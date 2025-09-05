import math

class GSEPlanner:
    class Node:
        def __init__(self, state, obstacles, si):
            self.eigen_state = state
            self.state = None
            self.shape_param = set()
            self.obs_size = len(obstacles)
            self.si = si
            for obs in obstacles:
                obs_rel = obs.colwise() - state
                rmin = max(math.sqrt(obs_rel.colwise().squaredNorm().minCoeff()) - 0.2, 0.0)
                obs_rel.colwise().normalize()
                normal = obs_rel.col(obs_rel.norm().argmin())
                obs_rel = normal.transpose() @ obs_rel
                theta = math.acos((obs_rel.array() > 0).select(obs_rel, 10).minCoeff())
                self.shape_param.add(self.obsParam(rmin, normal, theta))

        def shape(self, state):
            flag = False
            r = []
            f = []
            obs_rel = state - self.eigen_state
            for obs_param in self.shape_param:
                r.append(obs_param.satisfy(math.acos(obs_param.normal_.normalized() @ obs_rel)))
                if r[-1] == 0:
                    flag = True
                f.append(obs_param.satisfy(obs_param.rmin_ - obs_rel.norm()))
            if flag:
                if sum(r[1:], r[0]) == sum(f[1:], f[0]):
                    return True
                for i in range(1, self.obs_size):
                    if sum(r[i], 1) - 2 * r[i-1] + 1 == sum(f[i], f[i-1]):
                        return True
                return False
            else:
                return True

        def steer(self, state):
            if self.shape(state):
                return
            slope = (state - self.eigen_state).normalized()
            for obs_param in self.shape_param:
                state = self.eigen_state + slope * (obs_param.get() - obs_param.rmin_)
                if self.shape(state):
                    return

    class obsParam:
        def __init__(self, rmin, normal, theta):
            self.rmin_ = rmin
            self.normal_ = normal
            self.theta_ = theta

        def satisfy(self, x):
            return int(x > 0)

        def get(self):
            return self.rmin_

    def __init__(self, si, dim):
        self.dim_ = dim
        self.projector_ = None
        self.obstacles_ = []
        self.vertices_ = []
        self.nn_ = None
        self.adj_ = None
        self.specs_ = {"approximateSolutions": True}
        self.sampler_ = None
        self.si_ = si
        self.edges_ = []

    def solve(self, ptc):
        self.check_validity()
        r_state = self.si_.allocState()
        x_state = Eigen.VectorXd(self.dim_)
        x_near = Eigen.VectorXd(self.dim_)
        idx = 0
        solution = []

        goal = self.pdef_.getGoal().get()
        goal_s = dynamic_cast(goal, ompl.base.GoalState)
        if len(self.vertices_) == 0:
            self.projector_.project(goal_s.getState(), x_state)
            self.add_vertex(x_state)

        pis = self.pis_
        while pis.haveMoreStartStates():
            self.projector_.project(pis.nextStart(), x_state)
            self.add_vertex(x_state)

        if len(self.vertices_) <= 1:
            OMPL_ERROR("There are no valid initial states!")
            return ompl.base.PlannerStatus.INVALID_START

        if not self.sampler_:
            self.sampler_ = self.si_.allocStateSampler()

        OMPL_INFORM("Starting planning with %u states already in data structure", len(self.vertices_))

        while not ptc():
            self.sampler_.sampleUniform(r_state)
            self.projector_.project(r_state, x_state)
            idx = self.nn_.nearest(std.make_pair(x_state, -1)).second
            self.vertices_[idx].steer(x_state)
            idx = self.add_vertex(x_state)

            for i in range(idx):
                xN = self.vertices_[i].eigen_state_
                self.vertices_[idx].steer(xN)
                if self.vertices_[i].shape(xN):
                    xN = self.vertices_[idx].eigen_state_
                    self.vertices_[i].steer(xN)
                    if self.vertices_[idx].shape(xN):
                        dist = (self.vertices_[idx].eigen_state_ - self.vertices_[i].eigen_state_).norm()
                        self.adj_.addEdge(i, idx, dist)
                        self.adj_.addEdge(idx, i, dist)

            if self.adj_.inSameComponent(0, 1):
                self.adj_.dijkstra(1, 0, solution)
                break

        self.si_.freeState(r_state)
        OMPL_INFORM("Created %u states", len(self.vertices_))

        if len(solution) != 0:
            path = std.make_shared(ompl.geometric.PathGeometric(self.si_))
            for x in solution:
                path.append(self.vertices_[x].state_)
            self.pdef_.addSolutionPath(path, True, 0, self.getName())
            return ompl.base.PlannerStatus.EXACT_SOLUTION
        else:
            return ompl.base.PlannerStatus.TIMEOUT

    def clear(self):
        Planner.clear()
        self.sampler_.reset()
        if self.nn_:
            self.nn_.clear()
        self.vertices_ = []
        self.adj_.clear()

    def setup(self):
        Planner.setup()
        sc = ompl.tools.SelfConfig(self.si_, self.getName())

        if self.projector_.getDimension() != self.dim_:
            OMPL_ERROR("%s: Projector dimension does not match planner dimension: expected dimension %d but got %d",
                       self.getName(), self.dim_, self.projector_.getDimension())
            OMPL_INFORM("The GSE Algorithm works on the euclidean vectors space (R^n). To use GSE with OMPL, "
                        "the state spaces should have a default projector which maps to R^dim")

        if not self.nn_:
            self.nn_.reset(ompl.tools.SelfConfig.getDefaultNearestNeighbors(std.make_pair(Eigen.VectorXd, int)))

        self.nn_.setDistanceFunction(lambda a, b: self.distanceFcn(a, b))

    def getPlannerData(self, data):
        Planner.getPlannerData(data)

        data.addGoalVertex(ompl.base.PlannerDataVertex(self.vertices_[0].state_))
        data.addStartVertex(ompl.base.PlannerDataVertex(self.vertices_[1].state_))

        for i in range(len(self.vertices_)):
            nbrs = []
                        self.adj_.getNeighbors(i, nbrs)
            for j in nbrs:
                data.addEdge(ompl.base.PlannerDataVertex(self.vertices_[i].state_),
                             ompl.base.PlannerDataVertex(self.vertices_[j].state_))

    def add_vertex(self, state):
        idx = len(self.vertices_)
        self.vertices_.append(GSEPlanner.Node(state, self.obstacles_, self.si_))
        self.nn_.add(std.make_pair(state, idx


