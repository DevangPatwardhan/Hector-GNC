import numpy as np

def angle_between_vectors(X, Pi_X):
    dot_product = np.dot(X, Pi_X)
    cos_angle = dot_product / (np.linalg.norm(X) * np.linalg.norm(Pi_X))
    return np.arccos(np.clip(cos_angle, -1.0, 1.0))



def calculate_distance(X, Pi_X):
    # Calculate Euclidean distance between points X and Pi_X
    distance = np.linalg.norm(X - Pi_X)
    return distance


def construct_normal_vector(X, Pi_X):
    # Calculate the vector v from X to Pi_X
    v = Pi_X - X

    # Calculate the magnitude of v
    v_magnitude = np.linalg.norm(v)

    # Normalize the vector v to get the unit vector u
    u = v / v_magnitude

    # Construct the normal vector n_i;X by permuting components of the unit vector u
    n_i_X = np.array([u[1], u[2], -u[0]])

    return n_i_X

def calculate_ri(X, P_o , ni_X, theta_i):
    # Calculate V_i_X
    V_i_X = P_o - X
    # Calculate dot product1,1
    dot_product = np.dot(ni_X, V_i_X)
    # Calculate magnitudes
    magnitude_ni_X = np.linalg.norm(ni_X)
    magnitude_V_i_X = np.linalg.norm(V_i_X)
    # Calculate the angle
    #angle = np.arccos(np.clip(dot_product / (magnitude_ni_X * magnitude_V_i_X), -1.0, 1.0))
    angle = np.arccos((dot_product) / ((magnitude_ni_X) * (magnitude_V_i_X)))
    print("angle:", angle )
    # Calculate ri
    #ri = np.arccos(np.clip(dot_product / (magnitude_ni_X * magnitude_V_i_X), -1.0, 1.0)) - theta_i
    ri = np.arccos((dot_product) / ((magnitude_ni_X) * (magnitude_V_i_X))) - (theta_i)
    print("ri:",ri )

    #ri = angle - theta_i

    # Determine if Pi_X lies within the angle theta_i
    #if ri <= theta_i:
    if ri < 0:
        print("Point Pi_X lies within the angle theta_i")
    else:
        print("Point Pi_X lies outside the angle theta_i")

    return ri


# Get user input for X coordinates
X_coords = [float(coord) for coord in input("Enter coordinates of point X (comma separated x, y, z): ").split(',')]

# Get user input for Pi_X coordinates
Pi_X_coords = [float(coord) for coord in
               input("Enter coordinates of closest point on obstacle i to X (comma separated x, y, z): ").split(',')]

P_o = [float(coord) for coord in
               input("Enter coordinates of point point on obstacle i to X (comma separated x, y, z): ").split(',')]

# Convert input to numpy arrays
X = np.array(X_coords)
Pi_X = np.array(Pi_X_coords)

# Calculate distance ri_X
ri_X = calculate_distance(X, Pi_X)
print("Minimum distance from point X to obstacle i (ri_X):", ri_X)

# Construct normal vector n_i;X
n_i_X = construct_normal_vector(X, Pi_X)
print("Normal vector n_i;X:", n_i_X)


theta=angle_between_vectors(X, Pi_X)
print("angle between 2 vector:", theta)

#theta_i = float(input("Enter theta_i in radians: "))
theta_i = 1.74533

ri = calculate_ri(X, P_o, n_i_X, theta_i)
print("ri:", ri)