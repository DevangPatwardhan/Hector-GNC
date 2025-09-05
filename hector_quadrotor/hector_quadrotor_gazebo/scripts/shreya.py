import numpy as np
import matplotlib.pyplot as plt

# Data Generation and Setup
def setup_data():
    pt = np.array([0, 0, 0])  # position vector of transmitter wrt inertial frame
    pr = np.array([0, 0, 0])  # position vector of receiver wrt inertial frame
    m_vec = np.array([1, 0, 0])  # magnetic dipole moment
    num_points = 1000
    x_range = np.linspace(-30, 30, num_points)
    y_range = np.linspace(-30, 30, num_points)
    receiver_positions = np.array(np.meshgrid(x_range, y_range, [0])).T.reshape(-1, 3)
    B_scalar = np.zeros(len(receiver_positions))
    B_vectors = np.zeros((len(receiver_positions), 3))
    roll_t = 0  # Roll angle
    pitch_t = 0  # Pitch angle
    yaw_t = 0  # Yaw angle
    R_t_to_i = np.array([
        [np.cos(pitch_t)*np.cos(yaw_t), np.sin(roll_t)*np.sin(pitch_t)*np.cos(yaw_t) - np.cos(roll_t)*np.sin(yaw_t), np.cos(roll_t)*np.sin(pitch_t)*np.cos(yaw_t) + np.sin(roll_t)*np.sin(yaw_t)],
        [np.cos(pitch_t)*np.sin(yaw_t), np.sin(roll_t)*np.sin(pitch_t)*np.sin(yaw_t) + np.cos(roll_t)*np.cos(yaw_t), np.cos(roll_t)*np.sin(pitch_t)*np.sin(yaw_t) - np.sin(roll_t)*np.cos(yaw_t)],
        [-np.sin(pitch_t), np.sin(roll_t)*np.cos(pitch_t), np.cos(roll_t)*np.cos(pitch_t)]
    ])
    roll_r = 0  # Roll angle
    pitch_r = 0  # Pitch angle
    yaw_r = 0  
    R_r_to_i = np.array([
        [np.cos(pitch_r)*np.cos(yaw_r), np.sin(roll_r)*np.sin(pitch_r)*np.cos(yaw_r) - np.cos(roll_r)*np.sin(yaw_r), np.cos(roll_r)*np.sin(pitch_r)*np.cos(yaw_r) + np.sin(roll_r)*np.sin(yaw_r)],
        [np.cos(pitch_r)*np.sin(yaw_r), np.sin(roll_r)*np.sin(pitch_r)*np.sin(yaw_r) + np.cos(roll_r)*np.cos(yaw_r), np.cos(roll_r)*np.sin(pitch_r)*np.sin(yaw_r) - np.sin(roll_r)*np.cos(yaw_r)],
        [-np.sin(pitch_r), np.sin(roll_r)*np.cos(pitch_r), np.cos(roll_r)*np.cos(pitch_r)]
    ])
    turn_radius = float(input("Enter the turn radius (in meters): "))
    initial_point_input = input("Enter the initial point (x, y) comma-separated: ").split(',')
    initial_point = (float(initial_point_input[0]), float(initial_point_input[1]))
    for i, receiver_position in enumerate(receiver_positions):
        B_vectors[i] = get_arva_data(receiver_position, pt, R_t_to_i, R_r_to_i, m_vec)
        scalar_field_strength = np.linalg.norm(B_vectors[i])  
        B_scalar[i] = scalar_field_strength
    return pt, pr, m_vec, receiver_positions, B_scalar, R_t_to_i, R_r_to_i, turn_radius, initial_point

# Data Processing Functions
def get_arva_data(pr, pt, R_t_to_i, R_r_to_i, m_vec):
    R_i_to_t = np.transpose(R_t_to_i)
    r = pr - pt
    r = np.dot(R_i_to_t, r)
    A = np.array([[2*r[0]**2 - r[1]**2 - r[2]**2, 3*r[0]*r[1], 3*r[0]*r[2]],
                  [3*r[0]*r[1], 2*r[1]**2 - r[0]**2 - r[2]**2, 3*r[1]*r[2]],
                  [3*r[0]*r[2], 3*r[1]*r[2], 2*r[2]**2 - r[0]**2 - r[1]**2]])
    Am = np.dot(R_t_to_i, A)
    Am_x = A[0, 0]*m_vec[0] + A[0, 1]*m_vec[1] + A[0, 2]*m_vec[2]
    Am_y = A[1, 0]*m_vec[0] + A[1, 1]*m_vec[1] + A[1, 2]*m_vec[2]
    Am_z = A[2, 0]*m_vec[0] + A[2, 1]*m_vec[1] + A[2, 2]*m_vec[2]
    rd = np.linalg.norm(r)
    H = np.array([1/(4*np.pi*rd**5)*Am_x, 1/(4*np.pi*rd**5)*Am_y, 1/(4*np.pi*rd**5)*Am_z])
    R_i_to_r = np.transpose(R_r_to_i)
    Hb = np.dot(R_i_to_r, H)
    return Hb

def find_equidistant_points(turn_radius, initial_point, pt, R_t_to_i, R_r_to_i, m_vec, receiver_positions, B_scalar):
    center = (initial_point[0] - turn_radius, initial_point[1])
    angles = np.linspace(0, 2*np.pi, 100)
    x = center[0] + turn_radius * np.cos(angles)
    y = center[1] + turn_radius * np.sin(angles)
    plt.plot(x, y, color='blue', label='Trajectory')
    total_length = np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2))
    equidistant_length = total_length / 6
    accumulated_length = 0
    equidistant_points = []
    for i in range(len(x) - 1):
        length = np.sqrt((x[i+1] - x[i])**2 + (y[i+1] - y[i])**2)
        accumulated_length += length
        if accumulated_length >= equidistant_length:
            min_distance = float('inf')
            closest_position = None
            for position in receiver_positions:
                distance = np.linalg.norm(np.array([x[i], y[i], 0]) - np.array(position))
                if distance < min_distance:
                    min_distance = distance
                    closest_position = position
            equidistant_points.append(closest_position)
            accumulated_length = 0  # Reset accumulated length
    equidistant_points.append([x[-1], y[-1], 0])
    equidistant_points = np.array(equidistant_points)
    equidistant_points = np.round(equidistant_points).astype(int)
    return equidistant_points, equidistant_length

def calculate_equidistant_distances(equidistant_points_list):
    distances = []
    for i in range(len(equidistant_points_list) - 1):
        point1 = equidistant_points_list[i]
        point2 = equidistant_points_list[i + 1]
        distance = calculate_distance(point1, point2)
        distances.append(distance)
    return distances

def field_of_eq_point(equidistant_points, pt, R_t_to_i, R_r_to_i, m_vec):
    eq_field_strengths = []
    for eq_point in equidistant_points:
        field_strength = get_arva_data(eq_point, pt, R_t_to_i, R_r_to_i, m_vec)
        eq_field_strengths.append(np.linalg.norm(field_strength))
    return eq_field_strengths

def separate_dicts_and_lists(original_dict, decimals=0):
    separate_dicts = [{k: v} for k, v in original_dict.items()]
    value_lists = []
    for k, v in original_dict.items():
        rounded_unique_values = np.unique(np.round(v, decimals=decimals), axis=0)
        value_lists.append(rounded_unique_values.tolist()) 
    return separate_dicts, value_lists

def find_closest_scalar_indices(B_scalar, equidistant_points, pt, R_t_to_i, R_r_to_i, m_vec, receiver_positions):
    closest_indices = []
    receiver_positions_corresponding = []
    scalar_values_corresponding = []
    index_dict = {}  # Dictionary to store indices
    eq_field_strengths = field_of_eq_point(equidistant_points, pt, R_t_to_i, R_r_to_i, m_vec)
    scalar_tolerance = 1e-8
    for strength in eq_field_strengths:
        indices = np.where(np.abs(B_scalar - strength) < scalar_tolerance)[0]
        closest_indices.extend(indices)
        receiver_positions_corresponding.extend([receiver_positions[idx] for idx in indices])
        scalar_values_corresponding.extend([strength] * len(indices))  # Extend scalar values for each index
        index_dict[strength] = receiver_positions[indices]
    return closest_indices, receiver_positions_corresponding, scalar_values_corresponding, index_dict

def calculate_distance(point1, point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    return np.linalg.norm(point1 - point2)

def calculate_all_distances(list1, list2, distances):
    matching_points_list1 = []
    matching_points_list2 = []
    midpoints = []
    for i, point1 in enumerate(list1):
        distance = calculate_distance(point1, list2)
        midpoint = [(point1[0] + list2[0]) / 2, (point1[1] + list2[1]) / 2, (point1[2] + list2[2]) / 2]
        if np.abs(distance - distances[-1]) <= 0.3:  # Check if the distance is equal to the last value in distances
            matching_points_list1.append(point1)
            matching_points_list2.append(list2)
            midpoints.append(midpoint)
    return matching_points_list1, matching_points_list2, midpoints

def calulate_direction(pt, midpoints):
    direction_vectors = []
    for midpoint in midpoints:
        direction_vector = [midpoint[0] - pt[0], midpoint[1] - pt[1], midpoint[2] - pt[2]]
        direction_vectors.append(direction_vector)
    return direction_vectors

def plot_trajectory(equidistant_points):
    plt.scatter(equidistant_points[:, 0], equidistant_points[:, 1], color='r', label='Equidistant Points')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Equidistant Points on the Trajectory')
    plt.axis('equal')
    plt.grid(True)
    plt.legend(loc='upper right')
    plt.show()
   
def plot_data(receiver_positions, B_vectors_normalized, B_magnitude, equidistant_points, pt, matching_points_list1, matching_points_list2, direction_towards_transmitter):
    matching_points_list1 = np.array(matching_points_list1)
    matching_points_list2 = np.array(matching_points_list2)
    B_vectors_normalized = B_vectors_normalized / np.max(B_magnitude)
    angles = []
    plt.quiver(receiver_positions[:, 0], receiver_positions[:, 1], B_vectors_normalized[:, 0], B_vectors_normalized[:, 1], B_magnitude, cmap='viridis', scale=20)
    plt.scatter(pt[0],pt[1], color='r', label='transmitter')
    plt.scatter(matching_points_list1[:, 0], matching_points_list1[:, 1], color='green', label='Corresponding Points')
    plt.scatter(matching_points_list2[:, 0], matching_points_list2[:, 1], color='green', label='Corresponding Points')
    for direction, midpoint in zip(direction_towards_transmitter, midpoints):
        angle = np.degrees(np.arctan2(pt[1]-midpoint[1], pt[0]-midpoint[0]))
        angles.append(angle)
        plt.arrow(midpoint[0], midpoint[1], -midpoint[0]*0.5, -midpoint[0]*0.5, color='red', width=0.01, head_width=0.5, head_length=0.1)  # Plot arrow from midpoint towards transmitter
    print("\nangle:",angles)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Receiver Positions, Magnetic Field Vectors, and Trajectory')
    plt.axis('equal')
    plt.grid(True)
    plt.legend(loc='upper right')
    plt.show()
    

if __name__ == "__main__":
    pt, pr, m_vec, receiver_positions, B_scalar, R_t_to_i, R_r_to_i, turn_radius, initial_point = setup_data()
    B_vectors = np.array([get_arva_data(pr, pt, R_t_to_i, R_r_to_i, m_vec) for pr in receiver_positions])
    B_magnitude = np.linalg.norm(B_vectors, axis=1)
    B_vectors_normalized = B_vectors / B_magnitude[:, np.newaxis]
    equidistant_points, equidistant_length = find_equidistant_points(turn_radius, initial_point, pt, R_t_to_i, R_r_to_i, m_vec, receiver_positions, B_scalar)
    equidistant_points = np.array(equidistant_points)
    equidistant_points_list = equidistant_points.tolist()
    print("Equidistant Points:")
    for point in equidistant_points_list:
        print(point)
    distances = calculate_equidistant_distances(equidistant_points.tolist()) 
    closest_indices, receiver_positions_corresponding, scalar_values_corresponding, index_dict = find_closest_scalar_indices(B_scalar, equidistant_points, pt, R_t_to_i, R_r_to_i, m_vec, receiver_positions)
    separate_dicts, value_lists = separate_dicts_and_lists(index_dict)
    matching_points_list1, matching_points_list2, midpoints = calculate_all_distances(value_lists[-2], equidistant_points_list[-1], distances)
    direction_towards_transmitter = calulate_direction(pt, midpoints)
    plot_trajectory(equidistant_points)
    plot_data(receiver_positions, B_vectors_normalized, B_magnitude, equidistant_points, pt, matching_points_list1, matching_points_list2, direction_towards_transmitter)
    # Print matching points
    print("\nMatching Points:")
    for point1, point2 in zip(matching_points_list1, matching_points_list2):
        print("Point in list 1:", point1)
        print("Point in list 2:", point2)
