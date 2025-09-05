import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
import cv2
import yaml

def generate_occupancy_grid(png_file, yaml_file):
    # Load the image
    image = cv2.imread(png_file, cv2.IMREAD_GRAYSCALE)
    if image is None:
        rospy.logerr("Failed to load the PNG image.")
        return None

    # Load the YAML file
    with open(yaml_file, 'r') as f:
        yaml_data = yaml.safe_load(f)

    width = yaml_data.get('width', 0)
    height = yaml_data.get('height', 0)
    resolution = yaml_data.get('resolution', 0)
    origin = yaml_data.get('origin', [0, 0, 0])
    negate = yaml_data.get('negate', 0)
    occupied_thresh = yaml_data.get('occupied_thresh', 0)
    free_thresh = yaml_data.get('free_thresh', 0)

    # Convert origin to Pose message
    origin_pose = Pose()
    origin_pose.position.x = origin[0]
    origin_pose.position.y = origin[1]
    origin_pose.position.z = origin[2]
    origin_pose.orientation.w = 1.0

    occupancy_grid = OccupancyGrid()
    occupancy_grid.header = Header()
    occupancy_grid.header.frame_id = "map"
    occupancy_grid.info.width = width
    occupancy_grid.info.height = height
    occupancy_grid.info.resolution = resolution
    occupancy_grid.info.origin = origin_pose

    # Convert image to occupancy data
    map_data = []
    for row in image:
        for pixel in row:
            if pixel > occupied_thresh:
                map_data.append(100)  # Occupied
            elif pixel < free_thresh:
                map_data.append(0)  # Free
            else:
                map_data.append(-1)  # Unknown

    occupancy_grid.data = map_data

    return occupancy_grid

if __name__ == '__main__':
    rospy.init_node('occupancy_grid_publisher', anonymous=True)
    pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
    rate = rospy.Rate(1) # 1Hz

    png_file = 'simple.pgm'  # Replace with the path to your PNG image
    yaml_file = 'simple.yaml'  # Replace with the path to your YAML file

    occupancy_grid_msg = generate_occupancy_grid(png_file, yaml_file)

    while not rospy.is_shutdown():
        if occupancy_grid_msg is not None:
            occupancy_grid_msg.header.stamp = rospy.Time.now()
            pub.publish(occupancy_grid_msg)
        rate.sleep()
