import rospy

# ---- PARAMETERS FROM ROS
ROBOT_RADIUS = rospy.get_param('~robot_radius', 0.3)  # Get the radius, default to 0.3 if not found
MAP_RESOLUTION = rospy.get_param('~resolution', 0.05)
MAP_HEIGHT = rospy.get_param('~map_height', 20)
MAP_WIDTH = rospy.get_param('~map_width', 20)