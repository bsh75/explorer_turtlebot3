#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
import actionlib
from visualization_msgs.msg import Marker, MarkerArray



def find_unexplored_areas(costmap_data, resolution, origin_position, min_row, max_row, min_col, max_col, robot_radius_cells):
    """
    Finds unexplored areas in the search grid that are large enough for the robot.

    Args:
        costmap_data: 2D NumPy array representing the entire costmap.
        resolution: Resolution of the costmap (meters per cell).
        origin_position: Origin of the costmap in meters (x, y).
        min_row, max_row, min_col, max_col: Boundaries of the search area.
        robot_radius_cells: Robot's radius in cells.

    Returns:
        List of (x, y) coordinates in meters of the center of unexplored areas large enough for the robot, or None if none found.
    """
    # Extract search area
    search_grid = costmap_data[min_row:max_row + 1, min_col:max_col + 1]

    # Find potential unexplored cells
    unexplored_cells = np.argwhere(search_grid == -1)

    if not unexplored_cells.size:  # Check if any unexplored cells were found
        return None

    # Filter cells based on size and convert to map coordinates (meters)
    possible_goals = []
    for cell in unexplored_cells:
        row, col = cell  # Extract row and column

        # Check if the cell is part of a sufficiently large unexplored area
        if cell_is_big_enough(search_grid, row, col, robot_radius_cells):
            # Convert cell indices to map coordinates (meters) relative to global_frame
            x = (col + min_col) * resolution + origin_position.x + (resolution/2)
            y = (row + min_row) * resolution + origin_position.y + (resolution/2)
            possible_goals.append((x, y)) 

    rospy.logwarn(f"{len(possible_goals)} possible goals eg: {possible_goals[0]}")

    return possible_goals


def cell_is_big_enough(map_data, row, col, robot_radius_cells):
    # Extract the region around the cell
    region = map_data[
        row - robot_radius_cells : row + robot_radius_cells + 1,
        col - robot_radius_cells : col + robot_radius_cells + 1,
    ]

    # Check if all cells in the region are unexplored (-1)
    return np.all(region == -1)

class ExplorationNode:
    def __init__(self):
        rospy.init_node('exploration_node', anonymous=True)

        # Initialize action client for move_base
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()

        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.marker_pub = rospy.Publisher('/frontiers', MarkerArray, queue_size=10)
        self.marker_array = MarkerArray()

        self.global_map_data = None
        self.robot_pose = None

        self.timer = rospy.Timer(rospy.Duration(3), self.goal_timer_callback)

        rospy.loginfo("Exploration node initialized")

    def map_callback(self, msg: OccupancyGrid):
        self.global_map_data = msg

    def odom_callback(self, msg: Odometry):
        self.robot_pose = msg.pose.pose


    def find_goal(self):
        """
        Finds a goal location in the costmap by searching radially outwards from the robot.

        Returns:
            Tuple (x, y) representing the goal coordinates in meters, or None if no goal found.
        """
        costmap = self.global_map_data
        costmap_data = np.array(costmap.data).reshape(costmap.info.height, costmap.info.width)
        resolution = costmap.info.resolution

        search_size_cells = int(1 / resolution)  # Initial search size in cells
        expansion_factor = 2 # Double search with each iteration
        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y
        robot_yaw = euler_from_quaternion([
            self.robot_pose.orientation.x, self.robot_pose.orientation.y,
            self.robot_pose.orientation.z, self.robot_pose.orientation.w
        ])[2]

        robot_radius = 0.3  
        robot_radius_cells = int(np.ceil(robot_radius / resolution))

        # Robot position in cell indices
        robot_x_idx = int((robot_x - costmap.info.origin.position.x) / resolution)
        robot_y_idx = int((robot_y - costmap.info.origin.position.y) / resolution)

        half_size_cells = search_size_cells // 2

        while search_size_cells <= max(costmap.info.width, costmap.info.height):
            # Adjust center based on robot's orientation (in cell indices)
            center_x_idx = robot_x_idx + int(round(half_size_cells * np.cos(robot_yaw)))
            center_y_idx = robot_y_idx + int(round(half_size_cells * np.sin(robot_yaw)))

            # Calculate search window boundaries (in cell indices)
            min_row = max(0, center_y_idx - half_size_cells)
            max_row = min(costmap.info.height - 1, center_y_idx + half_size_cells)
            min_col = max(0, center_x_idx - half_size_cells)
            max_col = min(costmap.info.width - 1, center_x_idx + half_size_cells)

            # Find unexplored areas returns locations in m relative to global frame
            unexplored_areas = find_unexplored_areas(costmap_data, costmap.info.resolution, costmap.info.origin.position, min_row, max_row, min_col, max_col, robot_radius_cells)
            self.add_coord_markers(unexplored_areas)
            self.marker_pub.publish(self.marker_array)

            if unexplored_areas:
                # Select closest cell in the robot's direction, considering costmap origin and orientation
                target_location = max(unexplored_areas, key=lambda location:
                    # Calculate Euclidean distance to the robot
                    np.sqrt((location[0] - robot_x) ** 2 + (location[1] - robot_y) ** 2)
                )
                
                goal_x = target_location[1]
                goal_y = target_location[0]
                selected_goal = (goal_x, goal_y)
                
                rospy.logwarn(f"\nSelected goal: {selected_goal}\nROBOT: {self.robot_pose.position}")
                
                self.add_coord_markers([selected_goal], size=3, shape=Marker.SPHERE)
     
                self.marker_pub.publish(self.marker_array)
                rospy.logwarn("published marker array")
                return selected_goal

            # Expand the search area if no goal is found
            search_size_cells *= expansion_factor
            half_size_cells = search_size_cells // 2

        return None  # No suitable goal found


    def add_coord_markers(self, coords, size=1, shape=Marker.CUBE):
        try:
            # Publish a marker for a list of coordinates
            for i, (x, y) in enumerate(coords):
                try:
                    marker = self.create_marker(x, y, i+1, size*self.global_map_data.info.resolution, shape)
                except Exception as e:
                    rospy.logerr(f"Error in creating_marker: {e}")

                self.marker_array.markers.append(marker)
        except Exception as e:
            rospy.logerr(f"Markers wrong {e}")
        


    def create_marker(self, x, y, id, size, shape=Marker.CUBE, duration=3.0): 
        # Return a marker at a given coordinate
        marker = Marker()
        marker.header.frame_id = "map"  # Frame of the map
        marker.header.stamp = rospy.Time.now()
        marker.ns = "frontiers"
        marker.id = id
        marker.type = shape  # or Marker.SPHERE, Marker.CYLINDER, etc.
        marker.action = Marker.ADD
        marker.pose.position.x = x # * self.global_map_data.info.resolution + self.global_map_data.info.origin.position.x
        marker.pose.position.y = y # * self.global_map_data.info.resolution + self.global_map_data.info.origin.position.y
        marker.pose.orientation.w = 1.0
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = 0.1  # Adjust for visualization
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration(duration)
        return marker


    def send_goal(self, goal_coords):
        if goal_coords is None:
            return
        new_goal_msg = MoveBaseGoal()
        new_goal_msg.target_pose.header.frame_id = "map" # Baselink is robots reference fram, map is global
        new_goal_msg.target_pose.header.stamp = rospy.Time.now()
        new_goal_msg.target_pose.pose.position.y = goal_coords[0]
        new_goal_msg.target_pose.pose.position.x = goal_coords[1]
        new_goal_msg.target_pose.pose.orientation.w = 1.0

        self.move_base_client.send_goal(new_goal_msg, done_cb=self.goal_done_callback, feedback_cb=self.goal_feedback)

        rospy.logwarn(f"New goal set at: x={goal_coords[0]}, y={goal_coords[1]}")


    def goal_feedback(self, feedback):
        # Call back function executed when goal is set
        # rospy.loginfo(f"----Timer: {self.timer}")
        pass


    def goal_done_callback(self, state, result):
        # Callback function to be executed when the goal is done (reached or aborted)
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo(f"Goal RESULT: {result}")
        else:
            rospy.logwarn("Goal aborted")


    def goal_timer_callback(self, event):
        try:
            goal = self.find_goal()
            self.send_goal(goal)
        except Exception as e:
            rospy.logerr(f"Error in goal_timer_callback: {e}")


if __name__ == '__main__':
    try:
        node = ExplorationNode()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unhandled exception: {e}")
