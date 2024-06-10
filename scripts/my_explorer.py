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
from functions import bresenham_line

def find_unexplored_areas(search_grid, robot_radius_cells):
    """
    Finds unexplored areas in the search grid that are large enough for the robot.

    Args:
        search_grid: 2D NumPy array representing the search area of the costmap.
        robot_radius: Radius of the robot in meters.
        resolution: Resolution of the costmap (meters per cell).

    Returns:
        List of (row, col) indices of the center of unexplored areas large enough for the robot, or None if none found.
    """
    # Find potential unexplored cells
    unexplored_cells = np.argwhere(search_grid == -1) 

    if not unexplored_cells.size:  # Check if any unexplored cells were found
        return None

    # Filter cells based on size
    possible_goals = []
    for cell in unexplored_cells:
        y, x = cell  # Extract row and column

        # Check if the cell is part of a sufficiently large unexplored area
        if cell_is_big_enough(search_grid, y, x, robot_radius_cells):
            possible_goals.append((y, x))  # Add the cell's coordinates

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

        self.target_pub = rospy.Publisher('/targets', MarkerArray, queue_size=10)

        self.window_pub = rospy.Publisher('/window', Marker, queue_size=10)

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
        initial_search_size = 100
        expansion_factor = 2
        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y
        robot_yaw = euler_from_quaternion([
            self.robot_pose.orientation.x, self.robot_pose.orientation.y,
            self.robot_pose.orientation.z, self.robot_pose.orientation.w
        ])[2]

        costmap = self.global_map_data
        costmap_data = np.array(costmap.data).reshape(costmap.info.height, costmap.info.width)
        resolution = costmap.info.resolution
        robot_radius = 0.3  
        robot_radius_cells = int(np.ceil(robot_radius / resolution))

        search_size = initial_search_size + (initial_search_size % 2 == 0)  # Ensure odd size
        half_size = search_size // 2

        while search_size <= max(costmap.info.width, costmap.info.height):
            # Calculate the center of the search area in cell coordinates
            center_x = int(round(robot_x / resolution - costmap.info.origin.position.x))
            center_y = int(round(robot_y / resolution - costmap.info.origin.position.y))

            # Adjust center based on robot's orientation (this is where the error was likely occurring)
            center_x += int(half_size * np.cos(robot_yaw))
            center_y += int(half_size * np.sin(robot_yaw))

            window = self.create_marker(center_x, center_y, "window", search_size)
            self.window_pub.publish(window)

            # Define search area boundaries, ensuring they stay within the costmap
            min_row = max(0, center_y - half_size)
            max_row = min(costmap.info.height - 1, center_y + half_size)  # -1 for correct indexing
            min_col = max(0, center_x - half_size)
            max_col = min(costmap.info.width - 1, center_x + half_size)

            # Extract the search area from the costmap
            search_grid = costmap_data[min_row:max_row + 1, min_col:max_col + 1]  # +1 for slicing

            # Debugging: Log shapes to identify mismatches 
            rospy.logdebug(f"search_grid shape: {search_grid.shape}")
            
            # Resize bitmask to match the search grid
            bitmask = np.ones(search_grid.shape, dtype=bool)
            search_grid = search_grid * bitmask

            # Mark unknown areas in search grid as occupied to avoid selecting goals outside the bitmask
            search_grid[search_grid == 0] = 100

            # Find unexplored areas
            unexplored_cells = find_unexplored_areas(search_grid, robot_radius_cells)
            
            if unexplored_cells:
                # Filter cells by size
                filtered_cells = [(y + min_row, x + min_col) for y, x in unexplored_cells if cell_is_big_enough(costmap_data, y + min_row, x + min_col, robot_radius_cells)]
                if filtered_cells:
                    # Select closest cell in the robot's direction
                    goal_cell = min(filtered_cells, key=lambda cell: 
                        ((cell[1] * resolution + costmap.info.origin.position.x - robot_x) / np.cos(robot_yaw))**2 +
                        ((cell[0] * resolution + costmap.info.origin.position.y - robot_y) / np.sin(robot_yaw))**2  # Distance along robot's direction
                    )
                    goal_x = goal_cell[1] * resolution + costmap.info.origin.position.x
                    goal_y = goal_cell[0] * resolution + costmap.info.origin.position.y
                    selected_goal = (goal_x, goal_y)
                    self.show_coords([selected_goal])
                    return selected_goal

            # Expand the search area if no goal is found
            search_size *= expansion_factor
            half_size = search_size // 2
            max_distance = np.sqrt(2) * search_size / 2

        return None  # No suitable goal found


    def show_coords(self, coords):
        # Publish a marker for a list of coordinates
        targets_array = MarkerArray
        for i, (x, y) in enumerate(coords):
            try:
                marker = self.create_marker(x, y, i, size=3*self.global_map_data.info.resolution)
            except Exception as e:
                rospy.logerr(f"Error in creating_marker: {e}")

            targets_array.markers.append(marker)
        self.target_pub.publish(targets_array)
        rospy.logwarn("published marker array")


    def create_marker(self, x, y, id, size): 
        # Return a marker at a given coordinate
        marker = Marker()
        marker.header.frame_id = "map"  # Frame of the map
        marker.header.stamp = rospy.Time.now()
        marker.ns = "frontiers"
        marker.id = id
        marker.type = Marker.CUBE  # or Marker.SPHERE, Marker.CYLINDER, etc.
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
        marker.lifetime = rospy.Duration(5.0)
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

        rospy.logerr(f"New goal set at: x={goal_coords[0]}, y={goal_coords[1]}")


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
            # possible_goals = self.find_unexplored_areas()
            goal = self.find_goal()
            rospy.logerr(f"Selected goal: {goal}")
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
