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

class MapUtils:
    """Utility class for handling costmap operations."""

    @staticmethod
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

    @staticmethod
    def cell_is_big_enough(map_data, row, col, robot_radius_cells):
        # Extract the region around the cell
        region = map_data[
            row - robot_radius_cells : row + robot_radius_cells + 1,
            col - robot_radius_cells : col + robot_radius_cells + 1,
        ]

        # Check if all cells in the region are unexplored (-1)
        return np.all(region == -1)

class ExplorationNode:
    """ROS node for autonomous exploration."""

    def __init__(self):
        # ROS setup
        rospy.init_node('exploration_node')
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.frontier_pub = rospy.Publisher('/frontiers', MarkerArray, queue_size=10)
        self.global_map_data = None
        self.robot_pose = None
        self.timer = rospy.Timer(rospy.Duration(3), self.goal_timer_callback)

        # Parameters (consider moving to ROS parameter server)
        self.initial_search_size = 100
        self.expansion_factor = 2
        self.robot_radius = 0.3  # meters

        rospy.loginfo("Exploration node initialized")

    def map_callback(self, msg):
        self.global_map_data = msg

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose
        
    def find_goal(self):
        #... (find_goal implementation using self.initial_search_size, self.expansion_factor) 
        
    def send_goal(self, goal_coords):
        """Sends a navigation goal to move_base."""
        # ... (implementation same as before)

    def goal_feedback(self, feedback):
        # ... (optional feedback handling)

    def goal_done_callback(self, state, result):
        # ... (implementation same as before)

    def goal_timer_callback(self, event):
        """Periodically triggers the exploration logic."""
        if self.global_map_data and self.robot_pose:
            goal = self.find_goal()
            self.send_goal(goal)
        else:
            rospy.logwarn("Waiting for map and odometry data...")

    # Visualization methods (show_coords, create_marker) ... (no changes)

if __name__ == '__main__':
    try:
        ExplorationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 
