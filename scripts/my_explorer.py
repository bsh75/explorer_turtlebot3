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
from functions import *

# ---- ExplorationNode Class ----

class ExplorationNode:

    def __init__(self):
        rospy.init_node('exploration_node', anonymous=True)

        # Initialize action client for move_base
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()

        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.marker_pub = rospy.Publisher('/frontiers', MarkerArray, queue_size=100)

        self.global_map_data = None

        self.global_goal_msg = MoveBaseGoal()
        self.global_goal_msg.target_pose.header.frame_id = "map"
        self.global_goal_msg.target_pose.pose.orientation.w = 1.0 

        self.robot_pose = None

        # self.timer = rospy.Timer(rospy.Duration(3), self.goal_timer_callback)

    def map_callback(self, msg: OccupancyGrid):
        self.global_map_data = msg
        self.find_goal()


    def odom_callback(self, msg: Odometry):
        self.robot_pose = msg.pose.pose


    def find_goal(self):
        """Finds a goal location in the costmap by searching radially outwards from the robot."""
        costmap = self.global_map_data
        costmap_data = np.array(costmap.data).reshape(costmap.info.height, costmap.info.width)
        resolution = costmap.info.resolution

        search_size_cells = int(1 / resolution)
        expansion_factor = 2
        robot_x, robot_y, robot_yaw = self.get_robot_position_and_yaw()
        robot_radius_cells = int(np.ceil(0.3 / resolution))

        robot_x_idx, robot_y_idx = get_robot_indices(robot_x, robot_y, costmap.info.origin.position, resolution)
        half_size_cells = search_size_cells // 2

        while search_size_cells <= max(costmap.info.width, costmap.info.height):
            center_x_idx, center_y_idx = adjust_center(robot_x_idx, robot_y_idx, half_size_cells, robot_yaw)
            min_row, max_row, min_col, max_col = calculate_search_boundaries(center_x_idx, center_y_idx, half_size_cells, costmap.info.height, costmap.info.width)

            unexplored_areas = find_unexplored_areas(costmap_data, resolution, costmap.info.origin.position, min_row, max_row, min_col, max_col, robot_radius_cells)
            
            self.marker_array = MarkerArray() # Reset the marker array
            self.add_coord_markers(unexplored_areas, size=1, shape=Marker.CUBE)

            rospy.logwarn(f"{len(unexplored_areas)} possible goals eg: {unexplored_areas[0] if unexplored_areas else 'None'}")

            if unexplored_areas:
                goal = min(unexplored_areas, key=self._distance_to_robot)
                if goal not in unexplored_areas:
                    rospy.logerr(f"{goal} isnt one of options")
                
                goal_x, goal_y = goal

                self.add_coord_markers([goal], size=3, shape=Marker.SPHERE)
                self.marker_pub.publish(self.marker_array)
                self.global_goal_msg.target_pose.pose.position.x = goal_x
                self.global_goal_msg.target_pose.pose.position.y = goal_y
                self.publish_goal()
                return None # Leave function
            else:
                rospy.logwarn("No unexplored areas found within the search window. Expanding...")

            search_size_cells *= expansion_factor
            half_size_cells = search_size_cells // 2

        rospy.logerr("No valid goal found within the entire map!")
    

    def get_robot_position_and_yaw(self):
        """Retrieve robot's current position and yaw."""
        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y
        robot_yaw = euler_from_quaternion([
            self.robot_pose.orientation.x, self.robot_pose.orientation.y,
            self.robot_pose.orientation.z, self.robot_pose.orientation.w
        ])[2]
        return robot_x, robot_y, robot_yaw
    

    def _distance_to_robot(self, location):
        """Helper function to calculate Euclidean distance to the robot."""
        return np.sqrt((location[0] - self.robot_pose.position.x) ** 2 +
                       (location[1] - self.robot_pose.position.y) ** 2)

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
        

    def create_marker(self, x, y, marker_id, size, shape):
        """Create a visualization marker."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "exploration_goals"
        marker.id = marker_id
        marker.type = shape
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        marker.color.a = 1.0  # Don't forget to set the alpha!
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        return marker


    def publish_goal(self):
        self.global_goal_msg.target_pose.header.stamp = rospy.Time.now()
        self.move_base_client.send_goal(self.global_goal_msg, done_cb=self.goal_done_callback, feedback_cb=self.goal_feedback)
        rospy.logwarn(f"New goal set: {self.global_goal_msg}")


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


if __name__ == '__main__':
    try:
        node = ExplorationNode()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unhandled exception: {e}")
