#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import actionlib
from visualization_msgs.msg import Marker, MarkerArray
from message_filters import ApproximateTimeSynchronizer, Subscriber
from goal_selecter import GoalSelector
# from state_controller import StateController

from ros_parameters import RosParameters


class ExplorationNode:
    def __init__(self):
        rospy.init_node('exploration_node', anonymous=True)
        # ROS parameters from file
        self.ros_params = RosParameters()
        
        # Goal Selector and State Controller Instances
        self.goal_selector = GoalSelector(self.ros_params, initial_window_size=30, expansion_factor=1.2, clearance=0.1)
        # self.state_controller = StateController()

        # ROS Publishers and Action Client
        self.marker_pub = rospy.Publisher('/frontiers', MarkerArray, queue_size=100)
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        
        # Initialize the subscribers with message_filters
        map_sub = Subscriber("/map", OccupancyGrid)
        odom_sub = Subscriber("/odom", Odometry)
        ats = ApproximateTimeSynchronizer([map_sub, odom_sub], queue_size=10, slop=0.5)
        ats.registerCallback(self.sync_callback)

        # Callback rate
        self.rync_callback_rate = rospy.Rate(2)

        # Goal Message (for move_base)
        self.global_goal_msg = MoveBaseGoal()
        self.global_goal_msg.target_pose.header.frame_id = "map"
        self.global_goal_msg.target_pose.pose.orientation.w = 1.0

        # Marker array initialization
        self.marker_array = MarkerArray()



    def sync_callback(self, map_msg: OccupancyGrid, odom_msg: Odometry):
        goal = self.goal_selector.select_goal(map_msg, odom_msg.pose.pose)
        if goal:
            self.set_goal(goal)
            self.publish_goal()
            self.marker_array.markers.clear()  # Clear previous markers
            self.add_coord_markers(self.goal_selector.possible_locations)
            self.add_coord_markers([goal])
        else:
            rospy.logwarn("Goal not found...")
        self.rync_callback_rate.sleep()

    def set_goal(self, goal):
        """Sets the goal for move_base."""
        self.global_goal_msg.target_pose.pose.position.x = goal[0]
        self.global_goal_msg.target_pose.pose.position.y = goal[1]
        self.global_goal_msg.target_pose.pose.orientation.w = 1.0

    def add_coord_markers(self, coords, size=1, shape=Marker.CUBE):
        """Publish a marker for a list of coordinates."""
        try:
            for i, (x, y) in enumerate(coords):
                marker = self.create_marker(x, y, i + 1, size * self.ros_params.map_resolution, shape)
                self.marker_array.markers.append(marker)
            self.marker_pub.publish(self.marker_array)
        except Exception as e:
            rospy.logerr(f"Error in adding markers: {e}")

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
        """Send the goal to move_base."""
        self.global_goal_msg.target_pose.header.stamp = rospy.Time.now()
        self.move_base_client.send_goal(self.global_goal_msg, done_cb=self.goal_done_callback, feedback_cb=self.goal_feedback)
        rospy.loginfo(f"New goal set: {self.global_goal_msg.target_pose.pose.position.x}, {self.global_goal_msg.target_pose.pose.position.y}")

    def goal_feedback(self, feedback):
        """Callback function executed when goal feedback is received."""
        pass

    def goal_done_callback(self, state, result):
        """Callback function executed when the goal is done (reached or aborted)."""
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached successfully.")
        else:
            rospy.logwarn("Goal aborted or failed.")

if __name__ == '__main__':
    try:
        node = ExplorationNode()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unhandled exception: {e}")
