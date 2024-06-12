#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import actionlib
from message_filters import ApproximateTimeSynchronizer, Subscriber
from goal_selecter import GoalSelector
from ros_parameters import RosParameters
from visualization import MarkerArrayDisplay


class ExplorationNode:
    def __init__(self):
        rospy.init_node('exploration_node')  # Initialize node (anonymous=True not needed)

        # Parameters
        self.ros_params = RosParameters()
        self.goal_selector = GoalSelector(
            self.ros_params, initial_window_size=30, expansion_factor=1.2, clearance=0.1
        )
        self.publish_rate = rospy.Rate(1)  # Hz (Goals published per second)

        # Move Base Client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        self.global_goal_msg = MoveBaseGoal()
        self.global_goal_msg.target_pose.header.frame_id = "map"
        self.global_goal_msg.target_pose.pose.orientation.w = 1.0

        # Message Filter Subscriptions
        map_sub = Subscriber("/map", OccupancyGrid)
        odom_sub = Subscriber("/odom", Odometry)
        ts = ApproximateTimeSynchronizer([map_sub, odom_sub], queue_size=10, slop=0.5)
        ts.registerCallback(self.sync_callback)


    def sync_callback(self, map_msg, odom_msg):
        goal = self.goal_selector.select_goal(map_msg, odom_msg.pose.pose)
        if goal:
            self.set_goal(goal)
            self.publish_goal()
            self.set_displays(goal)
        else:
            rospy.logwarn("Goal not found...")
        self.publish_rate.sleep()


    def set_goal(self, goal):
        goal_x, goal_y = goal
        self.global_goal_msg.target_pose.pose.position.x = goal_x
        self.global_goal_msg.target_pose.pose.position.y = goal_y        


    def publish_goal(self):
        self.global_goal_msg.target_pose.header.stamp = rospy.Time.now()
        self.move_base_client.send_goal(self.global_goal_msg, 
                                        done_cb=self.goal_done_callback,
                                        feedback_cb=self.goal_feedback)
        rospy.loginfo("New goal sent: x=%.2f, y=%.2f" % (self.global_goal_msg.target_pose.pose.position.x, 
                                                         self.global_goal_msg.target_pose.pose.position.y))

    def goal_feedback(self, feedback):
        """Callback function executed when goal feedback is received."""
        pass

    def goal_done_callback(self, state, result):
        """Callback function executed when the goal is done (reached or aborted)."""
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached successfully.")
        else:
            rospy.logwarn("Goal aborted or failed.")

    def set_displays(self, goal):
        small_displays = self.goal_selector.possible_locations
        resolution = self.ros_params.map_resolution
        markers = MarkerArrayDisplay(small_displays, goal, resolution, duration=1.0)
        markers.refresh_targets_and_goal()
        


if __name__ == '__main__':
    try:
        node = ExplorationNode()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unhandled exception: {e}")
