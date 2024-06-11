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
from message_filters import ApproximateTimeSynchronizer, Subscriber # add imports
from goal_selecter import GoalSelector
# from state_controller import StateController


class ExplorationNode:
    def __init__(self):
        rospy.init_node('exploration_node', anonymous=True)

        # Goal Selector and State Controller Instances
        self.goal_selector = GoalSelector(initial_window_size=10, expansion_factor=1.2, clearance=0.1)
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

        # Goal Message (for move_base)
        self.global_goal_msg = MoveBaseGoal()
        self.global_goal_msg.target_pose.header.frame_id = "map"
        self.global_goal_msg.target_pose.pose.orientation.w = 1.0 


    def sync_callback(self, map_msg: OccupancyGrid, odom_msg: Odometry):
        goal = self.goal_selector.select_goal(map_msg, odom_msg.pose.pose)

    
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
