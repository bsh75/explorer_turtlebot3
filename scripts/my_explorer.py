#!/usr/bin/env python3

"""
my_explorer.py: ROS node for frontier-based autonomous exploration.

This node orchestrates the robot's exploration process:
- Subscribes to the map and odometry topics.
- Uses the GoalSelector to identify potential exploration goals.
- Publishes markers in RViz to visualize these goals.
- Sends goals to the 'move_base' action server for navigation.
"""

import rospy
import actionlib
import random
import os
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid, Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from message_filters import ApproximateTimeSynchronizer, Subscriber
from goal_selecter import GoalSelector
from ros_parameters import RosParameters
from visualization import GoalMarkerArray


class NodeStateManager:
    IDLE_THRESHOLD = 300  # 5 minutes of idle time

    def __init__(self):
        """
        Initializes the exploration node.
        
        Sets up ROS parameters, goal selector, move_base action client,
        marker publisher, and message filter subscribers.
        """
        rospy.init_node('exploration_node')
        self.ros_params = RosParameters()
        self.goal_selector = GoalSelector(self.ros_params, initial_window_size=50, expansion_factor=2, clearance=0.1)
        self.publish_rate = rospy.Rate(1)  # Publish new goals at 1 Hz

        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        self.global_goal_msg = MoveBaseGoal()
        self.global_goal_msg.target_pose.header.frame_id = "map"
        self.global_goal_msg.target_pose.pose.orientation.w = 1.0

        map_sub = Subscriber("/map", OccupancyGrid)
        odom_sub = Subscriber("/odom", Odometry)
        ts = ApproximateTimeSynchronizer([map_sub, odom_sub], queue_size=10, slop=0.5)
        ts.registerCallback(self.sync_callback)

        self.last_goal_time = rospy.Time.now()

        rospy.Timer(rospy.Duration(1), self.check_idle_time)

    def sync_callback(self, map_msg: OccupancyGrid, odom_msg: Odometry):
        """
        Callback triggered when new map and odometry messages arrive.

        This method selects a goal using the goal selector, sets it for navigation,
        and updates the marker display.
        """
        goal = self.goal_selector.get_goal(map_msg, odom_msg.pose.pose)
        if goal:
            self.set_goal(goal)
            self.publish_goal()
            self.set_displays(goal)
            self.last_goal_time = rospy.Time.now()
        else:
            rospy.logwarn("Goal not found...")

    def set_goal(self, goal):
        """
        Sets the coordinates of the goal message for move_base.
        Separate function to publishing allows for more flexibility in timing.
        """
        goal_x, goal_y = goal
        self.global_goal_msg.target_pose.pose.position.x = goal_x
        self.global_goal_msg.target_pose.pose.position.y = goal_y        

    def publish_goal(self):
        """
        Sends the goal message to the move_base action server.
        """
        self.global_goal_msg.target_pose.header.stamp = rospy.Time.now()
        self.move_base_client.send_goal(
            self.global_goal_msg, 
            done_cb=self.goal_done_callback,
            feedback_cb=self.goal_feedback
        )
        rospy.loginfo("New goal sent: x=%.2f, y=%.2f" % (self.global_goal_msg.target_pose.pose.position.x, 
                                                         self.global_goal_msg.target_pose.pose.position.y))

    def goal_feedback(self, feedback):
        """Callback for receiving goal feedback from move_base."""
        pass  # Currently not used, but can be implemented for custom logic

    def goal_done_callback(self, state, result):
        """
        Callback triggered when a goal is finished (reached or aborted).
        """
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached successfully.")
        else:
            rospy.logwarn("Goal aborted or failed.")
            self.handle_recovery()

    def handle_recovery(self):
        """
        Handles recovery by setting a new goal with a random offset.
        """
        rospy.logwarn("Rotate recovery failed. Setting a new random goal.")
        current_position = self.global_goal_msg.target_pose.pose.position
        random_offset_x = random.uniform(-1.0, 1.0)
        random_offset_y = random.uniform(-1.0, 1.0)
        new_goal_x = current_position.x + random_offset_x
        new_goal_y = current_position.y + random_offset_y
        self.set_goal((new_goal_x, new_goal_y))
        self.publish_goal()

    def set_displays(self, goal):
        """
        Updates the marker display in RViz with possible goal locations and the current goal.
        """
        markers = GoalMarkerArray()
        markers.display(self.goal_selector.filtered_coords, goal, self.ros_params.map_resolution, duration=1.0)   

    def check_idle_time(self, event):
        """
        Checks if the robot has been idle for too long and saves the map.
        """
        if (rospy.Time.now() - self.last_goal_time).to_sec() > self.IDLE_THRESHOLD:
            rospy.logwarn("Robot has been idle for too long. Saving the map.")
            self.save_map()

    def save_map(self):
        """
        Saves the current map to the 'generated_maps' folder.
        """
        try:
            rospy.wait_for_service('dynamic_map', timeout=10)
            get_map = rospy.ServiceProxy('dynamic_map', GetMap)
            response = get_map()

            map_dir = os.path.expanduser('~/generated_maps')
            if not os.path.exists(map_dir):
                os.makedirs(map_dir)

            map_file = os.path.join(map_dir, 'map.yaml')
            with open(map_file, 'w') as f:
                f.write(response.map.data)

            rospy.loginfo(f"Map saved to {map_file}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to save map: {e}")

if __name__ == '__main__':
    try:
        node = NodeStateManager()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unhandled exception: {e}")
