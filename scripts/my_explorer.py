#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
import actionlib

class ExplorationNode:
    def __init__(self):
        rospy.init_node('exploration_node', anonymous=True)

        # Initialize action client for move_base
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()

        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.map_data = None
        self.robot_pose = None

        self.timer = rospy.Timer(rospy.Duration(3), self.goal_timer_callback)

        rospy.loginfo("Exploration node initialized")

    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg

    def odom_callback(self, msg: Odometry):
        self.robot_pose = msg.pose.pose

    def find_unexplored_areas(self):
        if self.map_data is None:
            rospy.logerr("Map data not available yet")
            return None

        data = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))
        unexplored = np.where(data == -1)
        unexplored_points = list(zip(unexplored[0], unexplored[1]))

        return unexplored_points




    def select_goal(self, unexplored_points):
        if not unexplored_points or self.robot_pose is None:
            rospy.logerr("No unexplored points or robot pose available")
            return None

        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y
        orientation_q = self.robot_pose.orientation
        _, _, robot_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        selected_goal = None
        min_distance = float('inf')
        angle_lim = np.pi / 2

        for (y, x) in unexplored_points:
            map_x = x * self.map_data.info.resolution + self.map_data.info.origin.position.x
            map_y = y * self.map_data.info.resolution + self.map_data.info.origin.position.y
            distance = np.sqrt((map_x - robot_x) ** 2 + (map_y - robot_y) ** 2)
            direction = np.arctan2(map_y - robot_y, map_x - robot_x)
            angle_diff = abs(direction - robot_yaw)

            if distance < min_distance and angle_diff < angle_lim:
                min_distance = distance
                selected_goal = (map_x, map_y)

        rospy.logwarn(f"Selected goal: {selected_goal}")
        return selected_goal

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

        # self.goal_pub.publish(goal_msg)
        rospy.loginfo(f"New goal set at: x={goal_coords[0]}, y={goal_coords[1]}")


    def goal_feedback(self, feedback):
        # Call back function executed when goal is set
        # rospy.loginfo(f"----Timer: {self.timer}")
        pass


    def goal_done_callback(self, state, result):
        # Callback function to be executed when the goal is done (reached or aborted)
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal RESULT:", result)
        else:
            rospy.logwarn("Goal aborted")


    def goal_timer_callback(self, event):
        try:
            unexplored_points = self.find_unexplored_areas()
            # rospy.loginfo(f"Unexplored points: {unexplored_points}")
            goal = self.select_goal(unexplored_points)
            rospy.loginfo(f"Selected goal: {goal}")
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
