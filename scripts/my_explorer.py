#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

class ExplorationNode:
    def __init__(self):
        rospy.init_node('exploration_node', anonymous=True)

        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

        self.map_data = None
        self.robot_pose = None

        self.timer = rospy.Timer(rospy.Duration(3), self.goal_timer_callback)

        rospy.loginfo("Exploration node initialized")

    def map_callback(self, msg: OccupancyGrid):
        rospy.loginfo("---MAP CALLBACK")
        self.map_data = msg

    def odom_callback(self, msg: Odometry):
        rospy.loginfo("---ODOM CALLBACK")
        self.robot_pose = msg.pose.pose

    def find_unexplored_areas(self):
        if self.map_data is None:
            return None

        data = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))
        unexplored = np.where(data == -1)
        unexplored_points = list(zip(unexplored[0], unexplored[1]))

        return unexplored_points

    def select_goal(self, unexplored_points):
        if not unexplored_points or self.robot_pose is None:
            rospy.logerr("No unexplored points or robot pose available")
            return None

        rospy.logwarn("Selecting goal")

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

    def send_goal(self, goal):
        if goal is None:
            return

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.pose.position.x = goal[0]
        goal_msg.pose.position.y = goal[1]
        goal_msg.pose.orientation.w = 1.0

        self.goal_pub.publish(goal_msg)
        rospy.loginfo(f"New goal set at: x={goal[0]}, y={goal[1]}")

    def goal_timer_callback(self, event):
        try:
            rospy.logerr("Exploring started")
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
