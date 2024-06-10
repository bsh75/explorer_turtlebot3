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

class ExplorationNode:
    def __init__(self):
        rospy.init_node('exploration_node', anonymous=True)

        # Initialize action client for move_base
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()

        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.frontier_pub = rospy.Publisher('/frontiers', MarkerArray, queue_size=10)

        self.global_map_data = None
        self.robot_pose = None

        self.timer = rospy.Timer(rospy.Duration(3), self.goal_timer_callback)

        rospy.loginfo("Exploration node initialized")

    def map_callback(self, msg: OccupancyGrid):
        self.global_map_data = msg

    def odom_callback(self, msg: Odometry):
        self.robot_pose = msg.pose.pose


    def find_unexplored_areas(self):
        if self.global_map_data is None:
            rospy.logerr("Map data not available yet")
            return None
        
        start_time = rospy.get_time()  # Record the start time

        # Robot size (in meters) - adjust these values to match your robot
        robot_radius = 0.3 # Radius of the waffle robot plus clearance doubled

        # Map data and dimensions
        width = self.global_map_data.info.width
        height = self.global_map_data.info.height
        global_map_data = np.array(self.global_map_data.data).reshape((height, width))
        resolution = self.global_map_data.info.resolution  # Get resolution from the map data

        # Calculate robot size and clearance in cells
        robot_radius_cells = int(np.ceil(robot_radius / resolution))
        rospy.logwarn(f"robot_radius cells: {robot_radius_cells}")

        # Initialize unexplored points list
        unexplored_points = []

        # Create a padded version of the map to handle edge cases easily
        padded_map = np.pad(global_map_data, pad_width=robot_radius_cells, mode='constant', constant_values=100)

        # Find unexplored areas
        unexplored_count = np.sum(global_map_data == -1)
        bound_breaks = 0
        occupied_breaks = 0

        # Identify unexplored cells
        unexplored_cells = np.argwhere(global_map_data == -1)

        for cell in unexplored_cells:
            y, x = cell

            # Extract the region around the cell
            region = padded_map[y:y + 2 * robot_radius_cells + 1, x:x + 2 * robot_radius_cells + 1]

            # Check if all cells in the region are unexplored (-1)
            if np.all(region == -1):
                unexplored_points.append((y, x))
            else:
                occupied_breaks += 1

        rospy.logerr(f"found {len(unexplored_points)} out of {unexplored_count}")
        rospy.logerr(f"Bound b {bound_breaks}, Occ B {occupied_breaks}")

        end_time = rospy.get_time()  # Record the end time
        execution_time = end_time - start_time  # Calculate the execution time

        rospy.loginfo(f"Execution time for find_unexplored_areas: {execution_time} seconds")
        return unexplored_points


    def find_frontiers(self):
        rospy.logerr("finding frontiers....")
        if self.global_map_data is None:
            rospy.logerr("Map data not available yet")
            return None

        # Get map data and dimensions
        width = self.global_map_data.info.width
        height = self.global_map_data.info.height
        global_map_data = np.array(self.global_map_data.data).reshape((height, width))

        # Find potential frontier cells
        frontiers = []
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if global_map_data[y][x] == -1:
                    neighbors = [
                        global_map_data[y - 1][x],
                        global_map_data[y + 1][x],
                        global_map_data[y][x - 1],
                        global_map_data[y][x + 1],
                    ]
                    if 0 in neighbors:
                        frontiers.append((y, x))

        # Cluster frontier cells into groups
        frontier_groups = []
        visited = set()
        for y, x in frontiers:
            if (y, x) not in visited:
                group = self.flood_fill(global_map_data, y, x, visited)
                frontier_groups.append(group)

        # Filter out small frontier groups (optional)
        min_group_size = 5  
        frontier_groups = [group for group in frontier_groups if len(group) >= min_group_size]

        # Convert frontiers to map coordinates
        frontiers_map_coords = []
        for group in frontier_groups:
            group_x, group_y = zip(*group)
            centroid_x = np.mean(group_x) * self.global_map_data.info.resolution + self.global_map_data.info.origin.position.x
            centroid_y = np.mean(group_y) * self.global_map_data.info.resolution + self.global_map_data.info.origin.position.y
            frontiers_map_coords.append((centroid_y, centroid_x))
        
        rospy.logerr(frontiers_map_coords)
        return frontiers_map_coords


    def show_coords(self, coords):
        # Publish a marker for a list of coordinates
        marker_array = MarkerArray()
        for i, (x, y) in enumerate(coords):
            try:
                marker = self.create_marker(x, y, i)
            except Exception as e:
                rospy.logerr(f"Error in creating_marker: {e}")

            marker_array.markers.append(marker)
        self.frontier_pub.publish(marker_array)
        rospy.logwarn("published marker array")


    def create_marker(self, x, y, id): 
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
        marker.scale.x = self.global_map_data.info.resolution * 3
        marker.scale.y = self.global_map_data.info.resolution * 3
        marker.scale.z = 0.1  # Adjust for visualization
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration(5.0)
        rospy.logwarn(f"made marker id: {id}")
        return marker


    def flood_fill(self, global_map_data, y, x, visited):
        group = []
        queue = [(y, x)]
        visited.add((y, x))

        while queue:
            current_y, current_x = queue.pop(0)
            group.append((current_y, current_x))
            for dy, dx in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                neighbor_y, neighbor_x = current_y + dy, current_x + dx
                if 0 <= neighbor_y < self.global_map_data.info.height and \
                0 <= neighbor_x < self.global_map_data.info.width and \
                global_map_data[neighbor_y][neighbor_x] == -1 and \
                (neighbor_y, neighbor_x) not in visited:
                    queue.append((neighbor_y, neighbor_x))
                    visited.add((neighbor_y, neighbor_x))
        return group


    def select_goal(self, potential_goals):
        if not potential_goals or self.robot_pose is None:
            rospy.logerr("No unexplored points or robot pose available")
            return None

        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y
        orientation_q = self.robot_pose.orientation
        _, _, robot_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        selected_goal = None
        min_distance = float('inf')
        angle_lim = np.pi / 2

        for (y, x) in potential_goals:
            map_x = x * self.global_map_data.info.resolution + self.global_map_data.info.origin.position.x
            map_y = y * self.global_map_data.info.resolution + self.global_map_data.info.origin.position.y
            distance = np.sqrt((map_x - robot_x) ** 2 + (map_y - robot_y) ** 2)
            direction = np.arctan2(map_y - robot_y, map_x - robot_x)
            angle_diff = abs(direction - robot_yaw)

            if distance < min_distance and angle_diff < angle_lim:
                min_distance = distance
                selected_goal = (map_x, map_y)
        rospy.logwarn(f"Selected goal: {selected_goal}")
        try:
            self.show_coords([selected_goal])
        except Exception as e:
            rospy.logerr(f"Error in show_coords: {e}")
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
            unexplored_points = self.find_unexplored_areas()
            # frontier_points = self.find_frontiers()
            goal = self.select_goal(unexplored_points)
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
