#!/usr/bin/env python3
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion
from windows import SearchWindow, CellWindow
from ros_parameters import MAP_HEIGHT, MAP_RESOLUTION, MAP_WIDTH, ROBOT_RADIUS, MAP_ORIGIN


class GoalSelector:
    def __init__(self, initial_window_size=10, expansion_factor=1.2, clearance=0.1):
        # Configuration attributes
        self.expansion_factor = expansion_factor  # Factor at which search size increases
        self.clearance = clearance # (m)
        self.initial_window_size = initial_window_size

        # Attributes that are set when Parent node updates
        self.map_2D_array = None
        self.robot_index_x = None
        self.robot_index_y = None
        self.search_window = None

        # Attributes that GoalSelector finds (these are in meters)
        self.possible_locations = None
        self.goal = None


    def get_target_locations_in_window(self, unexplored_cells):
        """Gets a list of possible targets from the window"""
        rospy.loginfo("Getting target locations in the window...")
        explorable_locations = []

        if len(unexplored_cells) == 0:
            rospy.logwarn("No unexplored cells found.")
            return explorable_locations
        
        # Find Robot diameter in terms of cells to be used as size for CellWindow
        robot_diameter = 2 * int(np.ceil((ROBOT_RADIUS + self.clearance) / MAP_RESOLUTION))
        rospy.loginfo(f"CELL WINDOW SIZE: {robot_diameter}")

        for cell in unexplored_cells:
            x_global, y_global = cell
            cell_window = CellWindow(robot_diameter, x_global, y_global)
            if cell_window.is_unexplored(self.map_2D_array):
                explorable_locations.append(self.indices_to_meters(cell))
        # rospy.loginfo(f"{unexplored_cells[0]} becomes: {explorable_locations[0]} location")
        return explorable_locations


    def distance_to_robot(self, location):
        """Helper function to calculate distance to the robot (m)"""
        return np.sqrt((location[0] - self.robot_index_x) ** 2 +
                       (location[1] - self.robot_index_y) ** 2)


    def select_goal(self, map_msg, odom_pose):
        """Selects the location furthest in the direction the robot is facing."""
        rospy.loginfo("Selecting goal...")
        self.update_goal_selector(map_msg, odom_pose)

        if not self.possible_locations:
            rospy.logwarn("No possible locations found.")
            return None

        # Get robot's orientation in the map frame
        robot_yaw = euler_from_quaternion([odom_pose.orientation.x, odom_pose.orientation.y,
                                        odom_pose.orientation.z, odom_pose.orientation.w])[2]

        def goal_score(location):
            """Calculates a score for each location based on distance and alignment with yaw."""
            distance = self.distance_to_robot(location)  
            
            # Calculate angle to location and difference from robot's yaw
            dx = location[0] - self.robot_index_x
            dy = location[1] - self.robot_index_y
            angle_to_location = np.arctan2(dy, dx)
            angle_diff = abs(robot_yaw - angle_to_location)
            angle_diff = min(angle_diff, 2*np.pi - angle_diff)  # Normalize to [0, pi]

            # Weight distance and angle difference
            return distance - 0.2 * angle_diff  # Adjust weights as needed

        # Find the index of the location with the highest score (furthest in yaw direction)
        target_location_index = max(range(len(self.possible_locations)), key=lambda i: goal_score(self.possible_locations[i]))

        goal = self.possible_locations[target_location_index]
        return goal


    def update_possible_locations(self):
        """Finds a goal location in the costmap by searching radially outwards from the robot."""
        rospy.loginfo("Updating possible locations...")
        while self.search_window.size <= min(MAP_WIDTH, MAP_HEIGHT):
            unexplored_cells = self.search_window.get_unexplored_cells_global(self.map_2D_array) # relative to global frame
            possible_locations = self.get_target_locations_in_window(unexplored_cells)
            
            if len(possible_locations) > 0:
                self.possible_locations = possible_locations
                rospy.loginfo(f"{len(self.possible_locations)} locations out of {len(unexplored_cells)} cells")
                return
            else:
                rospy.logwarn("No unexplored areas found within the search window. Expanding...")
                self.search_window.expand_window(self.expansion_factor)

        rospy.logerr(f"No valid goal found within the entire map! SWS:{self.search_window.size}, {MAP_WIDTH}, {MAP_HEIGHT}")


    def indices_to_meters(self, cell):
        """Converts a cell thats in global indices to global meters"""
        # Convert target coordinates to global meters
        target_x_index, target_y_index = cell
        origin_x, origin_y = MAP_ORIGIN
        target_x_meters = (target_x_index + 0.5) * MAP_RESOLUTION + origin_x
        target_y_meters = (target_y_index + 0.5) * MAP_RESOLUTION + origin_y
        return (target_x_meters, target_y_meters)
    

    def meters_to_indices(self, coord):
        """Converts coordinat in meters to map indices."""
        map_origin_x, map_origin_y = MAP_ORIGIN
        x_meters, y_meters = coord
        x_idx = int((x_meters - map_origin_x)/ MAP_RESOLUTION)
        y_idx = int((y_meters - map_origin_y)/ MAP_RESOLUTION)
        rospy.loginfo(f"Robot position: x={x_meters}, y={y_meters}\nRobot pose indices: x={x_idx}, y={y_idx}")
        return x_idx, y_idx
    
    
    def shift_window_in_yaw_direction(self, robot_yaw):
        """Shifts the search window in the direction of robot_yaw by half the window size."""
        # Calculate the shift in the x and y directions
        shift_x = int((self.search_window.size // 2) * np.cos(robot_yaw))
        shift_y = int((self.search_window.size // 2) * np.sin(robot_yaw))

        # Update the search window's center coordinates
        self.search_window.shift_window(shift_x, shift_y)
        rospy.loginfo(f"Search window center after shift: x={self.search_window.center_x}, y={self.search_window.center_y}")


    def update_goal_selector(self, map_msg, odom_pose):
        rospy.loginfo("Updating goal selector...")
        rospy.loginfo(f"MAP INFO: {map_msg.info} ")
        rospy.loginfo(f"ODOM POSE: {odom_pose}")
        self.map_2D_array = np.array(map_msg.data).reshape(map_msg.info.height, map_msg.info.width)
        odom_coord = (odom_pose.position.x, odom_pose.position.y)
        self.robot_index_x, self.robot_index_y = self.meters_to_indices(odom_coord)
        
        self.search_window = SearchWindow(self.initial_window_size, self.robot_index_x, self.robot_index_y)
        robot_yaw = euler_from_quaternion([odom_pose.orientation.x, odom_pose.orientation.y, odom_pose.orientation.z, odom_pose.orientation.w])[2]
        self.shift_window_in_yaw_direction(robot_yaw)
        self.update_possible_locations()
