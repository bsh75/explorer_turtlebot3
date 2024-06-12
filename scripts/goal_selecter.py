import numpy as np
from tf.transformations import euler_from_quaternion
from windows import SearchWindow, CellWindow
from nav_msgs.msg import OccupancyGrid, Odometry
from ros_parameters import RosParameters
import rospy


class GoalSelector:
    def __init__(self, ros_params: RosParameters, initial_window_size=10, expansion_factor=1.2, clearance=0.1):
        """
        Initializes the GoalSelector with configuration parameters and ROS node information.
        
        Args:
            ros_params (RosParameters): Instance of RosParameters containing ROS parameter values.
            initial_window_size (int, optional): Initial size of the search window (in cells).
            expansion_factor (float, optional): Factor by which the search window expands if no goals are found.
            clearance (float, optional): Safety distance (in meters) around the robot.
        """
        # Configuration attributes
        self.ros_params = ros_params
        self.expansion_factor = expansion_factor  # Factor at which search size increases
        self.clearance = clearance # (m)
        self.initial_window_size = initial_window_size

        # Attributes that are set when Parent node updates
        self.global_map_origin = None
        self.global_map_size = None
        self.map_2D_array = None
        self.robot_coord = None
        self.robot_index = None
        self.robot_yaw = None

        # Attributes that GoalSelector updates (these are in meters)
        self.coords_in_window = [] # Undiscovered coordinates (global) in window
        self.filtered_coords = [] # Coords that have free space around them


    def update_goal_selector(self, map_msg: OccupancyGrid, odom_pose):
        """
        Updates the GoalSelector with the latest map data and robot pose information.
        
        Args:
            map_msg (OccupancyGrid): ROS message containing the map data.
            odom_pose (Odometry): ROS message containing the robot's pose (position and orientation).
        """
        rospy.loginfo("Updating goal selector...")
        rospy.loginfo(f"MAP INFO: {map_msg.info} ")
        rospy.loginfo(f"ODOM POSE: {odom_pose}")
        self.global_map_origin = (map_msg.info.origin.position.x, map_msg.info.origin.position.y)
        self.global_map_size = (map_msg.info.width, map_msg.info.height)
        self.map_2D_array = np.array(map_msg.data).reshape(map_msg.info.height, map_msg.info.width)
        
        self.robot_coord = (odom_pose.position.x, odom_pose.position.y)
        self.robot_index = self.meters_to_indices(self.robot_coord)
        self.robot_yaw = euler_from_quaternion([odom_pose.orientation.x, odom_pose.orientation.y, odom_pose.orientation.z, odom_pose.orientation.w])[2]


    def meters_to_indices(self, coord):
        """
        Converts a coordinate in meters (x, y) to the corresponding indices (row, column) in the map.

        Args:
            coord (tuple): A tuple (x, y) representing the coordinate in meters.

        Returns:
            tuple: A tuple (row, col) representing the indices in the map.
        """
        map_origin_x, map_origin_y = self.global_map_origin
        x_meters, y_meters = coord
        x_idx = int((x_meters - map_origin_x)/ self.ros_params.map_resolution)
        y_idx = int((y_meters - map_origin_y)/ self.ros_params.map_resolution)
        return (x_idx, y_idx)
    
    
    def calculate_window_shift(self, window_size):
        """
        Shifts the search window in the direction of the robot's yaw angle.

        Args:
            robot_yaw (float): The robot's yaw angle in radians.
        """
        
        # Calculate the shift in the x and y directions
        shift_x = int((window_size // 2) * np.cos(self.robot_yaw))
        shift_y = int((window_size // 2) * np.sin(self.robot_yaw))

        return shift_x, shift_y


    def expanding_search(self):
        """
        Searches for possible goal locations (frontiers) in the map by expanding a search window outward from the robot's position.
        The search prioritizes areas that are farther away and aligned with the robot's current heading.
        """
        rospy.loginfo("Updating possible locations...")
        ix, iy = self.robot_index
        search_window = SearchWindow(self.initial_window_size, ix, iy, self.global_map_size)
        rospy.loginfo(f"SEARCH WINDOW ORIGINAL: {search_window.get_window_ranges()}")

        map_x, map_y = self.global_map_size
        while search_window.size <= min(map_x, map_y):
            shift_x, shift_y = self.calculate_window_shift(search_window.size)
            search_window.shift_window(shift_x, shift_y)
            rospy.loginfo(f"SEARCH WINDOW IS: {search_window.get_window_ranges()}")

            cells_in_window = search_window.get_unoccupied_cells_in_window(self.map_2D_array) # relative to global frame
            # rospy.loginfo(f"{len(cells_in_window)} must be no more than {self.initial_window_size**2}")
            self.coords_in_window = [self.indices_to_meters(cell) for cell in cells_in_window]
            self.filtered_coords = self.coords_in_window
            # rospy.loginfo(f"cells: {cells_in_window} coords: {self.coords_in_window}")

            self.filter_coord_list()

            if len(self.filtered_coords) > 0:
                rospy.loginfo(f"{len(self.filtered_coords)} filtered from {len(self.coords_in_window)} coords")
                return None
            else:
                rospy.logwarn("No unexplored areas found within the search window. Expanding...")
                search_window.expand_window(self.expansion_factor)

        rospy.logerr(f"No valid goal found within the entire map!")
    

    def indices_to_meters(self, cell):
        """
        Converts a cell index (row, col) in the map to the corresponding coordinates (x, y) in meters.

        Args:
            cell (tuple): A tuple (row, col) representing the cell index.

        Returns:
            tuple: A tuple (x, y) representing the coordinates in meters.
        """
        map_origin_x, map_origin_y = self.global_map_origin
        target_x_index, target_y_index = cell
        target_x_meters = (target_x_index + 0.5) * self.ros_params.map_resolution + map_origin_x
        target_y_meters = (target_y_index + 0.5) * self.ros_params.map_resolution + map_origin_y
        return (target_x_meters, target_y_meters)
    

    def filter_coord_list(self):
        """
        Filters unexplored cells within the search window and returns valid goal locations that provide enough clearance for the robot.

        Args:
            coords_in_window (list): A list of tuples representing the indices of cells in window.

        Returns:
            list: A list of tuples representing the coordinates (in meters) of valid goal locations.
        """
        rospy.loginfo("Getting target locations in the window...")
        if len(self.coords_in_window) == 0:
            rospy.logwarn("No unexplored cells found.")
            return None
        
        self.filtered_coords = []
        # Find Robot diameter in terms of cells to be used as size for CellWindow
        window_size = self.ros_params.robot_diameter_cells
        rospy.loginfo(f"CELL WINDOW SIZE: {window_size}")

        for coord in self.coords_in_window:
            # Cell must be made with positional indices
            cell = self.meters_to_indices(coord)
            xi_global, yi_global = cell
            cell_window = CellWindow(window_size, xi_global, yi_global, self.global_map_size)
            if cell_window.is_unexplored(self.map_2D_array):
                self.filtered_coords.append(coord)


    def distance_to_robot(self, location):
        """
        Calculates the Euclidean distance between a loaction (x, y) and the robot's current position.

        Args:
            location (tuple): A tuple (x, y) representing the location in meters.

        Returns:
            float: The distance between the location and the robot in meters.
        """
        robot_x, robot_y = self.robot_coord
        return np.sqrt((location[0] - robot_x) ** 2 +
                (location[1] - robot_y) ** 2)

    def get_goal(self, map_msg, odom_pose):
        """
        Interface function: Selects the most promising goal location for exploration from the possible locations.
        The selection prioritizes locations that are farther away from the robot and aligned with its heading.
        
        Args:
            map_msg (OccupancyGrid): ROS message containing the map data.
            odom_pose (Odometry): ROS message containing the robot's pose (position and orientation).

        Returns:
            tuple or None: A tuple (x, y) representing the goal location in meters, or None if no suitable goal is found.
        """
        rospy.loginfo("Selecting goal...")
        self.update_goal_selector(map_msg, odom_pose)
        self.expanding_search() # Starts search for suitible targets

        if len(self.filtered_coords) == 0:
            rospy.logwarn("No possible locations found.") 
            return None
        
        def goal_score(location):
            """Calculates a score for each location based on distance and alignment with yaw."""
            distance = self.distance_to_robot(location)  
            robot_x, robot_y = self.robot_coord
            # Calculate angle to location and difference from robot's yaw
            dx = location[0] - robot_x
            dy = location[1] - robot_y
            angle_to_location = np.arctan2(dy, dx)
            angle_diff = abs(self.robot_yaw - angle_to_location)
            angle_diff = min(angle_diff, 2*np.pi - angle_diff)  # Normalize to [0, pi]

            # Weight distance and angle difference
            return distance - 0.2 * angle_diff  # Adjust weights as needed

        # Find the index of the location with the highest score (furthest in yaw direction)
        target_location_index = max(range(len(self.filtered_coords)), key=lambda i: goal_score(self.filtered_coords[i]))

        goal = self.filtered_coords[target_location_index]
        return goal