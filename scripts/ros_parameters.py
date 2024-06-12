"""
ros_parameters.py: ROS parameter loader and manager for robotic exploration.

This module handles loading and accessing ROS parameters necessary for the robot's exploration behavior. 
It provides a convenient way to centralize parameter management and ensure consistent access across different parts of the exploration node.
"""

import rospy
import yaml
import numpy as np
import os
import rospkg # add rospack import


class RosParameters:
    def __init__(self):
        self.robot_type = rospy.get_param('~robot_type', 'waffle')  # Default to 'burger' if not specified

        # Get resolution from local_costmap_params.yaml (not 100% accurate for global map but close enough)
        self.local_costmap_params = self.load_yaml('explorer_turtlebot3', 'local_costmap_params.yaml')
        self.map_resolution = self.local_costmap_params['local_costmap']['resolution']

        # Load footprint parameters based on robot type and select abs max to be radius
        footprint_file = f'costmap_common_params_{self.robot_type}.yaml'
        footprint_params = self.load_yaml('explorer_turtlebot3', footprint_file)
        self.robot_radius = np.max(np.abs(np.array(footprint_params['footprint']).flatten()))
        self.robot_diameter_cells = 2 * int(np.ceil((self.robot_radius) / self.map_resolution))

    def load_yaml(self, package_name, file_name):
        """Loads parameters from a YAML file in a ROS package."""
        try:
            # Use rospack to find the package path
            rp = rospkg.RosPack()
            package_path = rp.get_path(package_name)
            
            config_folder = rospy.get_param("~config_folder", "param")
            file_path = os.path.join(package_path, config_folder, file_name)

            rospy.loginfo(f"Loading YAML file from: {file_path}")

            with open(file_path, 'r') as f:
                params = yaml.safe_load(f)
            return params

        except (rospy.ROSException, FileNotFoundError, yaml.YAMLError) as e:
            rospy.logerr(f"Error loading parameters from YAML file: {e}")
            return {}

