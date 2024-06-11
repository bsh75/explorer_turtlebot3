#!/usr/bin/env python3
import numpy as np
from ros_parameters import MAP_HEIGHT, MAP_WIDTH, MAP_ORIGIN, MAP_RESOLUTION
from nav_msgs.msg import OccupancyGrid, Odometry
import rospy

class Window:
    def __init__(self, size, center_x, center_y):
        """Size is number of cells across x or y (square)
            Centers are indexs relative to global frame"""

        self.size = size
        self.center_x = center_x
        self.center_y = center_y

    def get_window_ranges(self):
        """Finds the min and max x and y indices of the window (capped by global map)."""
        min_ix = int(max(0, self.center_x - self.size // 2))
        max_ix = int(min(MAP_WIDTH - 1, self.center_x + self.size // 2))
        min_iy = int(max(0, self.center_y - self.size // 2))
        max_iy = int(min(MAP_HEIGHT -1, self.center_y + self.size // 2))
        return min_ix, max_ix, min_iy, max_iy

    def get_window_contents(self, global_map):
        """Gets the section of the global_map that the window covers."""
        min_ix, max_ix, min_iy, max_iy = self.get_window_ranges()
        window_contents = global_map[min_ix:max_ix + 1, min_iy:max_iy + 1]
        return window_contents

class SearchWindow(Window):
    def expand_window(self, expansion_factor):
        """Increases the size of the window by a factor."""
        self.size = int(expansion_factor * self.size)

    def shift_window(self, shift_x, shift_y):
        """Shifts the window in the direction of robot_yaw."""
        self.center_x += shift_x
        self.center_y += shift_y

    def get_unexplored_cells_global(self, global_map):
        """Returns unexplored cells within the window, converted to the global frame."""
        window_contents = self.get_window_contents(global_map)
        unexplored_cells = np.argwhere(window_contents == -1)
        rospy.logwarn(f"Search window \nsize: {self.size}\nX: {self.center_x}, Y: {self.center_y}")
        # Get origin offset from the map message
        
        global_unexplored_cells = []
        for (xi, yi) in unexplored_cells:
            xi_global = (xi + (self.center_x - self.size//2))  # Add 0.5 for cell center
            yi_global = (yi + (self.center_y - self.size//2))  # Add 0.5 for cell center
            global_unexplored_cells.append((xi_global, yi_global))
        return global_unexplored_cells


class CellWindow(Window):
    def is_unexplored(self, global_map):
        """Returns Boolean if all cells in the window are unexplored."""
        return np.all(self.get_window_contents(global_map) == -1)
    