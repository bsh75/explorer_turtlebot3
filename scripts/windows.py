#!/usr/bin/env python3
import numpy as np
import rospy


class Window:
    """
    Represents a rectangular window or region within a map.
    """

    def __init__(self, size, center_x, center_y, map_size):
        """
        Initializes a Window object.

        Args:
            size (int): The side length of the square window in cells.
            center_x (int): The x-coordinate (column) of the window's center in cell indices.
            center_y (int): The y-coordinate (row) of the window's center in cell indices.
            map_size (tuple): (width, height) of the map in cell indices.
        """
        self.size = size
        self.center_x = center_x
        self.center_y = center_y
        self.map_width, self.map_height = map_size

    def get_window_ranges(self):
        """
        Calculates the minimum and maximum row and column indices of the window,
        ensuring they stay within the bounds of the map.

        Returns:
            tuple: (min_col, max_col, min_row, max_row) in cell indices.
        """
        min_col = max(0, self.center_x - self.size // 2)
        max_col = min(self.map_width - 1, self.center_x + self.size // 2)
        min_row = max(0, self.center_y - self.size // 2)
        max_row = min(self.map_height - 1, self.center_y + self.size // 2)
        return min_col, max_col, min_row, max_row

    def get_window_contents(self, global_map):
        """
        Extracts the portion of the occupancy grid map that falls within the window's boundaries.

        Args:
            global_map (numpy.ndarray): The occupancy grid map as a 2D NumPy array.

        Returns:
            numpy.ndarray: A 2D NumPy array representing the window's contents.
        """
        min_col, max_col, min_row, max_row = self.get_window_ranges()
        return global_map[min_row:max_row + 1, min_col:max_col + 1]


class SearchWindow(Window):
    """
    A specialized Window for searching for unexplored areas in a map.
    """

    def expand_window(self, expansion_factor):
        """
        Expands the size of the search window by the given factor.

        Args:
            expansion_factor (float): The factor to multiply the window size by.
        """
        self.size = int(expansion_factor * self.size)

    def shift_window(self, shift_x, shift_y):
        """
        Shifts the center of the search window by the given amounts.

        Args:
            shift_x (int): The amount to shift the window center in the x direction (columns).
            shift_y (int): The amount to shift the window center in the y direction (rows).
        """
        self.center_x += shift_x
        self.center_y += shift_y

    def get_unexplored_cells_global(self, global_map):
        """
        Finds unexplored cells (-1 values) within the window and returns their coordinates in the global map frame.

        Args:
            global_map (numpy.ndarray): The occupancy grid map as a 2D NumPy array.

        Returns:
            list: A list of tuples representing the (x, y) coordinates (in meters) of unexplored cells.
        """
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
    """
    A small Window representing a single cell and its immediate surroundings.
    """

    def is_unexplored(self, global_map):
        """
        Checks if all cells within this window (including the center cell) are unexplored (-1).

        Args:
            global_map (numpy.ndarray): The occupancy grid map as a 2D NumPy array.

        Returns:
            bool: True if all cells are unexplored, False otherwise.
        """
        return np.all(self.get_window_contents(global_map) == -1)
