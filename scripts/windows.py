#!/usr/bin/env python3
import numpy as np
from ros_parameters import MAP_HEIGHT, MAP_WIDTH


class Window:
    def __init__(self, size, center_x, center_y):
        self.size = size
        self.center_x = center_x
        self.center_y = center_y

    def get_window_ranges(self):
        """Finds the min and max x and y indices of the window (capped by global map)."""
        min_ix = max(0, self.center_x - self.size // 2)
        max_ix = min(MAP_WIDTH - 1, self.center_x + self.size // 2)
        min_iy = max(0, self.center_y - self.size // 2)
        max_iy = min(MAP_HEIGHT -1, self.center_y + self.size // 2)
        return min_ix, max_ix, min_iy, max_iy

    def get_window_contents(self, global_map):
        """Gets the section of the global_map that the window covers."""
        min_ix, max_ix, min_iy, max_iy = self.get_window_ranges()
        return global_map[min_ix:max_ix + 1, min_iy:max_iy + 1]

class SearchWindow(Window):
    def expand_window(self, expansion_factor):
        """Increases the size of the window by a factor."""
        self.size = int(expansion_factor * self.size)

    def shift_window(self, shift_x, shift_y):
        """Shifts the window in the direction of robot_yaw."""
        self.center_x += shift_x
        self.center_y += shift_y

    def get_unexplored_contents(self, global_map):
        """Returns unexplored cells within the window, converted to the global frame."""
        unexplored_cells = np.argwhere(self.get_window_contents(global_map) == -1)
        global_unexplored_cells = []
        for (x, y) in unexplored_cells:
            x_global = x + self.center_x
            y_global = y + self.center_y
            global_unexplored_cells.append((x_global, y_global))
        return global_unexplored_cells


class CellWindow(Window):
    def is_unexplored(self, global_map):
        """Returns Boolean if all cells in the window are unexplored."""
        return np.all(self.get_window_contents(global_map) == -1)
    