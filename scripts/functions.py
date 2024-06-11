import numpy as np

def find_unexplored_areas(costmap_data, resolution, origin_position, min_row, max_row, min_col, max_col, robot_radius_cells):
    """
    Finds unexplored areas in the search grid that are large enough for the robot.

    Args:
        costmap_data: 2D NumPy array representing the entire costmap.
        resolution: Resolution of the costmap (meters per cell).
        origin_position: Origin of the costmap in meters (x, y).
        min_row, max_row, min_col, max_col: Boundaries of the search area.
        robot_radius_cells: Robot's radius in cells.

    Returns:
        List of (x, y) coordinates in meters of the center of unexplored areas large enough for the robot, or None if none found.
    """
    search_grid = costmap_data[min_row:max_row + 1, min_col:max_col + 1]
    unexplored_cells = np.argwhere(search_grid == -1)

    if not unexplored_cells.size:
        return None

    possible_goals = [
        cell_to_map_coordinates(row, col, min_row, min_col, resolution, origin_position)
        for row, col in unexplored_cells
        if cell_is_big_enough(search_grid, row, col, robot_radius_cells)
    ]

    return possible_goals if possible_goals else None


def cell_to_map_coordinates(row, col, min_row, min_col, resolution, origin_position):
    """Convert cell indices to map coordinates (meters)."""
    x = (col + min_col) * resolution + origin_position.x + (resolution / 2)
    y = (row + min_row) * resolution + origin_position.y + (resolution / 2)
    return x, y


def cell_is_big_enough(search_grid, row, col, robot_radius_cells):
    """Check if the cell is part of a sufficiently large unexplored area."""
    # Placeholder for actual implementation
    return True  # Replace with actual check


def get_robot_indices(robot_x, robot_y, origin_position, resolution):
    """Convert robot's position to cell indices."""
    robot_x_idx = int((robot_x - origin_position.x) / resolution)
    robot_y_idx = int((robot_y - origin_position.y) / resolution)
    return robot_x_idx, robot_y_idx


def adjust_center(robot_x_idx, robot_y_idx, half_size_cells, robot_yaw):
    """Adjust center based on robot's orientation."""
    center_x_idx = robot_x_idx + int(round(half_size_cells * np.cos(robot_yaw)))
    center_y_idx = robot_y_idx + int(round(half_size_cells * np.sin(robot_yaw)))
    return center_x_idx, center_y_idx


def calculate_search_boundaries(center_x_idx, center_y_idx, half_size_cells, height, width):
    """Calculate search window boundaries (in cell indices)."""
    min_row = max(0, center_y_idx - half_size_cells)
    max_row = min(height - 1, center_y_idx + half_size_cells)
    min_col = max(0, center_x_idx - half_size_cells)
    max_col = min(width - 1, center_x_idx + half_size_cells)
    return min_row, max_row, min_col, max_col

