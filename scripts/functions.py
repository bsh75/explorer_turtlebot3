
def bresenham_line(x0, y0, x1, y1):
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx - dy

    line = []

    while x0 != x1 or y0 != y1:
        line.append((x0, y0))

        error2 = 2 * error
        if error2 > -dy:
            error -= dy
            x0 += sx
        if error2 < dx:
            error += dx
            y0 += sy

    line.append((x1, y1))  # Add the final point
    return line
