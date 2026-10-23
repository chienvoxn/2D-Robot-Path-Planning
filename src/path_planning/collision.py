
"""Exact grid-cell collision checking for point robots."""

import math


def point_free(grid, point):
    """Check whether a point lies in a free grid cell."""
    x, y = point
    rows, cols = grid.shape

    return (
        0 <= x < rows
        and 0 <= y < cols
        and grid[math.floor(x), math.floor(y)] == 0
    )


def segment_free(grid, a, b):
    """Check whether a segment intersects any occupied grid cell."""
    if not point_free(grid, a) or not point_free(grid, b):
        return False

    x0, y0 = map(float, a)
    x1, y1 = map(float, b)

    rows, cols = grid.shape

    # Bounding box of candidate grid cells
    xmin = max(0, math.floor(min(x0, x1)) - 1)
    xmax = min(rows - 1, math.floor(max(x0, x1)) + 1)
    ymin = max(0, math.floor(min(y0, y1)) - 1)
    ymax = min(cols - 1, math.floor(max(y0, y1)) + 1)

    # Check segment intersection with occupied cells
    for i in range(xmin, xmax + 1):
        for j in range(ymin, ymax + 1):
            if grid[i, j] and _segment_intersects_box(
                x0, y0, x1, y1,
                i, j, i + 1, j + 1,
            ):
                return False

    return True


def _segment_intersects_box(
    x0, y0, x1, y1,
    xmin, ymin, xmax, ymax,
):
    """Liang-Barsky clipping; boundary contact counts as collision."""
    dx = x1 - x0
    dy = y1 - y0

    lo, hi = 0.0, 1.0

    for pv, qv in (
        (-dx, x0 - xmin),
        (dx, xmax - x0),
        (-dy, y0 - ymin),
        (dy, ymax - y0),
    ):
        if abs(pv) < 1e-12:
            if qv < 0:
                return False
        else:
            r = qv / pv

            if pv < 0:
                lo = max(lo, r)
            else:
                hi = min(hi, r)

            if lo > hi:
                return False

    return True
