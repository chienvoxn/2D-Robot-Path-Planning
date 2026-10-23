
"""Occupancy grid generation. Array indexed as grid[x, y]."""

import numpy as np


def create_grid_map(
    size=(100, 100),
    obstacle_count=500,
    start=(5, 10),
    goal=(95, 95),
    seed=42,
):
    """Generate a reproducible 2D occupancy grid."""

    if len(size) != 2 or min(size) <= 0:
        raise ValueError("size must contain two positive dimensions")

    rows, cols = size

    # Validate start and goal positions
    for x, y in (start, goal):
        if not (0 <= x < rows and 0 <= y < cols):
            raise ValueError("start/goal outside map")

    if obstacle_count < 0:
        raise ValueError("obstacle_count must be nonnegative")

    grid = np.zeros(size, dtype=np.uint8)
    rng = np.random.default_rng(seed)

    # Place random obstacles (samples may repeat)
    for x, y in rng.integers(
        [0, 0],
        [rows, cols],
        size=(obstacle_count, 2),
    ):
        if (int(x), int(y)) not in (start, goal):
            grid[x, y] = 1

    return grid


def example_map():
    """Create a small grid with predefined obstacles."""

    grid = np.zeros((20, 20), dtype=np.uint8)

    grid[5:7, 5:7] = 1
    grid[12:16, 12:16] = 1

    return grid
