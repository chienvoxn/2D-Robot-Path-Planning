
"""Basic RRT with goal bias, fixed step size, and collision-free edges."""

from dataclasses import dataclass
import math

import numpy as np

from .collision import point_free, segment_free


@dataclass
class Node:
    x: float
    y: float
    parent: int | None = None


def nearest(nodes, target):
    """Find the nearest node to a target point."""
    return min(
        range(len(nodes)),
        key=lambda i: (
            (nodes[i].x - target[0]) ** 2
            + (nodes[i].y - target[1]) ** 2
        ),
    )


def steer(node, target, step_size):
    """Move from a node toward the target by at most step_size."""
    dx = target[0] - node.x
    dy = target[1] - node.y
    d = math.hypot(dx, dy)

    if d < 1e-12:
        return None

    scale = min(step_size, d) / d

    return node.x + dx * scale, node.y + dy * scale


def rrt(
    grid,
    start,
    goal,
    step_size=3.0,
    max_iter=5000,
    goal_threshold=3.0,
    goal_sample_rate=0.10,
    seed=42,
):
    """Return (path or None, tree nodes). Start/goal are grid cell indices."""

    # Validate parameters
    if (
        step_size <= 0
        or goal_threshold < 0
        or max_iter < 0
        or not 0 <= goal_sample_rate <= 1
    ):
        raise ValueError("Invalid RRT parameters")

    s = (start[0] + 0.5, start[1] + 0.5)
    g = (goal[0] + 0.5, goal[1] + 0.5)

    if not point_free(grid, s) or not point_free(grid, g):
        raise ValueError("Start or goal is blocked or out of bounds")

    nodes = [Node(*s)]

    if s == g:
        return [s], nodes

    rng = np.random.default_rng(seed)
    rows, cols = grid.shape

    for _ in range(max_iter):
        # Random sampling with goal bias
        sample = (
            g
            if rng.random() < goal_sample_rate
            else (rng.uniform(0, rows), rng.uniform(0, cols))
        )

        # Find nearest node and extend the tree
        parent_idx = nearest(nodes, sample)
        parent = nodes[parent_idx]
        candidate = steer(parent, sample, step_size)

        # Reject collision paths
        if candidate is None or not segment_free(
            grid,
            (parent.x, parent.y),
            candidate,
        ):
            continue

        nodes.append(Node(*candidate, parent_idx))

        # Connect to goal if reachable
        if (
            math.dist(candidate, g) <= goal_threshold
            and segment_free(grid, candidate, g)
        ):
            nodes.append(Node(*g, len(nodes) - 1))

            # Reconstruct path
            path = []
            idx = len(nodes) - 1

            while idx is not None:
                n = nodes[idx]
                path.append((n.x, n.y))
                idx = n.parent

            return path[::-1], nodes

    return None, nodes
