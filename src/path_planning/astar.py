
"""Eight-connected A* search with Euclidean heuristic."""

import heapq
import itertools
import math

from .collision import point_free, segment_free


# 8 possible movement directions and their costs
MOTIONS = [
    (dx, dy, math.hypot(dx, dy))
    for dx in (-1, 0, 1)
    for dy in (-1, 0, 1)
    if dx or dy
]


def astar(grid, start, goal, record=False):
    """Return (path or None, expanded positions). Coordinates are cell indices."""

    start, goal = tuple(start), tuple(goal)

    # Validate start and goal
    if (
        not point_free(grid, (start[0] + 0.5, start[1] + 0.5))
        or not point_free(grid, (goal[0] + 0.5, goal[1] + 0.5))
    ):
        raise ValueError("Start or goal is blocked or out of bounds")

    counter = itertools.count()
    frontier = [(math.dist(start, goal), next(counter), start)]

    costs = {start: 0.0}
    parents = {start: None}
    expanded = []
    closed = set()

    while frontier:
        # Expand the node with the lowest f = g + h
        _, _, current = heapq.heappop(frontier)

        if current in closed:
            continue

        closed.add(current)
        expanded.append(current)

        # Reconstruct path when goal is reached
        if current == goal:
            path = []

            while current is not None:
                path.append(current)
                current = parents[current]

            return path[::-1], expanded

        # Explore valid neighbors
        for dx, dy, step in MOTIONS:
            nxt = (current[0] + dx, current[1] + dy)

            if not point_free(grid, (nxt[0] + 0.5, nxt[1] + 0.5)):
                continue

            if not segment_free(
                grid,
                (current[0] + 0.5, current[1] + 0.5),
                (nxt[0] + 0.5, nxt[1] + 0.5),
            ):
                continue

            if nxt in closed:
                continue

            # Update cost and priority queue
            new_cost = costs[current] + step

            if new_cost < costs.get(nxt, float("inf")) - 1e-12:
                costs[nxt] = new_cost
                parents[nxt] = current

                heapq.heappush(
                    frontier,
                    (
                        new_cost + math.dist(nxt, goal),
                        next(counter),
                        nxt,
                    ),
                )

    return None, expanded
