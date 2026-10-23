
"""Performance metrics for path planning algorithms."""

import math
import statistics


def path_length(path):
    """Calculate the total Euclidean length of a path."""
    if path is None:
        return None

    return sum(
        math.dist(a, b)
        for a, b in zip(path, path[1:])
    )


def summarize(rows):
    """Summarize benchmark results."""
    success = [r for r in rows if r["success"]]

    return {
        "runs": len(rows),
        "success_rate": (
            100 * len(success) / len(rows) if rows else 0
        ),
        "mean_time_ms": (
            statistics.mean(r["time_ms"] for r in rows)
            if rows else None
        ),
        "mean_path_length": (
            statistics.mean(r["path_length"] for r in success)
            if success else None
        ),
        "mean_nodes": (
            statistics.mean(r["nodes"] for r in rows)
            if rows else None
        ),
    }
