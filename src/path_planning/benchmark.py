
"""Repeatable comparisons on paired maps."""

import csv
from pathlib import Path
from time import perf_counter

from .maps import create_grid_map
from .astar import astar
from .rrt import rrt
from .metrics import path_length, summarize


def benchmark(levels=(100, 500, 1000), runs=20, rrt_iterations=5000):
    """Benchmark A* and RRT on the same seeded maps."""
    records = []

    for level in levels:
        for run in range(runs):
            # Generate the same map for both algorithms
            grid = create_grid_map(
                obstacle_count=level,
                seed=42 + run,
            )

            for name in ("A*", "RRT"):
                t0 = perf_counter()

                if name == "A*":
                    path, history = astar(grid, (5, 10), (95, 95))
                    count = len(history)
                else:
                    path, history = rrt(
                        grid,
                        (5, 10),
                        (95, 95),
                        max_iter=rrt_iterations,
                        seed=1000 + run,
                    )
                    count = len(history)

                # Record performance metrics
                ms = (perf_counter() - t0) * 1000

                records.append({
                    "algorithm": name,
                    "obstacles": level,
                    "run": run + 1,
                    "success": path is not None,
                    "time_ms": round(ms, 4),
                    "path_length": path_length(path),
                    "nodes": count,
                })

    return records


def save_csv(rows, output):
    """Save benchmark results to a CSV file."""
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w", newline="", encoding="utf8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "algorithm",
                "obstacles",
                "run",
                "success",
                "time_ms",
                "path_length",
                "nodes",
            ],
        )
        writer.writeheader()
        writer.writerows(rows)


def print_summary(rows):
    """Print summary statistics for each obstacle level."""
    for level in sorted({r["obstacles"] for r in rows}):
        for name in ("A*", "RRT"):
            filtered = [
                r for r in rows
                if r["obstacles"] == level
                and r["algorithm"] == name
            ]

            data = summarize(filtered)

            print(
                f"{level:5d} | {name:3s} | "
                f"success={data['success_rate']:5.1f}% | "
                f"time={data['mean_time_ms']:9.2f} ms | "
                f"path={data['mean_path_length']}"
            )
