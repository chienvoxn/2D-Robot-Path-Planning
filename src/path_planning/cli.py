
"""Command-line interface for 2D Robot Path Planning."""

import argparse

from .maps import create_grid_map
from .astar import astar
from .rrt import rrt
from .metrics import path_length
from .visualization import plot_search, animate_search
from .benchmark import benchmark, save_csv, print_summary


def main():
    # Configure CLI arguments
    parser = argparse.ArgumentParser(
        description="2D Robot Path Planning — A* vs RRT"
    )
    sub = parser.add_subparsers(dest="command", required=True)

    # Demo command
    demo = sub.add_parser(
        "demo",
        help="Run and visualize both algorithms",
    )
    demo.add_argument("--obstacles", type=int, default=500)
    demo.add_argument("--seed", type=int, default=42)
    demo.add_argument("--rrt-iterations", type=int, default=5000)
    demo.add_argument(
        "--gif",
        action="store_true",
        help="Export animations from actual algorithm histories",
    )

    # Benchmark command
    bench = sub.add_parser(
        "benchmark",
        help="Compare algorithms over repeated maps",
    )
    bench.add_argument("--runs", type=int, default=20)
    bench.add_argument("--rrt-iterations", type=int, default=5000)

    args = parser.parse_args()

    # Run benchmark
    if args.command == "benchmark":
        rows = benchmark(
            runs=args.runs,
            rrt_iterations=args.rrt_iterations,
        )

        save_csv(rows, "results/benchmark.csv")
        print_summary(rows)
        print("Saved results/benchmark.csv")
        return

    # Generate occupancy grid
    grid = create_grid_map(
        obstacle_count=args.obstacles,
        seed=args.seed,
    )
    start, goal = (5, 10), (95, 95)

    # Run A* and RRT
    solvers = [
        ("A*", lambda: astar(grid, start, goal)),
        (
            "RRT",
            lambda: rrt(
                grid,
                start,
                goal,
                max_iter=args.rrt_iterations,
                seed=args.seed,
            ),
        ),
    ]

    for name, solver in solvers:
        path, history = solver()

        print(
            f"{name}: success={path is not None}, "
            f"nodes={len(history)}, "
            f"path length={path_length(path)}"
        )

        stem = "astar" if name == "A*" else "rrt"

        # Save static visualization
        plot_search(
            grid,
            start,
            goal,
            path,
            history,
            name,
            f"results/{stem}_path.png",
        )

        # Export animated GIF if requested
        if args.gif:
            animate_search(
                grid,
                start,
                goal,
                path,
                history,
                name,
                f"assets/{stem}_demo.gif",
            )
            print(f"Saved assets/{stem}_demo.gif")

    print("Saved results/astar_path.png and results/rrt_path.png")


if __name__ == "__main__":
    main()
