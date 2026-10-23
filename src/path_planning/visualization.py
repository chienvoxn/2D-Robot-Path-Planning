
"""Static plots and GIF animations from actual search histories."""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter


def _base(grid, start, goal, title):
    """Create the base map with start and goal markers."""
    fig, ax = plt.subplots(figsize=(6.5, 6.5))
    rows, cols = grid.shape

    ax.imshow(
        grid.T,
        origin="lower",
        cmap="gray_r",
        interpolation="none",
        extent=(0, rows, 0, cols),
        vmin=0,
        vmax=1,
    )

    ax.scatter(
        [start[0] + 0.5],
        [start[1] + 0.5],
        c="#1d4ed8",
        s=65,
        label="Start",
        zorder=6,
    )

    ax.scatter(
        [goal[0] + 0.5],
        [goal[1] + 0.5],
        c="#f59e0b",
        s=130,
        marker="*",
        label="Goal",
        zorder=6,
    )

    ax.set(
        xlim=(0, rows),
        ylim=(0, cols),
        xlabel="X",
        ylabel="Y",
        title=title,
    )

    ax.set_aspect("equal")
    ax.legend(loc="upper left")
    fig.tight_layout()

    return fig, ax


def plot_search(grid, start, goal, path, history, algorithm, output):
    """Save a static visualization of the search result."""
    fig, ax = _base(
        grid,
        start,
        goal,
        f"{algorithm} — 2D Path Planning",
    )

    if algorithm == "A*":
        # Plot expanded nodes
        if history:
            pts = np.array(history, dtype=float) + 0.5
            ax.scatter(
                pts[:, 0],
                pts[:, 1],
                s=3,
                c="#93c5fd",
                alpha=0.55,
            )

        if path:
            pts = np.array(path, dtype=float) + 0.5
            ax.plot(
                pts[:, 0],
                pts[:, 1],
                c="#ef4444",
                lw=2.2,
            )

    else:
        # Plot RRT tree edges
        for node in history:
            if node.parent is not None:
                parent = history[node.parent]

                ax.plot(
                    (parent.x, node.x),
                    (parent.y, node.y),
                    c="#16a34a",
                    lw=0.45,
                    alpha=0.45,
                )

        if path:
            pts = np.array(path)
            ax.plot(
                pts[:, 0],
                pts[:, 1],
                c="#ef4444",
                lw=2.4,
            )

    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)

    fig.savefig(output, dpi=140)
    plt.close(fig)


def animate_search(
    grid,
    start,
    goal,
    path,
    history,
    algorithm,
    output,
    frames=90,
    fps=15,
):
    """Export a GIF showing search progress and the final path."""
    fig, ax = _base(
        grid,
        start,
        goal,
        f"{algorithm} — Search Progress",
    )

    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)

    total = len(history)

    # Reserve 85% of frames for exploration
    explore_frames = max(1, int(frames * 0.85))

    if algorithm == "A*":
        dots = ax.scatter(
            [],
            [],
            s=5,
            c="#60a5fa",
            alpha=0.65,
        )
        line, = ax.plot([], [], c="#ef4444", lw=2.6)

        def update(frame):
            n = min(
                total,
                round(
                    total
                    * min(frame + 1, explore_frames)
                    / explore_frames
                ),
            )

            pts = (
                np.array(history[:n], dtype=float)
                .reshape(-1, 2) + 0.5
            )
            dots.set_offsets(pts)

            if frame >= explore_frames and path:
                route = np.array(path, dtype=float) + 0.5
                line.set_data(route[:, 0], route[:, 1])

            return dots, line

    else:
        from matplotlib.collections import LineCollection

        lines = LineCollection(
            [],
            colors="#16a34a",
            linewidths=0.7,
            alpha=0.65,
        )
        ax.add_collection(lines)

        line, = ax.plot([], [], c="#ef4444", lw=2.6)

        # Collect RRT tree edges
        segments = []

        for node in history:
            if node.parent is not None:
                parent = history[node.parent]
                segments.append(
                    (
                        (parent.x, parent.y),
                        (node.x, node.y),
                    )
                )

        def update(frame):
            n = round(
                len(segments)
                * min(frame + 1, explore_frames)
                / explore_frames
            )

            lines.set_segments(segments[:n])

            if frame >= explore_frames and path:
                route = np.array(path, dtype=float)
                line.set_data(route[:, 0], route[:, 1])

            return lines, line

    # Render and save animation
    animation = FuncAnimation(
        fig,
        update,
        frames=frames,
        interval=1000 / fps,
        blit=False,
    )

    animation.save(
        output,
        writer=PillowWriter(fps=fps),
        dpi=85,
    )

    plt.close(fig)
