# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Rendering: ASCII frames, plus a matplotlib animation of a run log.

ASCII legend:
    ``#`` placed voxel
    ``x`` defective voxel (Slice 2)
    ``o`` blueprint cell not yet built
    ``=`` ground row
    ``.`` empty air

The animation side consumes the run-log JSON contract (v1) documented in
``sim/metrics.py`` — the same file the Three.js replay viewer loads — so
one recorded run feeds both. Quick check from the shell::

    python -m sim.render runs/latest.json            # interactive window
    python -m sim.render runs/latest.json out.gif    # save a GIF instead
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from .world import DEFECT, EMPTY, GROUND, VOXEL, World

_GLYPHS = {EMPTY: ".", GROUND: "=", VOXEL: "#", DEFECT: "x"}

# Display codes for the animation (occupancy legend + two overlay codes).
_GHOST = 4  # blueprint cell not yet built
_ROBOT = 5

_COLORS = {
    EMPTY: "#ffffff",   # air
    GROUND: "#4b5563",  # ground row
    VOXEL: "#2563eb",   # placed voxel
    DEFECT: "#dc2626",  # defective voxel
    _GHOST: "#dbeafe",  # pending blueprint (ghost)
    _ROBOT: "#f97316",  # robot
}


def render_ascii(world: World) -> str:
    """One multi-line ASCII frame of the current world state."""
    lines: list[str] = []
    for r in range(world.rows):
        chars: list[str] = []
        for c in range(world.cols):
            occ = int(world.occupancy[r, c])
            if occ == EMPTY and world.blueprint[r, c]:
                chars.append("o")
            else:
                chars.append(_GLYPHS[occ])
        lines.append("".join(chars))
    return "\n".join(lines)


def render_ascii3d(world) -> str:
    """Per-level ASCII slices of a 3D world, ground level omitted.

    Levels print bottom-up, side by side is left to the reader's terminal
    width — each level is its own block, labeled. Same glyphs as 2D.
    """
    blocks: list[str] = []
    for level in range(1, world.levels):
        lines = [f"level {level}:"]
        for r in range(world.rows):
            chars = []
            for c in range(world.cols):
                occ = int(world.occupancy[level, r, c])
                if occ == EMPTY and world.blueprint[level, r, c]:
                    chars.append("o")
                else:
                    chars.append(_GLYPHS[occ])
            lines.append("".join(chars))
        blocks.append("\n".join(lines))
    return "\n\n".join(blocks)


def _frame_robots(frame: dict[str, Any]) -> list[dict[str, Any]]:
    """Robot snapshots from a v2 ("robots") or v1 ("robot") frame."""
    if "robots" in frame:
        return [r for r in frame["robots"] if r is not None]
    robot = frame.get("robot")
    return [robot] if robot is not None else []


def _display_grid(frame: dict[str, Any], blueprint: list[list[int]]) -> list[list[int]]:
    """Occupancy grid with ghost-blueprint and robot overlays applied."""
    grid = [row[:] for row in frame["occupancy"]]
    for r, c in blueprint:
        if grid[r][c] == EMPTY:
            grid[r][c] = _GHOST
    for robot in _frame_robots(frame):
        rr, rc = robot["pos"]
        grid[rr][rc] = _ROBOT
    return grid


def animate_run(
    run: dict[str, Any],
    interval_ms: int = 120,
    save_path: str | Path | None = None,
    show: bool = True,
):
    """Animate a run-log dict (contract v1). Returns the FuncAnimation.

    With ``save_path`` (``.gif``) the animation is written to disk and not
    shown; otherwise a window opens (unless ``show=False``, for tests).
    Matplotlib is imported lazily so ASCII rendering stays dependency-light.
    """
    if run.get("version") == 3 or "levels" in run.get("meta", {}):
        raise ValueError(
            "3D run logs (v3) are replayed in replay_viewer.html; "
            "the matplotlib animation is 2D-only"
        )
    import matplotlib

    if save_path is not None or not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import animation
    from matplotlib.colors import ListedColormap

    frames = run["ticks"]
    blueprint = run["blueprint"]
    cmap = ListedColormap([_COLORS[i] for i in sorted(_COLORS)])

    fig, ax = plt.subplots(figsize=(6, 4.5))
    ax.set_axis_off()
    image = ax.imshow(
        _display_grid(frames[0], blueprint),
        cmap=cmap,
        vmin=0,
        vmax=max(_COLORS),
        interpolation="nearest",
    )
    title = ax.set_title("")

    def draw(i: int):
        frame = frames[i]
        image.set_data(_display_grid(frame, blueprint))
        crew = _frame_robots(frame)
        if len(crew) == 1:
            state = f"  robot: {crew[0]['state']}"
        elif crew:
            state = "  " + " ".join(f"r{j}:{r['state']}" for j, r in enumerate(crew))
        else:
            state = ""
        title.set_text(f"tick {frame['tick']}  placed {frame['placed']}{state}")
        return (image, title)

    anim = animation.FuncAnimation(
        fig, draw, frames=len(frames), interval=interval_ms, blit=False
    )
    if save_path is not None:
        anim.save(str(save_path), writer="pillow")
        plt.close(fig)
    elif show:
        plt.show()
    return anim


def animate_file(
    path: str | Path,
    save_path: str | Path | None = None,
    show: bool = True,
):
    """Load a run-log JSON file and animate it. See :func:`animate_run`."""
    import json

    run = json.loads(Path(path).read_text(encoding="utf-8"))
    return animate_run(run, save_path=save_path, show=show)


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        sys.exit("usage: python -m sim.render <run.json> [out.gif]")
    animate_file(sys.argv[1], save_path=sys.argv[2] if len(sys.argv) > 2 else None)
