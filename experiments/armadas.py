# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Shelter throughput sweep. The manuscript's listed future work.

Gregg et al. 2024 name "multiple feed stations" and "parallelization"
as ways the 100.4 h clock could come down. This run is that sweep on
the stand-in shed in :mod:`sim.armadas`: 1–4 feed stations by 1–6
robots of each role. Stiffness is the unfitted ``k = 1``. Sag is in
units of ``mass·g/k``. Action times are the module defaults; nothing
here is scaled to land on 100.4 h.

    python main.py armadas
    python main.py armadas smoke
"""

from __future__ import annotations

import csv
import platform
from pathlib import Path

from sim.armadas import (
    CONFIG_SOURCE,
    EDGE_MM,
    FACES_BOLTED,
    G,
    K,
    MASS,
    N_VOXELS,
    REPORTED_BUILD_DAYS,
    REPORTED_BUILD_HOURS,
    REPORTED_DAYS_AS_HOURS,
    STEP_SOURCE,
    ActionTimes,
    SimResult,
    feed_station_cells,
    inset_boxes_disjoint,
    shelter_model,
    simulate,
)
from sim.cuboct import VIEWER_INSET, cuboct_offsets, cuboct_triangles

FEEDS = (1, 2, 3, 4)
ROBOTS = (1, 2, 3, 4, 5, 6)

CSV_FIELDS = (
    "n_feeds",
    "robots_per_role",
    "makespan_s",
    "makespan_h",
    "reported_h",
    "reported_days_as_h",
    "ratio_to_reported_h",
    "cargo_util",
    "crane_util",
    "fasten_util",
    "feed_util",
    "bottleneck",
    "n_voxels",
    "n_seed",
    "n_placed",
    "n_fasten_tasks",
    "published_faces_bolted",
    "cargo_steps",
    "crane_steps",
    "fasten_steps",
    "finished_sag_mg_over_k",
    "peak_sag_mg_over_k",
    "worst_overhang",
    "k",
    "mass",
    "g",
    "step_s",
    "feed_load_s",
    "handoff_s",
    "place_s",
    "fasten_joint_s",
    "step_source",
    "other_action_source",
    "geometry",
)


def run_sweep(
    times: ActionTimes | None = None,
) -> list[SimResult]:
    """24 schedules. The gated order is built once."""
    model = shelter_model()
    clock = times or ActionTimes()
    rows: list[SimResult] = []
    for n_feeds in FEEDS:
        for n_robots in ROBOTS:
            result = simulate(
                model,
                n_feeds=n_feeds,
                robots_per_role=n_robots,
                times=clock,
                stations=feed_station_cells(n_feeds),
            )
            rows.append(result)
            print(
                f"feeds={n_feeds} robots/role={n_robots}: "
                f"{result.makespan_h:.3f} h  bottleneck={result.bottleneck}  "
                f"util feed={result.feed_util:.2f} cargo={result.cargo_util:.2f} "
                f"crane={result.crane_util:.2f} fasten={result.fasten_util:.2f}",
                flush=True,
            )
    return rows


def _row(result: SimResult, times: ActionTimes) -> dict[str, object]:
    model = shelter_model()
    return {
        "n_feeds": result.n_feeds,
        "robots_per_role": result.robots_per_role,
        "makespan_s": f"{result.makespan_s:.6f}",
        "makespan_h": f"{result.makespan_h:.6f}",
        "reported_h": REPORTED_BUILD_HOURS,
        "reported_days_as_h": REPORTED_DAYS_AS_HOURS,
        "ratio_to_reported_h": f"{result.ratio_to_reported():.6f}",
        "cargo_util": f"{result.cargo_util:.6f}",
        "crane_util": f"{result.crane_util:.6f}",
        "fasten_util": f"{result.fasten_util:.6f}",
        "feed_util": f"{result.feed_util:.6f}",
        "bottleneck": result.bottleneck,
        "n_voxels": N_VOXELS,
        "n_seed": len(model.seed),
        "n_placed": result.n_placed,
        "n_fasten_tasks": result.n_fasten_tasks,
        "published_faces_bolted": FACES_BOLTED,
        "cargo_steps": result.cargo_steps,
        "crane_steps": result.crane_steps,
        "fasten_steps": result.fasten_steps,
        "finished_sag_mg_over_k": f"{result.finished_sag:.6f}",
        "peak_sag_mg_over_k": f"{result.peak_sag:.6f}",
        "worst_overhang": result.worst_overhang,
        "k": K,
        "mass": MASS,
        "g": G,
        "step_s": times.step_s,
        "feed_load_s": times.feed_load_s,
        "handoff_s": times.handoff_s,
        "place_s": times.place_s,
        "fasten_joint_s": times.fasten_joint_s,
        "step_source": STEP_SOURCE,
        "other_action_source": CONFIG_SOURCE,
        "geometry": "stand-in-shed-not-fig5",
    }


def write_csv(rows: list[SimResult], path: Path, times: ActionTimes) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for result in rows:
            writer.writerow(_row(result, times))
    return path


def write_chart(rows: list[SimResult], path: Path) -> Path:
    """Makespan and the binding resource across the sweep."""
    import matplotlib.pyplot as plt

    path.parent.mkdir(parents=True, exist_ok=True)
    by_feed: dict[int, list[SimResult]] = {n: [] for n in FEEDS}
    for result in rows:
        by_feed[result.n_feeds].append(result)
    fig, axes = plt.subplots(1, 2, figsize=(11.0, 4.4))
    ax = axes[0]
    for n_feeds, series in by_feed.items():
        series.sort(key=lambda item: item.robots_per_role)
        ax.plot(
            [item.robots_per_role for item in series],
            [item.makespan_h for item in series],
            marker="o",
            label=f"{n_feeds} feed" + ("s" if n_feeds > 1 else ""),
        )
    ax.axhline(
        REPORTED_BUILD_HOURS,
        color="0.35",
        linestyle="--",
        linewidth=1,
        label="reported ~100.4 h",
    )
    ax.set_xlabel("robots per role")
    ax.set_ylabel("makespan (hours)")
    ax.set_xticks(list(ROBOTS))
    ax.set_title("Simulated makespan")
    ax.legend(frameon=False, fontsize=8)
    ax.grid(True, axis="y", color="0.9")

    # Two outcomes showed up on this shed: the cargo robot is busy, or
    # no pool is (the dependency chain is).
    grid = [[0.0 for _ in ROBOTS] for _ in FEEDS]
    annot = [["" for _ in ROBOTS] for _ in FEEDS]
    for result in rows:
        row = FEEDS.index(result.n_feeds)
        col = ROBOTS.index(result.robots_per_role)
        head = result.bottleneck.split("+")[0]
        grid[row][col] = 0.0 if head == "cargo" else 1.0
        annot[row][col] = "cargo" if head == "cargo" else "chain"
    ax2 = axes[1]
    ax2.imshow(grid, cmap="coolwarm", vmin=0, vmax=1, aspect="auto")
    ax2.set_xticks(range(len(ROBOTS)), [str(n) for n in ROBOTS])
    ax2.set_yticks(range(len(FEEDS)), [str(n) for n in FEEDS])
    ax2.set_xlabel("robots per role")
    ax2.set_ylabel("feed stations")
    ax2.set_title("Binding constraint")
    for row in range(len(FEEDS)):
        for col in range(len(ROBOTS)):
            ax2.text(
                col,
                row,
                annot[row][col],
                ha="center",
                va="center",
                color="white",
                fontsize=8,
            )
    fig.suptitle(
        "ARMADAS stand-in shed, gates on, k = 1 (not fitted to 100.4 h)",
        fontsize=11,
    )
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def write_summary(rows: list[SimResult], path: Path, times: ActionTimes) -> Path:
    model = shelter_model()
    baseline = next(
        item for item in rows if item.n_feeds == 1 and item.robots_per_role == 1
    )
    lines = [
        f"python {platform.python_version()}  {platform.system()}",
        "geometry: stand-in open-front shed, not Fig. 5 coordinates",
        f"voxels {len(model.cells)}  seed {len(model.seed)}  "
        f"placed {len(model.order)}  fasten tasks {sum(len(j) for j in model.joints)}",
        f"published faces bolted {FACES_BOLTED} (different geometry; not a target)",
        f"edge {EDGE_MM} mm  overhang {model.worst_overhang}  "
        f"finished sag {model.finished_sag:.4f} mass*g/k  "
        f"peak along the gated order {model.peak_sag:.4f}",
        f"k = {K}  mass = {MASS}  g = {G}  (unfitted)",
        f"step {times.step_s} s ({STEP_SOURCE})",
        f"feed/handoff/place/fasten {times.feed_load_s}/"
        f"{times.handoff_s}/{times.place_s}/{times.fasten_joint_s} s ({CONFIG_SOURCE})",
        f"reported approximately {REPORTED_BUILD_HOURS} h or {REPORTED_BUILD_DAYS} days "
        f"({REPORTED_DAYS_AS_HOURS:.1f} h). Not reconciled in the manuscript. "
        "Not a fit target.",
        f"baseline 1 feed, 1 robot/role: {baseline.makespan_h:.3f} h "
        f"({baseline.ratio_to_reported():.3f} × the 100.4 h figure), "
        f"bottleneck {baseline.bottleneck}",
        "makespan_h by feeds (rows) and robots per role (columns):",
    ]
    for n_feeds in FEEDS:
        cells = []
        for n_robots in ROBOTS:
            item = next(
                row for row in rows
                if row.n_feeds == n_feeds and row.robots_per_role == n_robots
            )
            cells.append(f"{item.makespan_h:.2f}/{item.bottleneck}")
        lines.append(f"  feeds={n_feeds}: " + "  ".join(cells))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def render_still(path: Path) -> Path:
    """Cuboctahedra at the viewer inset. Seed and eave are marked.

    The mesh is the same solid the replay viewer draws, scaled by
    ``VIEWER_INSET`` (0.96) so face-adjacent cells keep a gap. A cell
    is one voxel. The picture is the finished stand-in, not a frame
    from the lab CAD.
    """
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    model = shelter_model()
    if not inset_boxes_disjoint(model.cells, VIEWER_INSET):
        raise RuntimeError("inset cuboct bounding boxes overlap")
    offsets = cuboct_offsets(VIEWER_INSET)
    triangles = cuboct_triangles(1.0)
    seed = set(model.seed)
    fig = plt.figure(figsize=(8.2, 6.2))
    ax = fig.add_subplot(111, projection="3d")
    for cell in model.cells:
        verts = [
            (
                cell[2] + offset[2],
                cell[1] + offset[1],
                cell[0] + offset[0],
            )
            for offset in offsets
        ]
        faces = [[verts[i] for i in tri] for tri in triangles]
        if cell in seed:
            color = "#e0a100"
        elif cell[0] >= 6 and cell[1] == 0:
            color = "#d4652f"
        elif cell[0] >= 6:
            color = "#8aa0b4"
        else:
            color = "#c5ced6"
        ax.add_collection3d(
            Poly3DCollection(
                faces,
                facecolors=color,
                edgecolors=(0.15, 0.17, 0.2, 0.35),
                linewidths=0.15,
            )
        )
    xs = [cell[2] for cell in model.cells]
    ys = [cell[1] for cell in model.cells]
    zs = [cell[0] for cell in model.cells]
    ax.set_xlim(min(xs) - 0.6, max(xs) + 0.6)
    ax.set_ylim(min(ys) - 0.6, max(ys) + 0.6)
    ax.set_zlim(0, max(zs) + 0.8)
    ax.set_box_aspect(
        (
            max(xs) - min(xs) + 1.2,
            max(ys) - min(ys) + 1.2,
            max(zs) + 0.8,
        )
    )
    ax.view_init(elev=16, azim=-58)
    ax.set_xlabel("col")
    ax.set_ylabel("row")
    ax.set_zlabel("level")
    ax.set_title(
        "Stand-in shelter, 256 cuboctahedra\n"
        "gold = 5-voxel seed, orange = roof eave, gap = 0.96 inset"
    )
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def main(argv: list[str]) -> None:
    out = Path("docs")
    times = ActionTimes()
    if argv and argv[0] == "smoke":
        model = shelter_model()
        result = simulate(model, n_feeds=1, robots_per_role=1, times=times)
        print(
            f"smoke: {result.n_placed} placed, {result.n_fasten_tasks} fasten tasks, "
            f"{result.makespan_h:.3f} h vs reported ~{REPORTED_BUILD_HOURS} h "
            f"(ratio {result.ratio_to_reported():.3f}, not fitted), "
            f"bottleneck {result.bottleneck}, "
            f"peak sag {result.peak_sag:.4f} mass*g/k"
        )
        return
    rows = run_sweep(times)
    csv_path = write_csv(rows, out / "armadas_sweep.csv", times)
    chart_path = write_chart(rows, out / "armadas_sweep.png")
    summary_path = write_summary(rows, out / "armadas_summary.txt", times)
    still_path = render_still(out / "armadas_shelter.png")
    baseline = next(
        item for item in rows if item.n_feeds == 1 and item.robots_per_role == 1
    )
    print(
        f"baseline {baseline.makespan_h:.3f} h vs reported ~{REPORTED_BUILD_HOURS} h "
        f"and vs {REPORTED_BUILD_DAYS} days = {REPORTED_DAYS_AS_HOURS:.1f} h. "
        f"Ratio to 100.4 h is {baseline.ratio_to_reported():.3f}. Not fitted."
    )
    print(f"csv:     {csv_path}")
    print(f"chart:   {chart_path}")
    print(f"summary: {summary_path}")
    print(f"still:   {still_path}")
