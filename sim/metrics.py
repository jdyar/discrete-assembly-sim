# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Run logging: per-tick progress records and a replayable JSON run log.

Two consumers, one call:

- **Charts** — ``TickRecord`` scalars (placed count, progress) per tick.
- **Replay** — full per-tick frames (occupancy grid + robot snapshot) that
  serialize to the run-log JSON contract (v1) consumed by
  ``replay_viewer.html`` and ``sim.render.animate_run``.

Run-log JSON contract (v1)
--------------------------
::

    {
      "version": 1,
      "meta": {
        "rows": int, "cols": int,
        "legend": {"EMPTY": 0, "GROUND": 1, "VOXEL": 2},
        ...free-form extras (e.g. "depot": [row, col])
      },
      "blueprint": [[row, col], ...],          // static target cells
      "ticks": [
        {
          "tick": int,                          // 0 = initial state
          "occupancy": [[int, ...], ...],       // rows x cols, legend values
          "placed": int,                        // cumulative voxels placed
          "robot": null | {
            "pos": [row, col],
            "state": str,                       // robot state-machine label
            "carrying": bool                    // holding a voxel?
          }
        }, ...
      ]
    }

The robot passed to :meth:`RunLog.log_tick` is duck-typed: it must expose
``pos`` (``(row, col)``), ``state`` (``str``-able), and ``carrying``
(``bool``). Attribute errors propagate on purpose — a robot that doesn't
meet the contract should fail loudly, not log garbage.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from .world import DEFECT, EMPTY, GROUND, VOXEL, World

RUN_LOG_VERSION = 2  # v2 adds "robots": [snapshot, ...]; "robot" kept for v1 readers
RUN_LOG_VERSION_3D = 3  # v3 = v2 with 3D nodes: meta gains "levels",
# nodes are [level, row, col], occupancy is levels x rows x cols


@dataclass
class TickRecord:
    tick: int
    placed: int  # cumulative voxels placed
    blueprint_total: int

    @property
    def progress(self) -> float:
        if self.blueprint_total == 0:
            return 1.0
        return self.placed / self.blueprint_total


def _robot_snapshot(robot: Any) -> dict[str, Any] | None:
    """Serialize a robot per the contract; ``None`` stays ``None``."""
    if robot is None:
        return None
    return {
        "pos": [int(x) for x in robot.pos],  # (row,col) or (level,row,col)
        "state": str(robot.state),
        "carrying": bool(robot.carrying),
    }


@dataclass
class RunLog:
    """Accumulates one TickRecord + one replay frame per tick.

    Call :meth:`log_tick` once per tick (tick 0 for the initial state is
    encouraged — the replay viewer treats it as the first frame). ``robot``
    is optional so Slice 0 runs (no robot) still produce valid logs.
    """

    records: list[TickRecord] = field(default_factory=list)
    frames: list[dict[str, Any]] = field(default_factory=list)
    blueprint: list[list[int]] = field(default_factory=list)
    meta: dict[str, Any] = field(default_factory=dict)

    def log_tick(
        self,
        tick: int,
        world: World,
        robot: Any = None,
        robots: list | None = None,
    ) -> TickRecord:
        """Record one tick. Pass ``robots`` (list) for multi-robot runs
        (contract v2); ``robot`` stays for single-robot callers. The v2
        frame always carries "robots" (possibly empty) plus the v1
        "robot" field (first robot or None) so old readers keep working.
        """
        if "rows" not in self.meta:  # first tick; keep any pre-seeded extras
            if world.occupancy.ndim == 3:  # v3: 3D world
                self.meta.update(levels=world.levels)
            self.meta.update(
                rows=world.rows,
                cols=world.cols,
                legend={
                    "EMPTY": EMPTY,
                    "GROUND": GROUND,
                    "VOXEL": VOXEL,
                    "DEFECT": DEFECT,
                },
            )
            self.blueprint = [
                [int(x) for x in node] for node in zip(*world.blueprint.nonzero())
            ]
        rec = TickRecord(
            tick=tick,
            placed=world.built_count,
            blueprint_total=world.blueprint_count,
        )
        self.records.append(rec)
        crew = robots if robots is not None else ([robot] if robot else [])
        snapshots = [_robot_snapshot(r) for r in crew]
        self.frames.append(
            {
                "tick": tick,
                "occupancy": world.occupancy.tolist(),
                "placed": rec.placed,
                "robot": snapshots[0] if snapshots else None,  # v1 compat
                "robots": snapshots,
            }
        )
        return rec

    # -- export --------------------------------------------------------------

    def to_dict(self) -> dict[str, Any]:
        """The full run log as a dict matching the JSON contract."""
        return {
            "version": RUN_LOG_VERSION_3D if "levels" in self.meta else RUN_LOG_VERSION,
            "meta": self.meta,
            "blueprint": self.blueprint,
            "ticks": self.frames,
        }

    def save(self, path: str | Path) -> Path:
        """Write the run log JSON to ``path``, creating parent dirs."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.to_dict()), encoding="utf-8")
        return path

    # -- summaries -------------------------------------------------------------

    @property
    def ticks(self) -> int:
        return self.records[-1].tick if self.records else 0

    def ticks_per_voxel(self) -> list[int]:
        """Ticks spent on each voxel: gaps between placement ticks."""
        placement_ticks = []
        prev_placed = 0
        for rec in self.records:
            if rec.placed > prev_placed:
                placement_ticks.append(rec.tick)
                prev_placed = rec.placed
        out = []
        last = 0
        for t in placement_ticks:
            out.append(t - last)
            last = t
        return out

    def summary(self) -> str:
        if not self.records:
            return "no ticks recorded"
        last = self.records[-1]
        return (
            f"ticks={last.tick} placed={last.placed}/{last.blueprint_total} "
            f"progress={last.progress:.0%}"
        )


def yield_vs_p_chart(
    ps: list[float],
    corrected: dict[float, list[float]],
    baseline: dict[float, list[float]],
    path: str | Path,
    target: float = 0.99,
    target_p: float = 0.08,
) -> Path:
    """The Slice 2 chart: blueprint yield vs defect rate, with/without
    error correction. Mean lines with min-max bands over seeds; the 99%
    target and the p = 0.08 spec point are marked. Saved as PNG.
    """
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    def stats(data: dict[float, list[float]]):
        means = [sum(data[p]) / len(data[p]) for p in ps]
        lows = [min(data[p]) for p in ps]
        highs = [max(data[p]) for p in ps]
        return means, lows, highs

    c_mean, c_lo, c_hi = stats(corrected)
    b_mean, b_lo, b_hi = stats(baseline)

    fig, ax = plt.subplots(figsize=(8, 4.8))
    ax.fill_between(ps, b_lo, b_hi, color="#9ca3af", alpha=0.18, linewidth=0, zorder=2)
    ax.fill_between(ps, c_lo, c_hi, color="#2563eb", alpha=0.15, linewidth=0, zorder=2)
    ax.plot(ps, b_mean, color="#6b7280", linewidth=2, zorder=3,
            label="no error correction")
    ax.plot(ps, c_mean, color="#2563eb", linewidth=2, zorder=4,
            label="inspect + remove + replace")

    ax.axhline(target, color="#d1d5db", linewidth=1, linestyle="--", zorder=1)
    ax.annotate(f"{target:.0%} target", xy=(ps[len(ps) // 3], target),
                xytext=(0, -3), textcoords="offset points", ha="left",
                va="top", fontsize=9, color="#6b7280")
    ax.axvline(target_p, color="#d1d5db", linewidth=1, linestyle=":", zorder=1)
    ax.annotate(f"p = {target_p}", xy=(target_p + 0.002, min(min(b_lo), 0.8)),
                ha="left", va="bottom", fontsize=9, color="#6b7280")

    # Direct-label the series at their right ends (color carries identity).
    ax.annotate("corrected", xy=(ps[-1], c_mean[-1]), xytext=(4, 0),
                textcoords="offset points", va="center", fontsize=9,
                color="#2563eb")
    ax.annotate("baseline", xy=(ps[-1], b_mean[-1]), xytext=(4, 0),
                textcoords="offset points", va="center", fontsize=9,
                color="#6b7280")

    ax.set_title("Blueprint yield vs defect rate — digital error correction",
                 loc="left", fontsize=12)
    ax.set_xlabel("defect probability p per placement")
    ax.set_ylabel("yield (good voxels / blueprint)")
    ax.set_xlim(ps[0], ps[-1] * 1.06)
    ax.set_ylim(min(min(b_lo), 0.8) - 0.01, 1.008)
    ax.yaxis.set_major_formatter(lambda v, _: f"{v:.0%}")
    ax.grid(axis="y", color="#e5e7eb", linewidth=0.8, zorder=0)
    ax.spines[["top", "right"]].set_visible(False)
    ax.legend(loc="lower left", frameon=False, fontsize=9)
    fig.tight_layout()

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def speedup_vs_n_chart(ticks: dict[int, int], path: str | Path) -> Path:
    """Milestone chart (Slice 4a→4b): build ticks vs robot count, with the
    speedup factor over N=1 direct-labeled. Single series, project blue;
    the 2D run's degenerate x1.00 is the mental baseline this chart
    answers. Saved as PNG.
    """
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ns = sorted(ticks)
    base = ticks[ns[0]]
    values = [ticks[n] for n in ns]

    fig, ax = plt.subplots(figsize=(6.4, 4.2))
    bars = ax.bar([str(n) for n in ns], values, width=0.62,
                  color="#2563eb", zorder=3)
    for bar, n, v in zip(bars, ns, values):
        ax.annotate(
            f"{v} ticks",
            xy=(bar.get_x() + bar.get_width() / 2, v),
            xytext=(0, 4), textcoords="offset points",
            ha="center", va="bottom", fontsize=9, color="#374151",
        )
        if n != ns[0]:
            ax.annotate(
                f"x{base / v:.2f}",
                xy=(bar.get_x() + bar.get_width() / 2, v / 2),
                ha="center", va="center", fontsize=11, color="#ffffff",
                fontweight="bold",
            )
    ax.set_title("Build time vs robot count — 3D hollow box (40 voxels)",
                 loc="left", fontsize=12)
    ax.set_xlabel("robots")
    ax.set_ylabel("ticks to complete")
    ax.set_ylim(0, max(values) * 1.12)
    ax.grid(axis="y", color="#e5e7eb", linewidth=0.8, zorder=0)
    ax.spines[["top", "right"]].set_visible(False)
    fig.tight_layout()

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def ticks_vs_reach_chart(ticks: dict[int, int], path: str | Path) -> Path:
    """Slice 4c chart: build ticks vs reach radius, same blueprint, same
    coordination code — the motion-model-as-config claim in one figure."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    radii = sorted(ticks)
    values = [ticks[r] for r in radii]

    fig, ax = plt.subplots(figsize=(6.4, 4.2))
    bars = ax.bar([str(r) for r in radii], values, width=0.62,
                  color="#2563eb", zorder=3)
    for bar, v in zip(bars, values):
        ax.annotate(f"{v}", xy=(bar.get_x() + bar.get_width() / 2, v),
                    xytext=(0, 4), textcoords="offset points",
                    ha="center", va="bottom", fontsize=9, color="#374151")
    ax.set_title(
        "Build time vs placement reach — same coordination code, reach as config",
        loc="left", fontsize=11,
    )
    ax.set_xlabel("reach radius (snake-arm, voxels)")
    ax.set_ylabel("ticks to complete (2 robots)")
    ax.set_ylim(0, max(values) * 1.12)
    ax.grid(axis="y", color="#e5e7eb", linewidth=0.8, zorder=0)
    ax.spines[["top", "right"]].set_visible(False)
    fig.tight_layout()

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def ticks_per_voxel_chart(log: RunLog, path: str | Path) -> Path:
    """Bar chart of ticks spent per voxel over the build, saved as PNG.

    Single series, single hue (the sim's voxel blue); mean line for context;
    the costliest voxel is direct-labeled. Matplotlib imported lazily.
    """
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    costs = log.ticks_per_voxel()
    xs = list(range(1, len(costs) + 1))
    mean = sum(costs) / len(costs) if costs else 0.0

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.bar(xs, costs, width=0.72, color="#2563eb", zorder=3)
    ax.axhline(mean, color="#9ca3af", linewidth=1, linestyle="--", zorder=2)
    ax.annotate(
        f"mean {mean:.1f}",
        xy=(0.99, mean),
        xycoords=("axes fraction", "data"),
        ha="right",
        va="bottom",
        fontsize=9,
        color="#6b7280",
    )
    if costs:
        peak = max(range(len(costs)), key=costs.__getitem__)
        ax.annotate(
            str(costs[peak]),
            xy=(xs[peak], costs[peak]),
            ha="center",
            va="bottom",
            fontsize=9,
            color="#374151",
        )
    ax.set_title("Ticks per voxel over the build", loc="left", fontsize=12)
    ax.set_xlabel("voxel # (placement order)")
    ax.set_ylabel("ticks")
    ax.set_xticks(xs[:: max(1, len(xs) // 12)])
    ax.grid(axis="y", color="#e5e7eb", linewidth=0.8, zorder=0)
    ax.spines[["top", "right"]].set_visible(False)
    fig.tight_layout()

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path
