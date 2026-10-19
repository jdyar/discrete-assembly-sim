# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Monte Carlo of placement error on chains, bridges, and the shelter.

Three modes, same draws in distribution, not the same stream:

* ``corrected`` — ski/claw check, and a capture snaps to the mating
  neighbors. Seated centers sit on the ideal grid. Misses are retried,
  which is the repair path's new draw.
* ``uncorrected`` — the draw is kept. Nothing fails. Error walks down
  the growth front.
* ``no_snap`` — the check still fails a miss, but a capture keeps the
  draw. A chain can finish, truncated. A bridge's far cap is tested
  against a pier the deck has walked away from.

``k`` is not an input. Sag in the rest of the repository stays the
unfitted ``k = 1``. The IEEE Aerospace 2024 RMS definition is used for
the surface column. Their inch-valued strut-length runs are not targets.

    python main.py precision
    python main.py precision smoke
"""

from __future__ import annotations

import csv
import math
import random
import statistics
import sys
from pathlib import Path

from sim.armadas import shelter_model
from sim.precision import (
    CLAW_OUT_OF_PLANE_CM,
    PLACE_TOLERANCE_CM,
    PLACE_TOLERANCE_DEG,
    PITCH_M,
    SKI_IN_PLANE_CM,
    AssemblyResult,
    PrecisionModel,
    assemble,
    bridge_chain,
    cantilever_chain,
    expected_uniform_fail_rate,
    rms_line,
    rms_plane,
    rms_position,
    surface_points,
    tip_length_error,
)

ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs"

CANTILEVERS = (3, 6, 12, 24)
BRIDGES = (4, 6, 8, 12, 16)
N_BEAM = 40
N_SHELTER = 16
MAX_ATTEMPTS = 40
SCALES = (0.5, 2.0)
ENVELOPE_CANTILEVER = 12
ENVELOPE_BRIDGE = 8

MODES = ("corrected", "uncorrected", "no_snap")

COLUMNS = (
    "structure",
    "size",
    "mode",
    "envelope_scale",
    "seed",
    "n_cells",
    "n_datums",
    "n_placed",
    "n_attempts",
    "n_failures",
    "fail_rate",
    "complete",
    "rms_position_m",
    "rms_surface_m",
    "surface_fit",
    "tip_error_m",
    "length_error_m",
    "length_error_ratio",
    "analytic_fail_rate",
    "distribution",
    "pitch_m",
    "position_half_cm",
    "angle_half_deg",
    "ski_cm",
    "claw_cm",
    "stiffness",
)


def _mode_flags(mode: str) -> tuple[bool, bool]:
    if mode == "corrected":
        return True, True
    if mode == "uncorrected":
        return False, False
    if mode == "no_snap":
        return False, True
    raise ValueError(mode)


def _seed(structure: str, size: int, mode: str, index: int, scale: float) -> int:
    kind = {"cantilever": 1, "bridge": 2, "shelter": 3}[structure]
    mode_id = {"corrected": 1, "uncorrected": 2, "no_snap": 3}[mode]
    scale_id = int(round(float(scale) * 100))
    return 1_000_003 * kind + 1_009 * int(size) + 97 * mode_id + 13 * scale_id + int(index)


def _fmt(value) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        if math.isnan(value):
            return ""
        return f"{value:.8g}"
    return str(value)


def _shelter_spec():
    gated = shelter_model()
    return gated.seed, gated.order


def _roof_cells(seed, order):
    return tuple(cell for cell in tuple(seed) + tuple(order) if cell[0] == 7)


def _run_one(
    structure: str,
    size: int,
    mode: str,
    index: int,
    scale: float,
    *,
    spec,
    fit: str,
    surface_cells,
    root,
    tip,
    max_attempts: int,
) -> dict:
    snap, check = _mode_flags(mode)
    model = PrecisionModel(snap=snap, check=check, envelope_scale=scale)
    datums, order = spec
    seed = _seed(structure, size, mode, index, scale)
    result = assemble(
        model,
        order,
        datums,
        random.Random(seed),
        max_attempts=max_attempts,
    )
    points = surface_points(result, surface_cells)
    if fit == "plane":
        surface = rms_plane(points)
    else:
        surface = rms_line(points)
    tip_error, length_error, ratio = tip_length_error(result, root, tip, model.pitch_m)
    analytic = float("nan")
    if mode == "corrected":
        # Snap back to ideal, so the miss rate does not depend on the draws.
        analytic = expected_uniform_fail_rate(order, datums, model)
    return {
        "structure": structure,
        "size": size,
        "mode": mode,
        "envelope_scale": scale,
        "seed": seed,
        "n_cells": len(tuple(datums) + tuple(order)),
        "n_datums": len(tuple(datums)),
        "n_placed": result.n_placed,
        "n_attempts": result.n_attempts,
        "n_failures": result.n_failures,
        "fail_rate": result.fail_rate,
        "complete": int(result.complete),
        "rms_position_m": rms_position(result, model.pitch_m),
        "rms_surface_m": surface,
        "surface_fit": fit,
        "tip_error_m": tip_error,
        "length_error_m": length_error,
        "length_error_ratio": ratio,
        "analytic_fail_rate": analytic,
        "distribution": model.distribution,
        "pitch_m": model.pitch_m,
        "position_half_cm": model.position_half_cm,
        "angle_half_deg": model.angle_half_deg,
        "ski_cm": model.ski_cm,
        "claw_cm": model.claw_cm,
        "stiffness": "not_used",
    }


def _jobs(smoke: bool) -> list[dict]:
    if smoke:
        specs = [("cantilever", 3, cantilever_chain(3), "line")]
        specs.append(("bridge", 4, bridge_chain(4), "line"))
        modes = ("corrected", "uncorrected")
        n_trials = 2
        scales_extra: tuple[float, ...] = ()
        include_shelter = False
    else:
        specs = []
        for length in CANTILEVERS:
            datums, order = cantilever_chain(length)
            specs.append(("cantilever", length, (datums, order), "line"))
        for span in BRIDGES:
            datums, order = bridge_chain(span)
            specs.append(("bridge", span, (datums, order), "line"))
        modes = MODES
        n_trials = N_BEAM
        scales_extra = SCALES
        include_shelter = True
    rows_spec = []
    for structure, size, spec, fit in specs:
        datums, order = spec
        root = datums[0]
        tip = order[-1]
        surface = order
        for mode in modes:
            for index in range(n_trials):
                rows_spec.append(
                    dict(
                        structure=structure,
                        size=size,
                        mode=mode,
                        index=index,
                        scale=1.0,
                        spec=spec,
                        fit=fit,
                        surface_cells=surface,
                        root=root,
                        tip=tip,
                    )
                )
        if not smoke and structure == "cantilever" and size == ENVELOPE_CANTILEVER:
            for scale in scales_extra:
                for index in range(n_trials):
                    rows_spec.append(
                        dict(
                            structure=structure,
                            size=size,
                            mode="corrected",
                            index=index,
                            scale=scale,
                            spec=spec,
                            fit=fit,
                            surface_cells=surface,
                            root=root,
                            tip=tip,
                        )
                    )
        if not smoke and structure == "bridge" and size == ENVELOPE_BRIDGE:
            for scale in scales_extra:
                for index in range(n_trials):
                    rows_spec.append(
                        dict(
                            structure=structure,
                            size=size,
                            mode="corrected",
                            index=index,
                            scale=scale,
                            spec=spec,
                            fit=fit,
                            surface_cells=surface,
                            root=root,
                            tip=tip,
                        )
                    )
    if include_shelter:
        datums, order = _shelter_spec()
        roof = _roof_cells(datums, order)
        spec = (datums, order)
        for mode in MODES:
            for index in range(N_SHELTER):
                rows_spec.append(
                    dict(
                        structure="shelter",
                        size=len(tuple(datums) + tuple(order)),
                        mode=mode,
                        index=index,
                        scale=1.0,
                        spec=spec,
                        fit="plane",
                        surface_cells=roof,
                        root=datums[0],
                        tip=order[-1],
                    )
                )
        for scale in scales_extra:
            for index in range(N_SHELTER):
                rows_spec.append(
                    dict(
                        structure="shelter",
                        size=len(tuple(datums) + tuple(order)),
                        mode="corrected",
                        index=index,
                        scale=scale,
                        spec=spec,
                        fit="plane",
                        surface_cells=roof,
                        root=datums[0],
                        tip=order[-1],
                    )
                )
    return rows_spec


def run_rows(smoke: bool = False) -> list[dict]:
    jobs = _jobs(smoke)
    rows = []
    for job in jobs:
        rows.append(
            _run_one(
                job["structure"],
                job["size"],
                job["mode"],
                job["index"],
                job["scale"],
                spec=job["spec"],
                fit=job["fit"],
                surface_cells=job["surface_cells"],
                root=job["root"],
                tip=job["tip"],
                max_attempts=4 if smoke else MAX_ATTEMPTS,
            )
        )
    return rows


def write_csv(rows: list[dict], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: _fmt(row[key]) for key in COLUMNS})


def load_rows(path: Path) -> list[dict]:
    with path.open(encoding="utf-8", newline="") as handle:
        raw = list(csv.DictReader(handle))
    rows = []
    for item in raw:
        row = dict(item)
        for key in (
            "size",
            "seed",
            "n_cells",
            "n_datums",
            "n_placed",
            "n_attempts",
            "n_failures",
            "complete",
        ):
            row[key] = int(item[key]) if item[key] != "" else 0
        for key in (
            "envelope_scale",
            "fail_rate",
            "rms_position_m",
            "rms_surface_m",
            "tip_error_m",
            "length_error_m",
            "length_error_ratio",
            "analytic_fail_rate",
            "pitch_m",
            "position_half_cm",
            "angle_half_deg",
            "ski_cm",
            "claw_cm",
        ):
            row[key] = float(item[key]) if item[key] != "" else float("nan")
        rows.append(row)
    return rows


def _select(rows, **wanted):
    found = []
    for row in rows:
        if all(row[key] == value for key, value in wanted.items()):
            found.append(row)
    return found


def _median(values) -> float:
    clean = [float(v) for v in values if not (isinstance(v, float) and math.isnan(v))]
    if not clean:
        return float("nan")
    return float(statistics.median(clean))


def _pooled_fail(rows) -> float:
    attempts = sum(int(row["n_attempts"]) for row in rows)
    failures = sum(int(row["n_failures"]) for row in rows)
    if attempts == 0:
        return 0.0
    return failures / attempts


def _minmax(values) -> tuple[float, float]:
    clean = [float(v) for v in values if not (isinstance(v, float) and math.isnan(v))]
    if not clean:
        return (float("nan"), float("nan"))
    return min(clean), max(clean)


def summarize(rows: list[dict]) -> str:
    lines = [
        "Placement error Monte Carlo.",
        f"Distribution: uniform inside ±{PLACE_TOLERANCE_CM} cm and "
        f"±{PLACE_TOLERANCE_DEG}°. A requirement, not a measured histogram.",
        f"Capture: ski ±{SKI_IN_PLANE_CM} cm in-plane, "
        f"claw/gripper {CLAW_OUT_OF_PLANE_CM} cm out-of-plane.",
        "Angle gate: the same ±3°. Envelope scale multiplies ski and claw only.",
        "Snap is an exact elastic average. The printed <0.1 mm bound is not sampled.",
        "Stiffness is not an input. Sag elsewhere remains the unfitted k = 1.",
        "Surface fit: line for a chain or a deck, plane for the shelter roof",
        "(IEEE Aerospace 2024 plate definition, different error source).",
        f"Pitch {PITCH_M} m = {PITCH_M * 1000} mm. N per beam cell is the row count.",
        "",
    ]
    for structure in ("cantilever", "bridge", "shelter"):
        sizes = sorted({row["size"] for row in rows if row["structure"] == structure})
        for size in sizes:
            for mode in MODES:
                group = _select(
                    rows, structure=structure, size=size, mode=mode, envelope_scale=1.0
                )
                if not group:
                    continue
                rms = _median([row["rms_position_m"] for row in group])
                tip = _median([row["tip_error_m"] for row in group])
                surface = _median([row["rms_surface_m"] for row in group])
                rms_span = _minmax([row["rms_position_m"] for row in group])
                tip_span = _minmax([row["tip_error_m"] for row in group])
                pooled = _pooled_fail(group)
                complete = sum(int(row["complete"]) for row in group)
                analytic = group[0]["analytic_fail_rate"]
                if isinstance(analytic, str) or (
                    isinstance(analytic, float) and math.isnan(analytic)
                ):
                    analytic_text = "n/a"
                else:
                    analytic_text = f"{float(analytic):.6f}"
                lines.append(
                    f"{structure} size {size} {mode}: n={len(group)} "
                    f"complete {complete}/{len(group)} "
                    f"pooled fail {pooled:.6f} analytic {analytic_text} "
                    f"rms median {rms:.6g} m ({rms_span[0]:.6g}–{rms_span[1]:.6g}) "
                    f"tip median {tip:.6g} m ({tip_span[0]:.6g}–{tip_span[1]:.6g}) "
                    f"surface median {surface:.6g} m"
                )
    lines.append("")
    lines.append("Envelope scale, corrected only (ski and claw × scale).")
    for structure, size in (
        ("cantilever", ENVELOPE_CANTILEVER),
        ("bridge", ENVELOPE_BRIDGE),
        ("shelter", 256),
    ):
        for scale in (0.5, 1.0, 2.0):
            group = _select(
                rows,
                structure=structure,
                size=size,
                mode="corrected",
                envelope_scale=scale,
            )
            if not group:
                continue
            pooled = _pooled_fail(group)
            analytic = group[0]["analytic_fail_rate"]
            rms = _median([row["rms_position_m"] for row in group])
            lines.append(
                f"  {structure} {size} scale {scale}: n={len(group)} "
                f"pooled fail {pooled:.6f} analytic {analytic:.6f} "
                f"rms median {rms:.6g} m"
            )
    lines.append("")
    lines.append(
        "Reading. With snap on, a seated voxel is back on the ideal center, "
        "so RMS does not grow with length. The miss rate is the placement "
        "box against the claw, and a second mating axis (the bridge's far "
        "cap, the shelter's corners) makes it stricter. Without the check, "
        "the ±3° walk moves the tip. Without the snap but with the check, "
        "a chain still finishes and a long bridge does not: the far pier "
        "is outside the envelope of a deck that has already wandered. "
        "k is not used."
    )
    lines.append("")
    return "\n".join(lines)


def _style(ax) -> None:
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def write_charts(rows: list[dict], out_dir: Path) -> list[Path]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out_dir.mkdir(parents=True, exist_ok=True)
    written = []
    colors = {
        "corrected": "#1d4ed8",
        "uncorrected": "#b45309",
        "no_snap": "#0f766e",
    }
    labels = {
        "corrected": "snap",
        "uncorrected": "open loop",
        "no_snap": "gate, no snap",
    }

    fig, axes = plt.subplots(1, 2, figsize=(9.2, 4.2), sharey=True)
    for ax, structure, xlabel in (
        (axes[0], "cantilever", "free voxels"),
        (axes[1], "bridge", "pier gap (cells)"),
    ):
        for mode in MODES:
            xs = []
            ys = []
            lo = []
            hi = []
            sizes = sorted(
                {
                    row["size"]
                    for row in rows
                    if row["structure"] == structure and row["envelope_scale"] == 1.0
                }
            )
            for size in sizes:
                group = _select(
                    rows,
                    structure=structure,
                    size=size,
                    mode=mode,
                    envelope_scale=1.0,
                )
                if not group:
                    continue
                values = [row["rms_position_m"] * 100.0 for row in group]
                xs.append(size)
                ys.append(_median(values))
                span = _minmax(values)
                lo.append(span[0])
                hi.append(span[1])
            if not xs:
                continue
            ax.plot(xs, ys, color=colors[mode], marker="o", label=labels[mode])
            ax.fill_between(xs, lo, hi, color=colors[mode], alpha=0.15)
        ax.set_xlabel(xlabel)
        ax.set_title(structure)
        _style(ax)
    axes[0].set_ylabel("RMS position error (cm)")
    if axes[0].get_legend_handles_labels()[0]:
        axes[0].legend(frameon=False)
    fig.suptitle(
        "Placement error vs size. Snap sits on the axis. k is not used.",
        fontsize=11,
    )
    fig.tight_layout()
    path = out_dir / "precision_rms.png"
    fig.savefig(path, dpi=140)
    plt.close(fig)
    written.append(path)

    fig, axes = plt.subplots(1, 2, figsize=(9.2, 4.2), sharey=True)
    for ax, structure in ((axes[0], "cantilever"), (axes[1], "bridge")):
        for mode in ("corrected", "no_snap"):
            xs = []
            ys = []
            sizes = sorted(
                {
                    row["size"]
                    for row in rows
                    if row["structure"] == structure and row["envelope_scale"] == 1.0
                }
            )
            for size in sizes:
                group = _select(
                    rows,
                    structure=structure,
                    size=size,
                    mode=mode,
                    envelope_scale=1.0,
                )
                if not group:
                    continue
                xs.append(size)
                ys.append(_pooled_fail(group))
            if xs:
                ax.plot(xs, ys, color=colors[mode], marker="o", label=labels[mode])
        analytic_x = []
        analytic_y = []
        sizes = sorted(
            {
                row["size"]
                for row in rows
                if row["structure"] == structure
                and row["mode"] == "corrected"
                and row["envelope_scale"] == 1.0
            }
        )
        for size in sizes:
            group = _select(
                rows,
                structure=structure,
                size=size,
                mode="corrected",
                envelope_scale=1.0,
            )
            if group and not math.isnan(group[0]["analytic_fail_rate"]):
                analytic_x.append(size)
                analytic_y.append(group[0]["analytic_fail_rate"])
        if analytic_x:
            ax.plot(
                analytic_x,
                analytic_y,
                color="#64748b",
                linestyle="--",
                label="snap, closed form",
            )
        ax.set_xlabel("free voxels" if structure == "cantilever" else "pier gap (cells)")
        ax.set_title(structure)
        ax.set_ylim(-0.02, 1.02)
        _style(ax)
    axes[0].set_ylabel("failed-capture rate")
    if axes[0].get_legend_handles_labels()[0]:
        axes[0].legend(frameon=False, fontsize=8)
    fig.suptitle(
        "Miss rate. Open loop is absent: it never fails. k is not used.",
        fontsize=11,
    )
    fig.tight_layout()
    path = out_dir / "precision_fail.png"
    fig.savefig(path, dpi=140)
    plt.close(fig)
    written.append(path)

    fig, ax = plt.subplots(figsize=(6.4, 4.2))
    series = (
        ("cantilever", ENVELOPE_CANTILEVER, "#1d4ed8"),
        ("bridge", ENVELOPE_BRIDGE, "#b45309"),
        ("shelter", 256, "#0f766e"),
    )
    for structure, size, color in series:
        xs = []
        ys = []
        for scale in (0.5, 1.0, 2.0):
            group = _select(
                rows,
                structure=structure,
                size=size,
                mode="corrected",
                envelope_scale=scale,
            )
            if not group:
                continue
            xs.append(scale)
            ys.append(_pooled_fail(group))
        if xs:
            ax.plot(xs, ys, color=color, marker="o", label=f"{structure} {size}")
    ax.set_xlabel("ski and claw scale (1 = published)")
    ax.set_ylabel("failed-capture rate, snap on")
    ax.set_ylim(-0.02, 1.02)
    ax.set_title("How wide the correction envelope is. k is not used.")
    if ax.get_legend_handles_labels()[0]:
        ax.legend(frameon=False)
    _style(ax)
    fig.tight_layout()
    path = out_dir / "precision_envelope.png"
    fig.savefig(path, dpi=140)
    plt.close(fig)
    written.append(path)
    return written


def _visible_seed(n: int = 12) -> tuple[int, AssemblyResult, str]:
    """An open-loop cantilever whose transverse wander is easy to see."""
    datums, order = cantilever_chain(n)
    best = None
    for index in range(40):
        seed = _seed("cantilever", n, "uncorrected", index, 1.0)
        result = assemble(
            PrecisionModel(snap=False, check=False),
            order,
            datums,
            random.Random(seed),
            max_attempts=1,
        )
        points = surface_points(result, order)
        if len(points) == 0:
            continue
        spread = (
            float(points[:, 0].std()),
            float(points[:, 1].std()),
        )
        score = max(spread)
        axis = "x" if spread[0] >= spread[1] else "y"
        if best is None or score > best[0]:
            best = (score, seed, result, axis, index)
    assert best is not None
    return best[1], best[2], best[3]


def write_still(path: Path, n: int = 12) -> dict:
    """Side view of one open-loop arm and the snap model.

    Offsets are true scale when the wander is at least a fifth of a
    voxel. Otherwise they are multiplied by a labeled factor. This is
    not the sag viewer and not an exaggerated spring.
    """
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    datums, order = cantilever_chain(n)
    seed, open_loop, axis_name = _visible_seed(n)
    # The index is recoverable from the seed function by scanning.
    index = next(
        i
        for i in range(40)
        if _seed("cantilever", n, "uncorrected", i, 1.0) == seed
    )
    corrected = assemble(
        PrecisionModel(snap=True, check=True),
        order,
        datums,
        random.Random(_seed("cantilever", n, "corrected", index, 1.0)),
        max_attempts=MAX_ATTEMPTS,
    )
    pitch = PITCH_M
    axis = 0 if axis_name == "x" else 1

    def _transverse(result: AssemblyResult):
        values = []
        along = []
        for cell in ((0, 0, 0),) + order:
            pose = result.poses[cell]
            along.append(float(pose.t[2]))
            values.append(float(pose.t[axis]))
        return along, values

    open_along, open_trans = _transverse(open_loop)
    corr_along, corr_trans = _transverse(corrected)
    ideal_along = [i * pitch for i in range(n + 1)]
    ideal_trans = [0.0 for _ in ideal_along]
    max_offset = max(abs(v) for v in open_trans)
    factor = 1.0
    if max_offset < 0.20 * pitch:
        factor = (0.45 * pitch) / max_offset if max_offset > 0 else 1.0
    scale_note = (
        "true scale"
        if factor == 1.0
        else f"transverse offsets ×{factor:.2f}, labeled"
    )

    def _draw(ax, along, trans, title: str) -> None:
        half = 0.41 * pitch
        for z, y in zip(ideal_along, ideal_trans):
            ax.add_patch(
                Rectangle(
                    (z - half, y - half),
                    2 * half,
                    2 * half,
                    fill=False,
                    edgecolor="#94a3b8",
                    linewidth=0.8,
                    zorder=1,
                )
            )
        ax.plot(ideal_along, ideal_trans, color="#cbd5e1", linewidth=1, zorder=0)
        drawn_y = [v * factor for v in trans]
        for i, (z, y) in enumerate(zip(along, drawn_y)):
            color = "#b45309" if i == 0 else "#1d4ed8"
            ax.add_patch(
                Rectangle(
                    (z - half, y - half),
                    2 * half,
                    2 * half,
                    facecolor=color,
                    edgecolor="#0f172a",
                    linewidth=0.6,
                    alpha=0.92,
                    zorder=2,
                )
            )
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("along the chain (m)")
        ax.set_ylabel(f"transverse {axis_name} (m)")
        ax.set_title(title, fontsize=10)
        _style(ax)

    fig, axes = plt.subplots(1, 2, figsize=(10.4, 4.4))
    _draw(
        axes[0],
        open_along,
        open_trans,
        f"Open loop, seed index {index}. {scale_note}.",
    )
    _draw(
        axes[1],
        corr_along,
        corr_trans,
        f"Snap, seed index {index}. Centers on the grid.",
    )
    fig.suptitle(
        "Cantilever, 12 free voxels. ±1.78 cm and ±3°, ski/claw snap. "
        "Not sag. k is not used.",
        fontsize=11,
    )
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return {
        "path": str(path),
        "seed_index": index,
        "seed": seed,
        "axis": axis_name,
        "factor": factor,
        "max_offset_m": max_offset,
        "corrected_complete": corrected.complete,
        "open_rms_m": rms_position(open_loop, pitch),
        "corrected_rms_m": rms_position(corrected, pitch),
    }


def write_outputs(rows: list[dict], out_dir: Path, *, still: bool) -> dict:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "precision_campaign.csv"
    write_csv(rows, csv_path)
    summary = summarize(rows)
    (out_dir / "precision_summary.txt").write_text(summary, encoding="utf-8")
    charts = write_charts(rows, out_dir) if rows else []
    still_info = None
    if still:
        still_info = write_still(out_dir / "precision_cantilever.png")
        summary += (
            f"Still: {still_info['path']} seed index {still_info['seed_index']} "
            f"axis {still_info['axis']} factor {still_info['factor']:.3f} "
            f"open rms {still_info['open_rms_m']:.6g} m "
            f"snap rms {still_info['corrected_rms_m']:.6g} m\n"
        )
        (out_dir / "precision_summary.txt").write_text(summary, encoding="utf-8")
    return {
        "rows": rows,
        "summary": summary,
        "csv": csv_path,
        "charts": charts,
        "still": still_info,
    }


def run_campaign(out_dir: Path = DOCS) -> dict:
    rows = run_rows(smoke=False)
    return write_outputs(rows, out_dir, still=True)


def run_smoke(out_dir: Path) -> dict:
    rows = run_rows(smoke=True)
    return write_outputs(rows, out_dir, still=False)


def main(argv: list[str] | None = None) -> None:
    args = list(sys.argv[1:] if argv is None else argv)
    if args and args[0] == "smoke":
        out = Path(args[1]) if len(args) > 1 else ROOT / "runs" / "precision_smoke"
        result = run_smoke(out)
        print(result["summary"], end="")
        return
    out = Path(args[0]) if args else DOCS
    result = run_campaign(out)
    print(result["summary"], end="")


if __name__ == "__main__":
    main()
