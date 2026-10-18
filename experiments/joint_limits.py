# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Which builds are joint-limited, and which are sag-limited.

Every solve uses ``k = mass = g = 1``. Displacement is in units of
``mass·g/k``. Joint force is ``k`` times relative displacement, so it
is in units of ``mass·g``. The published ARMADAS face compression
(:func:`sim.stability.armadas_face_compression`) is in newtons. It is
printed in the summary and it is not used as a cap. Comparing it to a
``k = 1`` force would be a unit error.

A half-cap sets the checked capacity to half the finished demand of
that component and leaves the other components unchecked. The finished
structure then either still passes (that component was already ~0) or
is rejected. Scaffold cannot rescue a rejection of the finished
structure. A separate block, on bridges, sets the cap at the finished
demand and, when a one-sided deck shears more than the finished deck,
at the midpoint. That is the cap that can change the scaffold count.

    python main.py joints
    python main.py joints smoke
"""

from __future__ import annotations

import csv
import sys
import time
from pathlib import Path

from experiments.on_lattice import GAIT
from experiments.scaffold_campaign import inchworm_factory, plant_entry_pad
from experiments.stability_cost import build_bridge
from sim.armadas import shelter_cells
from sim.geometry3d import CubicLattice3D, MotionModel
from sim.planner import SearchBudgetExceeded
from sim.scaffold import (
    CRANE,
    caps_for_finished,
    geometry_factory,
    plan_with_scaffold,
)
from sim.world3d import World3D
from sim.stability import (
    JointCap,
    JointStrength,
    SagCap,
    StabilityPolicy,
    armadas_face_compression,
    estimate_sag,
    joint_loads,
    path_chain_joint_force,
)

ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs"

# One-sided shear above the finished shear by more than this is a gap
# a midpoint cap can sit in. Below it, the two demands are the same
# number for this model and no scaffold search is run at a midpoint.
GAP_ATOL = 1e-6
MAX_NODES = 40_000


def clamp_policy() -> StabilityPolicy:
    """Finite base under one root. The stock floor would plant the course."""
    return StabilityPolicy(
        up_axis=0,
        up_sign=1,
        ground_plane=None,
        base=((0, 0, 0),),
        max_cantilever=None,
    )


def clamp_bar(n_free: int, *, horizontal: bool) -> tuple[tuple, ...]:
    """Planted root plus ``n_free`` voxels in a straight line."""
    cells = [(1, 0, 0)]
    if horizontal:
        cells += [(1, 0, i) for i in range(1, n_free + 1)]
    else:
        cells += [(1 + i, 0, 0) for i in range(1, n_free + 1)]
    return tuple(cells)


def peak_forces(cells, policy, *, point_loads=None) -> dict:
    """Sag and the largest tension, compression, and shear, at k = 1."""
    estimate = estimate_sag(
        cells, policy, k=1.0, mass=1.0, g=1.0, point_loads=point_loads
    )
    loads = joint_loads(estimate, k=1.0)
    compression = 0.0
    tension = 0.0
    shear = 0.0
    for load in loads:
        if load.axial < 0:
            compression = max(compression, -load.axial)
        elif load.axial > 0:
            tension = max(tension, load.axial)
        shear = max(shear, load.shear)
    return {
        "sag": float(estimate.worst_sag),
        "compression": float(compression),
        "tension": float(tension),
        "shear": float(shear),
        "n_joints": len(loads),
    }


def _positive(value: float) -> float | None:
    if value < 1e-8:
        return None
    return float(value)


def strength_scaled(forces: dict, *, compression: float | None, shear: float | None) -> JointStrength:
    """Cap at ``scale × demand`` for the components that are actually loaded.

    A component whose finished demand is ~0 is left unchecked. Setting a
    tiny capacity on a zero force would not reject it, and setting a
    zero capacity is not a legal strength.
    """
    comp = forces["compression"] if compression is not None else None
    shr = forces["shear"] if shear is not None else None
    if comp is not None:
        comp = comp * compression
    if shr is not None:
        shr = shr * shear
    comp = _positive(comp) if comp is not None else None
    shr = _positive(shr) if shr is not None else None
    if comp is None and shr is None:
        raise ValueError("nothing to cap; both demands are ~0")
    return JointStrength(axial_compression=comp, shear=shr)


def half_rejects(cells, policy, forces: dict, component: str, *, point_loads=None) -> bool:
    """True when a cap at half this component's demand rejects the build."""
    if component == "sag":
        if forces["sag"] < 1e-8:
            return False
        cap = SagCap(max_sag=forces["sag"] * 0.5, k=1.0, mass=1.0, g=1.0)
        try:
            estimate = estimate_sag(
                cells, policy, k=cap.k, mass=cap.mass, g=cap.g, point_loads=point_loads
            )
        except ValueError:
            return True
        return float(estimate.worst_sag) > float(cap.max_sag) + 1e-9
    if forces[component] < 1e-8:
        return False
    if component == "compression":
        strength = strength_scaled(forces, compression=0.5, shear=None)
    else:
        strength = strength_scaled(forces, compression=None, shear=0.5)
    return not peak_passes(cells, policy, strength, point_loads=point_loads)


def peak_passes(cells, policy, strength: JointStrength, *, point_loads=None) -> bool:
    from sim.stability import joints_within

    cap = JointCap(strength, safety_factor=1.0, k=1.0, mass=1.0, g=1.0)
    return joints_within(cells, policy, cap, point_loads=point_loads)


def binder(forces: dict, comp_rejects: bool, shear_rejects: bool, sag_rejects: bool) -> str:
    """Which half-caps reject the finished structure."""
    parts = []
    if comp_rejects:
        parts.append("compression")
    if shear_rejects:
        parts.append("shear")
    if sag_rejects:
        parts.append("sag")
    if not parts:
        return "none"
    quiet = []
    if not comp_rejects and forces["compression"] < 1e-8:
        quiet.append("compression demand ~0")
    if not shear_rejects and forces["shear"] < 1e-8:
        quiet.append("shear demand ~0")
    label = "+".join(parts)
    if quiet:
        label += " (" + "; ".join(quiet) + ")"
    return label


def bridge_one_sided(span: int) -> tuple[tuple, ...]:
    """Near pier and the whole deck. The far pier is absent.

    That is the prefix a one-pier order builds before the far support
    exists. It is not a plan; it is the load case.
    """
    world, _depot = build_bridge(span)
    cells = [
        (int(level), int(row), int(col))
        for level, row, col in zip(*world.blueprint.nonzero())
    ]
    piers = [cell for cell in cells if cell[0] == 1]
    far = max(piers, key=lambda cell: cell[2])
    return tuple(cell for cell in cells if cell != far)


def _scaffold_case(span: int, gait: str):
    """Same worlds as the scaffold campaign, without running that search."""
    from experiments.scaffold_campaign import disjoint_top_stances

    world, depot = build_bridge(span)
    if gait == "inchworm":
        pad = plant_entry_pad(world)
        policy, sag = caps_for_finished(world, CubicLattice3D(world).stability_policy(0))
        factory = inchworm_factory(policy)
        stances = disjoint_top_stances(factory(world), pad)
        if not stances:
            raise RuntimeError(f"span {span} has no inchworm stance")
        depot = stances[0]
    elif gait == "crane":
        policy, sag = caps_for_finished(world, CubicLattice3D(world).stability_policy(0))
        factory = geometry_factory(CRANE)
    else:
        raise ValueError(gait)
    return world, depot, factory, policy, sag


def _count(world, depot, factory, policy, sag, joints) -> dict:
    t0 = time.perf_counter()
    try:
        plan = plan_with_scaffold(
            world,
            depot,
            factory,
            policy,
            sag,
            max_nodes=MAX_NODES,
            joints=joints,
        )
    except SearchBudgetExceeded as exc:
        return {
            "status": "undecided",
            "scaffold_count": "",
            "seconds": time.perf_counter() - t0,
            "detail": str(exc),
        }
    seconds = time.perf_counter() - t0
    if plan is None:
        return {
            "status": "infeasible",
            "scaffold_count": "",
            "seconds": seconds,
            "detail": "",
        }
    return {
        "status": "ok",
        "scaffold_count": plan.scaffold_count,
        "seconds": seconds,
        "detail": "",
    }


def demand_rows() -> list[dict]:
    """Finished-structure demands. No planner."""
    policy = clamp_policy()
    specs: list[tuple[str, tuple, StabilityPolicy]] = []
    for n in (3, 6):
        specs.append((f"cantilever n={n}", clamp_bar(n, horizontal=True), policy))
        specs.append((f"tower n={n}", clamp_bar(n, horizontal=False), policy))
    frame = StabilityPolicy(up_axis=0, up_sign=1, ground_plane=0, max_cantilever=None)
    for span in (4, 5, 6):
        world, _depot = build_bridge(span)
        cells = tuple(
            (int(a), int(b), int(c)) for a, b, c in zip(*world.blueprint.nonzero())
        )
        specs.append((f"bridge span={span}", cells, frame))
        specs.append((f"bridge span={span} one-sided", bridge_one_sided(span), frame))
    specs.append(("ARMADAS shelter", shelter_cells(), frame))
    rows = []
    for name, cells, pol in specs:
        forces = peak_forces(cells, pol)
        comp_r = half_rejects(cells, pol, forces, "compression")
        shear_r = half_rejects(cells, pol, forces, "shear")
        sag_r = half_rejects(cells, pol, forces, "sag")
        rows.append(
            {
                "build": name,
                "sag_mg_per_k": f"{forces['sag']:.6g}",
                "compression_mg": f"{forces['compression']:.6g}",
                "tension_mg": f"{forces['tension']:.6g}",
                "shear_mg": f"{forces['shear']:.6g}",
                "n_joints": forces["n_joints"],
                "half_compression_rejects": comp_r,
                "half_shear_rejects": shear_r,
                "half_sag_rejects": sag_r,
                "binder": binder(forces, comp_r, shear_r, sag_r),
                "k": 1,
                "mass": 1,
                "g": 1,
            }
        )
    # Tip load on the n=3 shear chain. Closed form: every spring gains P.
    cells = clamp_bar(3, horizontal=True)
    bare = peak_forces(cells, policy)
    tip = [((1, 0, 3), 2.0)]
    loaded = peak_forces(cells, policy, point_loads=tip)
    expected = path_chain_joint_force(1, 3, weight=1.0) + 2.0
    comp_r = half_rejects(cells, policy, loaded, "compression", point_loads=tip)
    shear_r = half_rejects(cells, policy, loaded, "shear", point_loads=tip)
    sag_r = half_rejects(cells, policy, loaded, "sag", point_loads=tip)
    rows.append(
        {
            "build": "cantilever n=3 tip load P=2",
            "sag_mg_per_k": f"{loaded['sag']:.6g}",
            "compression_mg": f"{loaded['compression']:.6g}",
            "tension_mg": f"{loaded['tension']:.6g}",
            "shear_mg": f"{loaded['shear']:.6g}",
            "n_joints": loaded["n_joints"],
            "half_compression_rejects": comp_r,
            "half_shear_rejects": shear_r,
            "half_sag_rejects": sag_r,
            "binder": (
                f"{binder(loaded, comp_r, shear_r, sag_r)}; "
                f"root shear {bare['shear']:.6g} -> {loaded['shear']:.6g} "
                f"(closed form {expected:g})"
            ),
            "k": 1,
            "mass": 1,
            "g": 1,
        }
    )
    return rows


def scaffold_rows(spans: tuple[int, ...], gaits: tuple[str, ...]) -> list[dict]:
    """Scaffold count with the sag cap alone, and with a joint cap added."""
    frame = StabilityPolicy(up_axis=0, up_sign=1, ground_plane=0, max_cantilever=None)
    rows = []
    for span in spans:
        finished = peak_forces(
            tuple(
                (int(a), int(b), int(c))
                for a, b, c in zip(*build_bridge(span)[0].blueprint.nonzero())
            ),
            frame,
        )
        one = peak_forces(bridge_one_sided(span), frame)
        gap = one["shear"] - finished["shear"]
        for gait in gaits:
            world, depot, factory, policy, sag = _scaffold_case(span, gait)
            # Loose sag would hide the sag gate. The campaign cap is the
            # finished sag, so a prefix that sags more than the finished
            # bridge already needs scaffold or a different order.
            sag_only = _count(world, depot, factory, policy, sag, None)
            at_finished = JointCap(
                strength_scaled(finished, compression=1.0, shear=1.0),
                safety_factor=1.0,
            )
            half = JointCap(
                strength_scaled(finished, compression=0.5, shear=0.5),
                safety_factor=1.0,
            )
            joint_finished = _count(world, depot, factory, policy, sag, at_finished)
            # Below the finished demand the blueprint itself fails, so the
            # search returns without trying a larger scaffold set.
            joint_half = _count(world, depot, factory, policy, sag, half)
            if gap > GAP_ATOL:
                mid_shear = 0.5 * (finished["shear"] + one["shear"])
                # Keep compression at the finished demand so the pier
                # check does not move; only the shear allowable sits in
                # the gap.
                mid_strength = JointStrength(
                    axial_compression=_positive(finished["compression"]),
                    shear=mid_shear,
                )
                mid = _count(
                    world,
                    depot,
                    factory,
                    policy,
                    sag,
                    JointCap(mid_strength, safety_factor=1.0),
                )
                gap_note = f"{gap:.6g}"
            else:
                mid = {
                    "status": "no-gap",
                    "scaffold_count": "",
                    "seconds": 0.0,
                    "detail": "one-sided shear is not above the finished shear",
                }
                gap_note = f"{gap:.6g}"
            rows.append(
                {
                    "span": span,
                    "gait": gait,
                    "finished_shear_mg": f"{finished['shear']:.6g}",
                    "onesided_shear_mg": f"{one['shear']:.6g}",
                    "shear_gap_mg": gap_note,
                    "finished_compression_mg": f"{finished['compression']:.6g}",
                    "sag_only_count": sag_only["scaffold_count"],
                    "sag_only_status": sag_only["status"],
                    "joint_at_finished_count": joint_finished["scaffold_count"],
                    "joint_at_finished_status": joint_finished["status"],
                    "joint_midgap_count": mid["scaffold_count"],
                    "joint_midgap_status": mid["status"],
                    "joint_half_count": joint_half["scaffold_count"],
                    "joint_half_status": joint_half["status"],
                    "k": 1,
                    "mass": 1,
                    "g": 1,
                }
            )
    return rows


def _fmt_count(count, status: str) -> str:
    """Zero is a measured count. A missing count is not a zero."""
    if status != "ok":
        return f"— ({status})"
    return f"{count} ({status})"


def _write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def member_rows() -> list[dict]:
    """Planner specimens the crane can stand beside. Not the clamp bar.

    The level-2 arm is the cantilever ``build_cantilever`` already uses.
    A pure clamp bar has no stock floor under the root, so the scaffold
    search is not run on it. The demands table is that clamp bar.
    """
    from experiments.stability_cost import build_cantilever

    rows = []
    specs = []
    for length in (3, 6):
        world, depot = build_cantilever(length)
        specs.append((f"level-2 cantilever length={length}", world, depot))
    for n_free in (3, 6):
        world = World3D(n_free + 3, 4, 6)
        depot = (1, 1, 1)
        for level in range(1, n_free + 2):
            world.blueprint[level, 1, 3] = True
        specs.append((f"tower n_free={n_free}", world, depot))
    crane = geometry_factory(CRANE)
    # climb=0 stays on the floor. Reach 4 cannot touch the top of a
    # 6-voxel tower, which would be a reach failure rather than a joint
    # failure. The tower uses a longer arm so the cap is the thing that
    # decides.
    tall = geometry_factory(MotionModel(reach_radius=8, stride=1, climb=0))
    for name, world, depot in specs:
        policy, sag = caps_for_finished(world, CubicLattice3D(world).stability_policy(0))
        factory = tall if name.startswith("tower") else crane
        cells = tuple(
            (int(a), int(b), int(c)) for a, b, c in zip(*world.blueprint.nonzero())
        )
        forces = peak_forces(cells, policy)
        sag_only = _count(world, depot, factory, policy, sag, None)
        if forces["shear"] >= 1e-8 or forces["compression"] >= 1e-8:
            at = JointCap(strength_scaled(forces, compression=1.0, shear=1.0))
            half = JointCap(strength_scaled(forces, compression=0.5, shear=0.5))
            joint_at = _count(world, depot, factory, policy, sag, at)
            joint_half = _count(world, depot, factory, policy, sag, half)
        else:
            joint_at = {"scaffold_count": "", "status": "no-demand"}
            joint_half = joint_at
        rows.append(
            {
                "build": name,
                "sag_mg_per_k": f"{forces['sag']:.6g}",
                "compression_mg": f"{forces['compression']:.6g}",
                "shear_mg": f"{forces['shear']:.6g}",
                "sag_only_count": sag_only["scaffold_count"],
                "sag_only_status": sag_only["status"],
                "joint_at_finished_count": joint_at["scaffold_count"],
                "joint_at_finished_status": joint_at["status"],
                "joint_half_count": joint_half["scaffold_count"],
                "joint_half_status": joint_half["status"],
                "k": 1,
                "mass": 1,
                "g": 1,
            }
        )
    return rows


def _summary(demands: list[dict], scaffolds: list[dict], members: list[dict], derived) -> str:
    lines = [
        "Joint limit vs sag limit. k = mass = g = 1.",
        "Force unit: mass·g. Sag unit: mass·g/k. Stiffness is not a fit.",
        "",
        "Published 3×3×3 cube (not applied to the rows below):",
        f"  strength {derived.published_strength_kpa} kPa "
        f"(SD {derived.published_strength_sd_kpa}), status {derived.published_strength_status}",
        f"  manuscript strength {derived.manuscript_strength_kpa} kPa "
        f"(±{derived.manuscript_strength_sd_kpa}); not averaged",
        f"  failure load {derived.failure_load_status}: "
        f"> {derived.failure_load_lower_bound_n:.0f} N",
        f"  stiffness {derived.stiffness_mpa} MPa, status {derived.stiffness_status}; "
        f"SD {derived.stiffness_sd_status} "
        f"(HTML '{derived.stiffness_sd_published_text}', "
        f"manuscript ±{derived.stiffness_sd_manuscript_mpa} MPa)",
        f"  gross area {derived.gross_area_m2:.8f} m^2 = (3 × {derived.voxel_edge_m} m)^2",
        f"  implied mean load {derived.implied_mean_load_n:.6f} N "
        f"({derived.implied_load_status})",
        f"  face compression {derived.face_compression_n:.6f} N "
        f"(SD {derived.face_compression_sd_n:.6f} N from the strength SD only)",
        f"  shear: {derived.shear_n}; moment: {derived.moment_nm}",
        f"  fastener tension {derived.fastener_tension_n} N ± "
        f"{derived.fastener_tension_sd_n}: {derived.fastener_tension_status}",
        "",
        "Finished demands",
    ]
    for row in demands:
        lines.append(
            f"  {row['build']}: sag {row['sag_mg_per_k']}, "
            f"compression {row['compression_mg']}, tension {row['tension_mg']}, "
            f"shear {row['shear_mg']}; half-caps reject "
            f"compression={row['half_compression_rejects']} "
            f"shear={row['half_shear_rejects']} sag={row['half_sag_rejects']}; "
            f"binder: {row['binder']}"
        )
    lines.append("")
    lines.append("Scaffold counts")
    for row in scaffolds:
        lines.append(
            f"  span {row['span']} {row['gait']}: "
            f"finished shear {row['finished_shear_mg']}, "
            f"one-sided shear {row['onesided_shear_mg']}, "
            f"gap {row['shear_gap_mg']}; "
            f"sag-only {_fmt_count(row['sag_only_count'], row['sag_only_status'])}; "
            f"joint at finished demand "
            f"{_fmt_count(row['joint_at_finished_count'], row['joint_at_finished_status'])}; "
            f"mid-gap {_fmt_count(row['joint_midgap_count'], row['joint_midgap_status'])}; "
            f"half finished demand {_fmt_count(row['joint_half_count'], row['joint_half_status'])}"
        )
    lines.append("")
    lines.append("Level-2 arms and towers (crane)")
    for row in members:
        lines.append(
            f"  {row['build']}: sag {row['sag_mg_per_k']}, "
            f"compression {row['compression_mg']}, shear {row['shear_mg']}; "
            f"sag-only {_fmt_count(row['sag_only_count'], row['sag_only_status'])}; "
            f"joint at finished demand "
            f"{_fmt_count(row['joint_at_finished_count'], row['joint_at_finished_status'])}; "
            f"half {_fmt_count(row['joint_half_count'], row['joint_half_status'])}"
        )
    lines.append("")
    lines.append(
        "Reading. A cap at the finished demand does not add scaffold: "
        "a balanced bridge order, and a propped inchworm order, stay at "
        "or under that demand, and the search stops at the sag-only count. "
        "A cap at half the finished demand rejects the blueprint itself. "
        "Scaffold is removed at the end, so it cannot rescue that rejection. "
        "The count goes from the sag-only number to infeasible. It does not "
        "step up. One-sided deck shear is higher than the finished beam, and "
        "that prefix fails the finished-demand cap; the crane has another "
        "order, so its count stays 0. k = 1 throughout. The newton face "
        "capacity above is not a row."
    )
    lines.append("")
    return "\n".join(lines) + "\n"


def _chart(scaffolds: list[dict], path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    labels = []
    sag_counts = []
    joint_counts = []
    for row in scaffolds:
        labels.append(f"span {row['span']}\n{row['gait']}")

        def _num(status, count):
            if status != "ok" or count == "":
                return 0
            return int(count)

        sag_counts.append(_num(row["sag_only_status"], row["sag_only_count"]))
        # Prefer the mid-gap count when that search ran; otherwise the
        # finished-demand count. Infeasible stays 0 and is named in the CSV.
        if row["joint_midgap_status"] == "ok":
            joint_counts.append(int(row["joint_midgap_count"]))
        elif row["joint_at_finished_status"] == "ok":
            joint_counts.append(int(row["joint_at_finished_count"]))
        else:
            joint_counts.append(0)
    if not labels:
        return
    xs = range(len(labels))
    width = 0.36
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    ax.bar([x - width / 2 for x in xs], sag_counts, width, label="sag cap only", color="#2563eb")
    ax.bar(
        [x + width / 2 for x in xs],
        joint_counts,
        width,
        label="sag cap + joint cap",
        color="#b45309",
    )
    ax.set_xticks(list(xs), labels)
    ax.set_ylabel("scaffold voxels")
    ax.set_title("Scaffold count at k = 1 (mass·g, not newtons)")
    ax.legend(frameon=False)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=120)
    plt.close(fig)


def run(smoke: bool = False) -> dict:
    derived = armadas_face_compression()
    demands = demand_rows()
    if smoke:
        scaffolds = scaffold_rows((4,), ("crane",))
    else:
        scaffolds = scaffold_rows((4, 5, 6), ("crane",))
        scaffolds.extend(scaffold_rows((4,), ("inchworm",)))
    members = [] if smoke else member_rows()
    text = _summary(demands, scaffolds, members, derived)
    DOCS.mkdir(parents=True, exist_ok=True)
    (DOCS / "joint_limits_summary.txt").write_text(text, encoding="utf-8")
    _write_csv(DOCS / "joint_limits_demands.csv", demands)
    _write_csv(DOCS / "joint_limits_scaffold.csv", scaffolds)
    _write_csv(DOCS / "joint_limits_members.csv", members)
    _chart(scaffolds, DOCS / "joint_limits.png")
    return {"summary": text, "demands": demands, "scaffolds": scaffolds, "derived": derived}


def main(argv: list[str] | None = None) -> None:
    args = list(sys.argv[1:] if argv is None else argv)
    smoke = "smoke" in args
    # GAIT is the inchworm configuration the scaffold campaign uses.
    # Referencing it keeps this module on that gait if the name moves.
    _ = GAIT
    result = run(smoke=smoke)
    print(result["summary"], end="")


if __name__ == "__main__":
    main()
