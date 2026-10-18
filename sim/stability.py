# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Structural checks for partial lattice builds (v0.2 milestone 1).

The coordination stack was built with no gravity. This module is the
first structural gate, and it stays out of the planner unless a caller
passes a :class:`StabilityPolicy`. Two questions, kept separate on
purpose:

* :func:`is_stable` — a combinatorial screen. Every voxel must face-connect
  to a column that bears on the base, and no voxel may sit more than
  ``max_cantilever`` horizontal face-steps from such a column.
* :func:`estimate_sag` — a small-displacement linear-elastic estimate of
  that same partial build under its own weight. Voxels are nodes, face
  joints are springs, ``Ku = f`` is solved with numpy. The number to
  report is the worst-case displacement.

The screen is the gate (dimensionless, no material data). The sag
estimate is a measurement, not a gate: a displacement limit needs a
calibrated stiffness, and this milestone does not invent one.

Coordinate frame
----------------
Cells are integer tuples. ``up_axis`` selects the vertical coordinate
and ``up_sign`` says which way is up (``+1`` if larger coordinates are
higher, as in the 3D ``(level, row, col)`` grid; ``-1`` if smaller
coordinates are higher, as in the 2D grid whose ground row is the
bottom). The base is either the whole plane ``coord[up_axis] ==
ground_plane`` (the stock worlds: a solid ground row or a solid level
0) or an explicit set of ``base`` cells. Stock lattices package this
as :meth:`sim.geometry.Geometry.stability_policy`.

What "supported" means
----------------------
A voxel is **planted** when the cell directly beneath it is base. It is
**column-supported** when walking straight down through occupied cells
reaches a planted voxel with no gap. Column-supported voxels have
overhang 0.

Overhang of every other voxel is the minimum, over face-adjacent paths
that stay inside the structure, of the number of *horizontal* steps on
the path. A vertical step costs 0, so:

* a block stacked on a cantilever inherits that cantilever's overhang
  (it does not stick out any farther);
* a block hanging under a beam inherits the beam's overhang too.

The second point is the bolted-lattice assumption. ARMADAS / BILL-E
fasteners can take tension, so a voxel is allowed to hang. This is not
a masonry rule. A compression-only screen (drop the downward edges)
is deliberately not implemented.

Face adjacency is 4-connected in 2D and 6-connected in 3D. A voxel that
only touches the structure at a corner or an edge carries no load and
is floating. Bonds are axis-aligned; there is no diagonal member.

``max_cantilever is None`` keeps the connectivity rule and allows an
unlimited span. ``0`` means every voxel must itself be column-supported
(a tower passes at any height; a bridge fails). Height is not a
failure in :func:`is_stable`. The sag estimate is what height costs.

Sag model
---------
Each face-adjacent pair of voxels is one linear spring. The spring
resists relative displacement of the two voxel centers:

* along the joint, with stiffness ``k_axial`` (force per length);
* along each perpendicular axis, with stiffness ``k_shear``.

Passing only ``k`` sets both equal. There are no rotational degrees of
freedom and no moment transmitted except what those springs produce.
Equilibrium is small-displacement, about the as-built geometry:
rest length never appears, because the spring wants *zero additional
relative displacement*. Geometric stiffness, buckling, contact, and
friction are absent.

Voxels fastened to the base (planted) have every degree of freedom
fixed. That is the build plate, not a compliant foundation. Every free
voxel gets the same downward load ``mass * g``. Defective voxels, if
they are passed in, are ordinary material: this model does not weaken
their joints. The coordination stack's defect flag is not a crack.

The three displacement axes decouple, and gravity has only a vertical
component, so the solved horizontal displacement is identically zero.
There is no Poisson effect.

Closed forms (the print checks)
-------------------------------
A one-voxel-wide bar of ``n`` free voxels in a straight line, rooted on
one fixed voxel, is a path of identical springs. Number the free voxels
``i = 1`` (nearest the root) through ``i = n`` (the tip). With weight
``w = mass * g`` and joint stiffness ``k`` in the loaded direction:

    u(i) = (w / k) * i * (2n - i + 1) / 2
    u(n) = (w / k) * n * (n + 1) / 2

Horizontal bar, load vertical: ``k`` is ``k_shear`` (a shear chain).
Vertical bar, load vertical: ``k`` is ``k_axial`` (a compression
chain). :func:`shear_cantilever_tip_sag` and
:func:`axial_tower_compression` are those tip values;
:func:`path_chain_deflection_at` is the whole profile.

Those are the easiest numbers to check against a physical build,
because each is one specimen and one length measurement:

1. **Cantilever.** Fasten one root voxel to a rigid plate and run
   ``n`` identical voxels straight out horizontally, in the air, joined
   face to face. In the model that plate is a finite ``base`` under the
   root only. Do not use the stock infinite ``ground_plane`` for this
   specimen: on that floor the whole first course is planted, a
   "cantilever" in the first course does not move, and a later course
   springs off a column that can itself compress, so the one-line
   formula does not apply. Measure the downward deflection of voxel
   ``i`` (and at least the tip) under self-weight. Compare to
   ``path_chain_deflection_at(i, n, weight=m*g, stiffness=k_shear)``.
   Two lengths (for example ``n = 3`` and ``n = 6``) tell this ``n²``
   shear-chain apart from an Euler-Bernoulli cantilever, whose
   self-weight tip sag scales as ``n⁴``. Fit ``k_shear`` on one length,
   predict the other.
2. **Tower.** The same ``n`` free voxels stacked on one fixed root.
   Top shortening is the axial formula with ``k_axial``. The ratio of
   cantilever tip sag to tower shortening, at the same ``n`` and mass,
   is ``k_axial / k_shear``.
3. **Bridge.** Two piers with a deck between them. After ``k_axial``
   and ``k_shear`` are fit, :attr:`SagEstimate.worst_sag` should sit
   on the midspan deck voxel (:attr:`SagEstimate.worst_sag_cell`), and
   the deck profile in :attr:`SagEstimate.displacements` should be
   symmetric. Measuring midspan drop relative to the pier tops is the
   check. This one has no closed form; it is the solver's output.

Units: if ``mass``, ``g``, and the stiffnesses are SI (kg, m/s², N/m),
displacements are meters. The defaults are all 1, so a result is in
units of ``mass * g / k``. The voxel pitch never enters. A joint that
is stiff compared with the block is the assumption behind lumping all
compliance into ``k``; a continuously fused beam, where the member
itself bends, will not follow the ``n(n+1)/2`` profile, and that
disagreement is information about the model.

What this will not do
---------------------
Robot weight, wind, payload, and fastener preload are not load cases.
A hollow-box roof is a span: ``max_cantilever=0`` rejects it, which is
why the gate is off by default and why the published speedup and yield
runs are unchanged. The solve is a dense numpy factorization. It is
the right size for a partial build you would print, and it is not run
inside the planner's search. Unanchored input (nothing fastened to the
base, or a voxel the springs cannot reach from a fastened voxel) raises
:class:`ValueError` instead of returning a finite number from a
singular matrix.

The gate and the search
-----------------------
:func:`sim.planner.plan_build_order` and :func:`sim.planner.validate_plan`
take ``stability=`` and refuse any prefix that fails :func:`is_stable`.
:func:`sim.choreographer.stability_gate` is the third placement gate,
beside connectivity and buildability, and a refusal takes the existing
kickback path so the sequencer tries a different cell. The gate counts
voxels that are already in the world, plus the cell being proposed. It
does not count another robot's unplaced deed: a reservation is not a
block, and gravity will not wait for one. Connectivity still counts
deeds, because a deeded cell *will* be solid and can trap a robot.
Those are different questions.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Iterable, NamedTuple

import numpy as np

from .world import DEFECT, VOXEL

Cell = tuple[int, ...]


class StabilityPolicy(NamedTuple):
    """How a lattice sits in gravity, plus the span limit.

    ``up_axis`` / ``up_sign`` / ``ground_plane`` / ``base`` are the
    frame (see the module docstring). ``max_cantilever`` is the span
    limit in horizontal face-steps; ``None`` means "connected to the
    base is enough". The default of ``0`` is strict columns, and it is
    only consulted when a caller actually passes a policy — the planner
    does not invent one.

    ``base`` is a tuple of cells so a policy stays hashable and
    comparable. An empty base with a ``ground_plane`` is the stock
    "the whole floor is plate" frame.
    """

    up_axis: int
    up_sign: int
    ground_plane: int | None = None
    base: tuple[Cell, ...] = ()
    max_cantilever: int | None = 0


@dataclass(frozen=True)
class StabilityAssessment:
    """Why :func:`is_stable` accepted or refused a set of voxels.

    ``overhang`` maps each voxel that can reach a grounded column to its
    horizontal-step distance. ``floating`` are the voxels that cannot.
    ``worst_overhang`` is the largest finite distance (0 when nothing
    is connected).
    """

    stable: bool
    floating: tuple[Cell, ...]
    overhang: tuple[tuple[Cell, int], ...]
    worst_overhang: int
    max_cantilever: int | None

    @property
    def reason(self) -> str:
        if self.stable:
            return "stable"
        if self.floating:
            n = len(self.floating)
            return (
                f"{n} voxel(s) not face-connected to a column that "
                f"bears on the base"
            )
        return (
            f"overhang {self.worst_overhang} exceeds cantilever limit "
            f"{self.max_cantilever}"
        )


class SagEstimate(NamedTuple):
    """Linear-elastic self-weight result for one partial build.

    ``worst_displacement`` is the largest Euclidean norm of a voxel's
    displacement (the headline number). ``worst_sag`` is the largest
    *downward* component, reported positive downward, at
    ``worst_sag_cell``. In this model those two coincide whenever the
    motion is purely vertical, which self-weight on an axis-aligned
    spring network is. ``displacements`` maps every voxel to its
    displacement vector in coordinate order (the sign is the coordinate
    direction, not "down"). ``n_fixed`` voxels are fastened to the base;
    ``n_free`` are solved.

    Ties break toward the lexicographically first cell.
    """

    worst_displacement: float
    worst_cell: Cell | None
    worst_sag: float
    worst_sag_cell: Cell | None
    displacements: dict
    n_fixed: int
    n_free: int


class JointStrain(NamedTuple):
    """Strain proxy on one face joint.

    ``axial`` is the relative displacement along the joint, divided by
    ``pitch``, positive when the joint lengthens (tension). ``shear`` is
    the magnitude of the relative displacement perpendicular to the
    joint, divided by ``pitch``. Both are dimensionless. The voxel pitch
    is the center-to-center distance the caller believes the displacement
    unit corresponds to; the sag solve itself never uses it.
    """

    a: Cell
    b: Cell
    axis: int
    axial: float
    shear: float


def occupied_voxels(world) -> tuple[Cell, ...]:
    """Voxels currently in ``world`` (good or defective), as cell tuples.

    Ground is not included: it is the base, not a placed part. A
    defective voxel is still a physical block, so it counts until the
    robot removes it. Order is numpy's C-order scan, which is stable
    for a given occupancy array.
    """
    mask = (world.occupancy == VOXEL) | (world.occupancy == DEFECT)
    found = np.argwhere(mask)
    return tuple(tuple(int(v) for v in row) for row in found)


def is_stable(
    occupied_cells: Iterable,
    policy: StabilityPolicy | None = None,
    **frame,
) -> bool:
    """Whether this partial build passes the structural screen.

    ``occupied_cells`` are the voxels (not the ground). Pass a
    :class:`StabilityPolicy`, or the frame fields directly
    (``up_axis``, ``up_sign``, and ``ground_plane`` and/or ``base``,
    plus optional ``max_cantilever``). Keyword fields override a policy
    when both are given, so a caller can tighten the span limit of a
    lattice's stock policy without rebuilding the frame.

    An empty set is stable. A nonempty set with nothing bearing on the
    base is not.
    """
    return assess_stability(occupied_cells, policy, **frame).stable


def assess_stability(
    occupied_cells: Iterable,
    policy: StabilityPolicy | None = None,
    **frame,
) -> StabilityAssessment:
    """:func:`is_stable`, plus the overhang of every connected voxel."""
    resolved = _resolve_policy(policy, frame)
    cells = _normalize(occupied_cells)
    if not cells:
        return StabilityAssessment(True, (), (), 0, resolved.max_cantilever)
    _check_axis(cells, resolved)
    occupied = set(cells)
    base = _base_set(resolved, len(cells[0]))
    sources = [
        c for c in cells if _column_supported(c, occupied, resolved, base)
    ]
    dist = _overhang_distances(occupied, sources, resolved)
    floating = tuple(sorted(occupied.difference(dist)))
    overhang = tuple(sorted(dist.items()))
    worst = max(dist.values()) if dist else 0
    limit = resolved.max_cantilever
    within = limit is None or all(d <= limit for d in dist.values())
    return StabilityAssessment(
        stable=not floating and within,
        floating=floating,
        overhang=overhang,
        worst_overhang=int(worst),
        max_cantilever=limit,
    )


def estimate_sag(
    occupied_cells: Iterable,
    policy: StabilityPolicy | None = None,
    *,
    k: float = 1.0,
    k_axial: float | None = None,
    k_shear: float | None = None,
    mass: float = 1.0,
    g: float = 1.0,
    **frame,
) -> SagEstimate:
    """Worst-case displacement of a partial build under self-weight.

    See the module docstring for the spring law, the fixed-base
    boundary condition, and the closed forms. ``k`` fills in whichever
    of ``k_axial`` and ``k_shear`` is omitted.

    Raises :class:`ValueError` when the springs do not tie every voxel
    to a voxel fastened to the base (the stiffness matrix would be
    singular, and a least-squares number would not be a sag).
    """
    resolved = _resolve_policy(policy, frame)
    cells = _normalize(occupied_cells)
    if not cells:
        return SagEstimate(0.0, None, 0.0, None, {}, 0, 0)
    _check_axis(cells, resolved)
    k_axial, k_shear = _stiffnesses(k, k_axial, k_shear)
    if mass < 0 or g < 0:
        raise ValueError("mass and g must be non-negative")

    occupied = set(cells)
    base = _base_set(resolved, len(cells[0]))
    planted = [c for c in cells if _is_planted(c, resolved, base)]
    if not planted:
        raise ValueError(
            "no voxel is fastened to the base; sag is undefined "
            "(the structure is a rigid-body mechanism)"
        )
    anchored = _reachable(planted, occupied)
    loose = occupied.difference(anchored)
    if loose:
        sample = min(loose)
        raise ValueError(
            f"{len(loose)} voxel(s) are not face-connected to a voxel "
            f"fastened to the base (for example {sample}); the stiffness "
            f"matrix is singular"
        )

    nodes = sorted(occupied)
    index = {node: i for i, node in enumerate(nodes)}
    ndim = len(nodes[0])
    n = len(nodes)
    ndof = n * ndim
    stiffness = np.zeros((ndof, ndof), dtype=float)
    load = np.zeros(ndof, dtype=float)
    weight = float(mass) * float(g)
    # Force component along up_axis. up_sign +1 means "up" is +coord, so
    # gravity points in the -coord direction.
    load_step = -resolved.up_sign * weight

    for a_i, a in enumerate(nodes):
        load[a_i * ndim + resolved.up_axis] = load_step
        for b in _face_neighbors(a):
            b_i = index.get(b)
            if b_i is None or b_i < a_i:
                continue
            axis = _joint_axis(a, b)
            ia = a_i * ndim
            ib = b_i * ndim
            for dof in range(ndim):
                kk = k_axial if dof == axis else k_shear
                stiffness[ia + dof, ia + dof] += kk
                stiffness[ib + dof, ib + dof] += kk
                stiffness[ia + dof, ib + dof] -= kk
                stiffness[ib + dof, ia + dof] -= kk

    planted_set = set(planted)
    fixed = [
        a_i * ndim + dof
        for a_i, a in enumerate(nodes)
        if a in planted_set
        for dof in range(ndim)
    ]
    n_fixed = len(planted)
    free = [i for i in range(ndof) if i not in set(fixed)]
    solved = np.zeros(ndof, dtype=float)
    if free:
        try:
            solved[np.array(free)] = np.linalg.solve(
                stiffness[np.ix_(free, free)], load[np.array(free)]
            )
        except np.linalg.LinAlgError as exc:
            raise ValueError(
                "stiffness matrix is singular; the structure is a mechanism "
                "under this joint model"
            ) from exc

    displacements: dict[Cell, tuple[float, ...]] = {}
    worst_norm = 0.0
    worst_cell: Cell | None = None
    worst_sag = 0.0
    worst_sag_cell: Cell | None = None
    for a_i, a in enumerate(nodes):
        vec = tuple(float(solved[a_i * ndim + dof]) for dof in range(ndim))
        displacements[a] = vec
        norm = float(np.linalg.norm(solved[a_i * ndim : (a_i + 1) * ndim]))
        sag = float(-resolved.up_sign * vec[resolved.up_axis])
        if norm > worst_norm:
            worst_norm = norm
            worst_cell = a
        if sag > worst_sag:
            worst_sag = sag
            worst_sag_cell = a
    return SagEstimate(
        worst_displacement=worst_norm,
        worst_cell=worst_cell,
        worst_sag=worst_sag,
        worst_sag_cell=worst_sag_cell,
        displacements=displacements,
        n_fixed=n_fixed,
        n_free=n - n_fixed,
    )


def joint_strains(
    estimate: SagEstimate, *, pitch: float = 1.0
) -> tuple[JointStrain, ...]:
    """Per-joint strain proxy from an :func:`estimate_sag` result.

    One entry per face-adjacent pair, ``a`` lexicographically before
    ``b``. ``pitch`` must be in the same length unit as the
    displacements (meters if the sag inputs were SI; ``1`` in the
    default model units, where a result of ``mass * g / k`` is treated
    as one voxel). Positive ``axial`` is extension.
    """
    if pitch <= 0:
        raise ValueError("pitch must be positive")
    disp = estimate.displacements
    if not disp:
        return ()
    present = set(disp)
    out: list[JointStrain] = []
    for a in sorted(present):
        ua = disp[a]
        for b in _face_neighbors(a):
            if b not in present or b < a:
                continue
            ub = disp[b]
            axis = _joint_axis(a, b)
            # +1 when b sits on the positive side of a along the joint,
            # so the projection is the change in length, not the raw
            # coordinate delta (which flips if the neighbor scan hits
            # the negative side first).
            direction = 1.0 if b[axis] > a[axis] else -1.0
            du = [ub[i] - ua[i] for i in range(len(a))]
            axial = direction * du[axis] / pitch
            shear_sq = 0.0
            for i, component in enumerate(du):
                if i == axis:
                    continue
                shear_sq += component * component
            out.append(
                JointStrain(
                    a=a,
                    b=b,
                    axis=axis,
                    axial=float(axial),
                    shear=float(shear_sq**0.5 / pitch),
                )
            )
    return tuple(out)


def path_chain_deflection_at(
    index: int,
    n_free: int,
    *,
    weight: float,
    stiffness: float,
) -> float:
    """Downward deflection at free voxel ``index`` of a one-voxel-wide bar.

    ``index`` is 1 at the voxel next to the fixed root and ``n_free`` at
    the tip. ``weight`` is ``mass * g`` per voxel. ``stiffness`` is
    ``k_shear`` for a horizontal cantilever and ``k_axial`` for a tower.
    The derivation is the module docstring (each spring carries the
    weight of every voxel outboard of it).
    """
    if n_free < 0 or not (1 <= index <= n_free):
        raise ValueError(
            f"index must be in 1..n_free (got index={index}, n_free={n_free})"
        )
    if stiffness <= 0:
        raise ValueError("stiffness must be positive")
    return (weight / stiffness) * index * (2 * n_free - index + 1) / 2.0


def path_chain_deflection(
    n_free: int, *, weight: float, stiffness: float
) -> float:
    """Tip deflection of the same bar. Zero when ``n_free`` is 0."""
    if n_free == 0:
        return 0.0
    return path_chain_deflection_at(
        n_free, n_free, weight=weight, stiffness=stiffness
    )


def shear_cantilever_tip_sag(
    n_free: int,
    *,
    mass: float = 1.0,
    g: float = 1.0,
    k_shear: float = 1.0,
) -> float:
    """Tip sag of a one-voxel-wide horizontal cantilever of ``n_free`` voxels.

    The root voxel that is fastened to the plate is not counted in
    ``n_free``. This is the number to compare with a printed cantilever
    of the same length once ``k_shear`` has been chosen (or to fit
    ``k_shear`` from a measured tip sag).
    """
    return path_chain_deflection(
        n_free, weight=mass * g, stiffness=k_shear
    )


def axial_tower_compression(
    n_free: int,
    *,
    mass: float = 1.0,
    g: float = 1.0,
    k_axial: float = 1.0,
) -> float:
    """Top shortening of a one-voxel-wide tower of ``n_free`` voxels.

    Same chain as the cantilever, with the axial spring in the loaded
    direction. Same ``n_free`` convention: the fastened root is extra.
    """
    return path_chain_deflection(
        n_free, weight=mass * g, stiffness=k_axial
    )


# -- internals ---------------------------------------------------------------


def _resolve_policy(
    policy: StabilityPolicy | None, frame: dict
) -> StabilityPolicy:
    frame = dict(frame)
    if "base" in frame:
        frame["base"] = tuple(
            tuple(int(x) for x in cell) for cell in frame["base"]
        )
    if policy is None:
        missing = [key for key in ("up_axis", "up_sign") if key not in frame]
        if missing:
            raise TypeError(
                "pass a StabilityPolicy, or up_axis and up_sign "
                f"(missing {missing})"
            )
        data = {
            "up_axis": int(frame.pop("up_axis")),
            "up_sign": int(frame.pop("up_sign")),
            "ground_plane": frame.pop("ground_plane", None),
            "base": frame.pop("base", ()),
            "max_cantilever": frame.pop("max_cantilever", 0),
        }
        if data["ground_plane"] is not None:
            data["ground_plane"] = int(data["ground_plane"])
    else:
        data = policy._asdict()
        for key in list(frame):
            if key in data:
                value = frame.pop(key)
                if key in ("up_axis", "up_sign", "ground_plane") and value is not None:
                    value = int(value)
                data[key] = value
    unknown = sorted(frame)
    if unknown:
        raise TypeError(f"unknown stability argument(s): {unknown}")
    if data["up_sign"] not in (1, -1):
        raise ValueError("up_sign must be +1 or -1")
    limit = data["max_cantilever"]
    if limit is not None and int(limit) < 0:
        raise ValueError("max_cantilever must be >= 0, or None for no span limit")
    if limit is not None:
        data["max_cantilever"] = int(limit)
    return StabilityPolicy(**data)


def _normalize(occupied_cells: Iterable) -> tuple[Cell, ...]:
    cells: list[Cell] = []
    seen: set[Cell] = set()
    for raw in occupied_cells:
        cell = tuple(int(x) for x in raw)
        if not cell:
            raise ValueError("a cell needs at least one coordinate")
        if cell not in seen:
            seen.add(cell)
            cells.append(cell)
    if not cells:
        return ()
    rank = len(cells[0])
    if any(len(cell) != rank for cell in cells):
        raise ValueError("all cells must have the same number of coordinates")
    return tuple(cells)


def _check_axis(cells: tuple[Cell, ...], policy: StabilityPolicy) -> None:
    rank = len(cells[0])
    if not 0 <= policy.up_axis < rank:
        raise ValueError(
            f"up_axis {policy.up_axis} is outside 0..{rank - 1} for these cells"
        )


def _base_set(policy: StabilityPolicy, rank: int) -> set[Cell]:
    base: set[Cell] = set()
    for cell in policy.base:
        if len(cell) != rank:
            raise ValueError(
                f"base cell {cell} has rank {len(cell)}, voxels have rank {rank}"
            )
        base.add(cell)
    return base


def _down(cell: Cell, policy: StabilityPolicy) -> Cell:
    nxt = list(cell)
    nxt[policy.up_axis] -= policy.up_sign
    return tuple(nxt)


def _is_base_cell(cell: Cell, policy: StabilityPolicy, base: set[Cell]) -> bool:
    if policy.ground_plane is not None and cell[policy.up_axis] == policy.ground_plane:
        return True
    return cell in base


def _is_planted(cell: Cell, policy: StabilityPolicy, base: set[Cell]) -> bool:
    return _is_base_cell(_down(cell, policy), policy, base)


def _column_supported(
    cell: Cell,
    occupied: set[Cell],
    policy: StabilityPolicy,
    base: set[Cell],
) -> bool:
    cur = cell
    # Each step moves exactly one unit toward the base, so a finite
    # occupied set is left after a finite number of steps.
    while cur in occupied:
        below = _down(cur, policy)
        if _is_base_cell(below, policy, base):
            return True
        cur = below
    return False


def _face_neighbors(cell: Cell):
    coords = list(cell)
    for axis in range(len(coords)):
        for delta in (-1, 1):
            coords[axis] += delta
            yield tuple(coords)
            coords[axis] -= delta


def _joint_axis(a: Cell, b: Cell) -> int:
    return next(axis for axis in range(len(a)) if a[axis] != b[axis])


def _overhang_distances(
    occupied: set[Cell],
    sources: list[Cell],
    policy: StabilityPolicy,
) -> dict[Cell, int]:
    """0-1 BFS: vertical steps cost 0, horizontal steps cost 1."""
    dist: dict[Cell, int] = {}
    queue: deque[Cell] = deque()
    for source in sources:
        if source not in dist:
            dist[source] = 0
            queue.append(source)
    while queue:
        cur = queue.popleft()
        dcur = dist[cur]
        for nxt in _face_neighbors(cur):
            if nxt not in occupied:
                continue
            # Same height: the step sticks out. A change in the up
            # coordinate stacks or hangs, and does not lengthen the span.
            horizontal = nxt[policy.up_axis] == cur[policy.up_axis]
            step = 1 if horizontal else 0
            nd = dcur + step
            if nd < dist.get(nxt, nd + 1):
                dist[nxt] = nd
                if step == 0:
                    queue.appendleft(nxt)
                else:
                    queue.append(nxt)
    return dist


def _reachable(seeds: Iterable[Cell], occupied: set[Cell]) -> set[Cell]:
    seen: set[Cell] = set()
    queue: deque[Cell] = deque()
    for seed in seeds:
        if seed in occupied and seed not in seen:
            seen.add(seed)
            queue.append(seed)
    while queue:
        cur = queue.popleft()
        for nxt in _face_neighbors(cur):
            if nxt in occupied and nxt not in seen:
                seen.add(nxt)
                queue.append(nxt)
    return seen


def _stiffnesses(
    k: float, k_axial: float | None, k_shear: float | None
) -> tuple[float, float]:
    if k <= 0:
        raise ValueError("k must be positive")
    axial = float(k if k_axial is None else k_axial)
    shear = float(k if k_shear is None else k_shear)
    if axial <= 0 or shear <= 0:
        raise ValueError("k_axial and k_shear must be positive")
    return axial, shear
