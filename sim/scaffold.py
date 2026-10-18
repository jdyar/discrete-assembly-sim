# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Temporary scaffold: insert a minimal set, build, then remove it.

A blueprint can pass the finished-structure caps and still have no
build order. The usual case in this repository is the on-lattice
inchworm (:class:`~sim.inchworm.OnLatticeGeometry`): both feet grip
faces of voxels that are already placed, so the robot cannot walk the
open ground to the far pier. The finished deck is inside the
cantilever limit and the sag cap, but every order that reaches that
pier cantilevers past both. Both checks reject the plain plan.
:class:`~sim.geometry3d.MotionModel` ``lattice_only`` is a separate,
default-off stand-in. It is not the gait the campaign reports.

:func:`plan_with_scaffold` then searches temporary support cells — the
open column under each blueprint cell, down toward the ground — by
increasing count. The first count that admits a placement order and a
removal order is minimal. Ties break on :func:`sequence_cost`, a
single-robot travel score: a depot round trip per task under the
stride heuristic, plus the pick/place/inspect or removal ticks. The
score still charges the return after the last task, which the swarm
does not wait for, and it ignores other robots. It is not a measured
makespan. The campaign reports the swarm tick count separately.

Scaffold is occupancy :data:`~sim.world.SCAFFOLD`. It is solid, so a
robot can stand on it, and it is not a blueprint cell. Removal is
accepted only when every intermediate, including the finished
blueprint with the scaffold gone, still passes the cantilever screen
and the sag cap. A finished blueprint that itself fails those caps
cannot be rescued: the last removal would violate them, and this
function returns ``None``.

Sag uses :func:`~sim.stability.estimate_sag`. With the default
:class:`~sim.stability.SagCap` that is units of ``mass·g/k`` at
``k = 1``. That stiffness is not a fit. An optional
:class:`~sim.stability.JointCap` applies the same rule to face-joint
forces: a prefix whose demand exceeds ``capacity × safety_factor`` is
refused, and a finished structure that already exceeds it is not
rescued. At ``k = 1`` those forces are ``mass·g``, not newtons.
"""

from __future__ import annotations

from itertools import combinations
from typing import Callable, Iterable

from .geometry import bfs_path, bfs_reachable
from .geometry3d import CubicLattice3D, MotionModel
from .planner import (
    SearchBudgetExceeded,
    Task,
    _approaches_for,
    plan_build_order,
    validate_plan,
)
from .stability import (
    JointCap,
    SagCap,
    StabilityPolicy,
    estimate_sag,
    is_stable,
    joints_within,
    occupied_voxels,
    sag_within,
)
from .world import EMPTY, SCAFFOLD
from .world3d import World3D

GeometryFactory = Callable[[World3D], CubicLattice3D]

# Ground-bound long arm. Walks the floor; places several cells away.
CRANE = MotionModel(reach_radius=4, stride=1, climb=0)
# On-lattice inchworm. Stride 2, reach 1, and the ground is not a highway.
INCHWORM = MotionModel(reach_radius=1, stride=2, climb=1, lattice_only=True)


class ScaffoldPlan(tuple):
    """Placement order, then a stable removal order.

    ``sequence_cost`` is a single-robot travel score: for each task,
    twice the stride-heuristic depot-to-stance distance, plus 3 ticks
    for a place (pick, place, inspect) or 1 tick for a removal. It is
    the secondary search key after scaffold count. The last return is
    charged even though the swarm is done when the last block is down,
    so a one-robot run can finish a few ticks under the score. It is
    not a measured multi-robot makespan.
    """

    __slots__ = ()

    def __new__(
        cls,
        scaffold: tuple,
        place: tuple[Task, ...],
        remove: tuple[Task, ...],
        sequence_cost: int,
    ):
        return tuple.__new__(cls, (tuple(scaffold), tuple(place), tuple(remove), int(sequence_cost)))

    @property
    def scaffold(self) -> tuple:
        return self[0]

    @property
    def place(self) -> tuple[Task, ...]:
        return self[1]

    @property
    def remove(self) -> tuple[Task, ...]:
        return self[2]

    @property
    def sequence_cost(self) -> int:
        return self[3]

    @property
    def scaffold_count(self) -> int:
        return len(self.scaffold)

    def operations(self) -> list[tuple[str, tuple]]:
        """Swarm script: every place, then every removal."""
        ops = [("place", task.cell) for task in self.place]
        ops.extend(("remove", task.cell) for task in self.remove)
        return ops


def geometry_factory(
    motion: MotionModel = MotionModel(), yard: Iterable = ()
) -> GeometryFactory:
    """Cubic geometry factory closed over a motion model and a yard."""
    held = frozenset(yard)

    def factory(world: World3D) -> CubicLattice3D:
        return CubicLattice3D(world, motion, held)

    return factory


def near_yard(world: World3D, depot: tuple) -> frozenset:
    """Level-1 cells on the near side of the leftmost pier, plus ``depot``.

    "Near" is every column at or left of the leftmost level-1 blueprint
    cell. An on-lattice robot can walk that yard and start the near
    pier. It cannot stroll across the span or around it on the floor.
    """
    pier_cols = [
        int(c)
        for level, _row, c in zip(*world.blueprint.nonzero())
        if int(level) == 1
    ]
    near = min(pier_cols) if pier_cols else depot[2]
    cells = []
    for row in range(world.rows):
        for col in range(near + 1):
            cell = (1, row, col)
            if world.occupancy[cell] == EMPTY:
                cells.append(cell)
    if world.occupancy[depot] == EMPTY:
        cells.append(tuple(depot))
    return frozenset(cells)


def support_candidates(world: World3D) -> tuple:
    """Open cells in the column under a blueprint cell, down to level 1.

    A cell the blueprint already claims is not a candidate: that block
    stays. The ground plane is not a candidate. Order is lexicographic
    so the subset search is deterministic.
    """
    found: set[tuple] = set()
    for raw in zip(*world.blueprint.nonzero()):
        level, row, col = (int(v) for v in raw)
        for below in range(level - 1, 0, -1):
            cell = (below, row, col)
            if world.blueprint[cell]:
                break
            if world.occupancy[cell] != EMPTY:
                break
            found.add(cell)
    return tuple(sorted(found))


def sequence_cost(place: Iterable[Task], remove: Iterable[Task], geometry, depot) -> int:
    """Single-robot depot-round-trip score, in ticks.

    Not a proved lower bound on swarm ticks: the return after the last
    task is included, and the swarm stops when the build is done.
    """
    total = 0
    for task in place:
        total += 2 * geometry.heuristic_distance(depot, task.approach) + 3
    for task in remove:
        total += 2 * geometry.heuristic_distance(depot, task.approach) + 1
    return total


def finished_cells(world: World3D) -> list[tuple]:
    """Blueprint cells plus voxels already placed, scaffold excluded.

    A planted entry pad is part of the structure that remains after the
    scaffold comes down. The caps have to accept that set, not the
    blueprint alone.
    """
    cells = [tuple(int(v) for v in raw) for raw in zip(*world.blueprint.nonzero())]
    seen = set(cells)
    for cell in occupied_voxels(world):
        if cell in seen or world.occupancy[cell] == SCAFFOLD:
            continue
        cells.append(cell)
        seen.add(cell)
    return cells


def caps_for_finished(
    world: World3D, frame: StabilityPolicy
) -> tuple[StabilityPolicy, SagCap]:
    """Tightest cantilever limit that accepts the finished structure, and its sag.

    The finished structure is the blueprint plus any voxels already
    placed (an entry pad). Scaffold is not part of it.

    ``SagCap.max_sag`` is that structure's ``worst_sag`` at
    ``k = mass = g = 1`` (units of ``mass·g/k``, stiffness not fit).
    A construction prefix that sags more than the finished bridge fails
    the cap. The finished bridge itself passes.
    """
    cells = finished_cells(world)
    limit = 0
    policy = frame._replace(max_cantilever=limit)
    while not is_stable(cells, policy):
        limit += 1
        if limit > 64:
            raise RuntimeError("finished structure is not stable at any modest cantilever")
        policy = frame._replace(max_cantilever=limit)
    worst = estimate_sag(cells, policy, k=1.0, mass=1.0, g=1.0).worst_sag
    return policy, SagCap(max_sag=float(worst), k=1.0, mass=1.0, g=1.0)


def permanent_passes(
    world: World3D,
    stability: StabilityPolicy,
    sag: SagCap,
    joints: JointCap | None = None,
) -> bool:
    """The finished structure, with no scaffold, is inside the caps."""
    cells = finished_cells(world)
    return (
        is_stable(cells, stability)
        and sag_within(cells, stability, sag)
        and joints_within(cells, stability, joints)
    )


def plan_with_scaffold(
    world: World3D,
    depot: tuple,
    geometry_factory: GeometryFactory,
    stability: StabilityPolicy,
    sag: SagCap,
    *,
    max_nodes: int = 50_000,
    max_scaffold: int | None = None,
    joints: JointCap | None = None,
) -> ScaffoldPlan | None:
    """Minimal scaffold, then minimal sequential travel cost.

    Returns ``None`` when every candidate set up to ``max_scaffold`` is
    proved not to work, including the empty set. Raises
    :class:`~sim.planner.SearchBudgetExceeded` if a search hits
    ``max_nodes`` before a smaller count has been ruled out — that is
    undecided, not a proof that scaffold cannot help.

    The world is not modified.
    """
    if not permanent_passes(world, stability, sag, joints):
        return None
    candidates = support_candidates(world)
    limit = len(candidates) if max_scaffold is None else min(max_scaffold, len(candidates))
    geometry = geometry_factory(world)

    for count in range(0, limit + 1):
        best: ScaffoldPlan | None = None
        undecided = False
        subsets = ((),) if count == 0 else combinations(candidates, count)
        for subset in subsets:
            try:
                plan = _plan_subset(
                    world,
                    depot,
                    geometry_factory,
                    geometry,
                    stability,
                    sag,
                    tuple(subset),
                    max_nodes,
                    joints,
                )
            except SearchBudgetExceeded:
                undecided = True
                continue
            if plan is None:
                continue
            if best is None or plan.sequence_cost < best.sequence_cost:
                best = plan
        if best is not None:
            return best
        if undecided:
            raise SearchBudgetExceeded(
                f"scaffold search hit {max_nodes} nodes at count {count}; "
                "minimality is undecided"
            )
    return None


def _plan_subset(
    world: World3D,
    depot: tuple,
    geometry_factory: GeometryFactory,
    geometry,
    stability: StabilityPolicy,
    sag: SagCap,
    subset: tuple,
    max_nodes: int,
    joints: JointCap | None = None,
) -> ScaffoldPlan | None:
    scratch = world.copy()
    for cell in subset:
        if scratch.blueprint[cell] or scratch.occupancy[cell] != EMPTY:
            return None
        scratch.blueprint[cell] = True
    places = plan_build_order(
        scratch,
        depot,
        geometry_factory,
        max_nodes=max_nodes,
        stability=stability,
        sag=sag,
        joints=joints,
    )
    if places is None:
        return None
    ok, _reason = validate_plan(
        scratch,
        places,
        depot,
        geometry_factory,
        stability=stability,
        sag=sag,
        joints=joints,
    )
    if not ok:
        return None
    built = world.copy()
    scaffold = set(subset)
    for task in places:
        built.place_voxel(task.cell, scaffold=task.cell in scaffold)
    removes = _best_removal(
        built, subset, depot, geometry_factory, stability, sag, joints
    )
    if removes is None:
        return None
    return ScaffoldPlan(
        subset,
        tuple(places),
        tuple(removes),
        sequence_cost(places, removes, geometry, depot),
    )


def _best_removal(
    built: World3D,
    scaffold: tuple,
    depot: tuple,
    geometry_factory: GeometryFactory,
    stability: StabilityPolicy,
    sag: SagCap,
    joints: JointCap | None = None,
) -> list[Task] | None:
    """Removal order of ``scaffold`` that stays inside the caps.

    The robot must be able to reach the stance while the cell is still
    there, and get back to the depot after it is gone. Among legal
    orders, the one with the smallest sequential travel cost wins.
    """
    if not scaffold:
        return []
    geometry = geometry_factory(built)
    best: list[Task] | None = None
    best_cost: int | None = None

    def dfs(remaining: tuple, acc: list[Task]) -> None:
        nonlocal best, best_cost
        partial = sequence_cost((), acc, geometry, depot)
        if best_cost is not None and partial >= best_cost:
            return
        if not remaining:
            if best_cost is None or partial < best_cost:
                best = list(acc)
                best_cost = partial
            return
        for index, cell in enumerate(remaining):
            reachable = set(bfs_reachable(geometry, depot))
            approaches = [
                stance
                for stance in _approaches_for(cell, geometry)
                if stance in reachable
            ]
            approaches.sort(key=lambda stance: geometry.heuristic_distance(depot, stance))
            rest = remaining[:index] + remaining[index + 1 :]
            for approach in approaches:
                built.occupancy[cell] = EMPTY
                legal = (
                    geometry.is_footing(approach)
                    and is_stable(occupied_voxels(built), stability)
                    and sag_within(occupied_voxels(built), stability, sag)
                    and joints_within(occupied_voxels(built), stability, joints)
                    and bfs_path(geometry, approach, {depot}) is not None
                )
                if legal:
                    acc.append(Task(cell, approach))
                    dfs(rest, acc)
                    acc.pop()
                built.occupancy[cell] = SCAFFOLD
                # One legal stance is enough to extend this branch; the
                # travel key already prefers the closest stance, and a
                # later stance cannot reduce that task's cost below it
                # while the recursive order is searched fully.
                if legal:
                    break

    dfs(tuple(scaffold), [])
    # Restore, in case a caller reuses the world. The search puts every
    # removed cell back before returning.
    return best


def check_assembly(
    world: World3D,
    plan: ScaffoldPlan,
    depot: tuple,
    geometry_factory: GeometryFactory,
    stability: StabilityPolicy,
    sag: SagCap,
    joints: JointCap | None = None,
) -> tuple[bool, str]:
    """Replay ``plan`` and refuse any prefix that breaks a gate.

    Places write :data:`~sim.world.SCAFFOLD` for scaffold cells, so the
    footing check is the one a robot would see. The world is copied.
    """
    scratch = world.copy()
    geometry = geometry_factory(scratch)
    scaffold = set(plan.scaffold)
    for index, (target, approach) in enumerate(plan.place):
        label = f"place {index} {target} from {approach}"
        if scratch.occupancy[target] != EMPTY:
            return False, f"{label}: cell is not empty"
        if target in scaffold:
            if scratch.blueprint[target]:
                return False, f"{label}: scaffold cell is in the blueprint"
        elif not scratch.blueprint[target]:
            return False, f"{label}: permanent cell is not in the blueprint"
        if not geometry.is_footing(approach):
            return False, f"{label}: approach is not a footing"
        if target not in geometry.reach_cells(approach):
            return False, f"{label}: target out of reach"
        if bfs_path(geometry, depot, {approach}) is None:
            return False, f"{label}: no path from the depot"
        scratch.place_voxel(target, scaffold=target in scaffold)
        if not is_stable(occupied_voxels(scratch), stability):
            return False, f"{label}: cantilever gate failed"
        if not sag_within(occupied_voxels(scratch), stability, sag):
            return False, f"{label}: sag cap failed"
        if not joints_within(occupied_voxels(scratch), stability, joints):
            return False, f"{label}: joint cap failed"
        if not geometry.is_footing(approach) or bfs_path(geometry, approach, {depot}) is None:
            return False, f"{label}: robot stranded"
    if not scratch.complete:
        return False, "places did not finish the blueprint"
    for index, (target, approach) in enumerate(plan.remove):
        label = f"remove {index} {target} from {approach}"
        if target not in scaffold:
            return False, f"{label}: not a scaffold cell"
        if scratch.occupancy[target] != SCAFFOLD:
            return False, f"{label}: scaffold is not standing"
        if not geometry.is_footing(approach):
            return False, f"{label}: approach is not a footing"
        if target not in geometry.reach_cells(approach):
            return False, f"{label}: target out of reach"
        if bfs_path(geometry, depot, {approach}) is None:
            return False, f"{label}: no path from the depot"
        scratch.remove_voxel(target)
        if not is_stable(occupied_voxels(scratch), stability):
            return False, f"{label}: cantilever gate failed after removal"
        if not sag_within(occupied_voxels(scratch), stability, sag):
            return False, f"{label}: sag cap failed after removal"
        if not joints_within(occupied_voxels(scratch), stability, joints):
            return False, f"{label}: joint cap failed after removal"
        if not geometry.is_footing(approach) or bfs_path(geometry, approach, {depot}) is None:
            return False, f"{label}: robot stranded by the removal"
    if any(scratch.occupancy[cell] == SCAFFOLD for cell in scaffold):
        return False, "scaffold remains"
    if not scratch.complete:
        return False, "blueprint incomplete after removal"
    return True, "ok"
