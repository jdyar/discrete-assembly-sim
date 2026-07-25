# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Robot state machine: a relative robot that crawls on what it builds.

Movement regime (decided 2026-07-05, machine-verified; see NOTES.md):
**BILL-E surface locomotion** — the robot is an inchworm-style relative
robot (MIT BILL-E / MILAbot lineage) that grips the structure it builds:

- **Grip rule** (`is_grip`): the robot may occupy any empty in-bounds cell
  that is 4-adjacent to at least one GROUND or VOXEL cell — tops, vertical
  faces, and undersides alike.
- **Moves** (`legal_moves`): one cell per tick to an orthogonally adjacent
  grip cell (crawling along a surface), or diagonally when rounding a
  corner — allowed iff exactly one of the two cells between start and
  target is solid (a corner to pivot around, not a gap to teleport across
  or a pinch to squeeze through).
- **Reach** (`reach_cells`): places a carried voxel into any of the 8 cells
  surrounding its grip cell.

The robot senses only adjacent cells plus its current task — no global
knowledge except the blueprint and depot. One action per tick: a single
move, a pick, or a place.

State machine (one action per tick)::

    IDLE ──task──> TO_DEPOT ──at depot──> PICK ──picked──> TO_SITE
      ^                                                       │
      │                                        at approach ── ┘
      │                                                       v
      └── queue empty ── INSPECT(ok) <────────────────────  PLACE
             ^                │(defective)
             │                v
       replan loaded ────  REMOVE ── discard, halt, needs_replan

Slice 2 error correction (decisions 2026-07-05, see NOTES.md): each
placement is defective with probability ``defect_p`` (bad bond, in place,
crawlable). INSPECT costs 1 tick from the placement stance — modeled on
ARMADAS's vision-free fastening feedback. REMOVE costs 1 tick, discards
the part, then the robot halts with ``needs_replan`` set; the executor
calls the planner on the current world and hands back a fresh queue via
:meth:`Robot.load_plan` (the robot never mutates a plan). With
``inspect_enabled=False`` the robot places blind — the no-error-correction
baseline for the yield chart.

Logging contract (see sim/metrics.py): instances expose ``pos`` (row, col),
``state`` (str), and ``carrying`` (bool) at every tick boundary.
"""

from __future__ import annotations

import random
from collections import deque
from typing import Iterator

from . import geometry as geometry_mod
from .geometry import Geometry, SquareLattice2D
from .world import World

Cell = tuple[int, int]

# States
IDLE = "IDLE"
TO_DEPOT = "TO_DEPOT"
PICK = "PICK"
TO_SITE = "TO_SITE"
PLACE = "PLACE"
INSPECT = "INSPECT"
REMOVE = "REMOVE"

# Geometry compatibility wrappers. The rules themselves live in
# sim.geometry.SquareLattice2D (the pluggable lattice seam, Slice 3a);
# these keep the historical (world, cell) call signature for existing
# callers and tests. New code should take a Geometry instead.


def is_grip(world: World, cell: Cell) -> bool:
    """True if the robot may occupy ``cell``: empty, gripping a solid neighbor."""
    return SquareLattice2D(world).is_footing(cell)


def legal_moves(world: World, cell: Cell) -> Iterator[Cell]:
    """Surface-crawl edges: orthogonal along a surface, diagonal at corners."""
    yield from SquareLattice2D(world).neighbors(cell)


def reach_cells(cell: Cell) -> list[Cell]:
    """The 8 cells around ``cell`` — where a gripping robot can place."""
    r, c = cell
    return [
        (r + dr, c + dc)
        for dr in (-1, 0, 1)
        for dc in (-1, 0, 1)
        if (dr, dc) != (0, 0)
    ]


def bfs_path(world: World, start: Cell, goals: set[Cell]) -> list[Cell] | None:
    """Shortest grip-cell path from ``start`` into ``goals`` (BFS).

    Returns the path including ``start`` and the reached goal, or ``None``
    if no goal is reachable. ``start`` must itself be a legal grip cell.
    """
    return geometry_mod.bfs_path(SquareLattice2D(world), start, goals)


class Robot:
    """One relative robot executing placement tasks a tick at a time.

    ``tasks`` is a queue of planner Tasks (``.cell`` target, ``.approach``
    stance) or bare target cells (the robot then uses any stance in reach —
    handy for tests). For each task the robot crawls to the depot, picks a
    voxel, crawls to the approach stance, and places. It replans its path
    (BFS) whenever it has no valid one — cheap at this scale and robust to
    the world changing under it as it builds. Per the plan/execute/replan
    contract it never mutates the plan; if a task is impossible it idles
    (Slice 2 turns this into an explicit replan request).
    """

    def __init__(
        self,
        pos: Cell,
        depot: Cell,
        tasks: list | None = None,
        defect_p: float = 0.0,
        rng: random.Random | None = None,
        inspect_enabled: bool = True,
        geometry: Geometry | None = None,
    ):
        # Lattice rules are injected; None = square lattice built per tick
        # from the world handed to tick() (keeps the historical call shape).
        self._geometry = geometry
        self.pos = pos
        self.depot = depot  # grip cell where voxels are picked
        self.state = IDLE
        self.carrying = False
        self.tasks: deque = deque(tasks or [])
        self._path: list[Cell] = []  # remaining cells to crawl, excludes pos
        self.idle_ticks = 0
        self.moves = 0
        # Slice 2 — error correction
        self.defect_p = defect_p  # each placement defective with this prob.
        self.rng = rng or random.Random()
        self.inspect_enabled = inspect_enabled  # False = Slice 1 baseline
        self.needs_replan = False  # set after removing a defect; executor replans
        self.inspections = 0
        self.defects_found = 0

    def load_plan(self, tasks: list) -> None:
        """Adopt a fresh plan from the planner (initial or replan)."""
        self.tasks = deque(tasks)
        self.needs_replan = False
        self._path = []
        if self.state == IDLE and self.tasks:
            self.state = TO_DEPOT

    # -- helpers -----------------------------------------------------------

    @staticmethod
    def _task_cell(task) -> Cell:
        return task.cell if hasattr(task, "cell") else task

    @staticmethod
    def _task_approach(task) -> Cell | None:
        return task.approach if hasattr(task, "approach") else None

    def _geom(self, world: World) -> Geometry:
        return self._geometry if self._geometry is not None else SquareLattice2D(world)

    def _stances(self, world: World, task) -> set[Cell]:
        """Footing nodes this task may be placed from."""
        geom = self._geom(world)
        fixed = self._task_approach(task)
        if fixed is not None:
            return {fixed} if geom.is_footing(fixed) else set()
        target = self._task_cell(task)
        return {
            node
            for node in geom.reach_cells(target)  # reach is symmetric (Geometry contract)
            if geom.is_footing(node)
        }

    def _step_along_path(self) -> None:
        self.pos = self._path.pop(0)
        self.moves += 1

    # -- the tick ----------------------------------------------------------

    def tick(self, world: World) -> None:
        """Advance one tick: exactly one action (move, pick, place, or wait)."""
        if self.state == IDLE:
            if self.needs_replan:
                self.idle_ticks += 1  # halted, awaiting a fresh plan
            elif self.tasks:
                self.state = TO_DEPOT
                self._path = []
            else:
                self.idle_ticks += 1
            return

        if self.state == TO_DEPOT:
            if self.pos == self.depot:
                self.state = PICK
                return
            if not self._path:
                path = geometry_mod.bfs_path(self._geom(world), self.pos, {self.depot})
                if path is None:
                    self.idle_ticks += 1  # unreachable now; retry next tick
                    return
                self._path = path[1:]
            self._step_along_path()
            return

        if self.state == PICK:
            self.carrying = True  # depot has unlimited stock for now
            self.state = TO_SITE
            self._path = []
            return

        if self.state == TO_SITE:
            stances = self._stances(world, self.tasks[0])
            if self.pos in stances:
                self.state = PLACE
                return
            if not self._path or self._path[-1] not in stances:
                path = (
                    geometry_mod.bfs_path(self._geom(world), self.pos, stances)
                    if stances
                    else None
                )
                if path is None:
                    self.idle_ticks += 1
                    return
                self._path = path[1:]
            self._step_along_path()
            return

        if self.state == PLACE:
            cell = self._task_cell(self.tasks[0])
            defective = self.defect_p > 0 and self.rng.random() < self.defect_p
            world.place_voxel(cell, defective=defective)
            self.carrying = False
            self._path = []
            if self.inspect_enabled:
                self.state = INSPECT  # task stays queued until it passes
            else:
                self.tasks.popleft()  # baseline: place blind, move on
                self.state = TO_DEPOT if self.tasks else IDLE
            return

        if self.state == INSPECT:
            # 1 tick: read the bond from the placement stance (ARMADAS-style
            # feedback at fastening time — no vision).
            cell = self._task_cell(self.tasks[0])
            self.inspections += 1
            if world.is_defective(cell):
                self.defects_found += 1
                self.state = REMOVE
            else:
                self.tasks.popleft()  # placement verified good
                self.state = TO_DEPOT if self.tasks else IDLE
            return

        if self.state == REMOVE:
            # 1 tick: reverse the placement and discard the bad part, then
            # halt and request a replan (plan/execute/replan contract — the
            # robot never mutates the plan itself).
            cell = self._task_cell(self.tasks[0])
            world.remove_voxel(cell)
            self.needs_replan = True
            self.state = IDLE
            self._path = []
            return
