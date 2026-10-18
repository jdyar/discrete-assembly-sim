# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Opt-in on-lattice inchworm locomotion.

The default geometry (:class:`sim.geometry3d.CubicLattice3D`) keeps a
robot in an empty cell next to structure and reaches out from there.
This module is the other regime: both feet grip faces of voxels that
are already placed, one foot stays attached while the other takes a
step, and a carried voxel is placed into an empty cell that shares a
face with the structure and is within reach of a foot.

It is off unless a caller builds :class:`OnLatticeGeometry`. The
coordination stack sees ordinary geometry nodes (a stance is a
hashable). Crane-mode plans do not enter this file.

What is taken from the hardware papers, and what is configuration
--------------------------------------------------------------------
BILL-E (Jenett and Cheung, AIAA 2017-1876, doi:10.2514/6.2017-1876)
is a bipedal inchworm. Two feet, each gripping one voxel. The
functional requirements include a straight traverse, an orthogonal
turn, concave and convex corners, and a step up or down a level.
Figure 12 shows the convex and concave corner sequence; Figure 16
shows the alternating "front foot, then back foot" step. Table I
lists the prototype mass as 520 g. The lattice pitch is 76.2 mm.
That paper does not give a voxel mass, so this module does not state
a BILL-E robot-to-voxel mass ratio. BILL-E walks the vertex-connected
octahedral lattice of Cheung and Gershenfeld 2013, which is not the
face-joined cubic grid this geometry steps on. The corner rules below
are the cubic-face abstraction of those maneuvers, not a replay of
the joint trajectory.

SOLL-E (Park et al., IROS 2023, NTRS 20230011353; the transport robot
in Gregg et al., Science Robotics 2024, doi:10.1126/scirobotics.adi2746)
is a 5-DoF bipedal inchworm with two foot grippers and, on the cargo
robot, a third cargo gripper. Gregg et al. describe the gait as one
foot gripped while the other reaches a nearby face on the same level,
one level up, or one level down, then the grippers alternate. The
crane role uses one foot gripper as the end effector and places one
voxel. Park et al. Table I: cargoSOLL-E 4.8 kg, craneSOLL-E 4.4 kg;
voxel pitch 0.3048 m; voxel mass approximately 380 g. Those are the
only published masses used when this repository states a hardware
ratio (about 12.6 cargo, about 11.6 crane, with the voxel mass
explicitly approximate). Link lengths L1=0.341 m, L2=0.152 m,
L3=0.116 m, L4=0.031 m are published and are **not** reduced here to
a foot separation in voxel units.

Configured, not measured:

* ``stride`` (default 1) is the number of surface-face edges the
  swing foot may cross in one tick. One edge is the discrete reading
  of "one unit step" / "a nearby face". A level change that transits
  a side face is two edges, so with the default stride it is a legal
  step only when that side face is itself a legal place to stop, or
  when ``stride`` is raised.
* ``span`` (default 2) is the maximum face-center Chebyshev distance
  between the two feet, in voxel pitches. 2 is what lets the body
  open by one voxel on the front-foot step and close on the back-foot
  step. It is not a measured SOLL-E stance.
* ``place_reach`` (default 1) is the Chebyshev radius from a foot's
  voxel to an empty cell that shares a face with the structure.
  SOLL-E's placement map (Park et al. Fig. 6) was not transcribed
  into a cell set.
* Which faces are walkable (``allow_top`` default True, ``allow_sides``
  and ``allow_inverted`` default False) and which corner classes are
  legal transitions (flat / convex / concave, convex and concave
  default True because both papers demonstrate them).
* The equal split of robot weight across the foot voxels
  (:func:`sim.stability.split_foot_loads`).

A tick-boundary pose has both feet attached. The swing happens inside
the step: exactly one foot moves, the other stays. There is no graph
node with a foot in the air, matching the discrete commands the
robots actually execute ("step forward", then both grippers closed).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterator

from .geometry import Geometry, Node
from .stability import robot_stance_ok
from .world import DEFECT, EMPTY, GROUND, SCAFFOLD, VOXEL
from .world3d import World3D

# Viewer offsets, in voxel pitches. The replay viewer duplicates these
# numbers; tests check that a foot center drawn with them sits outside
# the 0.96-cell solid (half-extent 0.48), not inside it and not floating
# a voxel away.
VIEW_FOOT_OUTSET = 0.56
VIEW_FOOT_HALF_THICK = 0.05
VIEW_BODY_LIFT = 0.20
VIEW_VOXEL_HALF = 0.48

_STANCE_TAG = "iw"
_FACE_TAG = "face"

_AXES = ((1, 0, 0), (0, 1, 0), (0, 0, 1))


def _unit_normals():
    for axis in _AXES:
        yield axis
        yield tuple(-c for c in axis)


def face_center(face: tuple) -> tuple[float, float, float]:
    """Continuous center of a face, half a pitch out from the voxel center."""
    cell, normal = face
    return tuple(c + 0.5 * n for c, n in zip(cell, normal))


def center_chebyshev(a: tuple, b: tuple) -> float:
    ca, cb = face_center(a), face_center(b)
    return max(abs(x - y) for x, y in zip(ca, cb))


def face_key(face: tuple) -> tuple:
    """Reservation key for one gripped face. Distinct from a cell node."""
    cell, normal = face
    return (_FACE_TAG, cell, normal)


def is_stance(node) -> bool:
    return (
        isinstance(node, tuple)
        and len(node) == 3
        and node[0] == _STANCE_TAG
        and isinstance(node[1], tuple)
        and isinstance(node[2], tuple)
    )


def stance_faces(node) -> tuple[tuple, tuple] | None:
    if not is_stance(node):
        return None
    return node[1], node[2]


def make_stance(face_a: tuple, face_b: tuple) -> tuple:
    """Canonical stance node. Order of the feet does not change the pose."""
    if face_b < face_a:
        face_a, face_b = face_b, face_a
    return (_STANCE_TAG, face_a, face_b)


@dataclass(frozen=True)
class InchwormConfig:
    """Gait and surface policy. See the module docstring for provenance.

    Defaults walk top faces only, one surface edge per step, body
    allowed to open to two pitches. Side faces, undersides, and a
    longer step are switches, not implied hardware.
    """

    stride: int = 1
    span: float = 2.0
    allow_top: bool = True
    allow_sides: bool = False
    allow_inverted: bool = False
    allow_flat: bool = True
    allow_convex: bool = True
    allow_concave: bool = True
    place_reach: int = 1
    up_axis: int = 0
    up_sign: int = 1
    voxel_mass: float = 1.0
    g: float = 1.0
    k: float = 1.0
    k_axial: float | None = None
    k_shear: float | None = None
    # When a sag cap is attached, every stance is checked as if the
    # gripper holds ``carried_mass``. The walk to a placement is a
    # carrying walk. The walk back uses the same graph, so a return
    # that would pass only when empty-handed can be refused.
    check_carried_mass: bool = True

    def __post_init__(self) -> None:
        if int(self.stride) < 1:
            raise ValueError("stride must be >= 1")
        if float(self.span) <= 0:
            raise ValueError("span must be positive")
        if int(self.place_reach) < 1:
            raise ValueError("place_reach must be >= 1")
        if self.up_sign not in (1, -1):
            raise ValueError("up_sign must be +1 or -1")
        if not (self.allow_top or self.allow_sides or self.allow_inverted):
            raise ValueError("at least one face class must be walkable")
        if not (self.allow_flat or self.allow_convex or self.allow_concave):
            raise ValueError("at least one transition class must be allowed")
        if self.voxel_mass < 0 or self.g < 0 or self.k <= 0:
            raise ValueError("voxel_mass and g must be >= 0 and k > 0")
        object.__setattr__(self, "stride", int(self.stride))
        object.__setattr__(self, "place_reach", int(self.place_reach))
        object.__setattr__(self, "span", float(self.span))


class InchwormStep:
    """One legal step, for tests and for the swept-face footprint.

    ``pivot`` stays attached the whole step. ``swing_from`` releases
    and ``swing_to`` reattaches. ``path`` is the swing foot's surface
    route, including both ends. Both endpoints are both-feet-down
    poses; the foot is in the air only along ``path``.
    """

    def __init__(self, pivot, swing_from, swing_to, path: tuple) -> None:
        self.pivot = pivot
        self.swing_from = swing_from
        self.swing_to = swing_to
        self.path = path

    @property
    def attach_states(self) -> tuple[str, ...]:
        """Foot-attach labels along the step, including the two rest poses."""
        mid = ("pivot_only",) * (len(self.path) - 1)
        return ("both",) + mid + ("both",)


class OnLatticeGeometry(Geometry):
    """Cubic grid whose robot nodes are two-foot stances on voxel faces.

    Nodes the planner treats as robot positions are stance tuples from
    :func:`make_stance`. Nodes it treats as voxels stay ``(level, row,
    col)`` cells. :meth:`reach_cells` is symmetric across those two
    kinds: a cell is reachable from a stance exactly when that stance
    is listed for the cell.
    """

    voxel_shape = "cubic"

    def __init__(self, world: World3D, config: InchwormConfig | None = None, policy=None) -> None:
        self.world = world
        self.config = config if config is not None else InchwormConfig()
        self.policy = policy
        self._stance_cache: tuple | None = None
        self._ok_cache: dict = {}
        # One plan mutates occupancy and then asks for the same stance's
        # neighbors again (a return-to-depot search per approach). The
        # lists depend only on occupancy and the node. Reservation checks
        # are not in this object.
        self._cells_cache: tuple | None = None
        self._neighbor_cache: tuple = (None, {})
        self._footing_cache: tuple = (None, {})
        # Face-center distance does not depend on occupancy.
        self._heur_cache: dict = {}

    # -- face geometry -------------------------------------------------------

    def _in_bounds(self, cell) -> bool:
        level, row, col = cell
        return (
            0 <= level < self.world.levels
            and 0 <= row < self.world.rows
            and 0 <= col < self.world.cols
        )

    def _blocks(self, cell) -> bool:
        """True when ``cell`` is material, so a face opening onto it is internal."""
        if not self._in_bounds(cell):
            return False
        return self.world.occupancy[cell] in (GROUND, VOXEL, DEFECT, SCAFFOLD)

    def _is_voxel(self, cell) -> bool:
        return self._in_bounds(cell) and self.world.occupancy[cell] in (
            VOXEL,
            DEFECT,
            SCAFFOLD,
        )

    def _exposed(self, cell, normal) -> bool:
        if not self._is_voxel(cell):
            return False
        outside = tuple(c + n for c, n in zip(cell, normal))
        return not self._blocks(outside)

    def _normal_class(self, normal) -> str:
        axis = self.config.up_axis
        if normal[axis] == self.config.up_sign and sum(abs(c) for c in normal) == 1:
            # The up component is the whole normal.
            if all(c == 0 or i == axis for i, c in enumerate(normal)):
                return "top"
        if normal[axis] == -self.config.up_sign and all(
            c == 0 or i == axis for i, c in enumerate(normal)
        ):
            return "inverted"
        if normal[axis] == 0:
            return "side"
        return "other"

    def _resting(self, normal) -> bool:
        kind = self._normal_class(normal)
        cfg = self.config
        if kind == "top":
            return cfg.allow_top
        if kind == "side":
            return cfg.allow_sides
        if kind == "inverted":
            return cfg.allow_inverted
        return False

    def _perp_normals(self, normal):
        for other in _unit_normals():
            if sum(a * b for a, b in zip(normal, other)) == 0:
                yield other

    def surface_neighbors(self, face) -> Iterator[tuple]:
        """Exposed faces one edge away, tagged flat / convex / concave.

        The neighbor does not have to be a legal resting face. A longer
        stride may cross a face the robot is not allowed to stop on.
        The edge class still has to be enabled.
        """
        cell, normal = face
        cfg = self.config
        if cfg.allow_flat:
            for axis, component in enumerate(normal):
                if component != 0:
                    continue
                for delta in (-1, 1):
                    nxt = list(cell)
                    nxt[axis] += delta
                    other = (tuple(nxt), normal)
                    if self._exposed(*other):
                        yield other, "flat"
        if cfg.allow_convex:
            for n2 in self._perp_normals(normal):
                other = (cell, n2)
                if self._exposed(cell, n2):
                    yield other, "convex"
        if cfg.allow_concave:
            for n2 in self._perp_normals(normal):
                other_cell = tuple(c + a - b for c, a, b in zip(cell, normal, n2))
                other = (other_cell, n2)
                if self._exposed(other_cell, n2):
                    yield other, "concave"

    def _within_span(self, face_a, face_b) -> bool:
        return center_chebyshev(face_a, face_b) <= self.config.span + 1e-9

    def _valid_pair(self, face_a, face_b) -> bool:
        if face_a == face_b:
            return False
        cell_a, normal_a = face_a
        cell_b, normal_b = face_b
        if not self._exposed(cell_a, normal_a) or not self._exposed(cell_b, normal_b):
            return False
        if not self._resting(normal_a) or not self._resting(normal_b):
            return False
        return self._within_span(face_a, face_b)

    def _swing_path(self, pivot, start, goal) -> tuple | None:
        """Shortest surface path from ``start`` to ``goal`` of length 1..stride.

        The pivot face is not entered: that foot is down. ``start`` and
        ``goal`` are included. ``None`` when no such path exists.
        """
        if start == goal or start == pivot or goal == pivot:
            return None
        stride = self.config.stride
        prev = {start: start}
        frontier = [start]
        for _depth in range(stride):
            nxt_frontier = []
            for cur in frontier:
                for other, _kind in self.surface_neighbors(cur):
                    if other == pivot or other in prev:
                        continue
                    prev[other] = cur
                    if other == goal:
                        path = [goal]
                        while path[-1] != start:
                            path.append(prev[path[-1]])
                        path.reverse()
                        return tuple(path)
                    nxt_frontier.append(other)
            frontier = nxt_frontier
        return None

    def classify_step(self, a, b) -> InchwormStep | None:
        """The one-foot step ``a -> b``, or ``None`` if it is not one."""
        faces_a = stance_faces(a)
        faces_b = stance_faces(b)
        if faces_a is None or faces_b is None or a == b:
            return None
        shared = set(faces_a) & set(faces_b)
        if len(shared) != 1:
            return None
        pivot = next(iter(shared))
        swing_from = next(face for face in faces_a if face != pivot)
        swing_to = next(face for face in faces_b if face != pivot)
        path = self._swing_path(pivot, swing_from, swing_to)
        if path is None:
            return None
        if not self._valid_pair(pivot, swing_to):
            return None
        return InchwormStep(pivot, swing_from, swing_to, path)

    # -- stance enumeration --------------------------------------------------

    def _resting_faces(self) -> list:
        faces = []
        occ = self.world.occupancy
        for level in range(self.world.levels):
            for row in range(self.world.rows):
                for col in range(self.world.cols):
                    if occ[level, row, col] not in (VOXEL, DEFECT, SCAFFOLD):
                        continue
                    cell = (level, row, col)
                    for normal in _unit_normals():
                        if self._resting(normal) and self._exposed(cell, normal):
                            faces.append((cell, normal))
        return faces

    def stances(self) -> tuple:
        """Every both-feet-down pose on the current structure."""
        key = self.world.occupancy.tobytes()
        cached = self._stance_cache
        if cached is not None and cached[0] == key:
            return cached[1]
        faces = self._resting_faces()
        found = []
        for i, face_a in enumerate(faces):
            for face_b in faces[i + 1 :]:
                if self._valid_pair(face_a, face_b):
                    found.append(make_stance(face_a, face_b))
        result = tuple(found)
        self._stance_cache = (key, result)
        self._ok_cache.clear()
        return result

    def _structurally_ok(self, node) -> bool:
        if self.policy is None:
            return True
        feet = self.foot_cells(node)
        carrying = bool(self.config.check_carried_mass)
        cells = self._voxel_tuples()
        key = (cells, feet, carrying, self.policy)
        hit = self._ok_cache.get(key)
        if hit is not None:
            return hit
        params = self.sag_parameters()
        ok, _reason = robot_stance_ok(
            cells,
            feet,
            self.policy,
            carrying=carrying,
            **params,
        )
        if len(self._ok_cache) > 4096:
            self._ok_cache.clear()
        self._ok_cache[key] = ok
        return ok

    def _voxel_tuples(self) -> tuple:
        key = self.world.occupancy.tobytes()
        cached = self._cells_cache
        if cached is not None and cached[0] == key:
            return cached[1]
        occ = self.world.occupancy
        found = []
        for level in range(self.world.levels):
            for row in range(self.world.rows):
                for col in range(self.world.cols):
                    if occ[level, row, col] in (VOXEL, DEFECT, SCAFFOLD):
                        found.append((level, row, col))
        result = tuple(found)
        self._cells_cache = (key, result)
        return result

    # -- Geometry interface --------------------------------------------------

    def is_footing(self, node) -> bool:
        key = self.world.occupancy.tobytes()
        cache = self._footing_cache
        if cache[0] != key:
            cache = (key, {})
            self._footing_cache = cache
        hit = cache[1].get(node)
        if hit is None:
            faces = stance_faces(node)
            hit = (
                faces is not None
                and self._valid_pair(*faces)
                and self._structurally_ok(node)
            )
            cache[1][node] = hit
        return hit

    def neighbors(self, node) -> Iterator[Node]:
        key = self.world.occupancy.tobytes()
        cache = self._neighbor_cache
        if cache[0] != key:
            cache = (key, {})
            self._neighbor_cache = cache
        hit = cache[1].get(node)
        if hit is None:
            hit = tuple(self._neighbor_list(node))
            cache[1][node] = hit
        yield from hit

    def _neighbor_list(self, node) -> Iterator[Node]:
        faces = stance_faces(node)
        if faces is None or not self.is_footing(node):
            return
        seen = {node}
        for pivot_index in (0, 1):
            pivot = faces[pivot_index]
            swing = faces[1 - pivot_index]
            # Destinations within stride, not the pivot, ending in a legal pair.
            prev = {swing: swing}
            frontier = [swing]
            for _depth in range(self.config.stride):
                nxt_frontier = []
                for cur in frontier:
                    for other, _kind in self.surface_neighbors(cur):
                        if other == pivot or other in prev:
                            continue
                        prev[other] = cur
                        nxt_frontier.append(other)
                        if not self._valid_pair(pivot, other):
                            continue
                        stance = make_stance(pivot, other)
                        if stance in seen or not self._structurally_ok(stance):
                            continue
                        seen.add(stance)
                        yield stance
                frontier = nxt_frontier

    def move_footprint(self, a, b) -> tuple:
        step = self.classify_step(a, b)
        if step is None:
            return ()
        # Intermediate swing faces, not the two rest poses.
        return tuple(face_key(face) for face in step.path[1:-1])

    def reservation_keys(self, node) -> tuple:
        faces = stance_faces(node)
        if faces is None:
            return (node,)
        return (node, face_key(faces[0]), face_key(faces[1]))

    def foot_cells(self, node) -> tuple:
        faces = stance_faces(node)
        if faces is None:
            return ()
        return tuple(face[0] for face in faces)

    def sag_parameters(self) -> dict:
        cfg = self.config
        return {
            "k": cfg.k,
            "k_axial": cfg.k_axial,
            "k_shear": cfg.k_shear,
            "mass": cfg.voxel_mass,
            "g": cfg.g,
        }

    def _placeable(self, node) -> list:
        faces = stance_faces(node)
        if faces is None:
            return []
        blocked = set()
        foot_voxels = set()
        for cell, normal in faces:
            foot_voxels.add(cell)
            blocked.add(tuple(c + n for c, n in zip(cell, normal)))
        reach = self.config.place_reach
        found: set = set()
        for cell, _normal in faces:
            level, row, col = cell
            for dl in range(-reach, reach + 1):
                for dr in range(-reach, reach + 1):
                    for dc in range(-reach, reach + 1):
                        if max(abs(dl), abs(dr), abs(dc)) > reach:
                            continue
                        if dl == dr == dc == 0:
                            continue
                        target = (level + dl, row + dr, col + dc)
                        if target in blocked or target in foot_voxels:
                            continue
                        if not self._in_bounds(target):
                            continue
                        # Empty cells are placements. A scaffold voxel is
                        # a removal target: the same stance that could
                        # have put it there can take it back. Other
                        # solids stay out of reach so a placed blueprint
                        # cell is not a second placement.
                        occ = self.world.occupancy[target]
                        if occ != EMPTY and occ != SCAFFOLD:
                            continue
                        if not self._against_structure(target):
                            continue
                        found.add(target)
        return sorted(found)

    def _against_structure(self, cell) -> bool:
        level, row, col = cell
        for dl, dr, dc in _AXES:
            for sign in (1, -1):
                other = (level + sign * dl, row + sign * dr, col + sign * dc)
                if self._is_voxel(other):
                    return True
        return False

    def reach_cells(self, node) -> list[Node]:
        if is_stance(node):
            if not self.is_footing(node):
                return []
            return self._placeable(node)
        if (
            isinstance(node, tuple)
            and len(node) == 3
            and all(isinstance(c, int) for c in node)
        ):
            return [stance for stance in self.stances() if node in self._placeable(stance) and self.is_footing(stance)]
        return []

    def heuristic_distance(self, a, b) -> int:
        key = (a, b)
        hit = self._heur_cache.get(key)
        if hit is not None:
            return hit
        stride = self.config.stride
        if is_stance(a) and is_stance(b):
            dist = min(
                center_chebyshev(fa, fb)
                for fa in stance_faces(a)
                for fb in stance_faces(b)
            )
            result = _ceil_div(dist, stride)
        else:
            cells_a = self._anchor_cells(a)
            cells_b = self._anchor_cells(b)
            if not cells_a or not cells_b:
                result = 0
            else:
                dist = min(
                    max(abs(x - y) for x, y in zip(p, q))
                    for p in cells_a
                    for q in cells_b
                )
                result = _ceil_div(dist, stride)
        self._heur_cache[key] = result
        return result

    def _anchor_cells(self, node) -> tuple:
        if is_stance(node):
            return self.foot_cells(node)
        if (
            isinstance(node, tuple)
            and len(node) == 3
            and all(isinstance(c, int) for c in node)
        ):
            return (node,)
        return ()

    def candidate_sort_key(self, target, depot) -> tuple:
        level, row, col = target
        below = (level - 1, row, col)
        supported = self._in_bounds(below) and self.world.occupancy[below] in (
            GROUND,
            VOXEL,
            DEFECT,
            SCAFFOLD,
        )
        anchors = self._anchor_cells(depot)
        if anchors:
            horiz = min(max(abs(row - a[1]), abs(col - a[2])) for a in anchors)
        else:
            horiz = 0
        return (not supported, level, -horiz)

    def bound_factory(self):
        config, policy = self.config, self.policy
        return lambda world: OnLatticeGeometry(world, config, policy)

    def future_view(self, solid_nodes) -> "OnLatticeGeometry":
        clone = self.world.copy()
        for node in solid_nodes:
            if (
                isinstance(node, tuple)
                and len(node) == 3
                and all(isinstance(c, int) for c in node)
                and self._in_bounds(node)
                and clone.occupancy[node] == EMPTY
            ):
                clone.occupancy[node] = VOXEL
        return OnLatticeGeometry(clone, self.config, self.policy)

    def stability_policy(self, max_cantilever: int | None = 0):
        from .stability import StabilityPolicy

        extra = {}
        if self.policy is not None:
            extra = {
                "max_sag": self.policy.max_sag,
                "robot_mass": self.policy.robot_mass,
                "carried_mass": self.policy.carried_mass,
            }
        return StabilityPolicy(
            up_axis=0,
            up_sign=1,
            ground_plane=0,
            max_cantilever=max_cantilever,
            **extra,
        )


def entry_stances(geometry: OnLatticeGeometry, seed_cells) -> list:
    """Stances whose feet both lie on ``seed_cells``.

    The seed is the depot the robot enters from: voxels already in the
    world, not cells the blueprint still has to place. Order is the
    enumeration order of :meth:`OnLatticeGeometry.stances`, which is
    stable for a given occupancy.
    """
    seed = {tuple(int(x) for x in cell) for cell in seed_cells}
    found = []
    for stance in geometry.stances():
        feet = geometry.foot_cells(stance)
        if feet and all(cell in seed for cell in feet) and geometry.is_footing(stance):
            found.append(stance)
    return found


def _ceil_div(distance: float, stride: int) -> int:
    if distance <= 1e-9:
        return 0
    return int(-(-distance // stride))
