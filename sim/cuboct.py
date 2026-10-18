# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Cuboctahedral voxels on the cubic cell grid (v0.2 milestone 4).

Optional geometry. The default lattice stays :class:`sim.geometry3d.CubicLattice3D`.
Nothing in the planner, choreographer, or reservation table branches on
this shape.

What this is
------------
A cuboctahedron built the way the ARMADAS voxels are described: six
square faces, joined to the neighbor across each square, four connection
points at the corners of that square. Packed on the integer cubic grid,
adjacent voxels meet in the plane halfway between their centers. Each
voxel therefore has the same six face-neighbors as a cube.

Citations (checked against the papers, not copied from memory)
--------------------------------------------------------------
* Kenneth C. Cheung and Neil Gershenfeld, "Reversibly Assembled
  Cellular Composite Materials," *Science* 341, 1219–1221 (2013),
  doi:10.1126/science.1240889. Their "cuboct" lattice is a cubic lattice
  of **vertex-connected octahedra** (coordination number 8), assembled
  from planar cross-shaped parts. It is the digital-materials ancestor.
  It is not the solid implemented here.
* Benjamin Jenett and Kenneth Cheung, "BILL-E: Robotic Platform for
  Locomotion and Manipulation of Lightweight Space Structures," AIAA
  2017-1876 (2017), doi:10.2514/6.2017-1876, NTRS 20170006219. Two
  authors. BILL-E walks on that octahedral Cuboct lattice (pitch
  P = 76.2 mm, strut length P√2/2). The locomotion *rules* in this
  repository were taken from the 2D surface-locomotion decision and
  generalized per axis-plane; this module does not re-derive BILL-E's
  gait.
* Christine E. Gregg et al., "Ultralight, strong, and self-reprogrammable
  mechanical metamaterials," *Science Robotics* 9, eadi2746 (2024),
  doi:10.1126/scirobotics.adi2746, published 17 January 2024, NTRS
  20230005194. The year 2018 sometimes attached to this title is wrong.
  ARMADAS voxels are cuboctahedra preassembled from six square faces
  (corner-to-corner 304.8 mm, so the bounding cube is 304.8 mm on a
  side) and joined **face to face** by four genderless fasteners at the
  corners of each square. That is the shape below. The published
  density, strength, and stiffness of the 256-voxel build are not
  inputs to this model.

Same as :class:`~sim.geometry3d.CubicLattice3D`
-----------------------------------------------
* Nodes ``(level, row, col)`` on :class:`sim.world3d.World3D`. Pitch is
  one cell. Centers sit on the integer coordinates.
* Six face-neighbors, along the three axes. Corner and edge contact
  still carries no load in :mod:`sim.stability`.
* Footing, orthogonal steps, corner rounding, snake-arm reach, stride,
  climb, and coupled motion. :class:`~sim.geometry3d.MotionModel` is the
  same object.
* :meth:`stability_policy`: up is ``+level``, the base plane is level 0.
* :func:`sim.stability.is_stable` and :func:`sim.stability.estimate_sag`.
  The spring graph is one lumped axial/shear spring per shared face, so
  a one-voxel-wide bar is still the path chain. The closed forms
  (:func:`~sim.stability.shear_cantilever_tip_sag`,
  :func:`~sim.stability.axial_tower_compression`,
  :func:`~sim.stability.path_chain_deflection_at`) apply to the same
  occupied sets as on the cube.

Different
---------
* The solid has 12 vertices, all permutations of ``(±pitch/2, ±pitch/2, 0)``
  about the center: the midpoints of the bounding cube's edges. A cube
  has 8 vertices at ``(±pitch/2, ±pitch/2, ±pitch/2)``. Those cube
  corners are outside the cuboctahedron (they lie past the triangular
  faces).
* Each connection is a square face, rotated 45° in the face plane
  relative to the bounding cube's face. Side length ``pitch/√2``,
  diagonal ``pitch`` (the published 304.8 mm is that diagonal, and the
  bounding-cube edge, at hardware scale). The four joint sites are the
  square's corners. A cube's joint sites are the four corners of the
  full face, which are not cuboctahedron vertices.
* The face *center* — the midpoint between the two voxel centers — is
  the same point for both shapes. That is where the lumped spring is
  anchored.

Sag assumptions (so the closed forms are not an accident)
---------------------------------------------------------
:func:`sim.stability.estimate_sag` does not grow degrees of freedom for
this shape. Each shared square face is one spring between the two voxel
centers:

* ``k_axial`` along the center-to-center axis;
* ``k_shear`` on each of the two axes in the face plane.

The four fastener sites are locations, not nodes. Splitting the spring
into four fasteners would let one face transmit a moment, and the
one-line path-chain formulas would stop applying. This module does not
do that. There is no plate stiffness for the six injection-molded
faces, no bolt preload, no robot weight, and no fit to the Science
Robotics modulus. A printed cuboctahedron whose compliance is bending
of the face plates will not follow ``n(n+1)/2``.

:meth:`CuboctahedralLattice3D.estimate_sag` is that solver, called with
this lattice's frame. :func:`face_joints` reports the sites and the
center the spring is lumped at.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

from .geometry3d import CubicLattice3D
from .world import EMPTY, VOXEL
from .world3d import World3D

VOXEL_SHAPE_CUBIC = "cubic"
VOXEL_SHAPE_CUBOCTAHEDRAL = "cuboctahedral"

# Viewer inset, matching replay_viewer.html's cubic BoxGeometry(0.96).
# The analytical joints use pitch 1; the mesh is shrunk so neighboring
# faces do not z-fight. It is a drawing gap, not a joint clearance.
VIEWER_INSET = 0.96


@dataclass(frozen=True)
class FaceJoint:
    """One shared face between two face-adjacent voxels.

    ``center`` is the midpoint between voxel centers, and the anchor of
    the single lumped spring :func:`sim.stability.estimate_sag` places
    on this face. ``sites`` are the four in-plane joint locations
    (fastener corners for the cuboctahedron; bounding-face corners for
    the cube). ``sites`` are not extra degrees of freedom.
    """

    a: tuple[int, ...]
    b: tuple[int, ...]
    axis: int
    center: tuple[float, float, float]
    sites: tuple[tuple[float, float, float], ...]
    shape: str

    @property
    def spring(self) -> str:
        return (
            "one lumped axial/shear spring at center; "
            "sites are joint locations, not extra degrees of freedom"
        )


def _pitch(pitch: float) -> float:
    if pitch <= 0:
        raise ValueError("pitch must be positive")
    return float(pitch)


def _shape(shape: str) -> str:
    if shape not in (VOXEL_SHAPE_CUBIC, VOXEL_SHAPE_CUBOCTAHEDRAL):
        raise ValueError(
            f"shape must be {VOXEL_SHAPE_CUBIC!r} or "
            f"{VOXEL_SHAPE_CUBOCTAHEDRAL!r} (got {shape!r})"
        )
    return shape


def cuboct_offsets(pitch: float = 1.0) -> tuple[tuple[float, float, float], ...]:
    """12 vertices of a cuboctahedron centered at the origin.

    Coordinate order is ``(axis0, axis1, axis2)``, which on
    :class:`World3D` is ``(level, row, col)``. Sorted lexicographically.
    """
    h = _pitch(pitch) / 2.0
    raw: list[tuple[float, float, float]] = []
    for zero in range(3):
        others = [i for i in range(3) if i != zero]
        for s0 in (-h, h):
            for s1 in (-h, h):
                point = [0.0, 0.0, 0.0]
                point[others[0]] = s0
                point[others[1]] = s1
                raw.append((point[0], point[1], point[2]))
    return tuple(sorted(raw))


def cubic_offsets(pitch: float = 1.0) -> tuple[tuple[float, float, float], ...]:
    """8 corners of the bounding cube, centered at the origin."""
    h = _pitch(pitch) / 2.0
    raw = [
        (x, y, z)
        for x in (-h, h)
        for y in (-h, h)
        for z in (-h, h)
    ]
    return tuple(sorted(raw))


def voxel_vertices(
    center: Sequence[int] | Sequence[float],
    shape: str = VOXEL_SHAPE_CUBOCTAHEDRAL,
    pitch: float = 1.0,
) -> tuple[tuple[float, float, float], ...]:
    """Vertices of one voxel whose center is ``center`` (cell coordinates)."""
    shape = _shape(shape)
    pitch = _pitch(pitch)
    if len(center) != 3:
        raise ValueError("cuboct and cubic voxel vertices are 3D")
    local = cuboct_offsets(pitch) if shape == VOXEL_SHAPE_CUBOCTAHEDRAL else cubic_offsets(pitch)
    base = tuple(float(c) * pitch for c in center)
    return tuple(
        (base[0] + p[0], base[1] + p[1], base[2] + p[2]) for p in local
    )


def _neighbor_axis(a: Sequence[int], b: Sequence[int]) -> int:
    if len(a) != 3 or len(b) != 3:
        raise ValueError("face joints are defined for 3D cells")
    diffs = [i for i in range(3) if int(a[i]) != int(b[i])]
    if len(diffs) != 1 or abs(int(a[diffs[0]]) - int(b[diffs[0]])) != 1:
        raise ValueError(
            f"{tuple(a)} and {tuple(b)} are not 6-neighbors; "
            "a cuboct or cubic face joint needs a shared square face"
        )
    return diffs[0]


def face_center(
    a: Sequence[int], b: Sequence[int], pitch: float = 1.0
) -> tuple[float, float, float]:
    """Midpoint between two face-adjacent voxel centers."""
    pitch = _pitch(pitch)
    _neighbor_axis(a, b)
    return (
        (float(a[0]) + float(b[0])) * 0.5 * pitch,
        (float(a[1]) + float(b[1])) * 0.5 * pitch,
        (float(a[2]) + float(b[2])) * 0.5 * pitch,
    )


def square_face_sites(
    a: Sequence[int],
    b: Sequence[int],
    shape: str = VOXEL_SHAPE_CUBOCTAHEDRAL,
    pitch: float = 1.0,
) -> tuple[tuple[float, float, float], ...]:
    """Four joint locations on the shared square face, sorted.

    Cuboctahedron: corners of the 45°-rotated square (a subset of the
    voxel's vertices). Cube: corners of the axis-aligned bounding face.
    Both sets lie in the shared plane; only the in-plane positions differ.
    """
    shape = _shape(shape)
    pitch = _pitch(pitch)
    axis = _neighbor_axis(a, b)
    center = face_center(a, b, pitch)
    h = pitch / 2.0
    transverse = [i for i in range(3) if i != axis]
    sites: list[tuple[float, float, float]] = []
    if shape == VOXEL_SHAPE_CUBIC:
        for s0 in (-h, h):
            for s1 in (-h, h):
                point = list(center)
                point[transverse[0]] += s0
                point[transverse[1]] += s1
                sites.append((point[0], point[1], point[2]))
    else:
        for t in transverse:
            for sign in (-h, h):
                point = list(center)
                point[t] += sign
                sites.append((point[0], point[1], point[2]))
    return tuple(sorted(sites))


def face_joint(
    a: Sequence[int],
    b: Sequence[int],
    shape: str = VOXEL_SHAPE_CUBOCTAHEDRAL,
    pitch: float = 1.0,
) -> FaceJoint:
    """The shared face of two 6-neighbors, sites ordered with ``a < b``."""
    shape = _shape(shape)
    aa = tuple(int(x) for x in a)
    bb = tuple(int(x) for x in b)
    if bb < aa:
        aa, bb = bb, aa
    return FaceJoint(
        a=aa,
        b=bb,
        axis=_neighbor_axis(aa, bb),
        center=face_center(aa, bb, pitch),
        sites=square_face_sites(aa, bb, shape, pitch),
        shape=shape,
    )


def face_joints(
    cells: Iterable[Sequence[int]],
    shape: str = VOXEL_SHAPE_CUBOCTAHEDRAL,
    pitch: float = 1.0,
) -> tuple[FaceJoint, ...]:
    """One :class:`FaceJoint` per face-adjacent pair, ``a`` before ``b``.

    These are the springs :func:`sim.stability.estimate_sag` assembles:
    same pairs, same axes. The shape changes ``sites`` only.
    """
    shape = _shape(shape)
    present = {tuple(int(x) for x in cell) for cell in cells}
    out: list[FaceJoint] = []
    for a in sorted(present):
        for axis in range(3):
            step = [0, 0, 0]
            step[axis] = 1
            b = (a[0] + step[0], a[1] + step[1], a[2] + step[2])
            if b in present:
                out.append(face_joint(a, b, shape, pitch))
    return tuple(out)


def _cross(u: Sequence[float], v: Sequence[float]) -> tuple[float, float, float]:
    return (
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
    )


def _sub(p: Sequence[float], q: Sequence[float]) -> tuple[float, float, float]:
    return (p[0] - q[0], p[1] - q[1], p[2] - q[2])


def _dot(u: Sequence[float], v: Sequence[float]) -> float:
    return u[0] * v[0] + u[1] * v[1] + u[2] * v[2]


def _wind_outward(
    indices: list[int],
    offsets: Sequence[Sequence[float]],
    outward: Sequence[float],
) -> tuple[int, ...]:
    """Cycle ``indices`` so the polygon normal points along ``outward``."""
    if len(indices) < 3:
        raise ValueError("a face needs at least 3 vertices")
    normal = _cross(
        _sub(offsets[indices[1]], offsets[indices[0]]),
        _sub(offsets[indices[2]], offsets[indices[0]]),
    )
    if _dot(normal, outward) < 0:
        indices = list(reversed(indices))
    return tuple(indices)


def cuboct_faces(
    pitch: float = 1.0,
) -> tuple[tuple[tuple[int, ...], tuple[float, float, float]], ...]:
    """14 faces as ``(vertex indices, outward direction)``.

    Indices refer to :func:`cuboct_offsets`. Eight triangles, six squares.
    """
    offsets = cuboct_offsets(pitch)
    index = {point: i for i, point in enumerate(offsets)}
    h = pitch / 2.0
    faces: list[tuple[tuple[int, ...], tuple[float, float, float]]] = []

    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                wanted = (
                    (sx * h, sy * h, 0.0),
                    (sx * h, 0.0, sz * h),
                    (0.0, sy * h, sz * h),
                )
                indices = [index[p] for p in wanted]
                outward = (sx, sy, sz)
                faces.append((_wind_outward(indices, offsets, outward), outward))

    for axis, sign in ((0, -1.0), (0, 1.0), (1, -1.0), (1, 1.0), (2, -1.0), (2, 1.0)):
        on_face = [
            i
            for i, point in enumerate(offsets)
            if abs(point[axis] - sign * h) < 1e-9
        ]
        if len(on_face) != 4:
            raise RuntimeError(f"square face on axis {axis} has {len(on_face)} vertices")
        transverse = [i for i in range(3) if i != axis]
        on_face.sort(
            key=lambda i: (
                # Angle about the face center in the transverse plane.
                _atan2(offsets[i][transverse[1]], offsets[i][transverse[0]])
            )
        )
        outward = tuple(sign if i == axis else 0.0 for i in range(3))
        faces.append((_wind_outward(on_face, offsets, outward), outward))
    return tuple(faces)


def _atan2(y: float, x: float) -> float:
    import math

    return math.atan2(y, x)


def cuboct_triangles(pitch: float = 1.0) -> tuple[tuple[int, int, int], ...]:
    """Triangulated cuboctahedron, outward winding, indices into :func:`cuboct_offsets`.

    Squares become two triangles ``(v0, v1, v2)`` and ``(v0, v2, v3)``
    along the angle-sorted boundary.
    """
    tris: list[tuple[int, int, int]] = []
    for indices, _outward in cuboct_faces(pitch):
        if len(indices) == 3:
            tris.append((indices[0], indices[1], indices[2]))
        elif len(indices) == 4:
            v0, v1, v2, v3 = indices
            tris.append((v0, v1, v2))
            tris.append((v0, v2, v3))
        else:
            raise RuntimeError(f"unexpected face size {len(indices)}")
    return tuple(tris)


def viewer_offsets(pitch: float = 1.0) -> tuple[tuple[float, float, float], ...]:
    """Cuboct vertices in replay-viewer order ``(x, y, z) = (col, level, row)``.

    Same vertex order as :func:`cuboct_offsets`, so :func:`cuboct_triangles`
    indexes either list. ``pitch`` 1 is the analytical solid; the viewer
    multiplies by :data:`VIEWER_INSET`.
    """
    return tuple((point[2], point[0], point[1]) for point in cuboct_offsets(pitch))


def annotate_run_log(log, shape: str) -> None:
    """Record ``meta.voxel_shape`` for the replay viewer.

    ``"cubic"`` is omitted. Logs written before this field, and every
    default cubic run, stay free of the key. The run-log version does
    not change.
    """
    shape = _shape(shape)
    if shape == VOXEL_SHAPE_CUBIC:
        log.meta.pop("voxel_shape", None)
        return
    log.meta["voxel_shape"] = shape


def example_world() -> World3D:
    """2×2×2 block on the ground: the cuboct demo blueprint.

    Eight voxels. Depot and start used by the example and by
    ``python main.py cuboct`` sit on the ground course beside it:
    depot ``(1, 0, 0)``, start ``(1, 5, 0)``, world ``World3D(4, 6, 6)``.
    """
    world = World3D(4, 6, 6)
    world.set_box_blueprint(width=2, depth=2, height=2, left=2, front=2)
    return world


EXAMPLE_DEPOT = (1, 0, 0)
EXAMPLE_START = (1, 5, 0)


class CuboctahedralLattice3D(CubicLattice3D):
    """Cuboctahedral voxels, cubic cell graph, cubic locomotion.

    Subclasses :class:`CubicLattice3D` so footing, steps, reach, stride,
    and climb cannot drift from the cubic rules. Overrides are only the
    ones that would otherwise rebuild a cubic geometry
    (:meth:`bound_factory`, :meth:`future_view`) plus the joint-location
    and sag helpers. See the module docstring for what stays the same.
    """

    voxel_shape = VOXEL_SHAPE_CUBOCTAHEDRAL

    def bound_factory(self):
        motion = self.motion
        return lambda world: CuboctahedralLattice3D(world, motion)

    def future_view(self, solid_nodes) -> "CuboctahedralLattice3D":
        clone = self.world.copy()
        for node in solid_nodes:
            if clone.occupancy[node] == EMPTY:
                clone.occupancy[node] = VOXEL
        return CuboctahedralLattice3D(clone, self.motion)

    def stability_policy(self, max_cantilever: int | None = 0):
        """Same frame as the cubic lattice: ``+level`` up, level 0 is the plate.

        The span limit is the caller's. The cell graph the screen walks
        is the 6-neighbor face graph, which this shape shares.
        """
        return super().stability_policy(max_cantilever)

    def face_joints(self, cells: Iterable[Sequence[int]] | None = None, pitch: float = 1.0):
        """Shared-face joints of ``cells``, or of the voxels in the world.

        Each entry is one lumped spring site. Ground is not a voxel, so
        a voxel standing on the ground plane has no face joint to the
        plate here; fastening to the base is the sag solver's boundary
        condition, not a row of fasteners in this list.
        """
        if cells is None:
            from .stability import occupied_voxels

            cells = occupied_voxels(self.world)
        return face_joints(cells, self.voxel_shape, pitch)

    def estimate_sag(self, occupied_cells: Iterable[Sequence[int]] | None = None, **kwargs):
        """:func:`sim.stability.estimate_sag` on this lattice's frame.

        Springs are the shared square faces: one lumped axial/shear
        spring between voxel centers, anchored at the face center
        (:class:`FaceJoint`). The four fastener sites are not separate
        springs. Keyword arguments are the sag solver's (``k``,
        ``k_axial``, ``k_shear``, ``mass``, ``g``, and frame overrides
        such as ``base``). Pass ``policy`` to replace the stock frame;
        otherwise ``max_cantilever`` is forwarded to
        :meth:`stability_policy` and does not enter the solve.
        """
        from .stability import estimate_sag as solve
        from .stability import occupied_voxels

        cells = occupied_voxels(self.world) if occupied_cells is None else occupied_cells
        policy = kwargs.pop("policy", None)
        if policy is None:
            policy = self.stability_policy(kwargs.pop("max_cantilever", 0))
        return solve(cells, policy, **kwargs)
