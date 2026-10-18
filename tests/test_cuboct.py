# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Cuboctahedral lattice: same face graph as the cube, different joints.

Planning, locomotion, the stability screen, and the sag closed forms are
pinned where the topology matches. The viewer contract is pinned by the
optional meta.voxel_shape key and by the offset/triangle tables in
replay_viewer.html, which must stay equal to sim.cuboct.
"""

from __future__ import annotations

import ast
import math
import re
import unittest
from pathlib import Path

from sim.cuboct import (
    EXAMPLE_DEPOT,
    EXAMPLE_START,
    VIEWER_INSET,
    VOXEL_SHAPE_CUBIC,
    VOXEL_SHAPE_CUBOCTAHEDRAL,
    CuboctahedralLattice3D,
    annotate_run_log,
    cubic_offsets,
    cuboct_offsets,
    cuboct_triangles,
    example_world,
    face_joint,
    face_joints,
    square_face_sites,
    viewer_offsets,
    voxel_vertices,
)
from sim.geometry import bfs_path
from sim.geometry3d import CubicLattice3D, MotionModel
from sim.metrics import RUN_LOG_VERSION_3D, RunLog, StructureLog
from sim.planner import plan_build_order, validate_plan
from sim.stability import (
    axial_tower_compression,
    estimate_sag,
    is_stable,
    shear_cantilever_tip_sag,
)
from sim.swarm import Swarm
from sim.world3d import World3D

_VIEWER = Path(__file__).resolve().parent.parent / "replay_viewer.html"


def _dist(a, b) -> float:
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def _js_array(source: str, name: str) -> list:
    match = re.search(
        rf"const {name} = (\[[\s\S]*?\]);",
        source,
    )
    if not match:
        raise AssertionError(f"{name} not found in replay_viewer.html")
    return ast.literal_eval(match.group(1))


class TestShapeAndJoints(unittest.TestCase):
    def test_vertices_differ_and_square_faces_meet(self) -> None:
        self.assertEqual(len(cuboct_offsets()), 12)
        self.assertEqual(len(cubic_offsets()), 8)
        self.assertTrue(set(cuboct_offsets()).isdisjoint(cubic_offsets()))

        a = (1, 2, 2)
        b = (1, 2, 3)
        shared = set(voxel_vertices(a)) & set(voxel_vertices(b))
        sites = set(square_face_sites(a, b, VOXEL_SHAPE_CUBOCTAHEDRAL))
        self.assertEqual(shared, sites)
        self.assertEqual(len(sites), 4)

        cuboct = face_joint(a, b, VOXEL_SHAPE_CUBOCTAHEDRAL)
        cube = face_joint(b, a, VOXEL_SHAPE_CUBIC)  # order swapped on purpose
        self.assertEqual(cuboct.a, cube.a)
        self.assertEqual(cuboct.b, cube.b)
        self.assertEqual(cuboct.center, cube.center)
        self.assertEqual(cuboct.center, (1.0, 2.0, 2.5))
        self.assertNotEqual(cuboct.sites, cube.sites)
        self.assertIn("lumped", cuboct.spring)
        # Face plane is the midpoint. Cuboct corners sit one half-pitch
        # off the center along a single transverse axis; cube corners sit
        # on the diagonal of that face.
        for site in cuboct.sites:
            self.assertAlmostEqual(site[2], cuboct.center[2])
            self.assertAlmostEqual(_dist(site, cuboct.center), 0.5)
        for site in cube.sites:
            self.assertAlmostEqual(site[2], cube.center[2])
            self.assertAlmostEqual(_dist(site, cube.center), math.sqrt(0.5))
        # Square side is pitch/√2; the diagonal is the pitch.
        ordered = sorted(cuboct.sites)
        sides = sorted(_dist(ordered[0], other) for other in ordered[1:])
        self.assertAlmostEqual(sides[0], 1.0 / math.sqrt(2))
        self.assertAlmostEqual(sides[-1], 1.0)

    def test_non_neighbors_have_no_face_joint(self) -> None:
        with self.assertRaises(ValueError):
            face_joint((0, 0, 0), (0, 1, 1))
        with self.assertRaises(ValueError):
            square_face_sites((0, 0, 0), (0, 0, 2))

    def test_face_joints_are_the_sag_pairs(self) -> None:
        cells = [(1, 0, 0), (1, 0, 1), (1, 1, 0)]
        joints = face_joints(cells, VOXEL_SHAPE_CUBOCTAHEDRAL)
        pairs = {(j.a, j.b) for j in joints}
        self.assertEqual(pairs, {((1, 0, 0), (1, 0, 1)), ((1, 0, 0), (1, 1, 0))})
        cubic = face_joints(cells, VOXEL_SHAPE_CUBIC)
        self.assertEqual([(j.a, j.b, j.axis) for j in joints], [(j.a, j.b, j.axis) for j in cubic])
        self.assertEqual([j.center for j in joints], [j.center for j in cubic])
        self.assertNotEqual([j.sites for j in joints], [j.sites for j in cubic])


class TestLocomotionMatchesCubic(unittest.TestCase):
    def test_rules_match_for_reach_and_stride(self) -> None:
        world = World3D(4, 5, 5)
        world.place_voxel((1, 2, 2))
        motion = MotionModel(reach_radius=2, stride=2, climb=1)
        cuboct = CuboctahedralLattice3D(world, motion)
        cubic = CubicLattice3D(world, motion)
        self.assertEqual(cuboct.voxel_shape, VOXEL_SHAPE_CUBOCTAHEDRAL)
        self.assertEqual(cubic.voxel_shape, VOXEL_SHAPE_CUBIC)
        for level in range(world.levels):
            for row in range(world.rows):
                for col in range(world.cols):
                    node = (level, row, col)
                    self.assertEqual(cuboct.is_footing(node), cubic.is_footing(node))
                    self.assertEqual(set(cuboct.neighbors(node)), set(cubic.neighbors(node)))
                    self.assertEqual(set(cuboct.reach_cells(node)), set(cubic.reach_cells(node)))
                    self.assertEqual(
                        cuboct.heuristic_distance(node, (2, 4, 4)),
                        cubic.heuristic_distance(node, (2, 4, 4)),
                    )
        # A stride-2 step sweeps the same intermediate the cubic lattice does.
        start = (1, 1, 1)
        goal = (1, 1, 3)
        self.assertIn(goal, set(cuboct.neighbors(start)))
        self.assertEqual(cuboct.move_footprint(start, goal), cubic.move_footprint(start, goal))
        self.assertEqual(cuboct.move_footprint(start, goal), ((1, 1, 2),))

    def test_rebound_geometry_stays_cuboct(self) -> None:
        world = example_world()
        geom = CuboctahedralLattice3D(world, MotionModel(reach_radius=2))
        rebound = geom.bound_factory()(world.copy())
        future = geom.future_view([(1, 2, 2)])
        self.assertIsInstance(rebound, CuboctahedralLattice3D)
        self.assertIsInstance(future, CuboctahedralLattice3D)
        self.assertEqual(rebound.motion, geom.motion)
        self.assertEqual(future.motion.reach_radius, 2)
        self.assertEqual(future.world.occupancy[(1, 2, 2)], 2)
        self.assertEqual(world.occupancy[(1, 2, 2)], 0)  # live world untouched

    def test_plan_matches_cubic_and_validates(self) -> None:
        world = example_world()
        cuboct_plan = plan_build_order(world, EXAMPLE_DEPOT, CuboctahedralLattice3D)
        cubic_plan = plan_build_order(world, EXAMPLE_DEPOT, CubicLattice3D)
        self.assertIsNotNone(cuboct_plan)
        self.assertEqual(
            [task.cell for task in cuboct_plan],
            [task.cell for task in cubic_plan],
        )
        ok, why = validate_plan(world, cuboct_plan, EXAMPLE_DEPOT, CuboctahedralLattice3D)
        self.assertTrue(ok, why)
        self.assertEqual(len(cuboct_plan), world.blueprint_count)

    def test_one_robot_builds_the_example(self) -> None:
        world = example_world()
        swarm = Swarm(
            world,
            EXAMPLE_DEPOT,
            starts=[EXAMPLE_START],
            geometry_factory=CuboctahedralLattice3D,
        )
        while not swarm.done and swarm.tick_count < 8000:
            swarm.tick()
            positions = [r.pos for r in swarm.robots]
            self.assertEqual(len(positions), len(set(positions)))
            geom = CuboctahedralLattice3D(world)
            for robot in swarm.robots:
                self.assertIsNotNone(bfs_path(geom, robot.pos, {EXAMPLE_DEPOT}))
        self.assertTrue(world.complete, f"incomplete after {swarm.tick_count} ticks")


class TestStabilityAndSag(unittest.TestCase):
    def test_policy_matches_cubic_frame(self) -> None:
        world = World3D(4, 4, 4)
        cuboct = CuboctahedralLattice3D(world).stability_policy(2)
        cubic = CubicLattice3D(world).stability_policy(2)
        self.assertEqual(cuboct, cubic)
        self.assertEqual((cuboct.up_axis, cuboct.up_sign, cuboct.ground_plane), (0, 1, 0))

    def test_screen_agrees_where_the_face_graph_agrees(self) -> None:
        column = [(1, 2, 2), (2, 2, 2), (3, 2, 2)]
        span = [(1, 2, 2), (2, 2, 2), (2, 2, 3), (2, 2, 4)]
        policy = CuboctahedralLattice3D(World3D(5, 5, 6)).stability_policy(0)
        cubic = CubicLattice3D(World3D(5, 5, 6)).stability_policy(0)
        self.assertTrue(is_stable(column, policy))
        self.assertTrue(is_stable(column, cubic))
        self.assertFalse(is_stable(span, policy))
        self.assertEqual(is_stable(span, policy), is_stable(span, cubic))

    def test_closed_forms_hold_on_the_cuboct_frame(self) -> None:
        # Finite clamp under the root only. The stock ground plane would
        # plant the whole first course and the shear chain would not apply.
        root = (1, 0, 0)
        arm = [root, (1, 0, 1), (1, 0, 2), (1, 0, 3)]
        policy = CuboctahedralLattice3D(World3D(3, 1, 4)).stability_policy(None)
        estimate = estimate_sag(
            arm,
            policy,
            base=( (0, 0, 0), ),
            ground_plane=None,
            k_axial=50.0,
            k_shear=4.0,
            mass=2.0,
            g=3.0,
        )
        tip = shear_cantilever_tip_sag(3, mass=2.0, g=3.0, k_shear=4.0)
        self.assertAlmostEqual(estimate.worst_sag, tip)
        self.assertEqual(estimate.worst_sag_cell, (1, 0, 3))
        self.assertAlmostEqual(estimate.displacements[(1, 0, 3)][1], 0.0)
        self.assertAlmostEqual(estimate.displacements[(1, 0, 3)][2], 0.0)

        tower = [root, (2, 0, 0), (3, 0, 0)]
        via_method = CuboctahedralLattice3D(World3D(4, 1, 1)).estimate_sag(
            tower,
            policy=policy,
            base=((0, 0, 0),),
            ground_plane=None,
            k_axial=4.0,
            k_shear=50.0,
            mass=2.0,
            g=3.0,
        )
        direct = estimate_sag(
            tower,
            policy,
            base=((0, 0, 0),),
            ground_plane=None,
            k_axial=4.0,
            k_shear=50.0,
            mass=2.0,
            g=3.0,
        )
        shortening = axial_tower_compression(2, mass=2.0, g=3.0, k_axial=4.0)
        self.assertAlmostEqual(via_method.worst_sag, shortening)
        self.assertAlmostEqual(via_method.worst_sag, direct.worst_sag)
        for cell in tower:
            for got, want in zip(via_method.displacements[cell], direct.displacements[cell]):
                self.assertAlmostEqual(got, want)

    def test_cluster_matches_cubic_sag_numerically(self) -> None:
        cells = [(1, 0, 0), (1, 0, 1), (1, 1, 0), (2, 0, 0)]
        frame = dict(up_axis=0, up_sign=1, ground_plane=None, base=((0, 0, 0),), max_cantilever=3)
        cuboct = CuboctahedralLattice3D(World3D(3, 2, 2)).estimate_sag(cells, k=2.5, **frame)
        cubic = estimate_sag(cells, k=2.5, **frame)
        self.assertAlmostEqual(cuboct.worst_sag, cubic.worst_sag)
        self.assertEqual(cuboct.worst_sag_cell, cubic.worst_sag_cell)
        joints = CuboctahedralLattice3D(World3D(3, 2, 2)).face_joints(cells)
        self.assertEqual(len(joints), 3)
        self.assertTrue(all(j.shape == VOXEL_SHAPE_CUBOCTAHEDRAL for j in joints))


class TestViewerLog(unittest.TestCase):
    def test_cubic_log_omits_the_shape_key(self) -> None:
        world = World3D(2, 2, 2)
        log = RunLog()
        log.log_tick(0, world)
        annotate_run_log(log, VOXEL_SHAPE_CUBIC)
        data = log.to_dict()
        self.assertEqual(data["version"], RUN_LOG_VERSION_3D)
        self.assertNotIn("voxel_shape", data["meta"])
        self.assertNotIn("voxel_shape", str(data["meta"]))

    def test_cuboct_sample_keeps_the_v3_structure_schema(self) -> None:
        world = World3D(2, 1, 3)
        policy = CuboctahedralLattice3D(world).stability_policy(None)
        # Same finite clamp the closed form needs.
        from sim.stability import StabilityPolicy

        clamp = StabilityPolicy(
            up_axis=0, up_sign=1, ground_plane=None, base=((0, 0, 0),), max_cantilever=None
        )
        self.assertEqual(policy.up_axis, clamp.up_axis)
        log = RunLog(structure=StructureLog(clamp, k_shear=4.0, k_axial=4.0, pitch=1.0))
        annotate_run_log(log, VOXEL_SHAPE_CUBOCTAHEDRAL)
        log.log_tick(0, world)
        world.place_voxel((1, 0, 0))
        log.log_tick(1, world)
        world.place_voxel((1, 0, 1))
        log.log_tick(2, world)
        data = log.to_dict()
        self.assertEqual(data["version"], RUN_LOG_VERSION_3D)
        self.assertEqual(data["meta"]["voxel_shape"], "cuboctahedral")
        frame = data["ticks"][2]["structure"]
        self.assertEqual(
            set(frame),
            {
                "worst_displacement",
                "worst_cell",
                "worst_sag",
                "worst_sag_cell",
                "n_fixed",
                "n_free",
                "displacements",
                "joints",
            },
        )
        self.assertAlmostEqual(
            frame["worst_sag"],
            shear_cantilever_tip_sag(1, k_shear=4.0),
        )
        # Dropping the shape key leaves a log the cubic viewer path accepts.
        del data["meta"]["voxel_shape"]
        self.assertEqual(data["version"], 3)
        self.assertNotIn("voxel_shape", data["meta"])
        self.assertIn("structure", data["ticks"][2])

    def test_viewer_tables_match_the_geometry_module(self) -> None:
        source = _VIEWER.read_text(encoding="utf-8")
        offsets = _js_array(source, "CUBOCT_OFFSETS")
        tris = _js_array(source, "CUBOCT_TRIS")
        expect = viewer_offsets()
        self.assertEqual(len(offsets), len(expect))
        for got, want in zip(offsets, expect):
            self.assertEqual(len(got), 3)
            for a, b in zip(got, want):
                self.assertAlmostEqual(a, b)
        self.assertEqual([tuple(tri) for tri in tris], list(cuboct_triangles()))
        self.assertIn("shape === \"cuboctahedral\" ? \"cuboctahedral\" : \"cubic\"", source)
        self.assertIn("[1, 2, 3].includes(data.version)", source)
        self.assertIn("body.is-2d #shapeWrap", source)
        self.assertIn(str(VIEWER_INSET), source)
        # Outward winding survives the (col, level, row) axis map.
        parsed = [tuple(p) for p in offsets]
        for i, j, k in tris:
            a, b, c = parsed[i], parsed[j], parsed[k]
            normal = (
                (b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1]),
                (b[2] - a[2]) * (c[0] - a[0]) - (b[0] - a[0]) * (c[2] - a[2]),
                (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]),
            )
            centroid = tuple((a[n] + b[n] + c[n]) / 3.0 for n in range(3))
            self.assertGreater(sum(normal[n] * centroid[n] for n in range(3)), 0.0)


class TestGatedColumn(unittest.TestCase):
    def test_column_plans_under_the_cuboct_screen(self) -> None:
        world = World3D(4, 4, 4)
        for level in (1, 2, 3):
            world.blueprint[level, 2, 2] = True
        depot = (1, 0, 0)
        policy = CuboctahedralLattice3D(world).stability_policy(0)
        plan = plan_build_order(
            world, depot, CuboctahedralLattice3D, stability=policy
        )
        self.assertIsNotNone(plan)
        self.assertEqual(len(plan), 3)
        ok, why = validate_plan(
            world, plan, depot, CuboctahedralLattice3D, stability=policy
        )
        self.assertTrue(ok, why)


if __name__ == "__main__":
    unittest.main()
