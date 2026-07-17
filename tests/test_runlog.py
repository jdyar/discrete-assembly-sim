# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Run-log contract (v1) and animation-fallback tests.

Uses a fake robot object (duck-typed per the contract in sim/metrics.py) so
these tests stay independent of the sim/robot.py implementation.
"""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from main import build_world, run
from sim.metrics import RUN_LOG_VERSION, RunLog
from sim.render import animate_run
from sim.world import VOXEL, World


class FakeRobot:
    """Minimal object satisfying the logging contract."""

    def __init__(self, pos=(5, 0), state="MOVING", carrying=True):
        self.pos = pos
        self.state = state
        self.carrying = carrying


class TestRunLogContract(unittest.TestCase):
    def _demo_log(self) -> RunLog:
        world = build_world()
        return run(world, verbose=False)

    def test_top_level_shape(self) -> None:
        data = self._demo_log().to_dict()
        self.assertEqual(data["version"], RUN_LOG_VERSION)
        self.assertEqual(data["meta"]["rows"], 7)
        self.assertEqual(data["meta"]["cols"], 10)
        self.assertIn("legend", data["meta"])
        self.assertEqual(len(data["blueprint"]), 24)
        # tick 0 initial frame + 24 placement ticks
        self.assertEqual(len(data["ticks"]), 25)
        self.assertEqual(data["ticks"][0]["tick"], 0)
        self.assertEqual(data["ticks"][0]["placed"], 0)
        self.assertEqual(data["ticks"][-1]["placed"], 24)

    def test_frames_have_full_grids_and_null_robot(self) -> None:
        data = self._demo_log().to_dict()
        for frame in data["ticks"]:
            self.assertEqual(len(frame["occupancy"]), 7)
            self.assertEqual(len(frame["occupancy"][0]), 10)
            self.assertIsNone(frame["robot"])  # Slice 0: no robot

    def test_robot_snapshot(self) -> None:
        world = World(rows=3, cols=3)
        log = RunLog()
        log.log_tick(0, world, robot=FakeRobot(pos=(2, 1), state="PLACING"))
        robot = log.frames[0]["robot"]
        self.assertEqual(robot, {"pos": [2, 1], "state": "PLACING", "carrying": True})

    def test_contract_violation_raises(self) -> None:
        class BadRobot:
            pos = (0, 0)  # no state / carrying

        with self.assertRaises(AttributeError):
            RunLog().log_tick(0, World(rows=2, cols=2), robot=BadRobot())

    def test_save_roundtrips_as_json(self) -> None:
        log = self._demo_log()
        with tempfile.TemporaryDirectory() as tmp:
            path = log.save(Path(tmp) / "nested" / "run.json")
            data = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(data, log.to_dict())


class TestRunLogV3(unittest.TestCase):
    def test_3d_world_logs_version_3(self) -> None:
        from sim.world3d import World3D

        world = World3D(3, 4, 5)
        world.blueprint[1, 1, 1] = True
        log = RunLog()
        log.log_tick(0, world, robots=[FakeRobot(pos=(1, 2, 2))])
        data = log.to_dict()
        self.assertEqual(data["version"], 3)
        self.assertEqual(data["meta"]["levels"], 3)
        self.assertEqual(data["meta"]["rows"], 4)
        self.assertEqual(data["meta"]["cols"], 5)
        self.assertEqual(data["blueprint"], [[1, 1, 1]])
        frame = data["ticks"][0]
        occ = frame["occupancy"]
        self.assertEqual((len(occ), len(occ[0]), len(occ[0][0])), (3, 4, 5))
        self.assertEqual(frame["robots"][0]["pos"], [1, 2, 2])
        self.assertEqual(frame["robot"]["pos"], [1, 2, 2])  # v1-field compat

    def test_animate_run_rejects_v3_logs(self) -> None:
        from sim.world3d import World3D

        log = RunLog()
        log.log_tick(0, World3D(2, 2, 2))
        with self.assertRaises(ValueError):
            animate_run(log.to_dict(), show=False)


class TestAnimationFallback(unittest.TestCase):
    def test_animation_builds_and_draws(self) -> None:
        world = build_world()
        data = run(world, verbose=False).to_dict()
        # Give one frame a robot so the overlay path is exercised.
        data["ticks"][1]["robot"] = {"pos": [6, 0], "state": "MOVING", "carrying": False}
        anim = animate_run(data, show=False)
        # Draw a mid-run frame; robot cell must not crash the overlay.
        anim._draw_frame(1)  # noqa: SLF001 - cheap smoke test of the draw fn
        self.assertEqual(len(data["ticks"]), 25)


if __name__ == "__main__":
    unittest.main()
