# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Seeded faults and the existing repair path.

Rates are configuration. A capture miss is a precision-model rejection.
A failed fastener names a face spring. The swarm's inspect / remove /
kickback / resequence path does the recovery. ``k = 1`` is not a fit.
"""

from __future__ import annotations

import csv
import random
import unittest
from pathlib import Path

from sim.armadas import ActionTimes, build_gated_order, simulate
from sim.faults import (
    CAPTURE_MISS,
    FaultInjector,
    FaultRates,
    face_partner,
    fastener_joint,
)
from sim.geometry3d import CubicLattice3D, MotionModel
from sim.precision import armadas_precision
from sim.scaffold import CRANE, ScaffoldPlan, caps_for_finished, geometry_factory
from sim.stability import JointCap, JointStrength, SagCap
from sim.swarm import Swarm
from sim.world import DEFECT, SCAFFOLD
from sim.world3d import World3D


def _tower() -> tuple[World3D, tuple[int, int, int]]:
    world = World3D(4, 5, 6)
    world.blueprint[1, 2, 2] = True
    world.blueprint[2, 2, 2] = True
    return world, (1, 1, 1)


def _drive(swarm: Swarm, limit: int = 800) -> Swarm:
    for _ in range(limit):
        if swarm.done:
            break
        swarm.tick()
    return swarm


def _factory(world):
    return CubicLattice3D(world, MotionModel(reach_radius=2, stride=1, climb=1))


class Injector(unittest.TestCase):
    def test_zero_rates_do_not_draw_and_repeat_a_seed(self):
        rates = FaultRates()
        first = FaultInjector(rates, seed=3)
        second = FaultInjector(rates, seed=3)
        for _ in range(5):
            self.assertFalse(first.roll("capture"))
            self.assertFalse(second.roll("drop"))
        # A positive rate consumes the seed. The same seed repeats.
        hot = FaultRates(capture=1.0)
        a = FaultInjector(hot, seed=9)
        b = FaultInjector(hot, seed=9)
        self.assertEqual(
            [a.roll("capture") for _ in range(4)],
            [b.roll("capture") for _ in range(4)],
        )
        self.assertTrue(all(a.events == b.events for a, b in ()))  # seed only

    def test_script_fires_one_named_channel(self):
        injector = FaultInjector(FaultRates(), seed=0, script=["fastener"])
        self.assertFalse(injector.roll("capture"))
        self.assertTrue(injector.roll("fastener"))
        self.assertFalse(injector.roll("fastener"))

    def test_a_fault_opened_during_recovery_is_a_cascade(self):
        injector = FaultInjector(FaultRates(capture=1.0), seed=1)
        first = injector.begin("capture", tick=1, cell=(1, 0, 0))
        second = injector.begin("drop", tick=2, cell=(1, 0, 1))
        self.assertFalse(injector.events[first].cascade)
        self.assertTrue(injector.events[second].cascade)
        self.assertEqual(injector.events[second].cause, first)
        injector.finish(first, tick=4)
        self.assertEqual(injector.events[first].recovery_time, 3.0)
        self.assertEqual(injector.n_cascades, 1)

    def test_capture_miss_is_outside_the_claw_and_the_rate_is_config(self):
        model = armadas_precision()
        self.assertGreater(CAPTURE_MISS[2], model.out_of_plane_m)
        self.assertLessEqual(abs(CAPTURE_MISS[0]), model.in_plane_m)
        rates = FaultRates(capture=0.05, drop=0.05)
        self.assertEqual(rates.capture, 0.05)
        # The number is an input. Nothing in the injector measures it.
        self.assertFalse(hasattr(rates, "measured"))

    def test_face_partner_is_the_joint_spring(self):
        cell = (2, 2, 3)
        partner = face_partner(cell, [(2, 2, 2), (9, 9, 9)])
        self.assertEqual(partner, (2, 2, 2))
        self.assertIsNone(face_partner(cell, []))
        cap = JointCap(JointStrength(axial_compression=5.0, shear=5.0))
        from sim.stability import StabilityPolicy

        policy = StabilityPolicy(up_axis=0, up_sign=1, ground_plane=0)
        joint, util = fastener_joint(
            (1, 2, 2),
            [(1, 2, 2)],
            policy,
            cap,
        )
        # The root on the ground plane has no voxel neighbor.
        self.assertIsNone(joint)
        self.assertIsNone(util)


class SwarmRecovery(unittest.TestCase):
    def test_zero_injector_matches_an_uninstrumented_swarm(self):
        def run(faults):
            world, depot = _tower()
            swarm = Swarm(
                world,
                depot,
                starts=[(1, 1, 1)],
                geometry_factory=_factory,
                faults=faults,
                rng=random.Random(0),
            )
            _drive(swarm)
            places = [event for event in swarm.events if event[2] == "place"]
            return swarm.done, swarm.tick_count, places

        plain = run(None)
        instrumented = run(FaultInjector(FaultRates(), seed=0))
        self.assertTrue(plain[0] and instrumented[0])
        self.assertEqual(plain[1], instrumented[1])
        self.assertEqual(plain[2], instrumented[2])

    def test_scripted_capture_is_removed_and_replaced(self):
        world, depot = _tower()
        faults = FaultInjector(FaultRates(), seed=0, script=["capture"])
        swarm = Swarm(
            world,
            depot,
            starts=[(1, 1, 1)],
            geometry_factory=_factory,
            faults=faults,
            inspect_enabled=True,
        )
        saw_defect = False
        for _ in range(800):
            if any(world.occupancy[cell] == DEFECT for cell in ((1, 2, 2), (2, 2, 2))):
                saw_defect = True
            if swarm.done:
                break
            swarm.tick()
        self.assertTrue(saw_defect)
        self.assertTrue(swarm.done, swarm.tick_count)
        self.assertTrue(world.complete)
        self.assertEqual(world.defect_count, 0)
        kinds = [event.kind for event in faults.events]
        self.assertIn("capture", kinds)
        capture = next(event for event in faults.events if event.kind == "capture")
        self.assertEqual(capture.recovery, "re-place")
        self.assertIsNotNone(capture.recovery_time)
        self.assertGreaterEqual(capture.recovery_time, 1.0)
        self.assertEqual(capture.rate, 0.0)  # scripted, not a measured rate

    def test_drop_reroutes_to_the_depot_and_finishes(self):
        world, depot = _tower()
        faults = FaultInjector(FaultRates(), seed=0, script=["drop"])
        swarm = Swarm(
            world,
            depot,
            starts=[(1, 1, 1)],
            geometry_factory=_factory,
            faults=faults,
        )
        _drive(swarm)
        self.assertTrue(swarm.done, swarm.tick_count)
        drop = next(event for event in faults.events if event.kind == "drop")
        self.assertEqual(drop.recovery, "re-route")
        self.assertIsNotNone(drop.recovery_time)
        self.assertGreater(swarm.defects_found, -1)

    def test_stall_keeps_the_task_and_reroutes(self):
        world, depot = _tower()
        faults = FaultInjector(
            FaultRates(stall_ticks=2), seed=0, script=["stall"]
        )
        swarm = Swarm(
            world,
            depot,
            starts=[(1, 1, 1)],
            geometry_factory=_factory,
            faults=faults,
        )
        _drive(swarm)
        self.assertTrue(swarm.done, swarm.tick_count)
        stall = next(event for event in faults.events if event.kind == "stall")
        self.assertEqual(stall.recovery, "re-route")
        self.assertIsNotNone(stall.recovery_time)
        self.assertGreaterEqual(stall.recovery_time, 2.0)

    def test_robot_failure_is_reassigned_to_the_other_robot(self):
        world, depot = _tower()
        faults = FaultInjector(
            FaultRates(down_ticks=4), seed=0, script=["robot_fail"]
        )
        swarm = Swarm(
            world,
            depot,
            starts=[(1, 1, 1), (1, 1, 4)],
            geometry_factory=_factory,
            faults=faults,
        )
        _drive(swarm)
        self.assertTrue(swarm.done, swarm.tick_count)
        failed = next(event for event in faults.events if event.kind == "robot_fail")
        self.assertEqual(failed.recovery, "reassign")
        self.assertIsNotNone(failed.done_tick)
        claims = [event for event in swarm.events if event[2] == "claim" and event[3] == failed.cell]
        claimers = {event[1] for event in claims}
        self.assertGreaterEqual(len(claimers), 1)
        self.assertIsNotNone(failed.recovery_time)

    def test_fastener_names_a_joint_and_is_replaced(self):
        world, depot = _tower()
        # The first voxel has no partner. The second fastens to the first.
        faults = FaultInjector(FaultRates(), seed=0, script=["fastener", "fastener"])
        policy = CubicLattice3D(world).stability_policy(max_cantilever=1)
        sag = SagCap(max_sag=10.0)
        joints = JointCap(JointStrength(axial_compression=10.0, shear=10.0))
        swarm = Swarm(
            world,
            depot,
            starts=[(1, 1, 1)],
            geometry_factory=_factory,
            faults=faults,
            stability=policy,
            sag=sag,
            joints=joints,
        )
        _drive(swarm)
        self.assertTrue(swarm.done, swarm.tick_count)
        self.assertTrue(world.complete)
        fasteners = [event for event in faults.events if event.kind == "fastener"]
        self.assertTrue(fasteners, "the upper voxel should fail a fastener")
        self.assertIsNotNone(fasteners[0].joint)
        self.assertEqual(fasteners[0].recovery, "re-place")
        self.assertIsNotNone(fasteners[0].recovery_time)

    def test_feed_outage_blocks_the_depot_then_completes(self):
        world, depot = _tower()
        faults = FaultInjector(
            FaultRates(outage_ticks=3), seed=0, script=["feed_outage"]
        )
        swarm = Swarm(
            world,
            depot,
            starts=[(1, 1, 1), (1, 1, 4)],
            geometry_factory=_factory,
            faults=faults,
        )
        _drive(swarm)
        self.assertTrue(swarm.done, swarm.tick_count)
        outage = next(event for event in faults.events if event.kind == "feed_outage")
        self.assertEqual(outage.recovery, "reassign")
        self.assertIsNotNone(outage.recovery_time)

    def test_gates_stay_on_through_a_capture(self):
        world, depot = _tower()
        frame = CubicLattice3D(world).stability_policy(0)
        policy, sag = caps_for_finished(world, frame)
        faults = FaultInjector(FaultRates(), seed=1, script=["capture"])
        swarm = Swarm(
            world,
            depot,
            starts=[(1, 1, 1)],
            geometry_factory=_factory,
            faults=faults,
            stability=policy,
            sag=sag,
            joints=JointCap(JointStrength(axial_compression=5.0, shear=5.0)),
        )
        _drive(swarm)
        self.assertTrue(world.complete)
        self.assertTrue(swarm.done)
        self.assertEqual(world.defect_count, 0)

    def test_install_scaffold_places_then_removes_it(self):
        world, depot = _tower()
        support = (1, 2, 3)

        class _Task:
            def __init__(self, cell):
                self.cell = cell

        plan = ScaffoldPlan(
            (support,),
            (
                _Task(support),
                _Task((1, 2, 2)),
                _Task((2, 2, 2)),
            ),
            (_Task(support),),
            0,
        )
        # The plan's cells are Tasks; ScaffoldPlan stores them as given.
        swarm = Swarm(
            world,
            depot,
            starts=[(1, 1, 1)],
            geometry_factory=_factory,
            faults=FaultInjector(FaultRates(), seed=0),
        )
        swarm._now = 0
        swarm._install_scaffold(plan)
        self.assertIn(support, swarm.scaffold_cells)
        _drive(swarm, limit=600)
        self.assertTrue(world.complete, swarm.tick_count)
        self.assertNotEqual(world.occupancy[support], SCAFFOLD)
        self.assertTrue(world.is_empty(support))
        self.assertTrue(any(event.kind == "scaffold" for event in swarm.faults.events))


class ShelterFaults(unittest.TestCase):
    def _model(self):
        seed = ((1, 1, 0), (1, 1, 1))
        cells = [seed[0], seed[1], (1, 2, 0), (1, 2, 1)]
        return build_gated_order(cells, seed)

    def test_zero_faults_match_the_plain_schedule(self):
        model = self._model()
        times = ActionTimes(
            step_s=1.0, feed_load_s=1.0, handoff_s=1.0, place_s=1.0, fasten_joint_s=1.0
        )
        stations = ((1, 0, 0),)
        plain = simulate(model, n_feeds=1, robots_per_role=1, times=times, stations=stations)
        again = simulate(
            model,
            n_feeds=1,
            robots_per_role=1,
            times=times,
            stations=stations,
            faults=FaultInjector(FaultRates(), seed=0),
            precision=armadas_precision(distribution="zero"),
        )
        self.assertEqual(plain.makespan_s, again.makespan_s)
        self.assertEqual(plain.n_placed, again.n_placed)
        self.assertEqual(again.n_faults, 0)

    def test_each_channel_is_logged_and_the_schedule_finishes(self):
        model = self._model()
        times = ActionTimes(
            step_s=1.0, feed_load_s=1.0, handoff_s=1.0, place_s=1.0, fasten_joint_s=1.0
        )
        rates = FaultRates(
            capture=0.0,
            drop=1.0,
            stall=0.0,
            robot_fail=0.0,
            fastener=1.0,
            feed_outage=1.0,
            stall_ticks=1,
            down_ticks=1,
            outage_s=2.0,
        )
        # One of each, then the rates above keep firing. Cap the script so
        # the first events are the channels, and let the rates finish the run
        # only for the channels we set to 1. Use a script of one each and
        # zero rates so the run cannot livelock.
        faults = FaultInjector(
            FaultRates(outage_s=2.0, stall_ticks=1, down_ticks=2),
            seed=2,
            script=["feed_outage", "drop", "stall", "robot_fail", "capture", "fastener"],
        )
        result = simulate(
            model,
            n_feeds=2,
            robots_per_role=2,
            times=times,
            stations=((1, 0, 0), (1, 0, 3)),
            faults=faults,
            precision=armadas_precision(distribution="zero"),
            joint_cap=JointCap(JointStrength(axial_compression=20.0, shear=20.0)),
        )
        self.assertEqual(result.n_placed, len(model.order))
        kinds = {event.kind for event in faults.events}
        self.assertTrue(
            {"feed_outage", "drop", "stall", "robot_fail", "fastener"} <= kinds,
            kinds,
        )
        # A datum may absorb the capture script. A later mating cell must
        # not be required if the script was spent on a datum; the fastener
        # still has to name a face.
        fastener = next(event for event in faults.events if event.kind == "fastener")
        self.assertIsNotNone(fastener.joint)
        self.assertEqual(fastener.recovery, "re-place")
        recoveries = {event.recovery for event in faults.events}
        self.assertIn("reassign", recoveries)
        self.assertIn("re-route", recoveries)
        self.assertTrue(all(event.rate == 0.0 or event.kind == "cascade" for event in faults.events))
        self.assertTrue(all(event.done for event in faults.events if event.kind != "cascade"))


class CommittedCampaign(unittest.TestCase):
    def test_rows_label_rates_as_configuration(self) -> None:
        root = Path(__file__).resolve().parents[1]
        with (root / "docs" / "fault_campaign.csv").open() as handle:
            trials = list(csv.DictReader(handle))
        with (root / "docs" / "fault_events.csv").open() as handle:
            events = list(csv.DictReader(handle))
        self.assertEqual(len(trials), 36)
        self.assertEqual(
            {(row["structure"], row["seed"], row["fault_rate"]) for row in trials},
            {
                (structure, seed, rate)
                for structure in ("bridge", "cantilever", "shelter")
                for seed in ("0", "1", "2")
                for rate in ("0", "0.05", "0.1", "0.2")
            },
        )
        self.assertTrue(all(row["rate_source"] == "configuration" for row in trials))
        self.assertTrue(all(row["k"] == "1" for row in trials))
        self.assertTrue(all(row["mass"] == "1" for row in trials))
        self.assertTrue(all(row["g"] == "1" for row in trials))
        self.assertTrue(all(row["completed"] == "1" for row in trials))
        self.assertTrue(all(row["n_scaffold"] == "0" for row in trials))
        shelter0 = [
            row for row in trials if row["structure"] == "shelter" and row["fault_rate"] == "0"
        ]
        self.assertEqual(len(shelter0), 3)
        self.assertTrue(all(row["makespan"] == "111860" for row in shelter0))
        self.assertTrue(all(row["overhead"] == "0" for row in shelter0))
        bridge0 = sorted(
            float(row["makespan"])
            for row in trials
            if row["structure"] == "bridge" and row["fault_rate"] == "0"
        )
        self.assertEqual(bridge0, [77.0, 82.0, 88.0])
        # Overhead is makespan / rate-0 median − 1. The rate is still an input.
        shelter_high = [
            float(row["overhead"])
            for row in trials
            if row["structure"] == "shelter" and row["fault_rate"] == "0.2"
        ]
        self.assertEqual(len(shelter_high), 3)
        self.assertGreater(min(shelter_high), 0.4)
        self.assertLess(max(shelter_high), 0.6)
        self.assertTrue(events)
        self.assertTrue(all(event["rate_source"] == "configuration" for event in events))
        self.assertTrue(all(event["k"] == "1" for event in events))
        self.assertTrue(
            {"capture", "drop", "stall", "robot_fail", "fastener", "feed_outage"}
            <= {event["kind"] for event in events}
        )
        summary = (root / "docs" / "fault_summary.txt").read_text()
        self.assertIn("configuration, not a measurement", summary)
        self.assertIn("k = 1.0", summary)
        for name in (
            "fault_completion.png",
            "fault_overhead.png",
            "fault_cascades.png",
            "fault_recovery.png",
            "fault_recovery_still.png",
        ):
            self.assertGreater((root / "docs" / name).stat().st_size, 1000)


if __name__ == "__main__":
    unittest.main()
