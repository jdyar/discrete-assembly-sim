# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Adversarial trap fixtures for the multi-robot choreographer (Slice 3a/3c).

Authored SPEC-FIRST, before sim/swarm.py exists (division-of-labor amendment
2026-07-14, docs/DESIGN.md, test discipline): these tests are written from the
roadmap's scenario names and the choreographer's published spec, not from
its implementation — they define the Swarm API contract and the invariants,
and skip until the implementation lands.

The three scenarios and why each one is a trap:

- **pocket_trap** — placements can imprison a robot. The blueprint's last
  voxel caps a cavity that sits on the natural ground-level crossing route;
  a coordination layer without the connectivity gate will happily seal a
  robot inside (its footing survives, its future doesn't). Passes only if
  the choreographer refuses connectivity-severing placements until the
  cavity is clear (gate + kickback), yet still finishes the build.

- **corridor_standoff** — a single shared crossing cell. Two robots must
  traverse a one-cell gap in a full-height wall in opposite directions at
  overlapping times. Naive independent planning meets head-on (swap or
  deadlock); reservation-correct planning resolves it purely by timing
  (one waits). Passes only with zero co-locations, zero swaps, and
  completion within budget — deadlock shows up as budget exhaustion.

- **repair_in_a_crowd** — a defect mid-swarm must not stop the world. With
  three robots working a small wall, one placement is forced defective.
  The detecting robot must remove it and the cell must be rebuilt (repair
  threaded through live reservations) while at least one OTHER robot makes
  real progress during the repair window. A design that replans by
  freezing the swarm fails the liveness assertion.

Shared invariants checked every tick in all scenarios: no two robots on
one node (collision), and every robot can still path to the depot on the
current world (no entrapment, the pocket trap's core assertion).
"""

import unittest

from sim.geometry import SquareLattice2D, bfs_path
from sim.world import World

try:
    from sim.swarm import Swarm
except ImportError:  # spec-first: implementation lands after these tests
    Swarm = None

MAX_TICKS = 3000


def run_swarm(test, swarm, world, depot, budget=MAX_TICKS):
    """Drive the swarm to completion, asserting per-tick invariants."""
    while not swarm.done and swarm.tick_count < budget:
        swarm.tick()
        positions = [r.pos for r in swarm.robots]
        test.assertEqual(
            len(positions), len(set(positions)),
            f"collision at tick {swarm.tick_count}: {positions}",
        )
        geom = SquareLattice2D(world)
        for r in swarm.robots:
            test.assertIsNotNone(
                bfs_path(geom, r.pos, {depot}),
                f"robot {r.id} entrapped at {r.pos}, tick {swarm.tick_count}",
            )
    test.assertTrue(
        swarm.done,
        f"not done after {budget} ticks — deadlock or livelock",
    )
    test.assertTrue(world.complete, "blueprint incomplete")


@unittest.skipIf(Swarm is None, "sim.swarm not implemented yet (spec-first fixture)")
class TestPocketTrap(unittest.TestCase):
    """Cap a cavity that sits on the ground crossing route.

    World (7x9, ground r=6, depot D=(5,0), towers T, cap C, cavity '.'):

        r3   . . T C T . .        cap (3,3) seals cavity (4,3),(5,3)
        r4   . . T . T . .        once both towers exist
        r5   D . T . T . .        <- (5,3) is the natural crossing cell
        r6   = = = = = = =        ground

    Blueprint = both towers + cap; cavity cells are NOT blueprint. The cap
    is placeable from (2,2)/(2,4) (atop a tower), so the build is always
    completable — the only failure mode is capping while a robot is inside.
    """

    def test_never_seals_a_robot_and_completes(self):
        world = World(7, 9)
        for cell in [(5, 2), (4, 2), (3, 2), (5, 4), (4, 4), (3, 4), (3, 3)]:
            world.blueprint[cell] = True
        depot = (5, 0)
        swarm = Swarm(world, depot, starts=[(5, 1), (5, 5)])
        run_swarm(self, swarm, world, depot)


@unittest.skipIf(Swarm is None, "sim.swarm not implemented yet (spec-first fixture)")
class TestCorridorStandoff(unittest.TestCase):
    """One crossing cell, two robots, opposite directions.

    World (6x11, ground r=5): a pre-built full-height wall at c=5 with a
    single gap at (2,5). Crossing = climb the face, pivot through the gap,
    descend the far side — every crossing shares node (2,5).

        r0   . . . . . W . . . . .
        r1   . . . . . W . . . . .
        r2   . . . . . _ . . . . .   <- the only crossing cell (2,5)
        r3   . . . . . W . . . . .
        r4   D . b . . W . . B a .   depot D=(4,0); blueprint a=(4,9)? see below
        r5   = = = = = = = = = = =   ground

    Robot A starts right of the wall at (4,9): its first move is a LEFTWARD
    crossing (to the depot). Robot B starts at (4,1), picks immediately and
    heads RIGHT through the same gap to place at (4,7). Their crossings
    overlap in time; the reservation table must make one wait.
    Blueprint: (4,7) and (4,8) — both right of the wall.
    """

    def test_opposite_crossings_resolve_without_deadlock(self):
        world = World(6, 11)
        for cell in [(4, 5), (3, 5), (1, 5), (0, 5)]:  # wall, gap at (2,5)
            world.place_voxel(cell)
        for cell in [(4, 7), (4, 8)]:
            world.blueprint[cell] = True
        depot = (4, 0)
        swarm = Swarm(world, depot, starts=[(4, 9), (4, 1)])
        run_swarm(self, swarm, world, depot)


@unittest.skipIf(Swarm is None, "sim.swarm not implemented yet (spec-first fixture)")
class TestRepairInACrowd(unittest.TestCase):
    """Forced defect mid-swarm: repair must thread through live reservations.

    Three robots build a 4x2 wall (8 voxels, rows 3-4, cols 6-9) on a
    6x12 world, depot at (4,0), all robots starting DEPOT-SIDE. The
    first placement at (4,7) is forced defective (defect_cells contract:
    the FIRST placement attempt at a listed cell is defective; retries
    are good). The detector removes it; the cell must be rebuilt; the
    world must complete with full good yield; and between defect
    detection and the replacement placement, at least one OTHER robot
    must move/pick/place — proof the swarm was never frozen for the
    repair.

    Fixture revisions (2026-07-14, same session it was authored):
    (1) the original geometry started two robots on the FAR side of the
    wall-to-be, where the growing wall cut them off from the depot —
    making the liveness assertion unsatisfiable by geometry regardless
    of coordination quality (it did expose a real claim-thrash
    livelock, fixed with a claim cooldown). (2) The second geometry was
    a width-1 ground corridor with the depot in a dead end: three
    robots + one head-on train there is physically unsolvable for any
    sequential-reservation planner (standard MAPF benchmarks exclude
    degree-1 dead-end corridors for this reason), and it is not
    representative — on a real lattice the structure itself provides
    alternate routes. A pre-built SERVICE RAIL (floating row at r=2,
    legal in the no-gravity world) now provides that routing freedom;
    the trap tests coordination, not corridor topology. The ladder at
    c=1 connects the depot zone to the rail — without it the rail is
    unreachable from the left until the wall itself is climbable.
    """

    def test_repair_completes_with_swarm_liveness(self):
        world = World(7, 12)
        world.set_wall_blueprint(width=4, height=2, left=6)
        for c in range(1, 11):  # service rail: alternate routing rows
            world.place_voxel((2, c))
        world.place_voxel((3, 1))  # ladder: rail <-> depot zone
        world.place_voxel((4, 1))
        depot = (5, 0)
        swarm = Swarm(
            world,
            depot,
            starts=[(5, 1), (5, 2), (5, 3)],
            defect_cells={(5, 7)},
        )
        run_swarm(self, swarm, world, depot)

        events = swarm.events  # list of (tick, robot_id, kind, node)
        defect_ticks = [e for e in events if e[2] == "defect_removed"]
        self.assertEqual(len(defect_ticks), 1, "exactly one forced defect")
        t_removed, detector, _, cell = defect_ticks[0]
        self.assertEqual(cell, (5, 7))
        replaced = [
            e for e in events
            if e[2] == "place" and e[3] == (5, 7) and e[0] > t_removed
        ]
        self.assertTrue(replaced, "defective cell was never rebuilt")
        t_replaced = replaced[0][0]
        others_alive = [
            e for e in events
            if t_removed <= e[0] <= t_replaced
            and e[1] != detector
            and e[2] in ("move", "place", "pick")
        ]
        self.assertTrue(
            others_alive,
            "no other robot progressed during the repair window — swarm froze",
        )


if __name__ == "__main__":
    unittest.main()
