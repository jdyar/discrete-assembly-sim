# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Adversarial 3D trap fixtures for the choreographer (Slice 4a).

Authored SPEC-FIRST, before sim/world3d.py or sim/geometry3d.py exist
(division-of-labor amendment 2026-07-14; 3D-first pivot 2026-07-15,
docs/DESIGN.md): these tests are written from the published
failure classes below, not from an implementation — they define the 3D
API contract and skip until it lands.

API contract they pin down:

- ``World3D(levels, rows, cols)`` — occupancy[level, row, col]; level 0
  is a solid GROUND plane; blueprint mask has the same shape; the World
  surface (place_voxel/remove_voxel/is_empty/copy/complete/...) is
  identical to the 2D World so the planner and swarm run unchanged.
- ``CubicLattice3D(world)`` — the Geometry implementation. BILL-E/SOLL-E
  surface locomotion generalized to 3D: footing = empty in-bounds cell
  face-adjacent (6-neighborhood) to a solid; movement = orthogonal to
  adjacent footing, or in-plane diagonal when rounding a corner (exactly
  one of the two between-cells solid — same rule as 2D, applied per
  axis-plane); reach = the 26-cell Chebyshev shell (radius parameterized
  later, Slice 4c — the default must stay symmetric).
- The Swarm, sequencer, choreographer, reservation table, and planner run
  UNCHANGED on 3D nodes (nodes are opaque; binding rule, docs/DESIGN.md).

The three scenarios and the published failure class each is built from:

- **tomb_trap** — enclosure by intermediate configuration. TERMES-class
  systems document that "inappropriate intermediate configurations can
  cause a deadlock" (Werfel/Petersen lineage; see also the collective
  robotic construction review, Science Robotics 2019). In 3D the sharpest
  form: a one-cell room. Once its four walls stand, a robot in the cavity
  is ALREADY unreachable-from-anywhere (the cell above the cavity has no
  solid neighbor until the roof itself arrives) — sealing happens on the
  4th WALL, not the roof. The connectivity gate must refuse any placement
  whose permanent-future graph strands any robot, yet the build must
  still complete (gate + kickback + reordering, never a stall).

- **tunnel_standoff** — the MAPF 1-wide-corridor swap pathology (Stern et
  al., MAPF benchmarks, arXiv:1906.08291: adversarial corridor-swap
  instances; corridors of width 1 are the documented hard case). A
  full-height, full-width slab with a single one-cell tunnel; two robots
  must cross in opposite directions at overlapping times. In 3D there is
  no over-the-top route (the slab reaches the world ceiling) — the
  reservation table must resolve the swap purely by timing. Deadlock
  shows up as budget exhaustion; collision as the per-tick assert.

- **backfill_trap** — single-path additive interiors. TERMES-class robots
  "cannot travel down narrow corridors" and their buildable structures
  are limited to single-path additive orders; a roofed dead-end channel
  must be filled deepest-first BY A ROBOT STANDING IN THE CHANNEL, which
  must then back out before the next cell is placed behind it. Wrong
  order (mouth first) makes the deep cells permanently unplaceable;
  wrong choreography (second robot enters, or places the mouth while the
  filler is inside) entombs the filler. Tests the coupled
  sequencer<->choreographer loop end-to-end in 3D.

Shared invariants checked every tick in all scenarios: no two robots on
one node, and every robot can still path to the depot on the current
world (no entrapment — the tomb trap's core assertion, but enforced
everywhere).

Node convention: ``(level, row, col)``, level 0 = ground plane, up is
+level. The depot and all robot starts stand ON the ground (level 1).
"""

import unittest

try:
    from sim.geometry3d import CubicLattice3D
    from sim.world3d import World3D
except ImportError:  # spec-first: implementation lands after these tests
    CubicLattice3D = None
    World3D = None

from sim.geometry import bfs_path
from sim.swarm import Swarm

MAX_TICKS = 3000

skip_unimplemented = unittest.skipIf(
    World3D is None, "sim.world3d / sim.geometry3d not implemented yet (spec-first)"
)


def run_swarm3d(test, swarm, world, depot, budget=MAX_TICKS):
    """Drive the swarm to completion, asserting per-tick 3D invariants."""
    while not swarm.done and swarm.tick_count < budget:
        swarm.tick()
        positions = [r.pos for r in swarm.robots]
        test.assertEqual(
            len(positions), len(set(positions)),
            f"collision at tick {swarm.tick_count}: {positions}",
        )
        geom = CubicLattice3D(world)
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


@skip_unimplemented
class TestTombTrap(unittest.TestCase):
    """Four walls + roof around a one-cell cavity; cavity is NOT blueprint.

    World 4 levels x 7 rows x 7 cols. Cavity at (1,3,3). Blueprint: its
    four level-1 orthogonal wall neighbors and the roof (2,3,3).

        level 1 (plan view, rows 2-4 / cols 2-4):     level 2:
              . W .            W = wall (blueprint)        . . .
              W c W            c = cavity (never built)    . R .   R = roof
              . W .                                        . . .

    Every wall is individually placeable from OUTSIDE the cavity (each
    has open orthogonal stances), and the roof is placeable from atop
    any wall (stand on a wall top's neighbor cell, target is within the
    Chebyshev shell) — so the build is always completable without ever
    standing in the cavity. The trap: the cavity is also a legal stance
    for all four walls, and it sits between the two robots' start sides,
    so an ungated coordinator will eventually have a robot in the cavity
    when the 4th wall goes up. Footing inside survives (walls are
    grippable); the robot's FUTURE dies — exactly what the permanent-
    future connectivity gate must catch.

    Depot far side at (1,3,0); robots start on opposite flanks so
    natural traffic flows past (and through) the room site.
    """

    def test_never_entombs_a_robot_and_completes(self):
        world = World3D(4, 7, 7)
        for cell in [(1, 2, 3), (1, 4, 3), (1, 3, 2), (1, 3, 4), (2, 3, 3)]:
            world.blueprint[cell] = True
        depot = (1, 3, 0)
        swarm = Swarm(
            world, depot,
            starts=[(1, 3, 1), (1, 3, 5)],
            geometry_factory=CubicLattice3D,
        )
        run_swarm3d(self, swarm, world, depot)


@skip_unimplemented
class TestTunnelStandoff(unittest.TestCase):
    """One tunnel cell through a ceiling-high slab; opposite crossings.

    World 4 levels x 5 rows x 9 cols. A pre-built slab fills col 4 at
    EVERY row and EVERY level 1-3 (the world ceiling — there is no
    over-the-top route, and the world edge means no way around), except
    the single tunnel cell at (1,2,4).

    Depot (1,2,0), left of the slab. Robot A starts RIGHT of the slab at
    (1,2,7): its first depot trip is a leftward crossing. Robot B starts
    left at (1,2,1), picks immediately and heads RIGHT through the same
    tunnel to place at the far-side blueprint. Their crossings overlap;
    the reservation table must make one wait (timing, not re-routing —
    there is no other route). Blueprint: (1,2,6) and (1,2,7), both
    right of the slab.
    """

    def test_opposite_crossings_resolve_without_deadlock(self):
        world = World3D(4, 5, 9)
        for level in (1, 2, 3):
            for row in range(5):
                if (level, row) != (1, 2):  # tunnel at (1,2,4)
                    world.place_voxel((level, row, 4))
        for cell in [(1, 2, 6), (1, 2, 7)]:
            world.blueprint[cell] = True
        depot = (1, 2, 0)
        swarm = Swarm(
            world, depot,
            starts=[(1, 2, 7), (1, 2, 1)],
            geometry_factory=CubicLattice3D,
        )
        run_swarm3d(self, swarm, world, depot)


@skip_unimplemented
class TestBackfillTrap(unittest.TestCase):
    """Fill a roofed dead-end channel: deepest-first, back out each time.

    World 4 levels x 7 rows x 10 cols. Pre-built: a one-cell-wide,
    three-cell-long channel at level 1 (cells (1,3,5),(1,3,6),(1,3,7) —
    these ARE the blueprint), walled on both sides (rows 2 and 4, cols
    5-7), roofed at level 2 (row 3, cols 5-7), and closed at the far end
    ((1,3,8) solid plus its wall/roof collar). The only access is the
    mouth, from (1,3,4).

    Correct play: place (1,3,7) standing at (1,3,6); back out; place
    (1,3,6) from (1,3,5); back out; place (1,3,5) from (1,3,4). Any
    other placement order kills the run (a mouth-first order makes deep
    cells unplaceable and the sequencer must already know it: the
    reachability-aware search never proposes it). The choreography trap
    is the second robot: with two robots hungry for tasks in a
    three-task world, the coordinator must keep robot B from entering
    the channel or placing behind robot A while A is inside — the
    per-tick entrapment invariant catches the entombment the moment it
    would happen.

    Grounding: TERMES-class systems exclude narrow-corridor interiors
    (robots cannot traverse them once built) — the published reason
    interiors must be sequenced as single-path additive orders.
    """

    def test_channel_backfills_without_entombment(self):
        world = World3D(4, 7, 10)
        for col in (5, 6, 7, 8):
            world.place_voxel((1, 2, col))  # side wall
            world.place_voxel((1, 4, col))  # side wall
            world.place_voxel((2, 3, col))  # roof
        world.place_voxel((1, 3, 8))  # closed far end
        for cell in [(1, 3, 5), (1, 3, 6), (1, 3, 7)]:
            world.blueprint[cell] = True
        depot = (1, 3, 0)
        swarm = Swarm(
            world, depot,
            starts=[(1, 3, 1), (1, 5, 1)],
            geometry_factory=CubicLattice3D,
        )
        run_swarm3d(self, swarm, world, depot)


if __name__ == "__main__":
    unittest.main()
