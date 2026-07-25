# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""3D cubic voxel-grid world (Slice 4a).

Node convention: ``(level, row, col)``; level 0 is a solid GROUND plane
and levels increase upward. The surface (place/remove/is_empty/copy/
progress) is identical to the 2D :class:`sim.world.World` so the planner,
choreographer, and swarm run unchanged — they index occupancy/blueprint
with opaque node tuples and never look inside them.

Lattice decision (docs/DESIGN.md, lattice decisions): a simple cubic
voxel grid, faithful to ARMADAS at the choreography level — cuboct voxels
are assembled in a cubic array (voxel centers form a cubic lattice; the
cuboct shape is parts/physics-level detail, not coordination-level).
"""

from __future__ import annotations

import numpy as np

from .world import DEFECT, EMPTY, GROUND, VOXEL

Cell3 = tuple[int, int, int]  # (level, row, col)


class World3D:
    """A bounded 3D voxel grid with occupancy and a build blueprint."""

    def __init__(self, levels: int, rows: int, cols: int) -> None:
        self.levels = levels
        self.rows = rows
        self.cols = cols
        self.occupancy = np.full((levels, rows, cols), EMPTY, dtype=np.int8)
        self.occupancy[0, :, :] = GROUND
        self.blueprint = np.zeros((levels, rows, cols), dtype=bool)

    def copy(self) -> "World3D":
        """Independent deep copy — scratch worlds for validation/search."""
        clone = World3D.__new__(World3D)
        clone.levels = self.levels
        clone.rows = self.rows
        clone.cols = self.cols
        clone.occupancy = self.occupancy.copy()
        clone.blueprint = self.blueprint.copy()
        return clone

    # -- blueprint ---------------------------------------------------------

    def set_box_blueprint(
        self,
        width: int,
        depth: int,
        height: int,
        left: int,
        front: int,
        hollow: bool = False,
    ) -> None:
        """Blueprint a ``width`` (cols) x ``depth`` (rows) x ``height``
        (levels) box resting on the ground plane, its near-left-bottom
        corner at column ``left``, row ``front``, level 1.

        ``hollow`` keeps only the shell (walls + roof; the ground plane
        is the floor) — the interesting multi-robot case: interior cells
        are traversable during the build and get sealed by the roof.
        """
        self.blueprint[:] = False
        self.blueprint[
            1 : 1 + height, front : front + depth, left : left + width
        ] = True
        if hollow and width > 2 and depth > 2 and height > 1:
            self.blueprint[
                1 : height, front + 1 : front + depth - 1, left + 1 : left + width - 1
            ] = False

    # -- occupancy ---------------------------------------------------------

    def is_empty(self, cell: Cell3) -> bool:
        return self.occupancy[cell] == EMPTY

    def place_voxel(self, cell: Cell3, defective: bool = False) -> None:
        """Put a voxel in an empty cell. Raises if the cell is taken."""
        if not self.is_empty(cell):
            raise ValueError(f"cell {cell} is not empty")
        self.occupancy[cell] = DEFECT if defective else VOXEL

    def is_defective(self, cell: Cell3) -> bool:
        return self.occupancy[cell] == DEFECT

    def remove_voxel(self, cell: Cell3) -> None:
        """Take a voxel (good or defective) back out of the lattice."""
        if self.occupancy[cell] not in (VOXEL, DEFECT):
            raise ValueError(f"cell {cell} holds no voxel")
        self.occupancy[cell] = EMPTY

    # -- progress ----------------------------------------------------------

    @property
    def built_count(self) -> int:
        return int(np.count_nonzero(self.blueprint & (self.occupancy == VOXEL)))

    @property
    def defect_count(self) -> int:
        return int(np.count_nonzero(self.occupancy == DEFECT))

    @property
    def blueprint_count(self) -> int:
        return int(np.count_nonzero(self.blueprint))

    @property
    def complete(self) -> bool:
        return self.built_count == self.blueprint_count
