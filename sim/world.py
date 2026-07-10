# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Grid, occupancy, and blueprint for the discrete assembly simulator.

The world is a 2D numpy grid of cells. Convention (numpy-native):

- Indexing is ``(row, col)``. Row 0 is the TOP of the printed frame; the
  bottom row (``rows - 1``) is the ground.
- ``occupancy`` holds what exists: ``EMPTY``, ``GROUND``, or ``VOXEL``.
- ``blueprint`` is a boolean mask of cells that should end up holding a voxel.

Slice 0 only needs "place a voxel, check progress"; later slices add robots
walking on the ground row + built structure.
"""

from __future__ import annotations

import numpy as np

Cell = tuple[int, int]  # (row, col)

EMPTY = 0
GROUND = 1
VOXEL = 2
DEFECT = 3  # voxel placed in the right cell but badly bonded (Slice 2)


class World:
    """A bounded 2D grid with occupancy and a build blueprint."""

    def __init__(self, rows: int, cols: int) -> None:
        self.rows = rows
        self.cols = cols
        self.occupancy = np.full((rows, cols), EMPTY, dtype=np.int8)
        self.occupancy[rows - 1, :] = GROUND
        self.blueprint = np.zeros((rows, cols), dtype=bool)

    # -- blueprint ---------------------------------------------------------

    def set_wall_blueprint(self, width: int, height: int, left: int) -> None:
        """Blueprint a ``width`` x ``height`` wall resting on the ground row.

        ``left`` is the column of the wall's left edge. The wall occupies the
        ``height`` rows directly above the ground row.
        """
        bottom = self.rows - 1  # ground row; wall sits on top of it
        self.blueprint[:] = False
        self.blueprint[bottom - height : bottom, left : left + width] = True

    def blueprint_cells(self) -> list[Cell]:
        """Blueprint cells in row-major order, bottom row first.

        Bottom-up ordering means Slice 0's fill loop never creates a floating
        voxel; real build-order logic arrives with the planner in Slice 1.
        """
        rows, cols = np.nonzero(self.blueprint)
        cells = [(int(r), int(c)) for r, c in zip(rows, cols)]
        cells.sort(key=lambda rc: (-rc[0], rc[1]))  # lowest rows first
        return cells

    # -- occupancy ---------------------------------------------------------

    def is_empty(self, cell: Cell) -> bool:
        return self.occupancy[cell] == EMPTY

    def place_voxel(self, cell: Cell, defective: bool = False) -> None:
        """Put a voxel in an empty cell. Raises if the cell is taken.

        A ``defective`` voxel occupies the cell and is crawlable like any
        solid, but does not count toward completion — it must be removed
        and replaced. Whether it is defective is only observable via
        :meth:`is_defective` (i.e. by inspecting).
        """
        if not self.is_empty(cell):
            raise ValueError(f"cell {cell} is not empty")
        self.occupancy[cell] = DEFECT if defective else VOXEL

    def is_defective(self, cell: Cell) -> bool:
        return self.occupancy[cell] == DEFECT

    def remove_voxel(self, cell: Cell) -> None:
        """Take a voxel (good or defective) back out of the lattice."""
        if self.occupancy[cell] not in (VOXEL, DEFECT):
            raise ValueError(f"cell {cell} holds no voxel")
        self.occupancy[cell] = EMPTY

    # -- progress ----------------------------------------------------------

    @property
    def built_count(self) -> int:
        """Number of blueprint cells holding a GOOD voxel (defects excluded)."""
        return int(np.count_nonzero(self.blueprint & (self.occupancy == VOXEL)))

    @property
    def defect_count(self) -> int:
        """Number of cells currently holding a defective voxel."""
        return int(np.count_nonzero(self.occupancy == DEFECT))

    @property
    def blueprint_count(self) -> int:
        return int(np.count_nonzero(self.blueprint))

    @property
    def complete(self) -> bool:
        return self.built_count == self.blueprint_count
