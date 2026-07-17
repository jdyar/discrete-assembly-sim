# Contributing

Thanks for looking. This project aims to be the open, legally-reusable
coordination layer for discrete lattice assembly — contributions that
sharpen that (bugs, traps, geometries, provenance corrections) matter
more than features.

## Setup

```bash
git clone https://github.com/jdyar/discrete-assembly-sim.git && cd discrete-assembly-sim
python -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -r requirements.txt  # numpy + matplotlib, nothing else
python -m unittest               # 82 tests, ~10s — all green before and after your change
```

No build step, no linter config to fight. Python 3.10+.

## Ground rules

1. **All tests green, always.** The 2D suite is the regression base for
   the 3D work — nothing may break it.
2. **The coordination stack never learns about lattices.** If your
   change adds coordinate math, row/col/level tuples, or adjacency
   assumptions to `swarm.py`, `choreographer.py`, `reservations.py`,
   `texgraph.py`, or `planner.py`, it will be asked to move behind the
   `Geometry`/`MotionModel` seam instead. That invariance is the
   project's core claim (see [docs/DESIGN.md](docs/DESIGN.md)).
3. **Clean-room provenance.** Implementations come from published
   papers, cited in docstrings. Do not port code from unlicensed
   repositories — if you've read one recently, say so in the PR and
   we'll figure out the right distance. MIT-licensed sources are fine
   with attribution.
4. **Traps are spec-first.** Adversarial fixtures are written against
   the published failure class and the API contract, ideally before the
   code that beats them. A trap PR that currently FAILS is a great PR —
   mark it `expectedFailure` and describe the failure class in the
   docstring like the existing fixtures do.
5. **Every behavior change shows its work**: a run log the replay
   viewer can load, or a chart, in the PR description.

## Good first contributions

- **Author a trap** (`tests/test_traps3d.py` style): a blueprint +
  start placement you believe deadlocks, starves, entombs, or strands
  the build. Cite the failure class if it has one.
- **A new `Geometry`**: hex lattice, cuboct strut-level, your lab's
  lattice. The triangle-lattice fake in `tests/test_reservations.py`
  shows the minimum contract; ~150 lines gets you a real one.
- **Performance**: the buildability gate re-runs the build-order search
  per gated placement — profiled hot path, memoization and incremental
  connectivity (union-find) are both open ideas.
- **Congestion experiments**: extend `speedup3d` past N=3, find the
  knee, chart it.
- **Viewer**: camera presets, per-robot trails, reservation-table
  overlay (show leases as ghosts through time).

## PR checklist

- [ ] `python -m unittest` green
- [ ] new behavior has a test (traps count)
- [ ] coordination modules still lattice-agnostic
- [ ] docstrings cite sources for any algorithmic claim
- [ ] run log or chart attached if behavior changed

Questions, half-formed ideas, "is this in scope?" — open a
[Discussion](../../discussions). Bugs and concrete proposals —
[Issues](../../issues).
