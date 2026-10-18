# Changelog

The package version in `pyproject.toml` and `sim/__init__.py` stays **0.1.0** until a tag. v0.2.0 is the structural work on `main`. It is not tagged. Tag it after a physical calibration Joshua accepts (`docs/VALIDATION.md`). The default build, with no `StabilityPolicy`, is unchanged.

## [0.2.0] — Unreleased

Do not tag this until the printed-cantilever log in `docs/VALIDATION.md` has a round whose diagnosis Joshua has signed, and the stiffness used in a released sag number is that fit. The results table is empty on purpose.

### Added

- Optional structural stability gate (`sim/stability.py`), off unless a caller passes a `StabilityPolicy`. `is_stable` requires every voxel to face-connect to a column on the base, and it limits horizontal overhang. A refusal is the existing kickback. Deeds are not supports.
- Small-displacement sag estimate: voxels as nodes, one lumped axial/shear spring per face, `Ku = mg`. Closed forms for a shear-chain cantilever and an axial tower (`shear_cantilever_tip_sag`, `axial_tower_compression`, `path_chain_deflection_at`). Not a gate, and not inside the search.
- Opt-in strain log (`StructureLog` on a `RunLog`) and replay coloring: displacement or joint strain, scale slider, worst-sag marker. Logs without samples play as before. Colors are the model. Default `k = 1` is not a fit.
- Gate campaign, 312 trials (`python main.py stability`, `docs/stability_cost.csv`). On the span-5 bridge, `max_cantilever` 2 cuts peak intermediate sag from a median of 15 (range 10–15, n = 6) to 6, and pays makespan and kickbacks. Stiffness in that file is `k = 1`.
- Optional cuboctahedral solid on the cubic grid (`sim/cuboct.py`). Same six face-neighbors and the same lumped springs, so the closed forms still hold. Different vertices and square-face joint sites. Cubic stays the default. Not a mechanical model of ARMADAS hardware.
- Finished-structure stability precheck (`necessary_stability_precheck`). Overhang in the finished set lower-bounds every prefix. The hollow 3×3×2 roof at cantilever limit 0 is a proved rejection (6/6, median 0.0965 ms) instead of an undecided 200-node search.
- Physical validation plan and empty log: `docs/VALIDATION.md`. One-piece PLA cantilevers, n = 3 and n = 6 free bays. Predicted self-weight tip ratio 3.5 (shear chain) against 16 (Euler–Bernoulli `n⁴`). No measured rows.
- Headless viewer capture: `scripts/render_demo.py` (Playwright + ffmpeg). About 60 seconds, two-robot bridge with displacement coloring and a display scale above the still, then a cuboct block, with title cards. The mp4 is a generated artifact, not a committed blob.

### Changed

- `python main.py stability` counts a finished structure past the cantilever limit as infeasible from the precheck. The campaign in `docs/stability_cost.csv` was re-run: 202 feasible, 110 infeasible, 0 undecided. Build metrics on the trials that still run are the same comparison the gate-cost write-up reports.
- Replay viewer draws cuboctahedra when `meta.voxel_shape` is `cuboctahedral`, and exposes `window.assemblerReplay` for the capture script.

### Tests

168 tests (`python -m unittest`): traps, 3D repair, fuzz, examples, the structural screen, the precheck, strain logging, the stability-gate smoke run, and the cuboctahedral geometry.

## [0.1.0] — 2026-07-25

Coordination stack: validated build orders, multi-robot choreography, mid-build repair, 3D cubic lattice, motion as configuration (reach, stride, climb, coupled pairs), trap suite, replayable logs. Structural checks did not exist. Default behavior of this tag is what v0.2 keeps when the gate is left off.
