#!/usr/bin/env python3
# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Stills of joint strain and utilization. Deformation is off.

The boxes are the viewer's 0.96 cubes on pitch-1 cells, so neighbors
keep a gap. k = mass = g = 1. Capacities are the finished demand of
the colored structure, in units of mass·g, so the worst joint of a
finished build sits at utilization 1. The one-sided deck uses the
finished bridge's capacities, and its scale is shared with that bridge
because both frames are in one log. The ARMADAS newton capacity is
not on these pictures.

    python scripts/render_joint_stills.py

Writes the PNGs under docs/ and, when that directory exists, under
/opt/cursor/artifacts/. The logs are runs/joint_stills_*.json
(gitignored).
"""

from __future__ import annotations

import http.server
import shutil
import socket
import sys
import threading
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from experiments.joint_limits import bridge_one_sided, peak_forces
from experiments.stability_cost import build_bridge
from sim.armadas import shelter_cells
from sim.geometry3d import CubicLattice3D
from sim.metrics import RunLog, StructureLog
from sim.stability import JointStrength, StabilityPolicy
from sim.world3d import World3D


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def _serve(port: int) -> http.server.ThreadingHTTPServer:
    handler = lambda *args, **kwargs: http.server.SimpleHTTPRequestHandler(  # noqa: E731
        *args, directory=str(ROOT), **kwargs
    )
    httpd = http.server.ThreadingHTTPServer(("127.0.0.1", port), handler)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    return httpd


def _world(cells: tuple[tuple[int, int, int], ...]) -> World3D:
    levels = max(cell[0] for cell in cells) + 2
    rows = max(cell[1] for cell in cells) + 3
    cols = max(cell[2] for cell in cells) + 3
    world = World3D(levels, rows, cols)
    for cell in cells:
        world.blueprint[cell] = True
    return world


def _occupy(world: World3D, cells) -> World3D:
    filled = world.copy()
    for cell in cells:
        if filled.occupancy[cell] != 0:
            continue
        filled.place_voxel(tuple(int(x) for x in cell))
    return filled


def _log(path: Path, policy: StabilityPolicy, strength: JointStrength, frames) -> None:
    """``frames`` is a list of (world, cells-to-place). Tick 0 is empty."""
    spec = StructureLog(
        policy,
        k=1.0,
        mass=1.0,
        g=1.0,
        pitch=1.0,
        every=1,
        strength=strength,
        safety_factor=1.0,
    )
    log = RunLog(structure=spec)
    log.log_tick(0, frames[0][0])
    for index, (world, _cells) in enumerate(frames[1:], start=1):
        log.log_tick(index, world)
        if "structure" not in log.frames[-1]:
            raise RuntimeError(f"{path.name} frame {index} has no structure sample")
    path.parent.mkdir(parents=True, exist_ok=True)
    log.save(path)


def _clamp_policy(root: tuple[int, int, int]) -> StabilityPolicy:
    level, row, col = root
    return StabilityPolicy(
        up_axis=0,
        up_sign=1,
        ground_plane=None,
        base=((level - 1, row, col),),
        max_cantilever=None,
    )


def _chain(n_free: int, *, horizontal: bool) -> tuple[tuple[int, int, int], ...]:
    root = (1, 2, 2)
    cells = [root]
    if horizontal:
        cells += [(1, 2, 2 + i) for i in range(1, n_free + 1)]
    else:
        cells += [(1 + i, 2, 2) for i in range(1, n_free + 1)]
    return tuple(cells)


def build_logs() -> list[dict]:
    shots = []

    arm = _chain(6, horizontal=True)
    arm_world = _world(arm)
    arm_full = _occupy(arm_world, arm)
    arm_policy = _clamp_policy(arm[0])
    arm_forces = peak_forces(arm, arm_policy)
    _log(
        ROOT / "runs" / "joint_stills_cantilever.json",
        arm_policy,
        JointStrength(shear=arm_forces["shear"]),
        [(arm_world, ()), (arm_full, arm)],
    )
    shots.append(
        {
            "file": "joint_cantilever_shear.png",
            "log": "runs/joint_stills_cantilever.json",
            "frame": 1,
            "color": "shear",
            "theta": 0.0,
            "phi": 1.25,
        }
    )
    shots.append(
        {
            "file": "joint_cantilever_utilization.png",
            "log": "runs/joint_stills_cantilever.json",
            "frame": 1,
            "color": "utilization",
            "theta": 0.0,
            "phi": 1.25,
        }
    )

    tower = _chain(6, horizontal=False)
    tower_world = _world(tower)
    tower_full = _occupy(tower_world, tower)
    tower_policy = _clamp_policy(tower[0])
    tower_forces = peak_forces(tower, tower_policy)
    _log(
        ROOT / "runs" / "joint_stills_tower.json",
        tower_policy,
        JointStrength(axial_compression=tower_forces["compression"]),
        [(tower_world, ()), (tower_full, tower)],
    )
    shots.append(
        {
            "file": "joint_tower_axial.png",
            "log": "runs/joint_stills_tower.json",
            "frame": 1,
            "color": "axial",
            "theta": 0.35,
            "phi": 1.45,
            "radius": 7,
        }
    )
    shots.append(
        {
            "file": "joint_tower_utilization.png",
            "log": "runs/joint_stills_tower.json",
            "frame": 1,
            "color": "utilization",
            "theta": 0.35,
            "phi": 1.45,
            "radius": 7,
        }
    )

    bridge, _depot = build_bridge(5)
    policy = CubicLattice3D(bridge).stability_policy(0)
    finished_cells = tuple(
        (int(level), int(row), int(col))
        for level, row, col in zip(*bridge.blueprint.nonzero())
    )
    finished = _occupy(bridge, finished_cells)
    one_sided_cells = bridge_one_sided(5)
    one_sided = _occupy(bridge, one_sided_cells)
    forces = peak_forces(finished_cells, policy)
    _log(
        ROOT / "runs" / "joint_stills_bridge.json",
        policy,
        JointStrength(
            axial_compression=forces["compression"],
            shear=forces["shear"],
        ),
        [(bridge, ()), (finished, finished_cells), (one_sided, one_sided_cells)],
    )
    shots.append(
        {
            "file": "joint_bridge_utilization.png",
            "log": "runs/joint_stills_bridge.json",
            "frame": 1,
            "color": "utilization",
            "theta": 0.4,
            "phi": 1.42,
            "radius": 9,
        }
    )
    shots.append(
        {
            "file": "joint_bridge_onesided_utilization.png",
            "log": "runs/joint_stills_bridge.json",
            "frame": 2,
            "color": "utilization",
            "theta": 0.4,
            "phi": 1.42,
            "radius": 9,
        }
    )

    shed = shelter_cells()
    shed_world = _world(shed)
    for cell in shed:
        shed_world.blueprint[cell] = True
    shed_full = _occupy(shed_world, shed)
    shed_policy = StabilityPolicy(
        up_axis=0, up_sign=1, ground_plane=0, max_cantilever=None
    )
    shed_forces = peak_forces(shed, shed_policy)
    _log(
        ROOT / "runs" / "joint_stills_shelter.json",
        shed_policy,
        JointStrength(
            axial_compression=shed_forces["compression"],
            shear=shed_forces["shear"],
        ),
        [(shed_world, ()), (shed_full, shed)],
    )
    shots.append(
        {
            "file": "joint_shelter_utilization.png",
            "log": "runs/joint_stills_shelter.json",
            "frame": 1,
            "color": "utilization",
            "theta": 0.7,
            "phi": 1.2,
            "radius": 16,
        }
    )
    return shots


def shoot(shots: list[dict], destinations: list[Path]) -> None:
    from playwright.sync_api import sync_playwright

    port = _free_port()
    httpd = _serve(port)
    base = f"http://127.0.0.1:{port}/replay_viewer.html"
    try:
        with sync_playwright() as playwright:
            browser = playwright.chromium.launch(
                executable_path=shutil.which("google-chrome")
                or shutil.which("google-chrome-stable")
                or shutil.which("chromium")
                or shutil.which("chromium-browser"),
                args=["--no-sandbox", "--disable-dev-shm-usage"],
            )
            page = browser.new_page(viewport={"width": 1280, "height": 720})
            for shot in shots:
                radius = shot.get("radius")
                radius_q = f"&radius={radius}" if radius is not None else ""
                url = (
                    f"{base}?log={shot['log']}&frame={shot['frame']}"
                    f"&color={shot['color']}&deform=0"
                    f"&theta={shot['theta']}&phi={shot['phi']}{radius_q}"
                )
                page.goto(url, wait_until="domcontentloaded")
                page.wait_for_function(
                    "() => window.assemblerReplay && window.assemblerReplay.ready() "
                    "&& window.assemblerReplay.nFrames() > 0",
                    timeout=45_000,
                )
                page.evaluate(
                    """async ({frame, color}) => {
                        const v = window.assemblerReplay;
                        v.setPlaying(false);
                        v.setColor(color);
                        v.setDeform(0);
                        v.setFrame(frame);
                        await new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)));
                    }""",
                    {"frame": shot["frame"], "color": shot["color"]},
                )
                png = page.screenshot()
                for directory in destinations:
                    directory.mkdir(parents=True, exist_ok=True)
                    (directory / shot["file"]).write_bytes(png)
                print(shot["file"], flush=True)
            browser.close()
    finally:
        httpd.shutdown()


def main() -> None:
    shots = build_logs()
    destinations = [ROOT / "docs", Path("/opt/cursor/artifacts")]
    shoot(shots, destinations)
    print("wrote stills", flush=True)


if __name__ == "__main__":
    main()
