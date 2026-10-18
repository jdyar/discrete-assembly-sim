#!/usr/bin/env python3
# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Stills of a span-5 inchworm bridge with temporary scaffold.

Material color, deformation off. One frame while both yellow supports
are standing under the finished deck, one after they have been removed,
and one in displacement color while they are still up (the supports stay
yellow; the blueprint voxels take the strain ramp). k = 1 is not a fit.

    python scripts/render_scaffold_stills.py

Writes docs/scaffold_standing.png, docs/scaffold_removed.png,
docs/scaffold_displacement.png, and the same files under
/opt/cursor/artifacts/ when that directory exists. The run log is
runs/scaffold_span5.json (gitignored).
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

from experiments.scaffold_campaign import plan_span, stance_starts
from sim.metrics import RunLog, StructureLog
from sim.swarm import Swarm
from sim.world import SCAFFOLD


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


def build_log(path: Path) -> dict:
    case = plan_span(5, "inchworm")
    plan = case["plan"]
    if plan is None or plan.scaffold_count < 1:
        raise RuntimeError(
            f"span-5 inchworm produced no scaffold ({case['status']} {case['detail']})"
        )
    world = case["world"]
    start = stance_starts(case["start_pool"], 1, 0)[0]
    swarm = Swarm(
        world,
        case["depot"],
        [start],
        defect_p=0.0,
        inspect_enabled=True,
        geometry_factory=case["factory"],
        stability=case["policy"],
        sag=case["sag"],
        operations=plan.operations(),
        scaffold_cells=plan.scaffold,
    )
    policy = case["policy"]
    log = RunLog(structure=StructureLog(policy, k=1.0, mass=1.0, g=1.0, pitch=1.0, every=1))
    log.log_tick(0, world, robots=swarm.robots)
    while not swarm.done and swarm.tick_count < 800:
        swarm.tick()
        log.log_tick(swarm.tick_count, world, robots=swarm.robots)
    if not swarm.done or not world.complete:
        raise RuntimeError(
            f"span-5 inchworm did not finish: ticks={swarm.tick_count} "
            f"kickbacks={swarm.kickbacks}"
        )
    if any(world.occupancy[cell] == SCAFFOLD for cell in plan.scaffold):
        raise RuntimeError("scaffold still standing at the end")
    path.parent.mkdir(parents=True, exist_ok=True)
    log.save(path)
    # Last frame that still has every support up. A later frame can
    # keep one cell while the other is already gone.
    needed = list(plan.scaffold)
    standing = None
    for index, frame in enumerate(log.frames):
        occ = frame["occupancy"]
        if all(occ[cell[0]][cell[1]][cell[2]] == SCAFFOLD for cell in needed):
            standing = index
    if standing is None:
        raise RuntimeError("no frame has every scaffold cell standing")
    if log.frames[standing]["placed"] != world.blueprint_count:
        raise RuntimeError(
            "scaffold frame is not the finished deck "
            f"(placed {log.frames[standing]['placed']})"
        )
    return {
        "standing": standing,
        "last": len(log.frames) - 1,
        "scaffold": list(plan.scaffold),
        "ticks": swarm.tick_count,
        "max_sag": case["sag"].max_sag,
        "max_cantilever": case["policy"].max_cantilever,
    }


def shoot(log_name: str, facts: dict, destinations: list[Path]) -> None:
    from playwright.sync_api import sync_playwright

    port = _free_port()
    httpd = _serve(port)
    base = f"http://127.0.0.1:{port}/replay_viewer.html"
    shots = [
        ("scaffold_standing.png", facts["standing"], "material", 0),
        ("scaffold_removed.png", facts["last"], "material", 0),
        ("scaffold_displacement.png", facts["standing"], "displacement", 0),
    ]
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
            for name, frame, color, deform in shots:
                url = (
                    f"{base}?log={log_name}&frame={frame}&color={color}"
                    f"&deform={deform}&theta=0.18&phi=1.32&radius=11"
                )
                page.goto(url, wait_until="domcontentloaded")
                page.wait_for_function(
                    "() => window.assemblerReplay && window.assemblerReplay.ready() "
                    "&& window.assemblerReplay.nFrames() > 0",
                    timeout=45_000,
                )
                page.evaluate(
                    """async ({frame, color, deform}) => {
                        const v = window.assemblerReplay;
                        v.setPlaying(false);
                        v.setColor(color);
                        v.setDeform(deform);
                        v.setFrame(frame);
                        await new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)));
                    }""",
                    {"frame": frame, "color": color, "deform": deform},
                )
                png = page.screenshot()
                for directory in destinations:
                    directory.mkdir(parents=True, exist_ok=True)
                    (directory / name).write_bytes(png)
            browser.close()
    finally:
        httpd.shutdown()


def main() -> None:
    log_path = ROOT / "runs" / "scaffold_span5.json"
    facts = build_log(log_path)
    print(facts, flush=True)
    destinations = [ROOT / "docs", Path("/opt/cursor/artifacts")]
    shoot("runs/scaffold_span5.json", facts, destinations)
    print("wrote stills", flush=True)


if __name__ == "__main__":
    main()
