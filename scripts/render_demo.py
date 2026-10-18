#!/usr/bin/env python3
# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Headless ~60 s capture of the replay viewer.

Two clips, with title cards:

1. Two robots building the two-pier bridge. Voxels are colored by the
   spring-network displacement and drawn on the cells where they were
   placed. Deformation stays off for that clip, so the deck remains a
   span. k = 1. The number on the title card is read from the log this
   script just wrote. A few seconds at the end turn the slider to 0.20
   of a voxel pitch (exaggerated and normalized, not mg/k lengths).
2. The 2×2×2 cuboct block (``python main.py cuboct``'s world). Display
   solid only. No stiffness.

Requires the repo dependencies (numpy), plus Playwright, ffmpeg, and
Google Chrome. Three.js is loaded from the CDN the viewer already uses,
so the capture needs network on first load.

    pip install playwright
    python scripts/render_demo.py
    python scripts/render_demo.py --mp4 /opt/cursor/artifacts/v02_demo.mp4 \\
        --gif docs/demo_bridge.gif

The mp4 is a generated artifact. A GIF is written only when the compressed
file is at or under ``--gif-max-bytes`` (default 1.5 MB). Logs go to
``runs/``, which is gitignored.

This is a picture of the model. It is not a measurement on a printed part.
"""

from __future__ import annotations

import argparse
import http.server
import shutil
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

# Slider fraction of the voxel pitch. 0 draws the as-built cells.
# 0.20 is the viewer's mild exaggeration: the longest arrow on screen
# is 0.20 pitches, whatever the mg/k magnitude is. The cap in the
# viewer is 0.40. The build clip stays at 0 so the span reads.
DEFORM = 0.0
DEFORM_HOLD = 0.20
FPS = 10
WIDTH = 1280
HEIGHT = 720
# 4s + 36s + 4s + 12s + 4s = 60s at FPS.
TITLE_S = 4.0
BRIDGE_S = 36.0
CUBOCT_TITLE_S = 4.0
CUBOCT_S = 12.0
END_S = 4.0
# Side view of the span (X across the frame). theta is a little off
# the pure profile so both deck rows read; phi stays low enough that
# the air under the deck is in frame. Radius frames the blueprint,
# not the empty ground around it.
BRIDGE_PHI = 1.35
BRIDGE_RADIUS = 9.2
BRIDGE_THETA0 = 0.06
BRIDGE_THETA1 = 0.22


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


def _build_logs() -> dict:
    """Write the two logs the recording plays. Return sag facts for the titles."""
    from main import (
        BRIDGE_CANTILEVER,
        BRIDGE_DEPOT,
        build_bridge_world,
        run_bridge,
        run_cuboct,
        run_swarm3d_build,
    )
    from sim.cuboct import VOXEL_SHAPE_CUBOCTAHEDRAL
    from sim.geometry3d import CubicLattice3D
    from sim.metrics import StructureLog

    runs = ROOT / "runs"
    runs.mkdir(exist_ok=True)

    world = build_bridge_world()
    policy = CubicLattice3D(world).stability_policy(BRIDGE_CANTILEVER)
    structure = StructureLog(policy, k=1.0, mass=1.0, g=1.0, pitch=1.0, every=1)
    # Second start is another ground-level footing, opposite the first,
    # not on the depot and not on a pier cell.
    log, swarm = run_swarm3d_build(
        world,
        n_robots=2,
        depot=BRIDGE_DEPOT,
        starts=[(1, 5, 12), (1, 1, 12)],
        stability=policy,
        structure=structure,
        verbose=False,
        max_ticks=8000,
    )
    if not world.complete:
        raise RuntimeError(
            f"two-robot bridge did not finish: {world.built_count}/{world.blueprint_count} "
            f"ticks={swarm.tick_count} kickbacks={swarm.kickbacks}"
        )
    bridge_path = runs / "demo_bridge.json"
    log.save(bridge_path)
    samples = [
        f["structure"]
        for f in log.frames
        if "structure" in f and "error" not in f["structure"]
    ]
    if not samples:
        raise RuntimeError("bridge log has no structural samples")
    peaks = [float(s["worst_sag"]) for s in samples]
    final = samples[-1]

    one_log, _one_swarm = run_bridge(verbose=False)
    cuboct_log, cuboct_swarm = run_cuboct(verbose=False)
    if cuboct_log.meta.get("voxel_shape") != VOXEL_SHAPE_CUBOCTAHEDRAL:
        raise RuntimeError("cuboct log did not record voxel_shape")
    cuboct_path = runs / "demo_cuboct.json"
    cuboct_log.save(cuboct_path)
    return {
        "bridge_frames": len(log.frames),
        "bridge_ticks": swarm.tick_count,
        "bridge_kickbacks": swarm.kickbacks,
        "final_sag": float(final["worst_sag"]),
        "final_cell": final.get("worst_sag_cell"),
        "peak_sag": max(peaks),
        "one_frames": len(one_log.frames),
        "cuboct_frames": len(cuboct_log.frames),
        "cuboct_ticks": cuboct_swarm.tick_count,
        "deform": DEFORM,
        "deform_hold": DEFORM_HOLD,
    }


def _card(title: str, lines: list[str]) -> str:
    body = "".join(f"<p>{line}</p>" for line in lines)
    return f"""<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8" />
<style>
  html, body {{ margin: 0; height: 100%; background: #0b1020; color: #e2e8f0;
    font: 28px/1.45 "Segoe UI", Helvetica, Arial, sans-serif; }}
  main {{ box-sizing: border-box; height: 100%; padding: 88px 96px; }}
  h1 {{ font-size: 54px; font-weight: 650; margin: 0 0 28px; letter-spacing: -0.02em; }}
  p {{ margin: 0 0 14px; max-width: 980px; color: #cbd5e1; }}
  p.strong {{ color: #f8fafc; }}
</style></head>
<body><main><h1>{title}</h1>{body}</main></body></html>"""


def _open_log(page, url: str) -> None:
    page.goto(url, wait_until="domcontentloaded")
    page.wait_for_function(
        "() => window.assemblerReplay && window.assemblerReplay.ready() "
        "&& window.assemblerReplay.nFrames() > 0",
        timeout=45_000,
    )


def _pose(page, *, sim: int, color: str | None, deform: float | None,
          theta: float, phi: float, radius: float) -> None:
    page.evaluate(
        """async ({sim, color, deform, theta, phi, radius}) => {
            const v = window.assemblerReplay;
            v.setPlaying(false);
            if (color) v.setColor(color);
            if (deform != null) v.setDeform(deform);
            v.setOrbit(theta, phi, radius);
            v.setFrame(sim);
            await new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)));
        }""",
        {
            "sim": sim,
            "color": color,
            "deform": deform,
            "theta": theta,
            "phi": phi,
            "radius": radius,
        },
    )


def _shot_cards(page, directory: Path, prefix: str, html: str, n: int) -> None:
    page.set_content(html, wait_until="load")
    page.screenshot(path=str(directory / f"{prefix}0000.png"))
    src = directory / f"{prefix}0000.png"
    for i in range(1, n):
        dest = directory / f"{prefix}{i:04d}.png"
        if not dest.exists():
            dest.write_bytes(src.read_bytes())


def _sim_index(i: int, n_video: int, n_sim: int, finish_by: float) -> int:
    """Map a video frame to a sim frame. finish_by < 1 holds the last pose."""
    if n_sim <= 1 or n_video <= 1:
        return 0
    if finish_by >= 1:
        return min(n_sim - 1, round(i / (n_video - 1) * (n_sim - 1)))
    cut = finish_by * (n_video - 1)
    if i >= cut:
        return n_sim - 1
    return min(n_sim - 1, int(round(i / cut * (n_sim - 1))))


def _drive(page, url: str, n_video: int, n_sim: int, prefix: str, directory: Path,
           color: str | None, deform: float | None, *,
           finish_by: float, phi: float, radius: float, theta0: float, theta1: float) -> None:
    page.goto(url, wait_until="domcontentloaded")
    page.wait_for_function(
        "() => window.assemblerReplay && window.assemblerReplay.ready() "
        "&& window.assemblerReplay.nFrames() > 0",
        timeout=45_000,
    )
    page.evaluate("() => window.assemblerReplay.setPlaying(false)")
    for i in range(n_video):
        sim = _sim_index(i, n_video, n_sim, finish_by)
        t = i / max(1, n_video - 1)
        theta = theta0 + (theta1 - theta0) * t
        page.evaluate(
            """async ({sim, color, deform, theta, phi, radius}) => {
                const v = window.assemblerReplay;
                if (color) v.setColor(color);
                if (deform != null) v.setDeform(deform);
                v.setOrbit(theta, phi, radius);
                v.setFrame(sim);
                await new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)));
            }""",
            {
                "sim": sim,
                "color": color,
                "deform": deform,
                "theta": theta,
                "phi": phi,
                "radius": radius,
            },
        )
        page.screenshot(path=str(directory / f"{prefix}{i:04d}.png"))


def _ffmpeg(args: list[str]) -> None:
    proc = subprocess.run(args, capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(
            "ffmpeg failed\n" + proc.stdout[-2000:] + "\n" + proc.stderr[-2000:]
        )


def _encode(directory: Path, mp4: Path, gif: Path | None, gif_max: int) -> str:
    mp4.parent.mkdir(parents=True, exist_ok=True)
    sequence = directory / "all_%04d.png"
    # Pieces were written as separate prefixes. Concatenate in story order.
    pieces = []
    for prefix in ("t1_", "br_", "t2_", "cu_", "t3_"):
        pieces.extend(sorted(directory.glob(f"{prefix}*.png")))
    if not pieces:
        raise RuntimeError("no frames captured")
    for i, src in enumerate(pieces):
        dest = directory / f"all_{i:04d}.png"
        if dest.resolve() != src.resolve():
            dest.write_bytes(src.read_bytes())
    _ffmpeg(
        [
            "ffmpeg", "-y", "-framerate", str(FPS),
            "-i", str(directory / "all_%04d.png"),
            "-c:v", "libx264", "-pix_fmt", "yuv420p",
            "-movflags", "+faststart",
            str(mp4),
        ]
    )
    note = f"mp4 {mp4} ({mp4.stat().st_size} bytes, {len(pieces) / FPS:.1f} s)"
    if gif is None:
        return note
    # A short excerpt of the bridge once the deck is colored, not the whole minute.
    gif.parent.mkdir(parents=True, exist_ok=True)
    palette = directory / "palette.png"
    # Finished span while deformation is still off. The exaggerated hold is
    # the last 4 s of the bridge clip, and this excerpt ends before it.
    clip_start = TITLE_S + 26.0
    _ffmpeg(
        [
            "ffmpeg", "-y", "-ss", f"{clip_start:.2f}", "-t", "5",
            "-i", str(mp4),
            "-vf", "fps=8,scale=560:-1:flags=lanczos,palettegen=stats_mode=diff",
            str(palette),
        ]
    )
    _ffmpeg(
        [
            "ffmpeg", "-y", "-ss", f"{clip_start:.2f}", "-t", "5",
            "-i", str(mp4), "-i", str(palette),
            "-lavfi", "fps=8,scale=560:-1:flags=lanczos[x];[x][1:v]paletteuse=dither=bayer",
            str(gif),
        ]
    )
    size = gif.stat().st_size
    if size > gif_max:
        gif.unlink()
        return note + f"; gif {size} bytes exceeded {gif_max}, not kept"
    return note + f"; gif {gif} ({size} bytes)"


def _write_stills(page, base: str, facts: dict, artifact_dir: Path, frames: Path) -> None:
    """Side views of the finished span, plus the one-robot README stills."""
    artifact_dir.mkdir(parents=True, exist_ok=True)
    side = (
        f"{base}?log=runs/demo_bridge.json&frame=last&color=displacement"
        f"&deform=0&theta={BRIDGE_THETA0}&phi={BRIDGE_PHI}&radius={BRIDGE_RADIUS}"
    )
    page.set_viewport_size({"width": WIDTH, "height": HEIGHT})
    _open_log(page, side)
    last = facts["bridge_frames"] - 1
    _pose(
        page, sim=last, color="displacement", deform=0,
        theta=BRIDGE_THETA0, phi=BRIDGE_PHI, radius=BRIDGE_RADIUS,
    )
    page.screenshot(path=str(artifact_dir / "v02_bridge_side.png"))
    _pose(
        page, sim=last, color="displacement", deform=DEFORM_HOLD,
        theta=BRIDGE_THETA0, phi=BRIDGE_PHI, radius=BRIDGE_RADIUS,
    )
    page.screenshot(path=str(artifact_dir / "v02_bridge_normalized.png"))

    one = (
        f"{base}?log=runs/bridge.json&frame=last&color=displacement"
        f"&deform=0&theta={BRIDGE_THETA0}&phi={BRIDGE_PHI}&radius={BRIDGE_RADIUS}"
    )
    page.set_viewport_size({"width": 1400, "height": 900})
    _open_log(page, one)
    one_last = facts["one_frames"] - 1
    _pose(
        page, sim=one_last, color="displacement", deform=0,
        theta=BRIDGE_THETA0, phi=BRIDGE_PHI, radius=BRIDGE_RADIUS,
    )
    still = ROOT / "docs" / "strain_bridge.png"
    page.screenshot(path=str(still))

    page.set_viewport_size({"width": 1100, "height": 700})
    gif_dir = frames / "strain"
    gif_dir.mkdir(exist_ok=True)
    n = facts["one_frames"]
    for i, frac in enumerate((0.15, 0.35, 0.55, 0.75, 1.0)):
        sim = min(n - 1, int(round(frac * (n - 1))))
        _pose(
            page, sim=sim, color="displacement", deform=0,
            theta=BRIDGE_THETA0 + 0.04 * i, phi=BRIDGE_PHI, radius=BRIDGE_RADIUS,
        )
        page.screenshot(path=str(gif_dir / f"st_{i:02d}.png"))
    _ffmpeg(
        [
            "ffmpeg", "-y", "-framerate", "2",
            "-i", str(gif_dir / "st_%02d.png"),
            "-lavfi", "scale=1100:-1:flags=lanczos,split[s0][s1];[s0]palettegen=stats_mode=diff[p];[s1][p]paletteuse=dither=bayer",
            str(ROOT / "docs" / "strain_bridge.gif"),
        ]
    )
    page.set_viewport_size({"width": WIDTH, "height": HEIGHT})
    cuboct = f"{base}?log=runs/demo_cuboct.json&frame=last&shape=cuboctahedral&theta=0.9&phi=1.05&radius=8"
    _open_log(page, cuboct)
    _pose(
        page, sim=facts["cuboct_frames"] - 1, color=None, deform=None,
        theta=0.9, phi=1.05, radius=8,
    )
    page.screenshot(path=str(artifact_dir / "v02_cuboct.png"))


def render(mp4: Path, gif: Path | None, gif_max: int) -> str:
    if shutil.which("ffmpeg") is None:
        raise SystemExit("ffmpeg is not on PATH")
    try:
        from playwright.sync_api import sync_playwright
    except ImportError as exc:
        raise SystemExit(
            "playwright is not installed. pip install playwright  "
            "(Chrome is launched via channel='chrome'; chromium is not downloaded.)"
        ) from exc

    facts = _build_logs()
    port = _free_port()
    httpd = _serve(port)
    frames = mp4.parent / "_demo_frames" if False else Path("/tmp/assembler-demo-frames")
    if frames.exists():
        shutil.rmtree(frames)
    frames.mkdir(parents=True)
    base = f"http://127.0.0.1:{port}/replay_viewer.html"
    n_title = int(TITLE_S * FPS)
    n_bridge = int(BRIDGE_S * FPS)
    n_c2 = int(CUBOCT_TITLE_S * FPS)
    n_cuboct = int(CUBOCT_S * FPS)
    n_end = int(END_S * FPS)
    sag = facts["final_sag"]
    peak = facts["peak_sag"]
    try:
        with sync_playwright() as pw:
            browser = pw.chromium.launch(
                channel="chrome",
                headless=True,
                args=["--no-sandbox", "--disable-dev-shm-usage"],
            )
            page = browser.new_page(viewport={"width": WIDTH, "height": HEIGHT})
            _shot_cards(
                page, frames, "t1_",
                _card(
                    "discrete-assembly-sim",
                    [
                        "<span class='strong'>Two robots build a span.</span> Color is the spring-network displacement.",
                        f"This run: peak sag {peak:.0f} mg/k, finished {sag:.0f} mg/k at k = 1 (unfitted).",
                        "Blocks are drawn where they were placed. Deformation is off. Color is not a measured deflection.",
                    ],
                ),
                n_title,
            )
            # True geometry for most of the clip. The last 4 seconds turn the
            # normalized exaggeration on, still on the finished span.
            hold_at = n_bridge - int(4 * FPS)
            page.goto("about:blank")
            _drive(
                page,
                (
                    f"{base}?log=runs/demo_bridge.json&frame=0&color=displacement"
                    f"&deform={DEFORM}&theta={BRIDGE_THETA0}&phi={BRIDGE_PHI}&radius={BRIDGE_RADIUS}"
                ),
                hold_at,
                facts["bridge_frames"],
                "br_",
                frames,
                color="displacement",
                deform=DEFORM,
                finish_by=0.72,
                phi=BRIDGE_PHI,
                radius=BRIDGE_RADIUS,
                theta0=BRIDGE_THETA0,
                theta1=BRIDGE_THETA1,
            )
            for i in range(hold_at, n_bridge):
                t = i / max(1, n_bridge - 1)
                theta = BRIDGE_THETA0 + (BRIDGE_THETA1 - BRIDGE_THETA0) * t
                page.evaluate(
                    """async ({sim, theta, deform, phi, radius}) => {
                        const v = window.assemblerReplay;
                        v.setColor('displacement');
                        v.setDeform(deform);
                        v.setOrbit(theta, phi, radius);
                        v.setFrame(sim);
                        await new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)));
                    }""",
                    {
                        "sim": facts["bridge_frames"] - 1,
                        "theta": theta,
                        "deform": DEFORM_HOLD,
                        "phi": BRIDGE_PHI,
                        "radius": BRIDGE_RADIUS,
                    },
                )
                page.screenshot(path=str(frames / f"br_{i:04d}.png"))
            _shot_cards(
                page, frames, "t2_",
                _card(
                    "Cuboctahedral voxels",
                    [
                        "Same six face-neighbors as the cube. The viewer draws the solid the log names.",
                        "Joint sites are the corners of each square face. The sag solver still uses one spring at the face center.",
                        "Not a stiffness measurement of ARMADAS hardware.",
                    ],
                ),
                n_c2,
            )
            _drive(
                page,
                f"{base}?log=runs/demo_cuboct.json&frame=0&shape=cuboctahedral&theta=0.5",
                n_cuboct,
                facts["cuboct_frames"],
                "cu_",
                frames,
                color=None,
                deform=None,
                finish_by=0.9,
                phi=1.05,
                radius=8,
                theta0=0.4,
                theta1=1.8,
            )
            _shot_cards(
                page, frames, "t3_",
                _card(
                    "Not a measurement",
                    [
                        "Stiffness is the default k = 1. Colors are model units of mg/k. The blocks were not moved by those numbers.",
                        "The print check (PLA cantilevers, n = 3 and n = 6) is docs/VALIDATION.md. The table is empty.",
                        "v0.2.0 stays untagged until that calibration.",
                    ],
                ),
                n_end,
            )
            _write_stills(page, base, facts, mp4.parent, frames)
            browser.close()
    finally:
        httpd.shutdown()
    return _encode(frames, mp4, gif, gif_max) + (
        f"; bridge ticks={facts['bridge_ticks']} kickbacks={facts['bridge_kickbacks']} "
        f"peak={peak:.4f} final={sag:.4f} at {facts['final_cell']}; "
        f"cuboct ticks={facts['cuboct_ticks']}"
    )


def render_on_lattice(out_dir: Path) -> str:
    """Still frames of an inchworm standing on a level-2 cantilever.

    Displacement color includes the robot's point loads (ratio 3, a
    sweep point, not a hardware mass). The default frame is true
    geometry, so the feet sit on the as-built faces. A second frame
    sets the longest arrow to 0.20 of a voxel pitch (the viewer's mild
    exaggeration, capped at 0.40). Feet follow the face that was drawn.
    """
    if shutil.which("ffmpeg") is None:
        raise SystemExit("ffmpeg is not on PATH")
    try:
        from playwright.sync_api import sync_playwright
    except ImportError as exc:
        raise SystemExit("playwright is not installed. pip install playwright") from exc

    from experiments.on_lattice import build_demo_log

    runs = ROOT / "runs"
    runs.mkdir(exist_ok=True)
    log = build_demo_log()
    log_path = runs / "on_lattice_demo.json"
    log.save(log_path)
    # A frame with the robot on the arm: the last tick is parked after
    # the build, often back on the seed. Prefer a tick whose feet are
    # on a level-2 cell.
    # Prefer the appended pose: both feet on adjacent level-2 tops.
    on_arm = len(log.frames) - 1
    for index, frame in enumerate(log.frames):
        robot = (frame.get("robots") or [None])[0]
        feet = (robot or {}).get("feet") or []
        if (
            len(feet) >= 2
            and all(foot["normal"] == [1, 0, 0] and foot["cell"][0] >= 2 for foot in feet)
            and feet[0]["cell"] != feet[1]["cell"]
        ):
            on_arm = index
            break
    port = _free_port()
    httpd = _serve(port)
    out_dir.mkdir(parents=True, exist_ok=True)
    shots = out_dir / "_on_lattice_frames"
    if shots.exists():
        shutil.rmtree(shots)
    shots.mkdir(parents=True)
    base = f"http://127.0.0.1:{port}/replay_viewer.html"
    notes = []
    try:
        with sync_playwright() as pw:
            browser = pw.chromium.launch(
                channel="chrome",
                headless=True,
                args=["--no-sandbox", "--disable-dev-shm-usage"],
            )
            page = browser.new_page(viewport={"width": WIDTH, "height": HEIGHT})
            for name, deform, frame_i in (
                ("stand", 0.0, on_arm),
                ("sag", DEFORM_HOLD, on_arm),
                ("start", 0.0, 0),
                ("done", 0.0, len(log.frames) - 1),
            ):
                url = (
                    f"{base}?log=runs/on_lattice_demo.json&frame={frame_i}"
                    f"&color=displacement&deform={deform}&theta=0.55&phi=0.95&radius=6.2"
                )
                page.goto(url, wait_until="domcontentloaded")
                page.wait_for_function(
                    "() => window.assemblerReplay && window.assemblerReplay.ready()",
                    timeout=45_000,
                )
                page.evaluate(
                    """async ({frame, deform}) => {
                        const v = window.assemblerReplay;
                        v.setPlaying(false);
                        v.setColor('displacement');
                        v.setDeform(deform);
                        v.setOrbit(0.55, 0.95, 6.2);
                        v.setFrame(frame);
                        await new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)));
                    }""",
                    {"frame": frame_i, "deform": deform},
                )
                dest = shots / f"{name}.png"
                page.screenshot(path=str(dest))
                notes.append(f"{name} frame={frame_i} deform={deform}")
            browser.close()
    finally:
        httpd.shutdown()
    # Keep the standing-on-arm stills next to the other docs pictures.
    kept = ROOT / "docs" / "on_lattice_demo.png"
    sag = ROOT / "docs" / "on_lattice_sag.png"
    shutil.copyfile(shots / "stand.png", kept)
    shutil.copyfile(shots / "sag.png", sag)
    artifact = Path("/opt/cursor/artifacts")
    if artifact.is_dir():
        for src in shots.glob("*.png"):
            shutil.copyfile(src, artifact / f"on_lattice_{src.name}")
    return f"on-lattice stills in {shots}; docs {kept} and {sag}; " + "; ".join(notes)


def render_bridge_stills(out_dir: Path) -> str:
    """Side view of the finished one-robot span.

    Deform 0 is the as-built deck (air under midspan). Deform 0.20 is
    the normalized exaggeration: the longest arrow is a fifth of a
    pitch, and block size shrinks so neighbors stay apart.
    """
    if shutil.which("ffmpeg") is None:
        raise SystemExit("ffmpeg is not on PATH")
    try:
        from playwright.sync_api import sync_playwright
    except ImportError as exc:
        raise SystemExit("playwright is not installed. pip install playwright") from exc

    from main import run_bridge

    runs = ROOT / "runs"
    runs.mkdir(exist_ok=True)
    log, _swarm = run_bridge(verbose=False)
    n = len(log.frames)
    port = _free_port()
    httpd = _serve(port)
    out_dir.mkdir(parents=True, exist_ok=True)
    base = f"http://127.0.0.1:{port}/replay_viewer.html"
    still = ROOT / "docs" / "strain_bridge.png"
    normalized = out_dir / "bridge_normalized.png"
    try:
        with sync_playwright() as pw:
            browser = pw.chromium.launch(
                channel="chrome",
                headless=True,
                args=["--no-sandbox", "--disable-dev-shm-usage"],
            )
            page = browser.new_page(viewport={"width": 1400, "height": 900})
            url = (
                f"{base}?log=runs/bridge.json&frame=last&color=displacement"
                f"&deform=0&theta={BRIDGE_THETA0}&phi={BRIDGE_PHI}&radius={BRIDGE_RADIUS}"
            )
            _open_log(page, url)
            last = n - 1
            _pose(
                page, sim=last, color="displacement", deform=0,
                theta=BRIDGE_THETA0, phi=BRIDGE_PHI, radius=BRIDGE_RADIUS,
            )
            page.screenshot(path=str(still))
            page.screenshot(path=str(out_dir / "bridge_true.png"))
            _pose(
                page, sim=last, color="displacement", deform=DEFORM_HOLD,
                theta=BRIDGE_THETA0, phi=BRIDGE_PHI, radius=BRIDGE_RADIUS,
            )
            page.screenshot(path=str(normalized))
            browser.close()
    finally:
        httpd.shutdown()
    artifact = Path("/opt/cursor/artifacts")
    if artifact.is_dir():
        shutil.copyfile(out_dir / "bridge_true.png", artifact / "bridge_true.png")
        shutil.copyfile(normalized, artifact / "bridge_normalized.png")
    return (
        f"bridge stills: true {still}; normalized {normalized}; "
        f"frames={n} deform=0 and {DEFORM_HOLD}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--on-lattice",
        action="store_true",
        help="Capture inchworm stills instead of the 60 s crane/cuboct demo.",
    )
    parser.add_argument(
        "--bridge-still",
        action="store_true",
        help="Capture the finished span at true geometry and at 0.20 pitch.",
    )
    parser.add_argument(
        "--mp4",
        type=Path,
        default=Path("/opt/cursor/artifacts/v02_demo.mp4"),
    )
    parser.add_argument(
        "--gif",
        type=Path,
        default=ROOT / "docs" / "demo_bridge.gif",
        help="Compressed excerpt. Omitted from disk if larger than --gif-max-bytes. Pass '' to skip.",
    )
    parser.add_argument("--gif-max-bytes", type=int, default=1_500_000)
    args = parser.parse_args()
    started = time.time()
    if args.on_lattice:
        print(render_on_lattice(Path("/tmp/on-lattice-demo")))
    elif args.bridge_still:
        print(render_bridge_stills(Path("/tmp/bridge-still")))
    else:
        gif = args.gif if str(args.gif) else None
        print(render(args.mp4, gif, args.gif_max_bytes))
    print(f"elapsed {time.time() - started:.1f} s")


if __name__ == "__main__":
    main()
