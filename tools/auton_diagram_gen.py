#!/usr/bin/env python3
"""auton_diagram_gen.py

Generates a Mermaid-flavoured Markdown state diagram from an FRC Team 302
auton XML file (validated against auton.dtd).

Usage
-----
  # One file
  python3 tools/auton_diagram_gen.py src/main/deploy/auton/MyAuton.xml

  # Multiple files (used by the GitHub Actions workflow)
  python3 tools/auton_diagram_gen.py src/main/deploy/auton/*.xml

The output file is always written to:
  documents/auton/<XmlBasename>.md   (relative to repo root)

The script resolves zone XML files relative to:
  src/main/deploy/auton/zones/<filename>
"""

import sys
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

# ---------------------------------------------------------------------------
# Paths (relative to repo root, which is two levels above this script)
# ---------------------------------------------------------------------------
SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
ZONES_DIR = REPO_ROOT / "src" / "main" / "deploy" / "auton" / "zones"
CHOREO_DIR = REPO_ROOT / "src" / "main" / "deploy" / "choreo"
DOCS_DIR = REPO_ROOT / "documents" / "auton"
TRAJ_SVG_DIR = REPO_ROOT / "documents" / "auton" / "svg" / "traj"

# ---------------------------------------------------------------------------
# Field + SVG constants (must match auton_zone_field_gen.py)
# ---------------------------------------------------------------------------
FIELD_W = 16.46   # metres
FIELD_H = 8.10    # metres
SVG_W   = 900     # pixels
SVG_H   = int(SVG_W * FIELD_H / FIELD_W)
SCALE   = SVG_W / FIELD_W

# Robot dimensions (metres)
ROBOT_W = 0.6096   # chassis width  (short side)
ROBOT_L = 0.762    # chassis length (long side,  front)
BUMPER_W = 0.762   # bumper width
BUMPER_L = 0.9144  # bumper length  (front side)


def mx(x: float) -> float:
    return x * SCALE

def my(y: float) -> float:
    return (FIELD_H - y) * SCALE

# Default attribute values from auton.dtd
DTD_DEFAULTS = {
    "id": "DO_NOTHING",
    "launcherState": "STATE_IDLE",
    "intakeState": "STATE_OFF",
    "climberState": "STATE_OFF",
    "time": "--",
    "choreoname": "",
}


def mermaid_safe_text(text: str) -> str:
    """Return Mermaid-safe ASCII label text.

    Mermaid state labels can fail on unexpected punctuation and non-ASCII chars.
    We normalize to plain ASCII, collapse whitespace, and avoid double quotes.
    """
    ascii_text = text.encode("ascii", errors="replace").decode("ascii")
    ascii_text = ascii_text.replace('"', "'").replace("\\", "/")
    return " ".join(ascii_text.split())


def parse_zone_effects(zone_filename: str) -> dict:
    """Return a dict of interesting attributes from a zone XML file.

    Keys returned (when present):
      pathUpdateOption, launcherState, allianceColor, shape (rect/circle)
    """
    zone_path = ZONES_DIR / zone_filename
    effects: dict = {}
    if not zone_path.exists():
        return effects
    try:
        root = ET.parse(zone_path).getroot()
        for zone_el in root.iter("zone"):
            for attr in ("pathUpdateOption", "launcherState", "allianceColor"):
                val = zone_el.get(attr)
                if val:
                    effects[attr] = val
    except ET.ParseError:
        pass
    return effects


def parse_zone_geometry(zone_filename: str) -> dict | None:
    """Return geometry + style attrs for one zone XML file, or None.

    Supported geometry:
      - circlex/circley/radius (radius stored in centimetres)
      - x1_rect/y1_rect/x2_rect/y2_rect
    """
    zone_path = ZONES_DIR / zone_filename
    if not zone_path.exists():
        return None

    try:
        root = ET.parse(zone_path).getroot()
    except ET.ParseError:
        return None

    for zone_el in root.iter("zone"):
        alliance = zone_el.get("allianceColor", "BOTH")
        path_opt = zone_el.get("pathUpdateOption", "NOTHING")

        if zone_el.get("circlex") is not None:
            try:
                cx = float(zone_el.get("circlex", 0.0))
                cy = float(zone_el.get("circley", 0.0))
                r = float(zone_el.get("radius", 0.0)) / 100.0
            except ValueError:
                return None
            return {
                "name": zone_filename.replace(".xml", ""),
                "type": "circle",
                "alliance": alliance,
                "pathUpdateOption": path_opt,
                "cx": cx,
                "cy": cy,
                "r": r,
            }

        if zone_el.get("x1_rect") is not None:
            try:
                x1 = float(zone_el.get("x1_rect", 0.0))
                y1 = float(zone_el.get("y1_rect", 0.0))
                x2 = float(zone_el.get("x2_rect", 0.0))
                y2 = float(zone_el.get("y2_rect", 0.0))
            except ValueError:
                return None
            return {
                "name": zone_filename.replace(".xml", ""),
                "type": "rect",
                "alliance": alliance,
                "pathUpdateOption": path_opt,
                "x1": min(x1, x2),
                "y1": min(y1, y2),
                "x2": max(x1, x2),
                "y2": max(y1, y2),
            }

    return None


def infer_auton_alliance(auton_stem: str) -> str:
    """Infer alliance from auton filename prefix: Blue*, Red*, or BOTH."""
    if auton_stem.startswith("Blue"):
        return "BLUE"
    if auton_stem.startswith("Red"):
        return "RED"
    return "BOTH"


def collect_element_zone_overlays(step_data: list[dict], auton_stem: str) -> list[dict]:
    """Collect zone geometries for files ending with Element.xml.

    Priority:
      1) Element zones explicitly referenced by primitives.
      2) If none are referenced, include alliance-matching Element zones.
    """
    element_zone_files: set[str] = set()
    for step in step_data:
        for zf in step.get("zones", []):
            if zf.endswith("Element.xml"):
                element_zone_files.add(zf)

    if not element_zone_files:
        alliance = infer_auton_alliance(auton_stem)
        for zpath in sorted(ZONES_DIR.glob("*Element.xml")):
            name = zpath.name
            if alliance == "BLUE" and not name.startswith("Blue"):
                continue
            if alliance == "RED" and not name.startswith("Red"):
                continue
            element_zone_files.add(name)

    overlays: list[dict] = []
    for zf in sorted(element_zone_files):
        geom = parse_zone_geometry(zf)
        if geom is not None:
            overlays.append(geom)
    return overlays


def _zone_overlay_style(alliance: str) -> tuple[str, str]:
    if alliance == "BLUE":
        return ("#4d7dff", "#9bb6ff")
    if alliance == "RED":
        return ("#ff6b6b", "#ffc0c0")
    return ("#66c08a", "#bde8cd")


def _element_zone_overlay_svg(shape: dict) -> str:
    """Render one Element-zone overlay on top of the field background."""
    stroke_col, text_col = _zone_overlay_style(shape.get("alliance", "BOTH"))
    label = shape["name"].replace("Blue", "").replace("Red", "")
    lines: list[str] = []

    if shape["type"] == "circle":
        cx, cy, r = shape["cx"], shape["cy"], shape["r"]
        if cx > FIELD_W * 1.5 or cy > FIELD_H * 1.5 or r <= 0:
            return ""
        svgcx, svgcy, svgr = mx(cx), my(cy), r * SCALE
        lines.append(
            f'  <circle cx="{svgcx:.1f}" cy="{svgcy:.1f}" r="{svgr:.1f}" '
            f'fill="none" stroke="{stroke_col}" stroke-width="1.6" stroke-dasharray="4 3" stroke-opacity="0.85"/>'
        )
        ly = svgcy - svgr - 4
        lines.append(
            f'  <text x="{svgcx:.1f}" y="{ly:.1f}" text-anchor="middle" font-size="8" '
            f'fill="{text_col}" font-family="monospace">{label}</text>'
        )
    else:
        x1, y1, x2, y2 = shape["x1"], shape["y1"], shape["x2"], shape["y2"]
        rx1 = mx(max(0.0, min(x1, FIELD_W)))
        ry1 = my(max(0.0, min(y2, FIELD_H)))
        rw = mx(max(0.0, min(x2, FIELD_W))) - rx1
        rh = my(max(0.0, min(y1, FIELD_H))) - ry1
        if rw <= 0 or rh <= 0:
            return ""
        lines.append(
            f'  <rect x="{rx1:.1f}" y="{ry1:.1f}" width="{rw:.1f}" height="{rh:.1f}" '
            f'fill="none" stroke="{stroke_col}" stroke-width="1.6" stroke-dasharray="4 3" stroke-opacity="0.85"/>'
        )
        lx = rx1 + 4
        ly = ry1 + 9
        lines.append(
            f'  <text x="{lx:.1f}" y="{ly:.1f}" text-anchor="start" font-size="8" '
            f'fill="{text_col}" font-family="monospace">{label}</text>'
        )

    return "\n".join(lines)


def zone_label(zone_filename: str, effects: dict) -> str:
    """Human-readable one-liner for a zone entry (used in legend only)."""
    stem = zone_filename.replace(".xml", "")
    notes = []
    if "pathUpdateOption" in effects:
        notes.append(effects["pathUpdateOption"])
    if "launcherState" in effects:
        notes.append(f"launcher: {effects['launcherState']}")
    suffix = f" ({', '.join(notes)})" if notes else ""
    return f"`{stem}`{suffix}"


def parse_traj(choreo_name: str) -> dict | None:
    """Parse a .traj file and return a summary dict, or None if not found.

    Returns:
      {
        "name":      str,
        "file":      str,           # repo-relative posix path
        "duration":  float | None,  # total time in seconds
        "waypoints": [              # from snapshot (design-time)
          {"x": float, "y": float, "heading_deg": float}
        ],
        "samples":   [              # decimated pose samples for rendering
          {"x": float, "y": float, "heading": float}  # heading in radians
        ]
      }
    """
    traj_path = CHOREO_DIR / f"{choreo_name}.traj"
    if not traj_path.exists():
        return None
    try:
        data = json.loads(traj_path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        return None

    # Design waypoints from snapshot
    snap_wps = data.get("snapshot", {}).get("waypoints", [])
    waypoints = [
        {
            "x": round(wp.get("x", 0.0), 3),
            "y": round(wp.get("y", 0.0), 3),
            "heading_deg": round(math.degrees(wp.get("heading", 0.0)) % 360, 1),
        }
        for wp in snap_wps
    ]

    # All trajectory samples
    all_samples = data.get("trajectory", {}).get("samples", [])
    duration = round(all_samples[-1]["t"], 3) if all_samples else None

    # Decimate to at most MAX_RENDER_SAMPLES evenly spaced samples
    MAX_RENDER_SAMPLES = 60
    if len(all_samples) <= MAX_RENDER_SAMPLES:
        chosen = all_samples
    else:
        step = (len(all_samples) - 1) / (MAX_RENDER_SAMPLES - 1)
        chosen = [all_samples[round(i * step)] for i in range(MAX_RENDER_SAMPLES)]
    samples = [{"x": s["x"], "y": s["y"], "heading": s["heading"]} for s in chosen]

    return {
        "name": choreo_name,
        "file": traj_path.relative_to(REPO_ROOT).as_posix(),
        "duration": duration,
        "waypoints": waypoints,
        "samples": samples,
    }


# ---------------------------------------------------------------------------
# SVG field background (lightweight inline version)
# ---------------------------------------------------------------------------
def _field_bg_svg() -> str:
    w, h = SVG_W, SVG_H
    lines = []
    lines.append(f'  <rect x="0" y="0" width="{w}" height="{h}" fill="#1a1a1a"/>')
    # Alliance region shading
    bw = mx(4.0)
    lines.append(f'  <rect x="0" y="0" width="{bw:.1f}" height="{h}" fill="#0d1a3a" fill-opacity="0.7"/>')
    lines.append(f'  <text x="{bw/2:.1f}" y="14" text-anchor="middle" font-size="10" fill="#4466cc" font-family="sans-serif">BLUE</text>')
    nzx, nzw = mx(4.8), mx(11.86) - mx(4.8)
    lines.append(f'  <rect x="{nzx:.1f}" y="0" width="{nzw:.1f}" height="{h}" fill="#111111" fill-opacity="0.7"/>')
    lines.append(f'  <text x="{nzx + nzw/2:.1f}" y="14" text-anchor="middle" font-size="10" fill="#888" font-family="sans-serif">NEUTRAL</text>')
    rx2 = mx(12.55)
    lines.append(f'  <rect x="{rx2:.1f}" y="0" width="{w - rx2:.1f}" height="{h}" fill="#3a0d0d" fill-opacity="0.7"/>')
    lines.append(f'  <text x="{rx2 + (w - rx2)/2:.1f}" y="14" text-anchor="middle" font-size="10" fill="#cc4444" font-family="sans-serif">RED</text>')
    # Field border
    lines.append(f'  <rect x="0" y="0" width="{w}" height="{h}" fill="none" stroke="#888" stroke-width="2"/>')
    # Axis ticks
    for xm in range(0, int(FIELD_W) + 1, 2):
        sx = mx(xm)
        lines.append(f'  <line x1="{sx:.1f}" y1="{h}" x2="{sx:.1f}" y2="{h-5}" stroke="#555" stroke-width="1"/>')
        lines.append(f'  <text x="{sx:.1f}" y="{h-7}" text-anchor="middle" font-size="7" fill="#666" font-family="monospace">{xm}m</text>')
    for ym in range(0, int(FIELD_H) + 1, 2):
        sy = my(ym)
        lines.append(f'  <line x1="0" y1="{sy:.1f}" x2="5" y2="{sy:.1f}" stroke="#555" stroke-width="1"/>')
        lines.append(f'  <text x="7" y="{sy:.1f}" dominant-baseline="middle" font-size="7" fill="#666" font-family="monospace">{ym}m</text>')
    return "\n".join(lines)


def _robot_svg(px: float, py: float, heading_rad: float,
               color: str, opacity: float, idx: int, is_waypoint: bool) -> str:
    """Return SVG elements for a robot pose.

    The robot chassis is ROBOT_L x ROBOT_W metres (long axis = forward = heading=0).
    A small forward triangle indicates the front.
    SVG heading is negated because SVG y-axis is flipped.
    """
    svgx, svgy = mx(px), my(py)
    # SVG rotate: positive = clockwise. Field heading positive = CCW.
    # Also note SVG y-flip inverts the rotation sign.
    rot_deg = -math.degrees(heading_rad)

    chassis_pw = ROBOT_L * SCALE   # side-to-side (long side = 0.762 m)
    chassis_pl = ROBOT_W * SCALE   # forward      (short side = 0.6096 m)
    bumper_pw  = BUMPER_L * SCALE  # side-to-side (long side = 0.9144 m)
    bumper_pl  = BUMPER_W * SCALE  # forward      (short side = 0.762 m)

    lines = []
    gid = f"pose{idx}"
    lines.append(f'  <g id="{gid}" transform="translate({svgx:.2f},{svgy:.2f}) rotate({rot_deg:.2f})">')

    if is_waypoint:
        # Full bumper + chassis + direction triangle
        # Bumper (outermost, semi-transparent)
        bx, by = -bumper_pl / 2, -bumper_pw / 2
        lines.append(
            f'    <rect x="{bx:.2f}" y="{by:.2f}" width="{bumper_pl:.2f}" height="{bumper_pw:.2f}" '
            f'rx="3" fill="{color}" fill-opacity="0.25" stroke="{color}" stroke-width="1.2" stroke-opacity="0.7"/>'
        )
        # Chassis
        cx, cy = -chassis_pl / 2, -chassis_pw / 2
        lines.append(
            f'    <rect x="{cx:.2f}" y="{cy:.2f}" width="{chassis_pl:.2f}" height="{chassis_pw:.2f}" '
            f'fill="{color}" fill-opacity="{opacity:.2f}" stroke="{color}" stroke-width="1.5"/>'
        )
        # Front triangle: points at +x direction (right in local frame = forward)
        tri_size = chassis_pw * 0.35
        tri_x = chassis_pl / 2
        lines.append(
            f'    <polygon points="{tri_x:.2f},0 {tri_x - tri_size:.2f},{tri_size:.2f} {tri_x - tri_size:.2f},{-tri_size:.2f}" '
            f'fill="#ffffff" fill-opacity="0.9"/>'
        )
        # Waypoint dot at centre
        lines.append(f'    <circle cx="0" cy="0" r="3" fill="#ffffff" fill-opacity="0.9"/>')
    else:
        # Path sample: just a tiny oriented tick
        tick = chassis_pl * 0.3
        lines.append(
            f'    <line x1="0" y1="0" x2="{tick:.2f}" y2="0" '
            f'stroke="{color}" stroke-width="1.5" stroke-opacity="0.6"/>'
        )
        lines.append(f'    <circle cx="0" cy="0" r="1.5" fill="{color}" fill-opacity="0.5"/>')

    lines.append('  </g>')
    return "\n".join(lines)


# Colour sequence for multiple trajectories on the same diagram
_TRAJ_COLORS = ["#00d4ff", "#ffcc00", "#ff6688", "#88ff44", "#ff8800", "#cc88ff"]


def render_traj_svg(trajs: list[tuple[int, str, dict, int]], zone_overlays: list[dict] | None = None) -> str:
    """Render one SVG showing the field + all provided trajectories overlaid.

    trajs: list of (step_number, choreo_name, traj_dict, color_index)
    color_index fixes which entry in _TRAJ_COLORS to use, so per-step SVGs
    use the same colour as the overview diagram.
    Returns the full SVG string.
    """
    body_lines = [_field_bg_svg()]

    for shape in (zone_overlays or []):
        overlay = _element_zone_overlay_svg(shape)
        if overlay:
            body_lines.append(overlay)

    for step_num, choreo_name, traj, color_idx in trajs:
        color = _TRAJ_COLORS[color_idx % len(_TRAJ_COLORS)]
        samples = traj["samples"]
        waypoints_xy = [(wp["x"], wp["y"]) for wp in traj["waypoints"]]
        waypoint_set = {(round(x, 2), round(y, 2)) for x, y in waypoints_xy}

        # Path polyline
        pts = " ".join(f"{mx(s['x']):.2f},{my(s['y']):.2f}" for s in samples)
        body_lines.append(
            f'  <polyline points="{pts}" '
            f'fill="none" stroke="{color}" stroke-width="2" stroke-opacity="0.85" '
            f'stroke-linejoin="round" stroke-linecap="round"/>'
        )

        # Render pose at every sample (small tick) and at waypoints (full robot)
        for si, s in enumerate(samples):
            is_wp = (round(s["x"], 2), round(s["y"], 2)) in waypoint_set
            body_lines.append(
                _robot_svg(s["x"], s["y"], s["heading"], color,
                           opacity=0.55, idx=color_idx * 1000 + si, is_waypoint=is_wp)
            )

        # Legend label near start of path
        start = samples[0]
        lx, ly = mx(start["x"]) + 6, my(start["y"]) - 8
        pill_w = len(choreo_name) * 5.5 + 8
        body_lines.append(
            f'  <rect x="{lx - 4:.1f}" y="{ly - 9:.1f}" width="{pill_w:.1f}" height="11" rx="3" '
            f'fill="#000000" fill-opacity="0.65"/>'
        )
        body_lines.append(
            f'  <text x="{lx:.1f}" y="{ly:.1f}" font-size="9" fill="{color}" '
            f'font-family="monospace" font-weight="bold">S{step_num}: {choreo_name}</text>'
        )

    body = "\n".join(body_lines) + "\n"
    return (
        f'<svg xmlns="http://www.w3.org/2000/svg" '
        f'width="{SVG_W}" height="{SVG_H}" viewBox="0 0 {SVG_W} {SVG_H}">\n'
        + body
        + "</svg>"
    )


def generate_traj_section(step_data: list[dict], auton_stem: str, zone_overlays: list[dict]) -> str:
    """Return a Markdown section with trajectory SVGs and waypoint tables."""
    traj_steps = [
        d for d in step_data
        if d["prim_id"] == "TRAJECTORY_DRIVE" and d["choreo"] and d["choreo"] != "--"
    ]
    if not traj_steps:
        return ""

    TRAJ_SVG_DIR.mkdir(parents=True, exist_ok=True)

    # --- Build list of (step, choreo, traj, color_idx) assigning colours in
    #     order of appearance -- same index used for both overview and per-step SVGs.
    all_trajs = []
    color_idx_map: dict[str, int] = {}   # choreo_name -> colour index
    for ci, d in enumerate(traj_steps):
        traj = parse_traj(d["choreo"])
        if traj:
            color_idx_map[d["choreo"]] = ci
            all_trajs.append((d["step"], d["choreo"], traj, ci))

    overview_svg_name = f"{auton_stem}_all_traj.svg"
    overview_svg_path = TRAJ_SVG_DIR / overview_svg_name
    if all_trajs:
        overview_svg_path.write_text(
            render_traj_svg(all_trajs, zone_overlays=zone_overlays), encoding="ascii", errors="replace"
        )

    lines = ["## Trajectory Details", ""]

    if all_trajs:
        # Legend for overview colours
        legend_items = " ".join(
            f'<span style="color:{_TRAJ_COLORS[ci % len(_TRAJ_COLORS)]}">'
            f'&#9632; S{step}: {name}</span>'
            for step, name, _, ci in all_trajs
        )
        rel = f"svg/traj/{overview_svg_name}"
        lines.append("### All Trajectories Overview")
        lines.append("")
        lines.append(f'<img src="{rel}" alt="All trajectories for {auton_stem}" width="{SVG_W}"/>')
        lines.append("")

    # --- Per-step section ---
    for d in traj_steps:
        choreo_name = d["choreo"]
        traj = parse_traj(choreo_name)
        traj_link = (
            f"[`{choreo_name}.traj`](../../{traj['file']})"
            if traj else f"`{choreo_name}.traj` *(not found)*"
        )
        duration_str = f"{traj['duration']} s" if traj and traj["duration"] is not None else "unknown"
        ci = color_idx_map.get(choreo_name, 0)
        color = _TRAJ_COLORS[ci % len(_TRAJ_COLORS)]

        lines.append(f"### Step {d['step']} -- {choreo_name}")
        lines.append("")
        lines.append(f"- **File:** {traj_link}")
        lines.append(f"- **Duration:** {duration_str}")
        lines.append(f"- **Timeout in auton XML:** {d['timeout']} s")
        lines.append(f"- **Colour in overview:** `{color}`")
        lines.append("")

        # Per-step SVG rendered with the same colour index as the overview
        if traj:
            svg_name = f"{auton_stem}_step{d['step']}_{choreo_name}.svg"
            svg_path = TRAJ_SVG_DIR / svg_name
            svg_path.write_text(
                render_traj_svg([(d["step"], choreo_name, traj, ci)], zone_overlays=zone_overlays),
                encoding="ascii", errors="replace"
            )
            rel = f"svg/traj/{svg_name}"
            lines.append(f'<img src="{rel}" alt="Trajectory {choreo_name}" width="{SVG_W}"/>')
            lines.append("")

        if traj and traj["waypoints"]:
            lines.append("| # | X (m) | Y (m) | Heading (deg) |")
            lines.append("|---|-------|-------|---------------|")
            for wi, wp in enumerate(traj["waypoints"], start=1):
                lines.append(f"| {wi} | {wp['x']} | {wp['y']} | {wp['heading_deg']} |")
            lines.append("")

    return "\n".join(lines)


def generate_markdown(xml_path: Path) -> str:
    """Parse an auton XML file and return the full Markdown string."""
    stem = xml_path.stem
    relative_xml = xml_path.relative_to(REPO_ROOT).as_posix()

    tree = ET.parse(xml_path)
    root = tree.getroot()

    primitives = list(root.iter("primitive"))

    # ------------------------------------------------------------------ #
    # 1. Mermaid stateDiagram-v2 block                                    #
    #                                                                      #
    # Mermaid stateDiagram-v2 composite states only support plain IDs and #
    # transitions inside them — no nested composites, no alias labels.    #
    # We use a flat diagram: one node per primitive, with state labels and #
    # all detail encoded on the transition arrow label.                    #
    # ------------------------------------------------------------------ #
    summary_rows: list[dict] = []
    zone_legend: dict[str, dict] = {}  # zone stem → effects

    # Pass 1: gather data
    step_data: list[dict] = []
    for i, prim in enumerate(primitives, start=1):
        prim_id = prim.get("id", DTD_DEFAULTS["id"])
        choreo = prim.get("choreoname", DTD_DEFAULTS["choreoname"]) or "--"
        timeout = prim.get("time", DTD_DEFAULTS["time"])
        intake = prim.get("intakeState", DTD_DEFAULTS["intakeState"])
        launcher = prim.get("launcherState", DTD_DEFAULTS["launcherState"])
        climber = prim.get("climberState", DTD_DEFAULTS["climberState"])

        zones = [z.get("filename", "") for z in prim.findall("zone") if z.get("filename")]
        zone_effects_list: list[tuple[str, dict]] = []
        for zf in zones:
            effects = parse_zone_effects(zf)
            stem_z = zf.replace(".xml", "")
            zone_legend[stem_z] = effects
            zone_effects_list.append((zf, effects))

        step_data.append(
            dict(
                step=i,
                prim_id=prim_id,
                choreo=choreo,
                timeout=timeout,
                intake=intake,
                launcher=launcher,
                climber=climber,
                zones=zones,
                zone_effects_list=zone_effects_list,
            )
        )
        # Trajectory link for summary table
        traj_path_exists = (CHOREO_DIR / f"{choreo}.traj").exists()
        if prim_id == "TRAJECTORY_DRIVE" and choreo and choreo != "--" and traj_path_exists:
            traj_rel = f"src/main/deploy/choreo/{choreo}.traj"
            choreo_display = f"[{choreo}](../../{traj_rel})"
        else:
            choreo_display = choreo

        summary_rows.append(
            {
                "step": i,
                "primitive": prim_id,
                "trajectory": choreo_display,
                "timeout": timeout,
                "intake": intake,
                "launcher": launcher,
                "climber": climber,
                "zones": ", ".join(z.replace(".xml", "") for z in zones) or "--",
            }
        )

    # Pass 2: emit Mermaid -- flat nodes, detail on transition labels
    # classDef colours: green = intake active, purple = intake off.
    # Only ASCII characters used -- non-ASCII breaks Mermaid diagram detection.
    mermaid_lines: list[str] = [
        "stateDiagram-v2",
        "    direction LR",
        "",
        "    %% Green = intake active, Purple = intake off",
        "    classDef intake  fill:#1a7a3a,color:#ffffff,stroke:#0d4a1f,font-weight:bold",
        "    classDef noIntake fill:#5a4a8a,color:#ffffff,stroke:#3a2a6a,font-weight:bold",
        "",
    ]

    # State label declarations  (id : description)
    for d in step_data:
        traj_part = f" {d['choreo']}" if d["choreo"] and d["choreo"] != "—" else ""
        state_label = mermaid_safe_text(f"Step {d['step']} - {d['prim_id']}{traj_part}")
        mermaid_lines.append(f'    Step{d["step"]} : "{state_label}"')
    mermaid_lines.append("")

    # Apply classDef per step based on intake state
    for d in step_data:
        css_class = "noIntake" if d["intake"].replace("STATE_", "") == "OFF" else "intake"
        mermaid_lines.append(f"    class Step{d['step']} {css_class}")
    mermaid_lines.append("")

    # Transitions
    if not step_data:
        # Mermaid requires transitions to/from concrete states; [*] --> [*] is invalid.
        mermaid_lines.append('    Empty : "No primitives defined"')
        mermaid_lines.append('    class Empty noIntake')
        mermaid_lines.append('    [*] --> Empty : "start"')
        mermaid_lines.append('    Empty --> [*] : "done"')
    else:
        mermaid_lines.append('    [*] --> Step1 : "start"')
    for d in step_data:
        src = f"Step{d['step']}"
        dst = f"Step{d['step'] + 1}" if d["step"] < len(step_data) else "[*]"

        # Build compact transition label.
        # IMPORTANT: Mermaid stateDiagram-v2 transition labels must NOT contain
        # parentheses, commas, or + — they break the lexer. Use plain spaces only.
        parts: list[str] = []
        intake_short = d["intake"].replace("STATE_", "")
        if intake_short != "OFF":
            parts.append(f"intake={intake_short}")
        launcher_short = d["launcher"].replace("STATE_", "")
        if launcher_short not in ("IDLE", "OFF"):
            parts.append(f"launcher={launcher_short}")
        climber_short = d["climber"].replace("STATE_", "")
        if climber_short not in ("OFF",):
            parts.append(f"climber={climber_short}")
        zone_labels_short: list[str] = []
        for zf, effects in d["zone_effects_list"]:
            stem_z = zf.replace(".xml", "").replace("Blue", "").replace("Red", "")
            eff_parts: list[str] = []
            if "pathUpdateOption" in effects:
                eff_parts.append(effects["pathUpdateOption"])
            if "launcherState" in effects:
                eff_parts.append(effects["launcherState"].replace("STATE_", ""))
            # Join effects with a space — no parens, commas, or +
            zone_labels_short.append(" ".join([stem_z] + eff_parts))
        if zone_labels_short:
            # Separate multiple zones with a space only
            parts.append("zones=" + " ".join(zone_labels_short))

        # Quote transition labels so Mermaid won't choke on punctuation.
        arrow_label = f': "{mermaid_safe_text(" | ".join(parts))}"' if parts else ""
        mermaid_lines.append(f"    {src} --> {dst}{arrow_label}")

    mermaid_block = "\n".join(mermaid_lines)

    # ------------------------------------------------------------------ #
    # 2. Summary table                                                     #
    # ------------------------------------------------------------------ #
    table_header = "| Step | Primitive | Trajectory | Timeout | Intake | Launcher | Climber | Zones |"
    table_sep = "|------|-----------|------------|---------|--------|----------|---------|-------|"
    table_rows = "\n".join(
        f"| {r['step']} | {r['primitive']} | {r['trajectory']} | {r['timeout']} s "
        f"| {r['intake']} | {r['launcher']} | {r['climber']} | {r['zones']} |"
        for r in summary_rows
    )

    # ------------------------------------------------------------------ #
    # 3. Zone legend                                                        #
    # ------------------------------------------------------------------ #
    legend_header = "| Zone file | Effect when entered |"
    legend_sep = "|-----------|---------------------|"
    legend_rows_list: list[str] = []
    for stem_z, effects in sorted(zone_legend.items()):
        parts = []
        if "pathUpdateOption" in effects:
            parts.append(f"`pathUpdateOption = {effects['pathUpdateOption']}`")
        if "launcherState" in effects:
            parts.append(f"`launcherState -> {effects['launcherState']}`")
        if not parts:
            parts.append("*(no tracked effects)*")
        legend_rows_list.append(f"| `{stem_z}` | {', '.join(parts)} |")
    legend_rows = "\n".join(legend_rows_list)

    # ------------------------------------------------------------------ #
    # 4. Trajectory details section                                        #
    # ------------------------------------------------------------------ #
    zone_overlays = collect_element_zone_overlays(step_data, stem)
    traj_section = generate_traj_section(step_data, stem, zone_overlays)

    # ------------------------------------------------------------------ #
    # 5. Assemble full document                                            #
    # ------------------------------------------------------------------ #
    doc = (
f"""# Auton: {stem}

> Auto-generated from [`{relative_xml}`](../../{relative_xml}).  
> Do not edit this file manually -- push a change to the XML and the [auton-diagrams workflow](../../.github/workflows/auton-diagrams.yml) will regenerate it.

## State Diagram

```mermaid
{mermaid_block}
```

## Primitive Summary

{table_header}
{table_sep}
{table_rows}

{traj_section}
## Zone Legend

{legend_header}
{legend_sep}
{legend_rows}
""")
    return doc


def main() -> None:
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <auton.xml> [auton2.xml ...]", file=sys.stderr)
        sys.exit(1)

    DOCS_DIR.mkdir(parents=True, exist_ok=True)

    for xml_arg in sys.argv[1:]:
        xml_path = Path(xml_arg).resolve()
        if not xml_path.exists():
            print(f"[SKIP] not found: {xml_arg}", file=sys.stderr)
            continue

        print(f"[GEN]  {xml_path.name} → documents/auton/{xml_path.stem}.md")
        try:
            md = generate_markdown(xml_path)
        except ET.ParseError as exc:
            print(f"[ERROR] XML parse error in {xml_path.name}: {exc}", file=sys.stderr)
            continue

        out_path = DOCS_DIR / f"{xml_path.stem}.md"
        out_path.write_text(md, encoding="ascii", errors="replace")
        print(f"[OK]   written → {out_path.relative_to(REPO_ROOT)}")


if __name__ == "__main__":
    main()
