#!/usr/bin/env python3
"""auton_zone_field_gen.py

Reads all zone XML files under src/main/deploy/auton/zones/ and generates
documents/auton/FieldZones.md containing an inline SVG field diagram with
every zone drawn to scale on the full FRC field.

Field coordinate system (from zone XML files and ZoneParser.cpp):
  - Full field: 16.46 m wide (x) x 8.10 m tall (y)
  - Blue alliance zone: x = 0..4.0
  - Neutral zone:       x = 4.8..11.86
  - Red alliance zone:  x = 12.55..16.55
  - Origin (0,0) is bottom-left; y increases upward in field coords.
  - SVG y-axis is flipped (0 = top), so we transform y -> (FIELD_H - y).
  - Radius values in zone XMLs are in CENTIMETRES (radius=70 -> 0.70 m).
  - Rect and circle x/y values are in METRES.

Usage:
  python3 tools/auton_zone_field_gen.py
"""

import xml.etree.ElementTree as ET
from pathlib import Path

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
ZONES_DIR = REPO_ROOT / "src" / "main" / "deploy" / "auton" / "zones"
OUT_PATH = REPO_ROOT / "documents" / "auton" / "FieldZones.md"
ZONE_PAGES_DIR = REPO_ROOT / "documents" / "auton" / "zones"
# SVG files live next to their .md files so relative <img> paths work on GitHub
SVG_DIR = REPO_ROOT / "documents" / "auton" / "svg"
ZONE_SVG_DIR = REPO_ROOT / "documents" / "auton" / "zones" / "svg"

# ---------------------------------------------------------------------------
# Field dimensions (metres)
# ---------------------------------------------------------------------------
FIELD_W = 16.46
FIELD_H = 8.10

# ---------------------------------------------------------------------------
# SVG canvas dimensions (pixels)
# ---------------------------------------------------------------------------
SVG_W = 900
SVG_H = int(SVG_W * FIELD_H / FIELD_W)
SCALE = SVG_W / FIELD_W   # pixels per metre


def mx(x_m: float) -> float:
    """Field x (metres) -> SVG x (pixels)."""
    return x_m * SCALE


def my(y_m: float) -> float:
    """Field y (metres) -> SVG y (pixels, flipped so 0 is bottom of field)."""
    return (FIELD_H - y_m) * SCALE


# ---------------------------------------------------------------------------
# Colour palette (all hex, ASCII-safe)
# Keyed by allianceColor attribute value; fallback = neutral grey.
# ---------------------------------------------------------------------------
ZONE_COLORS = {
    "BLUE":    {"fill": "#1a3a8a", "stroke": "#0d2060", "text": "#ffffff"},
    "RED":     {"fill": "#8a1a1a", "stroke": "#601010", "text": "#ffffff"},
    "BOTH":    {"fill": "#2a6a2a", "stroke": "#1a4a1a", "text": "#ffffff"},
    "NEUTRAL": {"fill": "#4a4a4a", "stroke": "#2a2a2a", "text": "#ffffff"},
}

# Effect -> badge colour
EFFECT_COLORS = {
    "DRIVE_OVER_BUMP":   "#e08000",
    "DRIVE_TO_DEPOT":    "#8000c0",
    "DRIVE_TO_OUTPOST":  "#006080",
    "DRIVE_TO_TOWER":    "#c06000",
    "DRIVE_TO_HUB":      "#008080",
    "PREPARE_TO_LAUNCH": "#c00040",
    "WANT_TO_CLIMB":     "#804000",
}

OPACITY = "0.55"


def parse_zone(xml_path: Path) -> dict | None:
    """Parse a single zone XML file and return a shape dict, or None on error."""
    try:
        root = ET.parse(xml_path).getroot()
    except ET.ParseError:
        return None

    for z in root.iter("zone"):
        shape: dict = {"name": xml_path.stem, "file": xml_path.name}

        # Alliance colour
        shape["alliance"] = z.get("allianceColor", "BOTH")

        # Effects
        effects = []
        puo = z.get("pathUpdateOption", "NOTHING")
        if puo and puo != "NOTHING":
            effects.append(puo.replace("DRIVE_", "").replace("_", " "))
        ls = z.get("launcherState", "STATE_IDLE")
        if ls and ls not in ("STATE_IDLE", "STATE_OFF"):
            effects.append(ls.replace("STATE_", "").replace("_", " "))
        cs = z.get("climberState", "STATE_OFF")
        if cs and cs != "STATE_OFF":
            effects.append(cs.replace("STATE_", "").replace("_", " "))
        shape["effects"] = effects
        shape["pathUpdateOption"] = puo

        # Shape geometry
        if z.get("circlex") is not None:
            shape["type"] = "circle"
            shape["cx"] = float(z.get("circlex", 0))
            shape["cy"] = float(z.get("circley", 0))
            # radius is stored in centimetres in the XML
            shape["r"] = float(z.get("radius", 0)) / 100.0
        elif z.get("x1_rect") is not None:
            shape["type"] = "rect"
            x1 = float(z.get("x1_rect", 0))
            y1 = float(z.get("y1_rect", 0))
            x2 = float(z.get("x2_rect", 0))
            y2 = float(z.get("y2_rect", 0))
            # Normalise so x1 < x2, y1 < y2
            shape["x1"] = min(x1, x2)
            shape["y1"] = min(y1, y2)
            shape["x2"] = max(x1, x2)
            shape["y2"] = max(y1, y2)
        else:
            return None  # no usable geometry

        return shape
    return None


def alliance_color_props(alliance: str) -> dict:
    return ZONE_COLORS.get(alliance, ZONE_COLORS["NEUTRAL"])


def effect_badge_color(effects: list[str]) -> str:
    for e in effects:
        key = e.replace(" ", "_").upper()
        for k, v in EFFECT_COLORS.items():
            if k in key:
                return v
    return "#555555"


def truncate_label(name: str, max_chars: int = 18) -> str:
    """Strip common prefixes to keep labels short."""
    for prefix in ("Blue", "Red", "Neutral"):
        if name.startswith(prefix):
            name = name[len(prefix):]
            break
    return name if len(name) <= max_chars else name[:max_chars - 2] + ".."


def svg_shape(shape: dict, idx: int) -> str:
    """Return SVG elements for one zone shape."""
    props = alliance_color_props(shape["alliance"])
    fill = props["fill"]
    stroke = props["stroke"]
    text_col = props["text"]
    badge_col = effect_badge_color(shape["effects"])
    name = shape["name"]
    label = truncate_label(name)
    clip_id = f"clip{idx}"
    lines = []

    if shape["type"] == "circle":
        cx, cy, r = shape["cx"], shape["cy"], shape["r"]
        # Skip circles whose centre is wildly off-field (old/unused zones)
        if cx > FIELD_W * 1.5 or cy > FIELD_H * 1.5:
            return f"  <!-- SKIPPED off-field circle: {name} cx={cx} cy={cy} -->\n"
        svgcx = mx(cx)
        svgcy = my(cy)
        svgr = r * SCALE
        lines.append(
            f'  <circle cx="{svgcx:.1f}" cy="{svgcy:.1f}" r="{svgr:.1f}" '
            f'fill="{fill}" fill-opacity="{OPACITY}" stroke="{stroke}" stroke-width="1.5"/>'
        )
        # Label at the TOP of the circle so overlapping zones don't pile text in the centre
        label_y = svgcy - svgr + 10
        # Dark pill background for legibility
        pill_w = len(label) * 5.5 + 6
        lines.append(
            f'  <rect x="{svgcx - pill_w/2:.1f}" y="{label_y - 8:.1f}" '
            f'width="{pill_w:.1f}" height="10" rx="2" '
            f'fill="#000000" fill-opacity="0.55"/>'
        )
        lines.append(
            f'  <text x="{svgcx:.1f}" y="{label_y:.1f}" '
            f'text-anchor="middle" dominant-baseline="auto" '
            f'font-size="9" fill="{text_col}" font-family="monospace">{label}</text>'
        )
        # Effect badge immediately below name pill
        if shape["effects"]:
            badge_text = " ".join(shape["effects"])[:20]
            badge_y = label_y + 11
            bpill_w = len(badge_text) * 4.5 + 6
            lines.append(
                f'  <rect x="{svgcx - bpill_w/2:.1f}" y="{badge_y - 8:.1f}" '
                f'width="{bpill_w:.1f}" height="10" rx="2" '
                f'fill="#000000" fill-opacity="0.55"/>'
            )
            lines.append(
                f'  <text x="{svgcx:.1f}" y="{badge_y:.1f}" '
                f'text-anchor="middle" dominant-baseline="auto" '
                f'font-size="7" fill="{badge_col}" font-family="monospace">{badge_text}</text>'
            )
    elif shape["type"] == "rect":
        x1, y1, x2, y2 = shape["x1"], shape["y1"], shape["x2"], shape["y2"]
        # Skip rects entirely off-field
        if x1 > FIELD_W * 1.5:
            return f"  <!-- SKIPPED off-field rect: {name} x1={x1} -->\n"
        # Clamp to field bounds for display
        rx1 = mx(max(0, min(x1, FIELD_W)))
        ry1 = my(max(0, min(y2, FIELD_H)))   # note: SVG y flip
        rw = mx(max(0, min(x2, FIELD_W))) - rx1
        rh = my(max(0, min(y1, FIELD_H))) - ry1
        if rw <= 0 or rh <= 0:
            return f"  <!-- SKIPPED zero-size rect: {name} -->\n"
        # Pin label to top-left corner of the rect (+ small padding)
        label_x = rx1 + 4
        label_y = ry1 + 10
        lines.append(
            f'  <rect x="{rx1:.1f}" y="{ry1:.1f}" width="{rw:.1f}" height="{rh:.1f}" '
            f'fill="{fill}" fill-opacity="{OPACITY}" stroke="{stroke}" stroke-width="1.5"/>'
        )
        # Dark pill background so text is legible over any fill
        pill_w = len(label) * 5.5 + 6
        lines.append(
            f'  <rect x="{label_x - 3:.1f}" y="{label_y - 8:.1f}" '
            f'width="{pill_w:.1f}" height="10" rx="2" '
            f'fill="#000000" fill-opacity="0.55"/>'
        )
        lines.append(
            f'  <text x="{label_x:.1f}" y="{label_y:.1f}" '
            f'text-anchor="start" dominant-baseline="auto" '
            f'font-size="9" fill="{text_col}" font-family="monospace">{label}</text>'
        )
        if shape["effects"]:
            badge_text = " ".join(shape["effects"])[:20]
            badge_y = label_y + 11
            bpill_w = len(badge_text) * 4.5 + 6
            lines.append(
                f'  <rect x="{label_x - 3:.1f}" y="{badge_y - 8:.1f}" '
                f'width="{bpill_w:.1f}" height="10" rx="2" '
                f'fill="#000000" fill-opacity="0.55"/>'
            )
            lines.append(
                f'  <text x="{label_x:.1f}" y="{badge_y:.1f}" '
                f'text-anchor="start" dominant-baseline="auto" '
                f'font-size="7" fill="{badge_col}" font-family="monospace">{badge_text}</text>'
            )
    return "\n".join(lines) + "\n"


def field_background() -> str:
    """SVG background: field outline + zone region shading + axis labels."""
    w, h = SVG_W, SVG_H
    lines = []

    # Field background
    lines.append(f'  <rect x="0" y="0" width="{w}" height="{h}" fill="#1a1a1a"/>')

    # Blue alliance region (x=0..4m)
    bw = mx(4.0)
    lines.append(f'  <rect x="0" y="0" width="{bw:.1f}" height="{h}" fill="#0d1a3a" fill-opacity="0.6"/>')
    lines.append(f'  <text x="{bw/2:.1f}" y="14" text-anchor="middle" font-size="11" fill="#5588ff" font-family="sans-serif">BLUE ALLIANCE</text>')

    # Neutral zone (x=4.8..11.86m)
    nzx = mx(4.8)
    nzw = mx(11.86) - nzx
    lines.append(f'  <rect x="{nzx:.1f}" y="0" width="{nzw:.1f}" height="{h}" fill="#1a1a1a" fill-opacity="0.6"/>')
    lines.append(f'  <text x="{nzx + nzw/2:.1f}" y="14" text-anchor="middle" font-size="11" fill="#aaaaaa" font-family="sans-serif">NEUTRAL ZONE</text>')

    # Red alliance region (x=12.55..16.46m)
    rx = mx(12.55)
    rw2 = w - rx
    lines.append(f'  <rect x="{rx:.1f}" y="0" width="{rw2:.1f}" height="{h}" fill="#3a0d0d" fill-opacity="0.6"/>')
    lines.append(f'  <text x="{rx + rw2/2:.1f}" y="14" text-anchor="middle" font-size="11" fill="#ff5555" font-family="sans-serif">RED ALLIANCE</text>')

    # Field border
    lines.append(f'  <rect x="0" y="0" width="{w}" height="{h}" fill="none" stroke="#888888" stroke-width="2"/>')

    # X-axis tick marks every 2m
    for xm in range(0, int(FIELD_W) + 1, 2):
        sx = mx(xm)
        lines.append(f'  <line x1="{sx:.1f}" y1="{h}" x2="{sx:.1f}" y2="{h - 6}" stroke="#666" stroke-width="1"/>')
        lines.append(f'  <text x="{sx:.1f}" y="{h - 8}" text-anchor="middle" font-size="8" fill="#888" font-family="monospace">{xm}m</text>')

    # Y-axis tick marks every 2m
    for ym in range(0, int(FIELD_H) + 1, 2):
        sy = my(ym)
        lines.append(f'  <line x1="0" y1="{sy:.1f}" x2="6" y2="{sy:.1f}" stroke="#666" stroke-width="1"/>')
        lines.append(f'  <text x="8" y="{sy:.1f}" dominant-baseline="middle" font-size="8" fill="#888" font-family="monospace">{ym}m</text>')

    return "\n".join(lines) + "\n"


def generate_svg(shapes: list[dict]) -> str:
    body = field_background()
    for i, s in enumerate(shapes):
        body += svg_shape(s, i)
    return (
        f'<svg xmlns="http://www.w3.org/2000/svg" '
        f'width="{SVG_W}" height="{SVG_H}" '
        f'viewBox="0 0 {SVG_W} {SVG_H}">\n'
        + body
        + "</svg>"
    )


def generate_table(shapes: list[dict]) -> str:
    header = "| Zone | Type | Key Coords (m) | Alliance | Effects |"
    sep = "|------|------|----------------|----------|---------|"
    rows = []
    for s in shapes:
        if s["type"] == "circle":
            coords = f"cx={s['cx']}, cy={s['cy']}, r={s['r']:.2f}"
        else:
            coords = f"({s['x1']},{s['y1']}) to ({s['x2']},{s['y2']})"
        effects = ", ".join(s["effects"]) if s["effects"] else "--"
        rows.append(f"| `{s['name']}` | {s['type']} | {coords} | {s['alliance']} | {effects} |")
    return header + "\n" + sep + "\n" + "\n".join(rows)


def generate_legend() -> str:
    lines = ["| Colour | Meaning |", "|--------|---------|"]
    lines.append("| Dark blue background | Blue alliance zone (x = 0 -- 4.0 m) |")
    lines.append("| Dark region (centre) | Neutral zone (x = 4.8 -- 11.86 m) |")
    lines.append("| Dark red background | Red alliance zone (x = 12.55 -- 16.46 m) |")
    lines.append("| Blue fill zones | Zones with allianceColor = BLUE |")
    lines.append("| Red fill zones | Zones with allianceColor = RED |")
    lines.append("| Green fill zones | Zones with allianceColor = BOTH |")
    lines.append("| Grey fill zones | Zones with no allianceColor set |")
    lines.append("| Orange badge text | DRIVE_OVER_BUMP effect |")
    lines.append("| Purple badge text | DRIVE_TO_DEPOT effect |")
    lines.append("| Teal badge text | DRIVE_TO_OUTPOST / HUB effect |")
    lines.append("| Red-pink badge text | PREPARE_TO_LAUNCH effect |")
    return "\n".join(lines)


def generate_individual_svg(focus: dict, all_shapes: list[dict]) -> str:
    """Full-field SVG that highlights one zone and dims all others."""
    lines = []
    w, h = SVG_W, SVG_H

    # --- background identical to the combined diagram ---
    lines.append(field_background().rstrip())

    # --- draw all other zones first, more transparent ---
    for i, s in enumerate(all_shapes):
        if s["name"] == focus["name"]:
            continue
        props = alliance_color_props(s["alliance"])
        fill = props["fill"]
        stroke = props["stroke"]
        if s["type"] == "circle":
            cx, cy, r = s["cx"], s["cy"], s["r"]
            if cx > FIELD_W * 1.5 or cy > FIELD_H * 1.5:
                continue
            svgcx, svgcy, svgr = mx(cx), my(cy), r * SCALE
            lines.append(
                f'  <circle cx="{svgcx:.1f}" cy="{svgcy:.1f}" r="{svgr:.1f}" '
                f'fill="{fill}" fill-opacity="0.18" stroke="{stroke}" stroke-width="0.8" stroke-opacity="0.4"/>'
            )
        elif s["type"] == "rect":
            x1, y1, x2, y2 = s["x1"], s["y1"], s["x2"], s["y2"]
            if x1 > FIELD_W * 1.5:
                continue
            rx1 = mx(max(0, min(x1, FIELD_W)))
            ry1 = my(max(0, min(y2, FIELD_H)))
            rw = mx(max(0, min(x2, FIELD_W))) - rx1
            rh = my(max(0, min(y1, FIELD_H))) - ry1
            if rw <= 0 or rh <= 0:
                continue
            lines.append(
                f'  <rect x="{rx1:.1f}" y="{ry1:.1f}" width="{rw:.1f}" height="{rh:.1f}" '
                f'fill="{fill}" fill-opacity="0.18" stroke="{stroke}" stroke-width="0.8" stroke-opacity="0.4"/>'
            )

    # --- draw the focus zone at full opacity with a bright highlight stroke ---
    props = alliance_color_props(focus["alliance"])
    fill = props["fill"]
    stroke = props["stroke"]
    text_col = props["text"]
    badge_col = effect_badge_color(focus["effects"])
    name = focus["name"]

    if focus["type"] == "circle":
        cx, cy, r = focus["cx"], focus["cy"], focus["r"]
        svgcx, svgcy, svgr = mx(cx), my(cy), r * SCALE
        # Glow ring
        lines.append(
            f'  <circle cx="{svgcx:.1f}" cy="{svgcy:.1f}" r="{svgr + 4:.1f}" '
            f'fill="none" stroke="#ffff00" stroke-width="2.5" stroke-opacity="0.7"/>'
        )
        lines.append(
            f'  <circle cx="{svgcx:.1f}" cy="{svgcy:.1f}" r="{svgr:.1f}" '
            f'fill="{fill}" fill-opacity="0.75" stroke="{stroke}" stroke-width="2"/>'
        )
        # Centre label
        label = name
        cx_label = svgcx
        cy_label = svgcy
        pill_w = len(label) * 6.5 + 10
        lines.append(
            f'  <rect x="{cx_label - pill_w/2:.1f}" y="{cy_label - 9:.1f}" '
            f'width="{pill_w:.1f}" height="13" rx="3" fill="#000000" fill-opacity="0.7"/>'
        )
        lines.append(
            f'  <text x="{cx_label:.1f}" y="{cy_label:.1f}" '
            f'text-anchor="middle" dominant-baseline="middle" '
            f'font-size="11" font-weight="bold" fill="{text_col}" font-family="monospace">{label}</text>'
        )
        if focus["effects"]:
            badge_text = " ".join(focus["effects"])
            by = cy_label + 16
            bpill_w = len(badge_text) * 5.5 + 10
            lines.append(
                f'  <rect x="{cx_label - bpill_w/2:.1f}" y="{by - 8:.1f}" '
                f'width="{bpill_w:.1f}" height="12" rx="3" fill="#000000" fill-opacity="0.7"/>'
            )
            lines.append(
                f'  <text x="{cx_label:.1f}" y="{by:.1f}" '
                f'text-anchor="middle" dominant-baseline="middle" '
                f'font-size="9" fill="{badge_col}" font-family="monospace">{badge_text}</text>'
            )

    elif focus["type"] == "rect":
        x1, y1, x2, y2 = focus["x1"], focus["y1"], focus["x2"], focus["y2"]
        rx1 = mx(max(0, min(x1, FIELD_W)))
        ry1 = my(max(0, min(y2, FIELD_H)))
        rw = mx(max(0, min(x2, FIELD_W))) - rx1
        rh = my(max(0, min(y1, FIELD_H))) - ry1
        # Glow outline
        lines.append(
            f'  <rect x="{rx1 - 3:.1f}" y="{ry1 - 3:.1f}" width="{rw + 6:.1f}" height="{rh + 6:.1f}" '
            f'fill="none" stroke="#ffff00" stroke-width="2.5" stroke-opacity="0.7" rx="2"/>'
        )
        lines.append(
            f'  <rect x="{rx1:.1f}" y="{ry1:.1f}" width="{rw:.1f}" height="{rh:.1f}" '
            f'fill="{fill}" fill-opacity="0.75" stroke="{stroke}" stroke-width="2"/>'
        )
        # Centre label
        label = name
        cx_label = rx1 + rw / 2
        cy_label = ry1 + rh / 2
        pill_w = len(label) * 6.5 + 10
        lines.append(
            f'  <rect x="{cx_label - pill_w/2:.1f}" y="{cy_label - 9:.1f}" '
            f'width="{pill_w:.1f}" height="13" rx="3" fill="#000000" fill-opacity="0.7"/>'
        )
        lines.append(
            f'  <text x="{cx_label:.1f}" y="{cy_label:.1f}" '
            f'text-anchor="middle" dominant-baseline="middle" '
            f'font-size="11" font-weight="bold" fill="{text_col}" font-family="monospace">{label}</text>'
        )
        if focus["effects"]:
            badge_text = " ".join(focus["effects"])
            by = cy_label + 16
            bpill_w = len(badge_text) * 5.5 + 10
            lines.append(
                f'  <rect x="{cx_label - bpill_w/2:.1f}" y="{by - 8:.1f}" '
                f'width="{bpill_w:.1f}" height="12" rx="3" fill="#000000" fill-opacity="0.7"/>'
            )
            lines.append(
                f'  <text x="{cx_label:.1f}" y="{by:.1f}" '
                f'text-anchor="middle" dominant-baseline="middle" '
                f'font-size="9" fill="{badge_col}" font-family="monospace">{badge_text}</text>'
            )

    body = "\n".join(lines) + "\n"
    return (
        f'<svg xmlns="http://www.w3.org/2000/svg" '
        f'width="{SVG_W}" height="{SVG_H}" '
        f'viewBox="0 0 {SVG_W} {SVG_H}">\n'
        + body
        + "</svg>"
    )


def generate_zone_page(shape: dict, svg_rel: str) -> str:
    """Generate full markdown content for an individual zone page.

    svg_rel -- path to the SVG file relative to the zone .md file.
    """
    name = shape["name"]
    alliance = shape["alliance"]
    effects = shape["effects"]
    stype = shape["type"]

    if stype == "circle":
        geom_lines = [
            f"| Type | Circle |",
            f"| Centre X | {shape['cx']} m |",
            f"| Centre Y | {shape['cy']} m |",
            f"| Radius | {shape['r']:.2f} m |",
        ]
    else:
        geom_lines = [
            f"| Type | Rectangle |",
            f"| X1 | {shape['x1']} m |",
            f"| Y1 | {shape['y1']} m |",
            f"| X2 | {shape['x2']} m |",
            f"| Y2 | {shape['y2']} m |",
            f"| Width | {shape['x2'] - shape['x1']:.3f} m |",
            f"| Height | {shape['y2'] - shape['y1']:.3f} m |",
        ]

    geom_table = "| Property | Value |\n|----------|-------|\n" + "\n".join(geom_lines)
    effects_str = ", ".join(effects) if effects else "None"

    return f"""# Zone: {name}

> Auto-generated from [`src/main/deploy/auton/zones/{name}.xml`](../../../src/main/deploy/auton/zones/{name}.xml).
> Do not edit manually -- changes to the XML will trigger regeneration via the
> [auton-diagrams workflow](../../../.github/workflows/auton-diagrams.yml).
>
> All other zones are shown dimmed for context. The highlighted zone (yellow outline) is `{name}`.

## Field Diagram

<img src="{svg_rel}" alt="Field diagram highlighting {name}" width="{SVG_W}"/>

## Zone Properties

| Property | Value |
|----------|-------|
| Name | `{name}` |
| Alliance | {alliance} |
| Effects | {effects_str} |

## Geometry

{geom_table}

---

[Back to all zones](../FieldZones.md)
"""


def main() -> None:
    zone_files = sorted(ZONES_DIR.glob("*.xml"))
    shapes = []
    skipped = []
    for zf in zone_files:
        s = parse_zone(zf)
        if s:
            shapes.append(s)
        else:
            skipped.append(zf.stem)

    # -----------------------------------------------------------------------
    # Write the combined field SVG and FieldZones.md
    # -----------------------------------------------------------------------
    SVG_DIR.mkdir(parents=True, exist_ok=True)
    combined_svg = generate_svg(shapes)
    combined_svg_path = SVG_DIR / "FieldZones.svg"
    combined_svg_path.write_text(combined_svg, encoding="ascii", errors="replace")
    print(f"[OK] written -> {combined_svg_path.relative_to(REPO_ROOT)}")

    table = generate_table(shapes)
    legend = generate_legend()

    skipped_note = ""
    if skipped:
        skipped_note = (
            "\n> **Note:** the following zone files had no parseable geometry "
            "and are omitted from the diagram: "
            + ", ".join(f"`{s}`" for s in skipped)
            + "\n"
        )

    zone_links = "\n".join(
        f"- [{s['name']}](zones/{s['name']}.md)" for s in shapes
    )

    # Path from FieldZones.md (documents/auton/) to the SVG (documents/auton/svg/)
    field_svg_rel = "svg/FieldZones.svg"

    doc = f"""# Auton Field Zones

> Auto-generated from [`src/main/deploy/auton/zones/`](../../src/main/deploy/auton/zones/).
> Do not edit this file manually -- push a change to a zone XML and the
> [auton-diagrams workflow](../../.github/workflows/auton-diagrams.yml) will regenerate it.
>
> Full field: **{FIELD_W} m wide x {FIELD_H} m tall**.
> Origin (0,0) is the bottom-left (Blue alliance wall).
> Circle radius values in the XML are stored in **centimetres** and converted to metres here.
{skipped_note}
## Field Diagram

<img src="{field_svg_rel}" alt="All auton field zones" width="{SVG_W}"/>

## Individual Zone Pages

{zone_links}

## Zone Legend

{legend}

## Zone Reference Table

{table}
"""
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text(doc, encoding="ascii", errors="replace")
    print(f"[OK] written -> {OUT_PATH.relative_to(REPO_ROOT)}")

    # -----------------------------------------------------------------------
    # Write one SVG + one .md page per zone
    # -----------------------------------------------------------------------
    ZONE_PAGES_DIR.mkdir(parents=True, exist_ok=True)
    ZONE_SVG_DIR.mkdir(parents=True, exist_ok=True)
    for shape in shapes:
        name = shape["name"]
        # Write SVG file
        zone_svg = generate_individual_svg(shape, shapes)
        zone_svg_path = ZONE_SVG_DIR / f"{name}.svg"
        zone_svg_path.write_text(zone_svg, encoding="ascii", errors="replace")
        print(f"[OK] written -> {zone_svg_path.relative_to(REPO_ROOT)}")

        # Path from zones/<Name>.md to zones/svg/<Name>.svg
        svg_rel = f"svg/{name}.svg"
        page_path = ZONE_PAGES_DIR / f"{name}.md"
        page_content = generate_zone_page(shape, svg_rel)
        page_path.write_text(page_content, encoding="ascii", errors="replace")
        print(f"[OK] written -> {page_path.relative_to(REPO_ROOT)}")

    if skipped:
        print(f"[SKIP] {len(skipped)} zone(s) had no geometry: {skipped}")


if __name__ == "__main__":
    main()
