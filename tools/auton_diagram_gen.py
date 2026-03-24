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
import os
import xml.etree.ElementTree as ET
from pathlib import Path

# ---------------------------------------------------------------------------
# Paths (relative to repo root, which is two levels above this script)
# ---------------------------------------------------------------------------
SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
ZONES_DIR = REPO_ROOT / "src" / "main" / "deploy" / "auton" / "zones"
DOCS_DIR = REPO_ROOT / "documents" / "auton"

# Default attribute values from auton.dtd
DTD_DEFAULTS = {
    "id": "DO_NOTHING",
    "launcherState": "STATE_IDLE",
    "intakeState": "STATE_OFF",
    "climberState": "STATE_OFF",
    "time": "--",
    "choreoname": "",
}


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
        summary_rows.append(
            {
                "step": i,
                "primitive": prim_id,
                "trajectory": choreo,
                "timeout": timeout,
                "intake": intake,
                "launcher": launcher,
                "climber": climber,
                "zones": ", ".join(z.replace(".xml", "") for z in zones) or "—",
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
        mermaid_lines.append(f"    Step{d['step']} : Step {d['step']} - {d['prim_id']}{traj_part}")
    mermaid_lines.append("")

    # Apply classDef per step based on intake state
    for d in step_data:
        css_class = "noIntake" if d["intake"].replace("STATE_", "") == "OFF" else "intake"
        mermaid_lines.append(f"    class Step{d['step']} {css_class}")
    mermaid_lines.append("")

    # Transitions
    mermaid_lines.append("    [*] --> Step1 : start")
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

        # Join parts with a space — no commas in the arrow label
        arrow_label = (": " + " ".join(parts)) if parts else ""
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
    # 4. Assemble full document                                            #
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
