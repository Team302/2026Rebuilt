#!/usr/bin/env python3
#====================================================================================================================================================
# Copyright 2026 Lake Orion Robotics FIRST Team 302
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
# IN THE SOFTWARE.
#====================================================================================================================================================

"""auton_docs_html_gen.py

Builds a single offline HTML page from generated auton markdown + SVG docs.
The output is intended for Kindle/offline viewing, so all image assets are
embedded directly into the HTML (no external files required).

Usage:
  python3 tools/auton_docs_html_gen.py
  python3 tools/auton_docs_html_gen.py --output documents/auton/AutonDocs.html --title "302 Auton Docs"
"""

from __future__ import annotations

import argparse
import base64
import datetime as dt
import html
import mimetypes
import re
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
DOCS_DIR = REPO_ROOT / "documents" / "auton"
ZONES_DIR = DOCS_DIR / "zones"
DEFAULT_OUTPUT = DOCS_DIR / "AutonDocs.html"


def slugify(text: str) -> str:
    slug = re.sub(r"[^a-z0-9]+", "-", text.lower()).strip("-")
    return slug or "section"


def inline_format(text: str) -> str:
    escaped = html.escape(text)
    escaped = re.sub(r"`([^`]+)`", r"<code>\1</code>", escaped)
    escaped = re.sub(r"\*\*([^*]+)\*\*", r"<strong>\1</strong>", escaped)
    escaped = re.sub(r"\*([^*]+)\*", r"<em>\1</em>", escaped)
    escaped = re.sub(r"\[([^\]]+)\]\(([^)]+)\)", r'<a href="\2">\1</a>', escaped)
    return escaped


def parse_markdown_table(lines: list[str], start_idx: int) -> tuple[str, int]:
    rows = []
    i = start_idx
    while i < len(lines):
        ln = lines[i]
        if "|" not in ln or not ln.strip().startswith("|"):
            break
        rows.append(ln)
        i += 1

    if len(rows) < 2:
        return f"<p>{inline_format(lines[start_idx])}</p>", start_idx + 1

    header_cells = [c.strip() for c in rows[0].strip().strip("|").split("|")]
    body_rows = []
    for row in rows[2:]:
        body_rows.append([c.strip() for c in row.strip().strip("|").split("|")])

    out = ["<table>", "<thead><tr>"]
    out.extend(f"<th>{inline_format(c)}</th>" for c in header_cells)
    out.append("</tr></thead>")
    out.append("<tbody>")
    for row in body_rows:
        out.append("<tr>")
        out.extend(f"<td>{inline_format(c)}</td>" for c in row)
        out.append("</tr>")
    out.append("</tbody></table>")
    return "\n".join(out), i


def wrap_svg_text(text: str, max_chars: int = 34) -> list[str]:
    words = text.split()
    if not words:
        return [""]

    lines: list[str] = []
    current = words[0]
    for word in words[1:]:
        candidate = f"{current} {word}"
        if len(candidate) <= max_chars:
            current = candidate
        else:
            lines.append(current)
            current = word
    lines.append(current)
    return lines


def parse_mermaid_state_diagram(code_text: str) -> dict | None:
    lines = [line.strip() for line in code_text.splitlines() if line.strip()]
    if not lines or lines[0] != "stateDiagram-v2":
        return None

    nodes: dict[str, str] = {}
    classes: dict[str, str] = {}
    transitions: list[dict[str, str]] = []

    for line in lines[1:]:
        if line.startswith("%%") or line.startswith("classDef") or line == "direction LR":
            continue

        label_match = re.match(r"^(Step\d+)\s*:\s*(.+)$", line)
        if label_match:
            nodes[label_match.group(1)] = label_match.group(2)
            continue

        class_match = re.match(r"^class\s+(Step\d+)\s+(\w+)$", line)
        if class_match:
            classes[class_match.group(1)] = class_match.group(2)
            continue

        transition_match = re.match(r"^(\[\*\]|Step\d+)\s+-->\s+(\[\*\]|Step\d+)(?::\s*(.+))?$", line)
        if transition_match:
            transitions.append(
                {
                    "src": transition_match.group(1),
                    "dst": transition_match.group(2),
                    "label": transition_match.group(3) or "",
                }
            )

    if not nodes:
        return None

    ordered_nodes = sorted(nodes.keys(), key=lambda name: int(name.replace("Step", "")))
    return {
        "nodes": ordered_nodes,
        "labels": nodes,
        "classes": classes,
        "transitions": transitions,
    }


def render_mermaid_state_svg(code_text: str) -> str:
    parsed = parse_mermaid_state_diagram(code_text)
    if parsed is None:
        code_html = html.escape(code_text)
        return f'<pre class="mermaid-block"><code>{code_html}</code></pre>'

    node_w = 720
    node_x = 90
    top_margin = 46
    node_gap = 92
    box_heights: dict[str, int] = {}
    node_positions: dict[str, tuple[int, int]] = {}

    current_y = top_margin
    for node_name in parsed["nodes"]:
        lines = wrap_svg_text(parsed["labels"][node_name])
        box_h = max(64, 28 + len(lines) * 18)
        box_heights[node_name] = box_h
        node_positions[node_name] = (node_x, current_y)
        current_y += box_h + node_gap

    svg_h = current_y + 60
    svg_w = 900

    palette = {
        "intake": {"fill": "#1a7a3a", "stroke": "#0d4a1f"},
        "noIntake": {"fill": "#5a4a8a", "stroke": "#3a2a6a"},
        "default": {"fill": "#4e5d6c", "stroke": "#2b3640"},
    }

    pieces = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{svg_w}" height="{svg_h}" viewBox="0 0 {svg_w} {svg_h}" class="state-diagram-svg">',
        "<defs>",
        '<marker id="arrowhead" markerWidth="10" markerHeight="8" refX="8" refY="4" orient="auto">',
        '<polygon points="0 0, 10 4, 0 8" fill="#4a4032"/>',
        "</marker>",
        "</defs>",
        f'<rect x="0" y="0" width="{svg_w}" height="{svg_h}" fill="#f9f6ee" rx="10"/>',
    ]

    start_x = svg_w // 2
    pieces.append(f'<circle cx="{start_x}" cy="22" r="10" fill="#4a4032"/>')

    first_node = parsed["nodes"][0]
    first_x, first_y = node_positions[first_node]
    pieces.append(
        f'<line x1="{start_x}" y1="32" x2="{start_x}" y2="{first_y}" stroke="#4a4032" stroke-width="2.5" marker-end="url(#arrowhead)"/>'
    )
    pieces.append(
        f'<rect x="{start_x + 14}" y="{(32 + first_y) / 2 - 12:.1f}" width="58" height="22" rx="11" fill="#efe8d8" stroke="#b8b09f"/>'
    )
    pieces.append(
        f'<text x="{start_x + 43}" y="{(32 + first_y) / 2 + 3:.1f}" text-anchor="middle" font-size="12" font-family="sans-serif" fill="#3a3126">start</text>'
    )

    transition_map = {(item["src"], item["dst"]): item["label"] for item in parsed["transitions"]}

    for idx, node_name in enumerate(parsed["nodes"]):
        x, y = node_positions[node_name]
        box_h = box_heights[node_name]
        label_lines = wrap_svg_text(parsed["labels"][node_name])
        class_name = parsed["classes"].get(node_name, "default")
        colors = palette.get(class_name, palette["default"])

        pieces.append(
            f'<rect x="{x}" y="{y}" width="{node_w}" height="{box_h}" rx="16" fill="{colors["fill"]}" stroke="{colors["stroke"]}" stroke-width="2.5"/>'
        )

        text_y = y + 26
        for line in label_lines:
            pieces.append(
                f'<text x="{x + node_w / 2:.1f}" y="{text_y}" text-anchor="middle" font-size="18" font-family="sans-serif" fill="#ffffff">{html.escape(line)}</text>'
            )
            text_y += 18

        if idx < len(parsed["nodes"]) - 1:
            next_name = parsed["nodes"][idx + 1]
            next_x, next_y = node_positions[next_name]
            line_y1 = y + box_h
            line_y2 = next_y
            center_x = x + node_w / 2
            pieces.append(
                f'<line x1="{center_x:.1f}" y1="{line_y1}" x2="{center_x:.1f}" y2="{line_y2}" stroke="#4a4032" stroke-width="2.5" marker-end="url(#arrowhead)"/>'
            )

            label = transition_map.get((node_name, next_name), "")
            if label:
                label_lines = wrap_svg_text(label, max_chars=36)
                pill_h = 20 + len(label_lines) * 16
                pill_y = line_y1 + (line_y2 - line_y1 - pill_h) / 2
                pieces.append(
                    f'<rect x="{center_x + 14:.1f}" y="{pill_y:.1f}" width="240" height="{pill_h}" rx="10" fill="#fffdf6" stroke="#b8b09f"/>'
                )
                label_y = pill_y + 16
                for line in label_lines:
                    pieces.append(
                        f'<text x="{center_x + 134:.1f}" y="{label_y:.1f}" text-anchor="middle" font-size="12" font-family="sans-serif" fill="#3a3126">{html.escape(line)}</text>'
                    )
                    label_y += 15

    last_name = parsed["nodes"][-1]
    last_x, last_y = node_positions[last_name]
    last_bottom = last_y + box_heights[last_name]
    last_center = last_x + node_w / 2
    end_outer_y = last_bottom + 42
    end_inner_y = end_outer_y
    pieces.append(
        f'<line x1="{last_center:.1f}" y1="{last_bottom}" x2="{last_center:.1f}" y2="{end_outer_y - 12:.1f}" stroke="#4a4032" stroke-width="2.5" marker-end="url(#arrowhead)"/>'
    )
    end_label = transition_map.get((last_name, "[*]"), "")
    if end_label:
        pieces.append(
            f'<rect x="{last_center + 14:.1f}" y="{last_bottom + 6:.1f}" width="240" height="24" rx="10" fill="#fffdf6" stroke="#b8b09f"/>'
        )
        pieces.append(
            f'<text x="{last_center + 134:.1f}" y="{last_bottom + 22:.1f}" text-anchor="middle" font-size="12" font-family="sans-serif" fill="#3a3126">{html.escape(end_label)}</text>'
        )
    pieces.append(f'<circle cx="{last_center:.1f}" cy="{end_outer_y:.1f}" r="12" fill="#ffffff" stroke="#4a4032" stroke-width="2.5"/>')
    pieces.append(f'<circle cx="{last_center:.1f}" cy="{end_inner_y:.1f}" r="6" fill="#4a4032"/>')
    pieces.append("</svg>")

    return '<div class="mermaid-diagram">' + "".join(pieces) + "</div>"


def markdown_to_html(md_text: str) -> str:
    lines = md_text.splitlines()
    out: list[str] = []
    i = 0

    in_code = False
    code_lang = ""
    code_lines: list[str] = []

    while i < len(lines):
        raw = lines[i]
        line = raw.rstrip("\n")
        stripped = line.strip()

        if stripped.startswith("```"):
            if not in_code:
                in_code = True
                code_lang = stripped[3:].strip().lower()
                code_lines = []
            else:
                if code_lang == "mermaid":
                    out.append(render_mermaid_state_svg("\n".join(code_lines)))
                else:
                    code_html = html.escape("\n".join(code_lines))
                    cls = f' class="code-{code_lang}"' if code_lang else ""
                    out.append(f"<pre{cls}><code>{code_html}</code></pre>")
                in_code = False
                code_lang = ""
                code_lines = []
            i += 1
            continue

        if in_code:
            code_lines.append(line)
            i += 1
            continue

        if stripped.startswith("<") and stripped.endswith(">"):
            out.append(stripped)
            i += 1
            continue

        if not stripped:
            i += 1
            continue

        if stripped.startswith("|") and i + 1 < len(lines) and re.match(r"^\s*\|?\s*[-:]+", lines[i + 1]):
            table_html, next_idx = parse_markdown_table(lines, i)
            out.append(table_html)
            i = next_idx
            continue

        if stripped.startswith("#"):
            level = min(6, len(stripped) - len(stripped.lstrip("#")))
            text = stripped[level:].strip()
            anchor = slugify(text)
            out.append(f'<h{level} id="{anchor}">{inline_format(text)}</h{level}>')
            i += 1
            continue

        if stripped.startswith(">"):
            quote_lines = []
            while i < len(lines) and lines[i].strip().startswith(">"):
                quote_lines.append(lines[i].strip()[1:].strip())
                i += 1
            out.append(f"<blockquote>{'<br/>'.join(inline_format(q) for q in quote_lines)}</blockquote>")
            continue

        if stripped.startswith("- "):
            out.append("<ul>")
            while i < len(lines) and lines[i].strip().startswith("- "):
                item = lines[i].strip()[2:].strip()
                out.append(f"<li>{inline_format(item)}</li>")
                i += 1
            out.append("</ul>")
            continue

        if re.match(r"^\d+\.\s+", stripped):
            out.append("<ol>")
            while i < len(lines) and re.match(r"^\d+\.\s+", lines[i].strip()):
                item = re.sub(r"^\d+\.\s+", "", lines[i].strip())
                out.append(f"<li>{inline_format(item)}</li>")
                i += 1
            out.append("</ol>")
            continue

        if stripped in ("---", "***"):
            out.append("<hr/>")
            i += 1
            continue

        para_lines = [stripped]
        i += 1
        while i < len(lines):
            nxt = lines[i].strip()
            if not nxt:
                i += 1
                break
            if nxt.startswith(("#", "- ", ">", "```", "|")) or re.match(r"^\d+\.\s+", nxt):
                break
            para_lines.append(nxt)
            i += 1
        out.append(f"<p>{inline_format(' '.join(para_lines))}</p>")

    return "\n".join(out)


def inline_image_tag(img_tag: str, base_dir: Path) -> str:
    src_match = re.search(r'src="([^"]+)"', img_tag)
    if not src_match:
        return img_tag

    src = src_match.group(1)
    if src.startswith("http://") or src.startswith("https://") or src.startswith("data:"):
        return img_tag

    asset_path = (base_dir / src).resolve()
    if not asset_path.exists():
        return img_tag

    if asset_path.suffix.lower() == ".svg":
        svg = asset_path.read_text(encoding="utf-8", errors="replace")
        # Keep width from img tag if present by wrapping in container; Kindle handles this well.
        return f'<div class="inline-svg">{svg}</div>'

    mime, _ = mimetypes.guess_type(asset_path.name)
    mime = mime or "application/octet-stream"
    data = base64.b64encode(asset_path.read_bytes()).decode("ascii")
    alt_match = re.search(r'alt="([^"]*)"', img_tag)
    alt = alt_match.group(1) if alt_match else "embedded image"
    return f'<img src="data:{mime};base64,{data}" alt="{html.escape(alt)}"/>'


def inline_local_assets(html_text: str, base_dir: Path) -> str:
    return re.sub(
        r"<img\b[^>]*>",
        lambda m: inline_image_tag(m.group(0), base_dir),
        html_text,
        flags=re.IGNORECASE,
    )


def rewrite_links(html_text: str, link_map: dict[str, str]) -> str:
    def repl(match: re.Match[str]) -> str:
        href = match.group(1)
        text = match.group(2)
        cleaned = href.strip()
        target = link_map.get(cleaned)
        if target:
            return f'<a href="#{target}">{text}</a>'
        if cleaned.startswith("#"):
            return match.group(0)
        if cleaned.startswith("http://") or cleaned.startswith("https://"):
            return match.group(0)
        return f'<span class="local-ref">{text}</span>'

    return re.sub(r'<a href="([^"]+)">(.*?)</a>', repl, html_text)


def section_block(section_id: str, title: str, body: str, compact: bool = False) -> str:
    compact_class = " compact" if compact else ""
    return (
        f'<section id="{section_id}" class="doc-section{compact_class}">\n'
        f'  <h2>{html.escape(title)}</h2>\n'
        f'  <div class="section-body">\n{body}\n  </div>\n'
        f'</section>'
    )


def collect_inputs() -> tuple[Path, list[Path], list[Path]]:
    field_zones = DOCS_DIR / "FieldZones.md"
    auton_docs = sorted(
        p for p in DOCS_DIR.glob("*.md")
        if p.name != "FieldZones.md"
    )
    zone_docs = sorted(ZONES_DIR.glob("*.md")) if ZONES_DIR.exists() else []
    return field_zones, auton_docs, zone_docs


def build_html(title: str) -> str:
    field_zones_md, auton_docs, zone_docs = collect_inputs()

    if not field_zones_md.exists():
        raise FileNotFoundError(f"Missing required file: {field_zones_md}")

    nav_items: list[str] = []
    page_sections: list[str] = []

    link_map: dict[str, str] = {
        "zones/": "zones",
        "../FieldZones.md": "field-zones",
        "FieldZones.md": "field-zones",
    }

    for z in zone_docs:
        link_map[f"zones/{z.name}"] = f"zone-{slugify(z.stem)}"

    nav_items.append('<a href="#overview">Overview</a>')

    generated_utc = dt.datetime.now(dt.timezone.utc).strftime('%Y-%m-%d %H:%M UTC')
    overview_body = (
        f"<p><strong>Generated:</strong> {generated_utc}</p>"
        "<p><strong>Offline mode:</strong> all SVG and image assets are embedded into this file.</p>"
        "<p><strong>Source:</strong> documents/auton/*.md, documents/auton/zones/*.md, documents/auton/svg/*</p>"
    )
    page_sections.append(section_block("overview", "Overview", overview_body, compact=True))

    # Field zones first
    fz_html = markdown_to_html(field_zones_md.read_text(encoding="utf-8", errors="replace"))
    fz_html = inline_local_assets(fz_html, field_zones_md.parent)
    fz_html = rewrite_links(fz_html, link_map)
    page_sections.append(section_block("field-zones", "Field Zones", fz_html))
    nav_items.append('<a href="#field-zones">Field Zones</a>')

    nav_items.append('<a href="#autons">Autons</a>')
    auton_nav_links = []
    auton_sections = ["<section id=\"autons\" class=\"doc-section\"><h2>Autons</h2><div class=\"section-body\">"]
    for md in auton_docs:
        sec_id = f"auton-{slugify(md.stem)}"
        link_map[md.name] = sec_id
        md_html = markdown_to_html(md.read_text(encoding="utf-8", errors="replace"))
        md_html = inline_local_assets(md_html, md.parent)
        md_html = rewrite_links(md_html, link_map)
        auton_nav_links.append(f'<a href="#{sec_id}">{html.escape(md.stem)}</a>')
        auton_sections.append(
            f'<article id="{sec_id}" class="auton-article"><h3>{html.escape(md.stem)}</h3>{md_html}</article>'
        )
    auton_sections.append("</div></section>")
    page_sections.append("\n".join(auton_sections))

    if zone_docs:
        nav_items.append('<a href="#zones">Zone Details</a>')
        zone_nav_links = []
        zone_sections = ["<section id=\"zones\" class=\"doc-section\"><h2>Zone Details</h2><div class=\"section-body\">"]
        for md in zone_docs:
            sec_id = f"zone-{slugify(md.stem)}"
            md_html = markdown_to_html(md.read_text(encoding="utf-8", errors="replace"))
            md_html = inline_local_assets(md_html, md.parent)
            md_html = rewrite_links(md_html, link_map)
            zone_nav_links.append(f'<a href="#{sec_id}">{html.escape(md.stem)}</a>')
            zone_sections.append(
                f'<article id="{sec_id}" class="zone-article"><h3>{html.escape(md.stem)}</h3>{md_html}</article>'
            )
        zone_sections.append("</div></section>")
        page_sections.append("\n".join(zone_sections))
    else:
        zone_nav_links = []

    zone_links_block = ""
    if zone_nav_links:
        zone_links_block = (
            "<div class=\"toc-subtitle\">Zone List</div>"
            f"<div class=\"toc-list\">{' '.join(zone_nav_links)}</div>"
        )

    nav_block = (
        "<nav class=\"toc\">"
        "<div class=\"toc-title\">Navigate</div>"
        f"<div class=\"toc-major\">{' '.join(nav_items)}</div>"
        "<div class=\"toc-subtitle\">Auton List</div>"
        f"<div class=\"toc-list\">{' '.join(auton_nav_links)}</div>"
        f"{zone_links_block}"
        "</nav>"
    )

    styles = """
:root {
  --bg: #f0efe8;
  --panel: #fffdf6;
  --ink: #1d1d1b;
  --muted: #5f5b52;
  --line: #b8b09f;
  --accent: #0c5d57;
  --accent-soft: #d8efe9;
}
html, body {
  margin: 0;
  padding: 0;
  background: var(--bg);
  color: var(--ink);
  font-family: Georgia, "Times New Roman", serif;
  font-size: 18px;
  line-height: 1.45;
}
a { color: #004b70; text-decoration: underline; }
a:visited { color: #3f2f73; }
.layout {
    display: flex;
    gap: 18px;
    align-items: flex-start;
  max-width: 1600px;
  margin: 0 auto;
  padding: 14px;
}
.toc {
    width: 300px;
    min-width: 240px;
  position: sticky;
  top: 0;
  max-height: 100vh;
  overflow: auto;
  background: var(--panel);
  border: 1px solid var(--line);
  border-radius: 8px;
  padding: 12px;
}
.toc-title { font-weight: bold; font-size: 1.1rem; margin-bottom: 8px; }
.toc-subtitle { font-weight: bold; margin: 10px 0 4px 0; }
.toc-major a, .toc-list a {
  display: block;
  padding: 8px 6px;
  margin: 2px 0;
  border-radius: 6px;
  background: transparent;
}
.toc-major a { background: var(--accent-soft); }
main {
    flex: 1;
    min-width: 0;
  background: var(--panel);
  border: 1px solid var(--line);
  border-radius: 8px;
  padding: 16px;
}
.doc-section { margin-bottom: 24px; border-bottom: 1px solid var(--line); padding-bottom: 14px; }
.doc-section.compact p { margin: 0.35rem 0; }
.doc-section h2 { margin: 0 0 12px 0; font-size: 1.5rem; }
.auton-article, .zone-article {
  border: 1px solid #d6cfbf;
  border-radius: 8px;
  padding: 12px;
  margin: 12px 0;
  background: #fff;
}
.auton-article h3, .zone-article h3 {
  margin-top: 0;
  font-size: 1.2rem;
  border-bottom: 1px solid #e8e2d4;
  padding-bottom: 6px;
}
.inline-svg svg { width: 100%; height: auto; }
.mermaid-diagram {
    overflow-x: auto;
    margin: 12px 0;
    padding: 6px 0;
}
.state-diagram-svg {
    width: 100%;
    height: auto;
    display: block;
}
pre {
  overflow: auto;
  background: #f5f2e8;
  border: 1px solid #ccc5b4;
  border-radius: 6px;
  padding: 10px;
  font-size: 0.92rem;
}
.mermaid-block {
  background: #f0f7f4;
  border-left: 6px solid #0c5d57;
}
table {
  border-collapse: collapse;
  width: 100%;
  margin: 12px 0;
  font-size: 0.95rem;
}
th, td {
  border: 1px solid #bcb3a2;
  padding: 6px 8px;
  vertical-align: top;
}
th { background: #efe8d8; }
blockquote {
  margin: 10px 0;
  border-left: 4px solid #0c5d57;
  padding: 4px 10px;
  background: #f4faf8;
}
.local-ref {
  color: var(--muted);
  text-decoration: none;
}
@media (max-width: 1024px) {
  .layout {
        display: block;
    padding: 10px;
  }
  .toc {
    position: static;
        width: auto;
        min-width: 0;
    max-height: none;
        margin-bottom: 12px;
  }
}
"""

    page_title = html.escape(title)
    body = "\n".join(page_sections)
    return f"""<!doctype html>
<html lang=\"en\">
<head>
  <meta charset=\"utf-8\"/>
  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"/>
  <title>{page_title}</title>
  <style>{styles}</style>
</head>
<body>
  <div class=\"layout\">
    {nav_block}
    <main>
      <h1>{page_title}</h1>
      {body}
    </main>
  </div>
</body>
</html>
"""


def validate_offline(html_text: str) -> list[str]:
    issues = []
    ext_refs = re.findall(r'(?:src|href)="(https?://[^\"]+)"', html_text, flags=re.IGNORECASE)
    if ext_refs:
        issues.append("Found external src/href references (http/https).")
    if re.search(r"<img\b[^>]*src=\"(?!data:)[^#\"]+\"", html_text, flags=re.IGNORECASE):
        issues.append("Found non-embedded image src references.")
    return issues


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate single-file offline auton HTML docs.")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="Output HTML file path.")
    parser.add_argument("--title", default="Team 302 Auton Reference", help="Page title.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_path = args.output.resolve()
    html_text = build_html(args.title)

    issues = validate_offline(html_text)
    if issues:
        for issue in issues:
            print(f"[WARN] {issue}")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(html_text, encoding="utf-8")
    rel = output_path.relative_to(REPO_ROOT)
    print(f"[OK] written -> {rel}")


if __name__ == "__main__":
    main()
