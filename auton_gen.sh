#!/bin/bash
# Regenerate autonomous and zone documentation locally

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
readonly ROOT="$SCRIPT_DIR"
readonly AUTON_DOCS_HTML="$ROOT/documents/auton/AutonDocs.html"

cd "$ROOT"
printf "Working from: %s\n" "$ROOT"

printf "\nRegenerating zone fields...\n"
python3 tools/auton_zone_field_gen.py

printf "\nGenerating autonomous documentation...\n"
find src/main/deploy/auton -maxdepth 1 -name "*.xml" -print0 | sort -z | while IFS= read -r -d '' xmlfile; do
  printf "  Processing: %s\n" "$(basename "$xmlfile")"
  python3 tools/auton_diagram_gen.py "$xmlfile"
done
auton_count=$(find src/main/deploy/auton -maxdepth 1 -name "*.xml" | wc -l)

printf "\nBuilding offline HTML...\n"
python3 tools/auton_docs_html_gen.py --output "$AUTON_DOCS_HTML"

if [[ ! -f "$AUTON_DOCS_HTML" ]]; then
  printf "Error: HTML generation failed\n" >&2
  exit 1
fi

printf "\n✓ Generated %d auton page(s)\n" "$auton_count"
ls -lh "$AUTON_DOCS_HTML"
