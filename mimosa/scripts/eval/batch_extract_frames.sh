#!/usr/bin/env bash
# Extract specific camera frames (at 15s, 40%, 90%) from every run_* bag dir under
# the given parent directory.
#
# Usage:
#   batch_extract_frames.sh <parent_dir> [glob]
#   batch_extract_frames.sh /mnt/.../19_05
#   batch_extract_frames.sh /mnt/.../19_05 'run_2026*'

set -euo pipefail

PARENT="${1:-}"
GLOB="${2:-run_*}"
FORCE="${3:-}"
if [[ -z "$PARENT" || ! -d "$PARENT" ]]; then
  echo "usage: $0 <parent_dir> [glob] [--force]"; exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

shopt -s nullglob
runs=("$PARENT"/$GLOB)
shopt -u nullglob

if [[ ${#runs[@]} -eq 0 ]]; then
  echo "[error] no bags matched ${PARENT}/${GLOB}"; exit 1
fi

echo "[info] processing ${#runs[@]} bag(s) under $PARENT"

for bag in "${runs[@]}"; do
  [[ -d "$bag" ]] || continue
  [[ -f "$bag/metadata.yaml" ]] || { echo "[skip] $bag has no metadata.yaml"; continue; }
  name="$(basename "$bag")"
  echo
  echo "=============================="
  echo "[bag] $name"
  echo "=============================="

  OUTDIR="$bag/frames"

  if [[ "$FORCE" != "--force" && -d "$OUTDIR" && -n "$(ls -A "$OUTDIR" 2>/dev/null)" ]]; then
    echo "[skip] frames already extracted in $OUTDIR"
  else
    echo "[step] extract frames"
    python3 "$SCRIPT_DIR/extract_frames.py" "$bag" "$OUTDIR" || echo "[warn] extraction failed for $name"
  fi
done

echo
echo "[done] extracted all frames for bags under $PARENT"
