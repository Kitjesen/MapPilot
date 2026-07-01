#!/bin/bash
# ============================================================
# clone_semantic_deps.sh — Bootstrap VLN third-party dependencies
#
# Usage:
#   bash scripts/build/clone_semantic_deps.sh
#
# This script clones all third-party repos needed for the
# semantic navigation (VLN) pipeline into third_party/.
# Model weights are NOT downloaded automatically — see the
# printed instructions at the end.
# ============================================================

set -euo pipefail

find_repo_root() {
  local dir="$1"
  while [[ "$dir" != "/" ]]; do
    if [[ -f "$dir/pyproject.toml" && -f "$dir/AGENTS.md" ]]; then
      printf '%s\n' "$dir"
      return 0
    fi
    dir="$(dirname "$dir")"
  done
  return 1
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(find_repo_root "$SCRIPT_DIR")"
THIRD_PARTY="$REPO_ROOT/third_party"

mkdir -p "$THIRD_PARTY"
cd "$THIRD_PARTY"

# ── Helper ──────────────────────────────────────────────────
clone_if_missing() {
  local name="$1"
  local url="$2"
  if [ -d "$name" ]; then
    echo "[skip] $name already exists"
  else
    echo "[clone] $name ← $url"
    git clone --depth 1 "$url" "$name"
  fi
}

# ── Core dependencies (must clone) ─────────────────────────

echo ""
echo "=== [1/4] LOVON — Quadruped Open-Vocabulary Navigator (MIT) ==="
clone_if_missing "LOVON" "https://github.com/DaojiePENG/LOVON.git"

echo ""
echo "=== [2/4] GroundingDINO — Open-Vocabulary Detection (Apache 2.0) ==="
clone_if_missing "GroundingDINO" "https://github.com/IDEA-Research/GroundingDINO.git"

echo ""
echo "=== [3/4] ConceptGraphs — 3D Semantic Scene Graph (MIT) ==="
clone_if_missing "concept-graphs" "https://github.com/concept-graphs/concept-graphs.git"

# ── Reference (prompt templates / architecture patterns) ────

echo ""
echo "=== [4/4] SG-Nav — Scene Graph + LLM Navigation (MIT) ==="
clone_if_missing "SG-Nav" "https://github.com/bagh2178/SG-Nav.git"

# ── Summary ─────────────────────────────────────────────────

echo ""
echo "============================================================"
echo " All repos cloned into: $THIRD_PARTY"
echo "============================================================"
echo ""
echo " Next steps (manual):"
echo ""
echo "   1. Install GroundingDINO:"
echo "      cd third_party/GroundingDINO && pip install -e ."
echo ""
echo "   2. Download GroundingDINO weights (~900 MB):"
echo "      mkdir -p third_party/GroundingDINO/weights"
echo "      wget -P third_party/GroundingDINO/weights/ \\"
echo "        https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha2/groundingdino_swinb_cogcoor.pth"
echo ""
echo "   3. Install ConceptGraphs:"
echo "      cd third_party/concept-graphs && pip install -e ."
echo ""
echo "   4. Download MobileSAM checkpoint:"
echo "      mkdir -p third_party/concept-graphs/checkpoints"
echo "      wget -P third_party/concept-graphs/checkpoints/ \\"
echo "        https://github.com/ChaoningZhang/MobileSAM/raw/master/weights/mobile_sam.pt"
echo ""
echo "   5. (Optional) Install LOVON dependencies:"
echo "      cd third_party/LOVON && pip install -r requirements.txt"
echo ""
