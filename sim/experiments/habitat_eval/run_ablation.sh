#!/bin/bash
# ============================================================
# NaviMind 消融实验批量运行脚本
# 在 HM3D ObjectNav 上运行 5 个变体, 输出对比表
#
# 用法:
#   bash run_ablation.sh                    # 完整 val (2000 episodes)
#   bash run_ablation.sh --max-episodes 50  # 快速验证
#   bash run_ablation.sh --resume           # 断点续评
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
EVAL_SCRIPT="$SCRIPT_DIR/eval_objectnav.py"
RESULTS_DIR="$SCRIPT_DIR/results"
EXTRA_ARGS="${@}"

mkdir -p "$RESULTS_DIR"

# 激活评测环境 (bsrl GPU 服务器: venv at /home/bsrl/hongsenpang/habitat/conda_env)
BASE=/home/bsrl/hongsenpang/habitat/navimind_eval
PY=/home/bsrl/hongsenpang/habitat/conda_env/bin/python
export PYTHONPATH=$BASE:$BASE/src

# SeetaCloud 回退 (root/miniconda)
if [ ! -f "$PY" ]; then
    PY="$(which python3 || which python)"
    BASE="$(dirname "$EVAL_SCRIPT")"
    export PYTHONPATH=$BASE
fi

echo "============================================"
echo "  NaviMind 消融实验"
echo "  额外参数: $EXTRA_ARGS"
echo "  结果目录: $RESULTS_DIR"
echo "============================================"

ABLATIONS=("full" "no_belief" "no_fov" "no_hierarchy" "always_llm")

for ablation in "${ABLATIONS[@]}"; do
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  运行: $ablation"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    OUTPUT="$RESULTS_DIR/results_${ablation}.json"

    "$PY" "$EVAL_SCRIPT" \
        --ablation "$ablation" \
        --output "$OUTPUT" \
        $EXTRA_ARGS \
        2>&1 | tee "$RESULTS_DIR/log_${ablation}.txt"

    echo "  ✓ $ablation 完成, 结果: $OUTPUT"
done

# 汇总对比
echo ""
echo "============================================"
echo "  消融实验汇总"
echo "============================================"

"$PY" -c "
import json, os, sys
from collections import defaultdict

results_dir = '$RESULTS_DIR'
ablations = ['full', 'no_belief', 'no_fov', 'no_hierarchy', 'always_llm']

print(f\"{'Variant':<18} {'SR%':>8} {'SPL':>8} {'SoftSPL':>8} {'DTG':>8} {'Steps':>8}\")
print('-' * 60)

for abl in ablations:
    path = os.path.join(results_dir, f'results_{abl}.json')
    if not os.path.exists(path):
        print(f'{abl:<18} (missing)')
        continue
    with open(path) as f:
        data = json.load(f)
    results = data.get('results', data) if isinstance(data, dict) else data
    n = len(results)
    if n == 0:
        print(f'{abl:<18} (empty)')
        continue
    sr = sum(1 for r in results if r['success']) / n * 100
    spl = sum(r['spl'] for r in results) / n
    sspl = sum(r.get('soft_spl', 0) for r in results) / n
    dtg = sum(r['distance_to_goal'] for r in results) / n
    steps = sum(r['steps'] for r in results) / n
    print(f'{abl:<18} {sr:>7.1f}% {spl:>8.4f} {sspl:>8.4f} {dtg:>7.2f}m {steps:>8.1f}')

print()
print('LaTeX Table Row (copy to paper):')
for abl in ablations:
    path = os.path.join(results_dir, f'results_{abl}.json')
    if not os.path.exists(path):
        continue
    with open(path) as f:
        data = json.load(f)
    results = data.get('results', data) if isinstance(data, dict) else data
    n = len(results)
    if n == 0:
        continue
    sr = sum(1 for r in results if r['success']) / n * 100
    spl = sum(r['spl'] for r in results) / n
    sspl = sum(r.get('soft_spl', 0) for r in results) / n
    dtg = sum(r['distance_to_goal'] for r in results) / n
    name = {'full': r'\\textbf{NaviMind (Ours)}', 'no_belief': 'w/o BA-HSG Belief',
            'no_fov': 'w/o FOV-aware', 'no_hierarchy': 'w/o Hierarchy',
            'always_llm': 'Always LLM'}.get(abl, abl)
    print(f'{name} & {sr:.1f} & {spl:.3f} & {sspl:.3f} & {dtg:.2f} \\\\\\\\')
" 2>/dev/null || echo "(汇总脚本需要 Python)"

echo ""
echo "全部消融实验完成!"
