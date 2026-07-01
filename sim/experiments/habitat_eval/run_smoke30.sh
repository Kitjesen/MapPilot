#!/bin/bash
BASE=/home/bsrl/hongsenpang/habitat/navimind_eval
cd $BASE
mkdir -p results
export PYTHONPATH=$BASE:$BASE/src
nohup /home/bsrl/hongsenpang/habitat/conda_env/bin/python eval_objectnav.py \
  --max-episodes 30 --verbose \
  --output results/results_honest_full_30.json \
  > results/log_smoke30.txt 2>&1 &
echo "PID=$!"
wait
