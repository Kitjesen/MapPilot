#!/bin/bash
BASE=/home/bsrl/hongsenpang/habitat/navimind_eval
cd $BASE
mkdir -p results
export PYTHONPATH=$BASE:$BASE/src
/home/bsrl/hongsenpang/habitat/conda_env/bin/python eval_objectnav.py \
  --max-episodes 5 --verbose \
  --output results/results_test5.json
