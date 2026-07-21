#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="${ROOT:-/home/sunrise/hongsenpang/Yolov11_project/vehicle_parking_detection}"
KEEP_RUNS="${KEEP_RUNS:-1}"
EXPECTED_ROOT="/home/sunrise/hongsenpang/Yolov11_project/vehicle_parking_detection"

if [ "$ROOT" != "$EXPECTED_ROOT" ]; then
  echo "ERROR: refusing to clean unexpected ROOT=$ROOT"
  echo "expected: $EXPECTED_ROOT"
  exit 2
fi

if ! [[ "$KEEP_RUNS" =~ ^[0-9]+$ ]]; then
  echo "ERROR: KEEP_RUNS must be a non-negative integer, got: $KEEP_RUNS"
  exit 3
fi

if [ ! -d "$ROOT" ]; then
  echo "ERROR: missing module root: $ROOT"
  exit 4
fi

echo "=== vehicle parking board storage cleanup ==="
date
echo "root=$ROOT"
echo "keep_latest_runs=$KEEP_RUNS"
echo

echo "=== disk before ==="
df -h "$ROOT" || true
echo

echo "=== delete package temp files ==="
rm -f \
  "$ROOT/vehicle_parking_return.tgz" \
  "$ROOT/vehicle_parking_return.tgz.sha256" \
  "$ROOT"/vehicle_parking_return.tgz.part.* \
  "$ROOT/vehicle_parking_alarms.tgz" \
  "$ROOT/vehicle_parking_alarms.tgz.sha256" \
  "$ROOT"/vehicle_parking_alarms.tgz.part.* 2>/dev/null || true
echo "package temp files cleaned"
echo

echo "=== clean old output runs ==="
if [ ! -d "$ROOT/output" ]; then
  echo "no output directory: $ROOT/output"
else
  mapfile -t runs < <(find "$ROOT/output" -mindepth 1 -maxdepth 1 -type d | sort)
  total="${#runs[@]}"
  delete_count=$((total - KEEP_RUNS))
  if [ "$delete_count" -lt 0 ]; then
    delete_count=0
  fi

  echo "total_runs=$total delete_old_runs=$delete_count keep_runs=$((total - delete_count))"
  for i in "${!runs[@]}"; do
    run_path="${runs[$i]}"
    case "$run_path" in
      "$ROOT"/output/*) ;;
      *)
        echo "ERROR: refusing suspicious output path: $run_path"
        exit 5
        ;;
    esac

    if [ "$i" -lt "$delete_count" ]; then
      echo "DELETE old output: $run_path"
      rm -rf -- "$run_path"
    else
      echo "KEEP latest output: $run_path"
    fi
  done
fi
echo

echo "=== remaining sizes ==="
du -sh "$ROOT/output" "$ROOT/logs" 2>/dev/null || true
echo

echo "=== disk after ==="
df -h "$ROOT" || true
echo
echo "CLEANUP_DONE"
