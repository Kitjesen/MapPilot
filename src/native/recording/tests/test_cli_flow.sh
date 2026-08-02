#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -ne 5 ]; then
  echo "usage: test_cli_flow.sh RECORDER PLAYER PUBLISHER SUBSCRIBER IDL" >&2
  exit 2
fi

RECORDER="$1"
PLAYER="$2"
PUBLISHER="$3"
SUBSCRIBER="$4"
IDL="$5"
DOMAIN=198
REPLAY_DOMAIN=200
TMP_DIR="$(mktemp -d)"
RECORDER_PID=""
SUBSCRIBER_PID=""
cleanup() {
  if [ -n "$RECORDER_PID" ]; then
    kill "$RECORDER_PID" 2>/dev/null || true
  fi
  if [ -n "$SUBSCRIBER_PID" ]; then
    kill "$SUBSCRIBER_PID" 2>/dev/null || true
  fi
  rm -rf -- "$TMP_DIR"
}
trap cleanup EXIT

"$PLAYER" --list-topics >"$TMP_DIR/topics.log"
grep -Eq '^/imu/raw[[:space:]]+rt/imu/raw[[:space:]]+lingtu\.dds\.Imu[[:space:]]+replayable$' "$TMP_DIR/topics.log"
grep -Eq '^/nav/cmd_vel[[:space:]]+rt/nav/cmd_vel[[:space:]]+lingtu\.dds\.FinalVelocityCommand[[:space:]]+record-only$' "$TMP_DIR/topics.log"

"$RECORDER" \
  --output "$TMP_DIR/session.mcap" \
  --idl "$IDL" \
  --domain "$DOMAIN" \
  --seconds 1.2 \
  --require-topic /imu/raw >"$TMP_DIR/recorder.log" 2>&1 &
RECORDER_PID=$!
sleep 0.2
"$PUBLISHER" "$DOMAIN"
wait "$RECORDER_PID"
RECORDER_PID=""

test -s "$TMP_DIR/session.mcap"
test ! -e "$TMP_DIR/session.mcap.tmp"
"$PLAYER" --info "$TMP_DIR/session.mcap" >"$TMP_DIR/info.log"
grep -Eq '^profile=lingtu\.dds\.v1 messages=[1-9][0-9]* duration_ns=[0-9]+$' "$TMP_DIR/info.log"
grep -Eq '^topic=/imu/raw dds_topic=rt/imu/raw count=[1-9][0-9]* idl_type=lingtu\.dds\.Imu policy=replayable$' "$TMP_DIR/info.log"

"$PLAYER" \
  "$TMP_DIR/session.mcap" \
  --idl "$IDL" \
  --domain 84 \
  --dry-run >"$TMP_DIR/player.log"
grep -Eq '^validated=[1-9][0-9]* domain=84 rate=1' "$TMP_DIR/player.log"

"$SUBSCRIBER" "$REPLAY_DOMAIN" >"$TMP_DIR/subscriber.log" 2>&1 &
SUBSCRIBER_PID=$!
sleep 0.2
"$PLAYER" \
  "$TMP_DIR/session.mcap" \
  --idl "$IDL" \
  --domain "$REPLAY_DOMAIN" >"$TMP_DIR/replay.log"
wait "$SUBSCRIBER_PID"
SUBSCRIBER_PID=""
grep -Eq '^replayed=[1-9][0-9]* domain=200 rate=1' "$TMP_DIR/replay.log"

if "$PLAYER" \
  "$TMP_DIR/session.mcap" \
  --idl "$IDL" \
  --domain 0 \
  --dry-run >"$TMP_DIR/live-domain.log" 2>&1; then
  echo "player accepted live DDS domain without explicit opt-in" >&2
  exit 1
fi
grep -Fq 'live field domain' "$TMP_DIR/live-domain.log"

"$PLAYER" \
  "$TMP_DIR/session.mcap" \
  --idl "$IDL" \
  --domain 0 \
  --allow-live-domain \
  --dry-run >"$TMP_DIR/live-domain-opt-in.log"
grep -Eq '^validated=[1-9][0-9]* domain=0 rate=1' "$TMP_DIR/live-domain-opt-in.log"

if "$RECORDER" \
  --output "$TMP_DIR/empty.mcap" \
  --idl "$IDL" \
  --domain 199 \
  --seconds 0.05 \
  /imu/raw >"$TMP_DIR/empty.log" 2>&1; then
  echo "recorder committed a session without sensor samples" >&2
  exit 1
fi
grep -Fq 'no sensor samples were captured' "$TMP_DIR/empty.log"
test ! -e "$TMP_DIR/empty.mcap"
test -e "$TMP_DIR/empty.mcap.tmp"

REQUIRED_DOMAIN=201
"$RECORDER" \
  --output "$TMP_DIR/incomplete.mcap" \
  --idl "$IDL" \
  --domain "$REQUIRED_DOMAIN" \
  --seconds 1.2 \
  --require-topic /slam/odometry \
  /imu/raw /slam/odometry >"$TMP_DIR/incomplete.log" 2>&1 &
RECORDER_PID=$!
sleep 0.2
"$PUBLISHER" "$REQUIRED_DOMAIN"
if wait "$RECORDER_PID"; then
  echo "recorder committed a session without required odometry evidence" >&2
  exit 1
fi
RECORDER_PID=""
grep -Fq 'required recording topic captured no samples: /slam/odometry' \
  "$TMP_DIR/incomplete.log"
test ! -e "$TMP_DIR/incomplete.mcap"
test -e "$TMP_DIR/incomplete.mcap.tmp"
