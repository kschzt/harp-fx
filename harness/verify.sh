#!/usr/bin/env bash
# verify.sh — §8.8 audio.fx end-to-end check: pace demos/dry.wav through harp-fx-deviced as
# an EFFECT and assert the WET return is BIT-EXACT to the src/reverb.c reverb oracle.
#
#   cmake -S device -B device/build -DHARP_ROOT=/path/to/harp && cmake --build device/build
#   harness/verify.sh
#
# Exits non-zero unless the device's float32 wet stream is bit-for-bit the oracle's.
set -u
cd "$(dirname "$0")/.."
DEV=device/build/harp-fx-deviced
TEST=device/build/fx-audio-test
PORT="${PORT:-48050}"; HPP="${HPP:-48051}"; SERIAL="${SERIAL:-FX-0001}"
DEVDIR=$(mktemp -d); DEVLOG=$(mktemp); WET="${TMPDIR:-/tmp}/fx_devwet.wav"
[ -x "$DEV" ]  || { echo "verify: $DEV not built (cmake --build device/build)"; exit 1; }
[ -x "$TEST" ] || { echo "verify: $TEST not built";  exit 1; }

DP=""
trap '[ -n "$DP" ] && kill -9 "$DP" 2>/dev/null; rm -rf "$DEVDIR" "$DEVLOG"' EXIT
"$DEV" --serial "$SERIAL" --port "$PORT" --state-dir "$DEVDIR" --panel-sock "" >"$DEVLOG" 2>&1 & DP=$!
for _ in $(seq 1 30); do grep -q "listening on $PORT" "$DEVLOG" 2>/dev/null && break; sleep 0.2; done
grep -q "listening on $PORT" "$DEVLOG" || { echo "verify: device did not start"; cat "$DEVLOG"; exit 1; }

perl -e 'alarm 90; exec @ARGV' "$TEST" "127.0.0.1:$PORT" demos/dry.wav "$WET" "$HPP"
rc=$?
[ "$rc" = 142 ] && { echo "verify: harness HUNG"; cat "$DEVLOG"; exit 1; }
if [ "$rc" = 0 ]; then echo "VERIFY PASS: §8.8 audio.fx device reproduces the reverb BIT-EXACT (device float32 == oracle)"
else echo "VERIFY FAIL (rc=$rc)"; cat "$DEVLOG"; fi
exit "$rc"
