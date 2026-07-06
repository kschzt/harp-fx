#!/usr/bin/env bash
# rebuild.sh — full KR260 reverb re-synth on the sopuli build host, FROM GIT.
#
# Syncs the canonical kernel + scaffolding out of the harp-fx checkout into the
# Vivado build dir, then runs HLS -> block design -> bitstream -> dtbo. The build
# dir (~/kria-reverb) holds only heavy Vivado artifacts; sources are single-sourced
# from ~/src/harp-fx so a stale copy can never miss a fix again.
#
# Usage:  ~/src/harp-fx/fabric/vivado/rebuild.sh          # full pipeline
#         ~/src/harp-fx/fabric/vivado/rebuild.sh --no-pull # skip git pull
set -euo pipefail

CHECKOUT="$HOME/src/harp-fx"
BUILD="$HOME/kria-reverb"
VITIS_SETTINGS="$HOME/xilinx/2025.1/Vitis/settings64.sh"
LOG="$BUILD/resynth-$(date +%Y%m%d-%H%M%S).log"

mkdir -p "$BUILD"
exec > >(tee "$LOG") 2>&1

echo "=== reverb re-synth $(date) ==="
echo "checkout=$CHECKOUT  build=$BUILD  log=$LOG"

# [0] sync sources from git (single source of truth)
if [ "${1:-}" != "--no-pull" ]; then git -C "$CHECKOUT" pull --ff-only; fi
cp -v "$CHECKOUT"/fabric/reverb_defs.h "$CHECKOUT"/fabric/reverb_kernel.cpp "$BUILD"/
cp -v "$CHECKOUT"/fabric/vivado/{build_bd.tcl,build_bits.tcl,run_export.tcl,fan.xdc,make_dtbo.sh,mkdts150.tcl} "$BUILD"/
echo "--- fmul in the source being built (expect round-to-nearest, +half-LSB) ---"
grep -n 'fmul(fx' "$BUILD"/reverb_defs.h

# back up the currently-deployed artifacts if not already saved
for f in reverb.bit.bin reverb.dtbo reverb.xsa; do
  [ -f "$BUILD/$f" ] && [ ! -f "$BUILD/$f.prev" ] && cp "$BUILD/$f" "$BUILD/$f.prev" && echo "saved $f -> $f.prev" || true
done

cd "$BUILD"
# shellcheck disable=SC1090
source "$VITIS_SETTINGS"

echo "=== [1/4] HLS csynth + export IP ($(date +%T)) ==="
vitis_hls -f run_export.tcl

echo "=== [2/4] block design ($(date +%T)) ==="
vivado -mode batch -source build_bd.tcl

echo "=== [3/4] synth + impl + bitstream ($(date +%T)) ==="
vivado -mode batch -source build_bits.tcl
grep -E 'BITS_WNS|BITSTREAM_DONE' vivado.log || true

echo "=== [4/4] device-tree overlay ($(date +%T)) ==="
./make_dtbo.sh

echo "=== DONE $(date) ==="
ls -la reverb.bit.bin reverb.dtbo reverb.xsa
md5sum reverb.bit.bin reverb.dtbo
echo "deploy: scp reverb.bit.bin reverb.dtbo -> kria:/lib/firmware/xilinx/reverb/ ; xmutil unloadapp && xmutil loadapp reverb"
