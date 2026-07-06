#!/usr/bin/env bash
# make_dtbo.sh — createdts from reverb.xsa, then compile pl.dtsi -> reverb.dtbo
set -e
cd ~/kria-reverb
source /home/jak/xilinx/2025.1/Vitis/settings64.sh 2>/dev/null
rm -rf dtg150
xsct mkdts150.tcl 2>&1 | tail -5
PLDTSI=$(find dtg150 -name pl.dtsi | head -1)
# reverb_kernel must bind uio_pdrv_genirq: createdts emits xlnx,reverb-kernel-1.0; the
# host driver mmaps it as /dev/uio4, so force generic-uio before compiling the overlay.
sed -i 's/"xlnx,reverb-kernel-1.0"/"generic-uio"/' "$PLDTSI"
echo "PLDTSI=$PLDTSI"
DTDIR=$(dirname "$PLDTSI")
# compile the overlay: system-top includes pl.dtsi; build the PL overlay dtbo
dtc -@ -O dtb -o reverb.dtbo -I dts "$PLDTSI" 2>&1 | tail -5 || true
ls -la reverb.dtbo reverb.bit.bin 2>&1
