# fabric/vivado — KR260 reverb bitstream build scaffolding

The Vivado/Vitis flow that turns the pure-integer reverb DSP (`../reverb_kernel.cpp`
+ `../reverb_defs.h`) into a loadable KR260 PL app: **HLS IP → block design → bitstream
→ device-tree overlay**. This is the "on-target HW" half of HARP §8.8 audio.fx — the C
engine is golden/scaffolding; *this* produces the deliverable that runs on the fabric.

> Committed here because it was almost lost once: the block design + build scripts lived
> only on the build host (`sopuli`) and are not reproducible from anything else. Now they are.

## Toolchain / host
- **sopuli** (x86-64 Linux, user `jak`), **Vivado + Vitis 2025.1** at `~/xilinx/2025.1/`.
- Build working directory: **`~/kria-reverb`** (heavy Vivado artifacts live there, not in git).
- Part `xck26-sfvc784-2LV-c`, board `xilinx.com:kr260_som:part0:1.1`.

## The kernel sources are canonical in `../`
`reverb_kernel.cpp` and `reverb_defs.h` are the SAME files as `fabric/reverb_kernel.cpp`
and `fabric/reverb_defs.h` (the bit-exact Q26 engine). Copy them into `~/kria-reverb/`
before building so the bitstream matches the software golden. Keep them in sync — a stale
copy on the build host is exactly how the DC-bias fix (`fmul` round-to-nearest) missed the
last synthesized bitstream.

## Build pipeline (run in `~/kria-reverb`)
```sh
source ~/xilinx/2025.1/Vitis/settings64.sh          # HLS + xsct
vitis_hls -f run_export.tcl                          # 1. HLS csynth -> reverb_ip + reverb_kernel_ip.zip
vivado  -mode batch -source build_bd.tcl             # 2. PS + kernel IP + fan route -> design_1.bd
vivado  -mode batch -source build_bits.tcl           # 3. synth + impl -> reverb.bit.bin + reverb.xsa
./make_dtbo.sh                                        # 4. XSA -> reverb.dtbo (forces generic-uio)
```
Outputs to deploy: **`reverb.bit.bin`** + **`reverb.dtbo`** →
`/lib/firmware/xilinx/reverb/` on the Kria, then `xmutil unloadapp && xmutil loadapp reverb`.

## What each script does
| file | step | notes |
|---|---|---|
| `run_export.tcl` | HLS csynth + `export_design` IP | top `reverb_kernel`, part xck26, 5 ns clock (200 MHz target) |
| `build_bd.tcl` | block design | Zynq US+ PS (board preset) + reverb HLS IP + smartconnects + **fan route**; `PL0=150 MHz`, `S_AXI_HP0` for DMA |
| `build_bits.tcl` | synth/impl/bits | reads `fan.xdc`, `ExtraTimingOpt` + `AggressiveExplore`, writes `.bin` + XSA |
| `make_dtbo.sh` + `mkdts150.tcl` | overlay | `createdts` from the XSA, then `sed` the kernel node to `generic-uio` so the host mmaps `/dev/uio4` |
| `fan.xdc` | pin constraint | `fan_en_b` → pin **A12**, LVCMOS33 |

## Fan route (and the known post-reboot "dead fan")
`build_bd.tcl` routes **TTC0 WAVEOUT → xlslice bit0 → NOT (invert) → `fan_en_b` (A12)**;
polarity is **inverted** (a `0` duty drives the fan hardest, `255`/max = off). TTC0's EMIO
`clk_in` is tied to a 3-bit `0` constant so the counter free-runs on its PS clock; the Linux
`ttc-pwm` driver programs the waveform.

**Known issue:** after a reboot with the reverb app loaded, the fan stays off even at high
drive. The bitstream route is fine (works on the base shell); the suspect is a **channel/bit
mismatch** — the route taps wave **bit 0** (`FAN_WAVE_BIT 0`), but the deployed overlay's
`pwm-fan` node drove a different TTC channel. Fix is in the **device tree** (`make_dtbo.sh`
output / `pwm-fan` `pwms` channel), NOT a re-synth. The board runs thermally safe fanless
(~56 °C plateau) regardless.
