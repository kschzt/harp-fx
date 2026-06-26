/* rev_fabric.h — the reusable KR260 reverb-fabric driver for harp-fx-deviced.
 *
 * Factored verbatim from fabric/reverb_fabric.c's `#ifdef FABRIC` path (the known-good
 * standalone driver): mmap the kernel's AXI-Lite control regs (/dev/uio4) + the u-dma-buf
 * DDR scratch (/dev/udmabuf0), write the 4 DDR pointers + coefficients once, then per block
 * fill XIN (Q26), pulse ap_start, spin on ap_done, read the wet stereo (YOUT, Q26).
 *
 * Built only into the -DHARPFX_FABRIC device variant (the KR260 target). The coefficients are
 * NOT recomputed here — load_coeffs reads them straight off the engine's `reverb` struct
 * (rev_init's output in reverb_engine_core.h), so there is ONE source of truth for the DSP math
 * and the fabric stays bit-exact to the software path / the standalone reverb_fabric tool.
 *
 * Device-memory discipline (aarch64, u-dma-buf O_SYNC mapping = Device memory): every DDR store
 * MUST be an aligned scalar through a `volatile` pointer — NO memset, NO NEON vector stores
 * (they SIGBUS). The .c keeps that discipline; callers just pass plain Q26 buffers.
 */
#ifndef REV_FABRIC_H
#define REV_FABRIC_H

#include "reverb_engine_core.h" /* the `reverb` struct (rev_init output) + fx */
#include "reverb_defs.h"        /* struct rev_coeffs + RESN + BS (DDR layout) */

/* open + mmap /dev/uio4 (ctrl) and /dev/udmabuf0 (DDR), program the 4 DDR base pointers.
 * Returns 0 on success, <0 on failure (perror'd). Call once before load_coeffs/block. */
int  rev_fabric_open(void);

/* push the frozen coefficients (a1/r2/gi + n_res/diff_on/dg/cg/widthf/wetg_k) from the engine's
 * already-computed reverb to DDR — ONCE per stream. R is the rev_init() result. */
void rev_fabric_load_coeffs(const reverb *R);

/* process ONE block on the fabric: bs mono Q26 inputs (xin) -> 2*bs interleaved Q26 wet stereo
 * (yout_stereo). do_init=1 on the first block of a stream (kernel zeroes state + caches coeffs).
 * One ap_start/ap_done handshake; emits silence if the fabric was never opened. */
void rev_fabric_block(const fx *xin, int bs, int do_init, fx *yout_stereo);

#endif /* REV_FABRIC_H */
