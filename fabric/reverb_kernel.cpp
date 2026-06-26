/* reverb_kernel.cpp — src/reverb.c rev_process() as a Vitis HLS kernel (HARP §8.8 audio-in FX).
 *
 * Fabric partition (mirrors the modal port): the PS daemon does WAV i/o, the double→Q26/Q55
 * coefficient setup, and the float→Q26 input + Q26→int16 output quantization; the PL holds ALL
 * reverb STATE (resonator s1/s2, the 4 input-diffusion allpass rings, the 2x3 decorrelation
 * allpass rings, the bandpass input history) in BRAM persisting across ap_ctrl_hs calls and runs
 * the bit-exact integer recurrence.
 *
 * AUDIO-IN (the new thing vs the modal synth): this kernel takes an INPUT AUDIO block (xin) and
 * produces a WET STEREO output block (yout). Per block: read n dry samples from DDR (gmem0), run
 * rev_process per sample, write n stereo wet samples to DDR (gmem1). Coefficients arrive once
 * (do_init=1): they are DMA'd into BRAM and all state is zeroed.
 *
 * src/reverb.c is already PURE Q26 integer (the one ex-float line — the wet scale — is now the
 * shared wet_scale(): a Q55 host coefficient times a 128-bit product). So there is NO float in
 * this kernel; a bit-exact match to src/reverb.c (hence GOLDEN_wet.wav) is the goal.
 *
 * Two builds:
 *   - Vitis HLS (__VITIS_HLS__): real interface pragmas; loops kept ROLLED/sequential.
 *   - plain C++ (software testbench): pragmas compile out; same int logic, linked into
 *     reverb_fabric.c (built without -DFABRIC) and validated bit-exact vs src/reverb.c.
 */
#include "reverb_defs.h"

#ifdef __VITIS_HLS__
  #include <ap_int.h>
  #define HLS_PRAGMA(x) _Pragma(#x)
  typedef ap_int<128> i128;
#else
  #define HLS_PRAGMA(x)
  typedef __int128 i128;
#endif

/* wet scale, PURE INTEGER — identical to src/reverb.c wet_scale().
 * wf = round(sum * wetg_k / 2^55), 128-bit product, round-half-away-from-zero (== old llround).
 * Q55 wetg_k captures the wet gain to full double precision → bit-exact to the float original. */
static fx wet_scale(int64_t sum, int64_t wetg_k){
    i128 p = (i128)sum * (i128)wetg_k;
    i128 h = (i128)1 << (WSHIFT - 1);
    int64_t r = (p >= 0) ? (int64_t)(( p + h) >> WSHIFT)
                         : -(int64_t)((-p + h) >> WSHIFT);
    return (fx)r;
}

/* canonical Schroeder allpass — identical math to src/reverb.c ap_proc(); the ring position is
 * advanced by the CALLER (avoids a pointer-to-scalar arg, friendlier to HLS). */
static fx ap_proc(fx buf[], int pos, fx g, fx x){
    fx wd = buf[pos];
    fx w  = x + fmul(g, wd);
    fx y  = wd - fmul(g, w);
    buf[pos] = w;
    return y;
}

/* persistent fabric state — lives in BRAM across ap_start pulses.
 * 2*RESN (state) + 3*RESN (coeffs) int32 = 60 KB + tiny allpass rings; fits the XCK26. */
struct rev_state {
    fx s1[RESN], s2[RESN];                 /* resonator state               */
    fx a1[RESN], r2[RESN], gi[RESN];       /* cached coeffs (loaded on init) */
    fx d0[D0], d1[D1], d2[D2], d3[D3];     /* input-diffusion rings          */
    int dp0, dp1, dp2, dp3;
    fx ca0[A0], ca1[A1], ca2[A2];          /* L decorrelation rings          */
    fx cb0[B0], cb1[B1], cb2[B2];          /* R decorrelation rings          */
    int pa0, pa1, pa2, pb0, pb1, pb2;
    fx xm1, xm2;                           /* bandpass input history         */
    int32_t n_res, diff_on, dg, cg, widthf;/* cached scalars                 */
    int64_t wetg_k;
};

/* TOP. Process n dry samples → n wet stereo samples for the current state.
 * Block-render handshake (== modal_kernel / harp render_output): the PS writes coeffs+audio to
 * DDR, pulses ap_start; the kernel reads them, fills yout. Kernel is its own AXI master. */
extern "C" void reverb_kernel(const struct rev_coeffs *cf,   /* gmem0  scalars        */
                              const fx                *coef, /* gmem0  a1|r2|gi packed*/
                              const fx                *xin,  /* gmem0  dry  [n]       */
                              fx                      *yout, /* gmem1  wet  [2*n]     */
                              int                      n,
                              int                      do_init)
{
    HLS_PRAGMA(HLS INTERFACE m_axi port=cf   bundle=gmem0 offset=slave depth=1)
    HLS_PRAGMA(HLS INTERFACE m_axi port=coef bundle=gmem0 offset=slave depth=9000)
    HLS_PRAGMA(HLS INTERFACE m_axi port=xin  bundle=gmem0 offset=slave depth=512)
    HLS_PRAGMA(HLS INTERFACE m_axi port=yout bundle=gmem1 offset=slave depth=1024)
    HLS_PRAGMA(HLS INTERFACE s_axilite port=cf      bundle=ctrl)
    HLS_PRAGMA(HLS INTERFACE s_axilite port=coef    bundle=ctrl)
    HLS_PRAGMA(HLS INTERFACE s_axilite port=xin     bundle=ctrl)
    HLS_PRAGMA(HLS INTERFACE s_axilite port=yout    bundle=ctrl)
    HLS_PRAGMA(HLS INTERFACE s_axilite port=n       bundle=ctrl)
    HLS_PRAGMA(HLS INTERFACE s_axilite port=do_init bundle=ctrl)
    HLS_PRAGMA(HLS INTERFACE s_axilite port=return  bundle=ctrl)

    static rev_state S;                    /* BRAM, persists across calls */

    /* (re)load coefficients + zero all state. do_init=1 on the first block only. */
    if (do_init) {
        struct rev_coeffs CF = *cf;
        S.n_res = CF.n_res; S.diff_on = CF.diff_on; S.dg = CF.dg;
        S.cg = CF.cg; S.widthf = CF.widthf; S.wetg_k = CF.wetg_k;
        for (int i = 0; i < RESN; i++) {
            HLS_PRAGMA(HLS PIPELINE off)
            S.a1[i] = coef[i];
            S.r2[i] = coef[RESN + i];
            S.gi[i] = coef[2 * RESN + i];
            S.s1[i] = 0; S.s2[i] = 0;
        }
        for (int i = 0; i < D0; i++) S.d0[i] = 0;
        for (int i = 0; i < D1; i++) S.d1[i] = 0;
        for (int i = 0; i < D2; i++) S.d2[i] = 0;
        for (int i = 0; i < D3; i++) S.d3[i] = 0;
        for (int i = 0; i < A0; i++) S.ca0[i] = 0;
        for (int i = 0; i < A1; i++) S.ca1[i] = 0;
        for (int i = 0; i < A2; i++) S.ca2[i] = 0;
        for (int i = 0; i < B0; i++) S.cb0[i] = 0;
        for (int i = 0; i < B1; i++) S.cb1[i] = 0;
        for (int i = 0; i < B2; i++) S.cb2[i] = 0;
        S.dp0 = S.dp1 = S.dp2 = S.dp3 = 0;
        S.pa0 = S.pa1 = S.pa2 = S.pb0 = S.pb1 = S.pb2 = 0;
        S.xm1 = S.xm2 = 0;
    }

    /* the audio recurrence. PIPELINE off on the sample loop: it carries the per-sample resonator
     * state (s2=s1; s1=y) — sequential is correct AND keeps HLS from force-unrolling the inner
     * resonator loop (the DSP/LUT explosion). The resonator loop is UNROLL off so all RESN
     * resonators time-share ONE rolled engine. Tiny + low-DSP — right for an offline render. */
    for (int t = 0; t < n; t++) {
        HLS_PRAGMA(HLS LOOP_TRIPCOUNT min=1 max=512)
        HLS_PRAGMA(HLS PIPELINE off)

        fx x   = xin[t];
        fx xbp = x - S.xm2; S.xm2 = S.xm1; S.xm1 = x;     /* bandpass src: zeros DC+Nyquist */

        fx d = xbp;
        if (S.diff_on) {                                  /* 4 short Schroeder allpasses    */
            d = ap_proc(S.d0, S.dp0, S.dg, d); S.dp0 = (S.dp0 + 1 >= D0) ? 0 : S.dp0 + 1;
            d = ap_proc(S.d1, S.dp1, S.dg, d); S.dp1 = (S.dp1 + 1 >= D1) ? 0 : S.dp1 + 1;
            d = ap_proc(S.d2, S.dp2, S.dg, d); S.dp2 = (S.dp2 + 1 >= D2) ? 0 : S.dp2 + 1;
            d = ap_proc(S.d3, S.dp3, S.dg, d); S.dp3 = (S.dp3 + 1 >= D3) ? 0 : S.dp3 + 1;
        }

        int64_t sum = 0;
        for (int i = 0; i < S.n_res; i++) {               /* the resonator network          */
            HLS_PRAGMA(HLS LOOP_TRIPCOUNT min=1 max=3000)
            HLS_PRAGMA(HLS UNROLL off)
            HLS_PRAGMA(HLS PIPELINE off)
            fx exc = fmul(S.gi[i], d);
            fx y   = fmul(S.a1[i], S.s1[i]) - fmul(S.r2[i], S.s2[i]) + exc;
            S.s2[i] = S.s1[i]; S.s1[i] = y;
            sum += y;
        }

        fx wf = wet_scale(sum, S.wetg_k);                 /* mono wet (pure int)            */

        fx la = wf;                                       /* L decorrelation chain          */
        la = ap_proc(S.ca0, S.pa0, S.cg, la); S.pa0 = (S.pa0 + 1 >= A0) ? 0 : S.pa0 + 1;
        la = ap_proc(S.ca1, S.pa1, S.cg, la); S.pa1 = (S.pa1 + 1 >= A1) ? 0 : S.pa1 + 1;
        la = ap_proc(S.ca2, S.pa2, S.cg, la); S.pa2 = (S.pa2 + 1 >= A2) ? 0 : S.pa2 + 1;
        fx rb = wf;                                       /* R decorrelation chain          */
        rb = ap_proc(S.cb0, S.pb0, S.cg, rb); S.pb0 = (S.pb0 + 1 >= B0) ? 0 : S.pb0 + 1;
        rb = ap_proc(S.cb1, S.pb1, S.cg, rb); S.pb1 = (S.pb1 + 1 >= B1) ? 0 : S.pb1 + 1;
        rb = ap_proc(S.cb2, S.pb2, S.cg, rb); S.pb2 = (S.pb2 + 1 >= B2) ? 0 : S.pb2 + 1;

        fx l = wf + fmul(S.widthf, la - wf);              /* blend mono <-> decorrelated    */
        fx r = wf + fmul(S.widthf, rb - wf);
        yout[t * 2]     = l;
        yout[t * 2 + 1] = r;
    }
}
