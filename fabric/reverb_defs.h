/* reverb_defs.h — shared layout for the harp-fx reverb fabric port (host PS + HLS PL).
 *
 * Both reverb_kernel.cpp and reverb_fabric.c include this so the DDR struct layout is
 * byte-identical on both sides. PURE int32/int64 fields (no ap_fixed members → portable,
 * deterministic DDR layout). Q26 fixed-point throughout — identical number system to
 * src/reverb.c, so the fabric reproduces GOLDEN_wet.wav BIT-EXACT.
 *
 * This is the AUDIO-IN FX path of HARP §8.8: dry audio in → wet stereo out.
 */
#ifndef REVERB_DEFS_H
#define REVERB_DEFS_H
#include <stdint.h>

#define FB     26                      /* Q26 (== src/reverb.c)                  */
#define ONE    ((int64_t)1 << FB)
#define WSHIFT 55                      /* wet-gain fixed-point scale (Q55)        */
#define RATE   48000.0

/* fixed network topology — must match src/reverb.c exactly (BRAM ring sizes). */
#define RESN  3000                     /* bandpass resonators (the network)      */
#define NDIFF 4                        /* input-diffusion Schroeder allpasses     */
#define D0 59
#define D1 89
#define D2 127
#define D3 173                         /* diffusion delays {59,89,127,173}        */
#define A0 241
#define A1 151
#define A2 97                          /* L decorrelation delays {241,151,97}     */
#define B0 199
#define B1 317
#define B2 113                         /* R decorrelation delays {199,317,113}    */
#define BS 512                         /* render block (samples/handshake)        */

typedef int32_t fx;
/* the ONE arithmetic primitive — identical to src/reverb.c fmul().
 * int64 intermediate, arithmetic >>26; never overflows int32 on the result. */
static inline fx fmul(fx a, fx b){ return (fx)(((int64_t)a * (int64_t)b) >> FB); }

/* scalar coefficients, written once to DDR (m_axi gmem0). Computed on the HOST in
 * double then frozen to fixed-point (Q26 gains; wetg as Q55), exactly like the modal
 * port — the kernel sample loop is then PURE INTEGER. 6*int32 + int64 = 32 bytes. */
struct rev_coeffs {
    int32_t n_res;                     /* active resonators (== RESN for golden)  */
    int32_t diff_on;                   /* 1 = input diffusion enabled             */
    int32_t dg;                        /* diffusion allpass gain   (Q26)          */
    int32_t cg;                        /* decorrelation allpass gain (Q26)        */
    int32_t widthf;                    /* stereo width             (Q26)          */
    int32_t _pad;                      /* align the int64 below                   */
    int64_t wetg_k;                    /* wet gain                 (Q55)          */
};

#endif
