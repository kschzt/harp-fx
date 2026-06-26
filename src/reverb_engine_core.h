/* reverb_engine_core.h — the RME-grade resonator-NETWORK reverb DSP, factored out of
 * reverb.c so BOTH the standalone `reverb` tool AND the harp-fx device bridge
 * (device/reverb_engine.c) link the SAME engine. Pure fixed-point (Q26), fabric-ready,
 * deterministic == FPGA ap_fixed. Header-only: every entry point is `static` (or `static
 * inline`), so each translation unit gets its own copy — the tool and the bridge are
 * separate binaries and never collide.
 *
 * Signal path:  in → [input diffusion: 4 Schroeder allpass] → [N bandpass resonators] →
 *               [decorrelated L/R output taps] → wet (host mixes the dry, §8.8).
 *
 * The bandpass bank is the dense "resonator network" — each resonator selectively rings at
 * the input's frequencies (true bandpass: zeros at DC+Nyquist), no global feedback, so it is
 * unconditionally stable. Diffusion spreads transients in time (smooth, dense early field);
 * the decorrelated taps give a wide stereo tail. No hard clipping anywhere (RME bar).
 */
#ifndef REVERB_ENGINE_CORE_H
#define REVERB_ENGINE_CORE_H

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#define FB  26
#define ONE ((int64_t)1<<FB)
#define WSHIFT 55                                /* wet-gain fixed-point scale (Q55)*/
typedef int32_t fx;
static inline fx fmul(fx a,fx b){ return (fx)(((int64_t)a*(int64_t)b)>>FB); }
static inline fx FX(double x){ return (fx)llround(x*(double)ONE); }
/* wet scale, PURE INTEGER (fabric-bit-exact). wetg is frozen to a Q55 int (wetg_k) on
 * the host; here we just do wf = round(sum * wetg_k / 2^55) with a 128-bit product and
 * round-half-away-from-zero (matches the old llround). No float in the per-sample path —
 * this is the one line that previously used a double, now silicon-identical. */
static inline fx wet_scale(int64_t sum,int64_t wetg_k){
    __int128 p=(__int128)sum*(__int128)wetg_k;
    int64_t r = (p>=0)? (int64_t)(( p+((__int128)1<<(WSHIFT-1)))>>WSHIFT)
                      :-(int64_t)((-p+((__int128)1<<(WSHIFT-1)))>>WSHIFT);
    return (fx)r;
}
#define RATE 48000.0
#define NDIFF 4                                  /* input diffusion allpass stages */

typedef struct {
    int   N;
    fx   *s1,*s2,*a1,*r2,*gi;                     /* per-resonator state + coeffs   */
    /* output stereo decorrelation: two allpass chains (different delays) scramble PHASE
     * differently per channel — flat magnitude (no coloration), genuine width. A weighted
     * sum of the shared phase-locked resonators cannot decorrelate; a phase split can. */
    fx   *cbufA[3],*cbufB[3]; int clenA[3],clenB[3],cposA[3],cposB[3]; fx cg,widthf;
    /* input diffusion: NDIFF Schroeder allpasses */
    fx   *dbuf[NDIFF]; int dlen[NDIFF],dpos[NDIFF]; fx dg; int diff_on;
    fx   xm1,xm2;                                 /* input history (bandpass source)*/
    double  wetg;                                  /* host-side wet gain (double)     */
    int64_t wetg_k;                                /* …frozen to Q55 int for the kernel*/
} reverb;

/* canonical Schroeder allpass: w=x+g*w[n-D]; y=w[n-D]-g*w */
static inline fx ap_proc(fx *buf,int len,int *pos,fx g,fx x){
    int r=*pos; fx wd=buf[r];
    fx w=x+fmul(g,wd);
    fx y=wd-fmul(g,w);
    buf[r]=w; *pos=(r+1>=len)?0:r+1;
    return y;
}

static reverb* rev_init(int N,double t60,double wet,double hidamp,double diffuse,double width){
    reverb*R=calloc(1,sizeof*R); R->N=N;
    R->s1=calloc(N,4);R->s2=calloc(N,4);R->a1=calloc(N,4);R->r2=calloc(N,4);R->gi=calloc(N,4);
    unsigned rng=99173;
    for(int i=0;i<N;i++){
        rng=rng*1103515245u+12345u; double jit=((rng>>9)/4194304.0-1.0)*0.5;
        double frac=(double)i/(N-1);
        double f=30.0*pow(18000.0/30.0,frac)*pow(2.0,jit/12.0); if(f>0.45*RATE)f=0.45*RATE;
        double w=2*M_PI*f/RATE, t=t60*(1.0-hidamp*frac*0.85); if(t<0.05)t=0.05;
        double r=exp(-6.9078/(t*RATE));
        R->a1[i]=FX(2*r*cos(w)); R->r2[i]=FX(r*r);
        R->gi[i]=FX((1.0-r*r)*(0.5+0.5*((rng>>11)&255)/255.0));   /* bandpass excite ∝(1-r²) */
    }
    /* short, mutually-prime allpasses (~1..3ms) — long ones comb-color; gentle gain */
    int dl[NDIFF]={59,89,127,173};
    R->dg=FX(0.45*diffuse); R->diff_on=(diffuse>0.001);
    for(int k=0;k<NDIFF;k++){ R->dlen[k]=dl[k]; R->dbuf[k]=calloc(dl[k],4); R->dpos[k]=0; }
    int dA[3]={241,151,97}, dB[3]={199,317,113};                   /* decorrelation: distinct primes/ch */
    R->cg=FX(0.5); R->widthf=FX(width>1?1:width<0?0:width);
    for(int k=0;k<3;k++){ R->clenA[k]=dA[k]; R->cbufA[k]=calloc(dA[k],4); R->cposA[k]=0;
                          R->clenB[k]=dB[k]; R->cbufB[k]=calloc(dB[k],4); R->cposB[k]=0; }
    R->wetg=wet*12.0/sqrt((double)N);
    R->wetg_k=(int64_t)llround(R->wetg*(double)((int64_t)1<<WSHIFT)); /* freeze → Q55 int */
    return R;
}
static void rev_free(reverb*R){ if(!R)return;
    free(R->s1);free(R->s2);free(R->a1);free(R->r2);free(R->gi);
    for(int k=0;k<NDIFF;k++)free(R->dbuf[k]);
    for(int k=0;k<3;k++){free(R->cbufA[k]);free(R->cbufB[k]);} free(R); }

/* one stereo sample of WET output for input x */
static void rev_process(reverb*R,fx x,double*outL,double*outR){
    fx xin=x-R->xm2; R->xm2=R->xm1; R->xm1=x;          /* bandpass source: zeros DC+Nyquist */
    fx d=xin; if(R->diff_on) for(int k=0;k<NDIFF;k++) d=ap_proc(R->dbuf[k],R->dlen[k],&R->dpos[k],R->dg,d);
    int64_t sum=0;
    for(int i=0;i<R->N;i++){
        fx exc=fmul(R->gi[i],d);
        fx y=fmul(R->a1[i],R->s1[i])-fmul(R->r2[i],R->s2[i])+exc;
        R->s2[i]=R->s1[i]; R->s1[i]=y;
        sum += y;                                          /* mono wet (decorrelate to stereo next) */
    }
    fx wf=wet_scale(sum,R->wetg_k);                       /* mono wet sample (fx, pure int) */
    fx la=wf; for(int k=0;k<3;k++) la=ap_proc(R->cbufA[k],R->clenA[k],&R->cposA[k],R->cg,la);
    fx rb=wf; for(int k=0;k<3;k++) rb=ap_proc(R->cbufB[k],R->clenB[k],&R->cposB[k],R->cg,rb);
    fx l=wf+fmul(R->widthf,la-wf), r=wf+fmul(R->widthf,rb-wf);   /* blend mono↔phase-decorrelated */
    *outL=(double)l/ONE; *outR=(double)r/ONE;
}

#endif /* REVERB_ENGINE_CORE_H */
