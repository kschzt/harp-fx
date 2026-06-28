/* implicit_mesh.c — IMPLICIT-SOLVE coupled-mesh reverb. Pure Q26 fixed-point, FPGA datapath.
 *
 * Lineage: explore/ring_neighbor.c (which itself came from src/reverb.c +
 * reverb_engine_core.h). KEEPS, byte-for-byte / behaviour-for-behaviour:
 *   - the WAV I/O (16-bit mono read / stereo write) and the softceil output stage,
 *   - the Q26 fixed-point primitives  fmul / FX / wet_scale  (== fabric ap_fixed datapath),
 *   - the input diffusion (4 Schroeder allpasses) and the bandpass input source,
 *   - the by-log-frequency resonator ring + per-resonator 2-pole bandpass,
 *   - the stereo decorrelation taps.
 *
 * WHAT CHANGES — the coupling goes from EXPLICIT to IMPLICIT.
 *
 *   ring_neighbor stepped the discrete Laplacian EXPLICITLY, reading PREVIOUS-sample
 *   neighbours:   y[i] = bandpass_i + kc*( yprev[i-1] - 2*yprev[i] + yprev[i+1] ).
 *   That is forward-Euler diffusion. It is bounded only under a CFL limit (kc * 4 < ~2):
 *   crank kc past ~0.5 and the alternating spatial mode (ring-Laplacian eigenvalue -4)
 *   flips its update sign and SCREECHES. And reading yprev[] makes it a 1-sample-delay
 *   lattice step a GPU can do in parallel.
 *
 *   implicit_mesh instead SOLVES, every sample, the linear system on the frequency ring
 *
 *        (I - kc*L) y = rhs        L = cyclic discrete Laplacian (tridiagonal + wrap corners)
 *
 *   for y THIS sample, where  rhs[i] = a1_i*s1_i - r2_i*s2_i + gi_i*d  is the per-resonator
 *   bandpass recurrence + excitation. Row i reads:
 *
 *        (1 + 2*kc) * y[i]  -  kc*( y[i-1] + y[i+1] )  =  rhs[i].
 *
 *   This is BACKWARD-Euler diffusion. (I - kc*L) is, for any kc>0, symmetric
 *   positive-definite AND strictly diagonally dominant (diagonal 1+2kc > 2kc = sum|offdiag|),
 *   so its inverse is a contraction: every spatial mode m has gain 1/(1 - kc*lambda_m) in
 *   (0,1] because lambda_m = 2cos(theta_m)-2 <= 0. The solve can therefore ONLY remove or
 *   redistribute energy, NEVER amplify — UNCONDITIONALLY STABLE, no CFL screech. So kc can
 *   finally be pushed into the strongly-audible regime that blew the explicit version up,
 *   giving a big, long, lush space whose tail slides in pitch (energy diffuses around the
 *   log-frequency ring over the tail = "a room retuning itself").
 *
 *   For identical resonators the closed loop even diagonalises per spatial mode m to a
 *   2-pole with poles r/sqrt(D_m), D_m = 1 - kc*lambda_m >= 1 : the smooth (low spatial
 *   frequency) modes keep the full t60 long tail, the rough modes get extra damping ->
 *   a coherent, smooth-across-pitch lush tail. (Heterogeneous resonators perturb this;
 *   we verify boundedness empirically via internal_state_peak + peak/clip/NaN.)
 *
 * THE ONLY-KRIA FLEX (airtight, and NOT what meanfield/triangular missed):
 *   The per-sample solve is a NON-rank-1 SPARSE linear solve across all N resonators. We run
 *   it as IMPLICIT Gauss-Seidel / SOR sweeps:
 *        y[i] <- recip * ( rhs[i] + kc*( y[i-1] + y[i+1] ) ),  recip = 1/(1+2kc) (precomputed)
 *   The backward neighbour y[i-1] is the value JUST written earlier in THIS sweep — a
 *   same-sample sequential recurrence that wavefronts forward around the ring. An FPGA
 *   pipelines this dependent sweep (II=1 carried recurrence); a GPU cannot close N*nsweep
 *   dependent steps inside one 48 kHz sample. It is NOT the rank-1 mean-field shortcut
 *   (meanfield.c) — the inverse Green's function couples neighbours with exponential-in-
 *   ring-distance weight, a genuinely banded full-rank operator, so there is no two-pass
 *   GPU collapse. The diagonal is pre-inverted to a Q26 reciprocal so the sweep is pure
 *   multiply-add — no per-sample division — == the fabric datapath.
 *
 *   cc -O3 -Wall explore/implicit_mesh.c -o /tmp/imesh -lm
 *   imesh in.wav out.wav [N t60 wet hidamp diffuse width kc nsweep omega order cmode]
 *     kc     : coupling strength (>=0). 0 = decoupled bank. STRONG (0.4..4) is now allowed.
 *     nsweep : implicit Gauss-Seidel/SOR sweeps per sample (solve accuracy). default 8.
 *     omega  : SOR over-relaxation (1.0 = plain Gauss-Seidel; 1.0..1.7 accelerates).
 *     order  : 0=by-frequency(default)  1=interleaved(neighbours far in pitch)
 *     cmode  : WHICH per-sample implicit cyclic-tridiagonal solve closes the coupling:
 *              2 = ADVECTION-COUPLED (THE DELIVERABLE — loud + long + moving + stable): solve
 *                  (I - kc*D) y = [a1 s1 - r2 s2 + gi d] where D is the SKEW-symmetric cyclic
 *                  difference (D y)[i] = y[i+1] - y[i-1]. D's eigenvalues are PURELY IMAGINARY
 *                  (i*2 sin theta), so (I-kc*D) has |eigenvalue| = sqrt(1+(2kc sin theta)^2) >= 1
 *                  -> its inverse is a contraction (unconditionally STABLE, backward-Euler
 *                  advection, NO CFL screech) BUT, crucially, D has a ZERO diagonal, so the
 *                  solve's diagonal is exactly 1: the resonator's own r2 decay + a1 tuning pass
 *                  through UNSCALED -> FULL t60, FULL level (no over-damp, no detune). The skew
 *                  coupling ADVECTS energy around the log-frequency ring at a kc-set speed =
 *                  a literal traveling pitch ("the room retuning itself"), lossless to first
 *                  order. recip is unused (diagonal 1). GS converges for kc < 0.5.
 *              0 = STATE-COUPLED: solve (I-kc*L)u = s1 and
 *                  drive the resonator with a1*u. The implicit diagonal lands on the a1
 *                  (frequency) term, so per-mode poles stay at exactly r -> FULL t60 long
 *                  tail for EVERY spatial mode, while M^{-1}'s off-diagonal transports energy
 *                  between nearby-pitch resonators over the tail (the traveling self-retune).
 *                  Unconditionally bounded (|pole|=r<1 for any kc) -> crank kc, no screech.
 *              1 = OUTPUT-COUPLED (the literal (I-kc*L)y = [a1 s1 - r2 s2 + gi d] from the
 *                  brief): also unconditionally STABLE (no screech at strong kc), but the
 *                  (1+2kc) diagonal divides the r2 decay term too, so it OVER-DAMPS the
 *                  narrowband high-Q resonators to near-silence. Kept to honestly show the
 *                  wall the literal scheme hits. Both are genuine non-rank-1 sparse solves.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ---- Q26 fixed-point primitive: identical to reverb_engine_core.h / fabric ---- */
#define FB  26
#define ONE ((int64_t)1<<FB)
#define WSHIFT 55
#define RATE 48000.0
#define NDIFF 4
#define YMAX ((fx)(8*ONE))      /* per-resonator clamp: keeps every state well inside int32 */
typedef int32_t fx;
static inline fx fmul(fx a,fx b){ return (fx)(((int64_t)a*(int64_t)b)>>FB); }
/* 64-bit-accumulating multiply: a is Q26, b is a Q26 value in an int64 accumulator. */
static inline int64_t fmul64(int64_t a,int64_t b){ return (a*b)>>FB; }
static inline fx FX(double x){ return (fx)llround(x*(double)ONE); }
static inline fx wet_scale(int64_t sum,int64_t wetg_k){
    __int128 p=(__int128)sum*(__int128)wetg_k;
    int64_t r = (p>=0)? (int64_t)(( p+((__int128)1<<(WSHIFT-1)))>>WSHIFT)
                      :-(int64_t)((-p+((__int128)1<<(WSHIFT-1)))>>WSHIFT);
    return (fx)r;
}
/* canonical Schroeder allpass: w=x+g*w[n-D]; y=w[n-D]-g*w (diffusion + decorrelation) */
static inline fx ap_proc(fx *buf,int len,int *pos,fx g,fx x){
    int r=*pos; fx wd=buf[r];
    fx w=x+fmul(g,wd);
    fx y=wd-fmul(g,w);
    buf[r]=w; *pos=(r+1>=len)?0:r+1;
    return y;
}

/* ---- WAV io (copied verbatim from src/reverb.c / ring_neighbor.c) ---- */
static float* wav_read(const char*p,long*nout){
    FILE*f=fopen(p,"rb"); if(!f){perror(p);return NULL;}
    char id[4]; unsigned u; int ch=1,bits=16; float*buf=NULL;
    if(fread(id,1,4,f)!=4){fclose(f);return NULL;} if(fread(&u,4,1,f)!=1){} if(fread(id,1,4,f)!=4){}
    while(fread(id,1,4,f)==4){ if(fread(&u,4,1,f)!=1)break;
        if(!memcmp(id,"fmt ",4)){ unsigned char fmt[40]; long want=u<40?u:40; if(fread(fmt,1,want,f)!=(size_t)want){}
            ch=fmt[2]|(fmt[3]<<8); bits=fmt[14]|(fmt[15]<<8); if((long)u>want)fseek(f,u-want,SEEK_CUR);
        } else if(!memcmp(id,"data",4)){ long frames=(long)u/(bits/8)/ch; buf=malloc(sizeof(float)*frames);
            for(long i=0;i<frames;i++){ double a=0; for(int c=0;c<ch;c++){ short v=0; if(fread(&v,2,1,f)!=1){} a+=v/32768.0; } buf[i]=(float)(a/ch); }
            *nout=frames; fclose(f); return buf;
        } else fseek(f,u,SEEK_CUR);
    } fclose(f); return NULL;
}
static void wav_write_stereo(const char*p,short*lr,long n){
    FILE*f=fopen(p,"wb"); long nd=n*2*2; unsigned u; unsigned short s;
    fwrite("RIFF",1,4,f);u=36+nd;fwrite(&u,4,1,f);fwrite("WAVE",1,4,f);fwrite("fmt ",1,4,f);
    u=16;fwrite(&u,4,1,f);s=1;fwrite(&s,2,1,f);s=2;fwrite(&s,2,1,f);u=48000;fwrite(&u,4,1,f);
    u=48000*4;fwrite(&u,4,1,f);s=4;fwrite(&s,2,1,f);s=16;fwrite(&s,2,1,f);fwrite("data",1,4,f);u=nd;fwrite(&u,4,1,f);
    fwrite(lr,2,n*2,f); fclose(f);
}
static inline double softceil(double v){
    if(v> 0.9)return 0.9+0.1*tanh((v-0.9)/0.1);
    if(v<-0.9)return -0.9-0.1*tanh((v+0.9)/0.1); return v; }

/* ---- the implicit-solve coupled resonator mesh ---- */
typedef struct {
    int N;
    fx *s1,*s2,*a1,*r2,*gi;       /* per-resonator state + 2-pole coeffs (as in template)   */
    fx *y;                       /* working solution buffer for the implicit solve (persists)*/
    fx *rhs;                     /* this-sample right-hand side of the cyclic-tridiag solve  */
    int *im1,*ip1;              /* ring neighbour indices (precomputed for any --order)     */
    fx kc;                       /* coupling gain (Q26)                                     */
    fx recip;                    /* 1/(1+2kc) in Q26 — pre-inverted diagonal, no /per-sample */
    fx omega;                    /* SOR over-relaxation factor (Q26); ONE = plain GS         */
    int nsweep;                  /* implicit GS/SOR sweeps per sample                        */
    int cmode;                   /* 0=state-coupled (solve M u=s1)  1=output-coupled (M y=rhs)*/
    /* shared front/back ends (verbatim behaviour from the template) */
    fx *dbuf[NDIFF]; int dlen[NDIFF],dpos[NDIFF]; fx dg; int diff_on;
    fx *cbufA[3],*cbufB[3]; int clenA[3],clenB[3],cposA[3],cposB[3]; fx cg,widthf;
    fx xm1,xm2;
    int64_t wetg_k;
    /* diagnostics */
    double istate_peak;          /* max |resonator state|  (internal boundedness)           */
    double resid_peak;           /* max relative solver residual seen (solve fidelity)       */
    long   clampn;               /* times the YMAX safety clamp engaged (should be 0)        */
} mesh;

static mesh* mesh_init(int N,double t60,double wet,double hidamp,double diffuse,double width,
                       double kc,int nsweep,double omega,int order,int cmode){
    mesh*R=calloc(1,sizeof*R); R->N=N; R->nsweep=nsweep; R->cmode=cmode;
    R->s1=calloc(N,4);R->s2=calloc(N,4);R->a1=calloc(N,4);R->r2=calloc(N,4);R->gi=calloc(N,4);
    R->y=calloc(N,4);R->rhs=calloc(N,4);R->im1=calloc(N,sizeof(int));R->ip1=calloc(N,sizeof(int));
    if(kc<0) kc=0;
    R->kc=FX(kc);
    R->recip=FX(1.0/(1.0+2.0*kc));   /* pre-inverted constant diagonal -> no per-sample divide */
    R->omega=FX(omega);
    unsigned rng=99173;
    for(int i=0;i<N;i++){
        rng=rng*1103515245u+12345u; double jit=((rng>>9)/4194304.0-1.0)*0.5;
        double frac=(double)i/(N-1);
        double f=30.0*pow(18000.0/30.0,frac)*pow(2.0,jit/12.0); if(f>0.45*RATE)f=0.45*RATE;
        double w=2*M_PI*f/RATE, t=t60*(1.0-hidamp*frac*0.85); if(t<0.05)t=0.05;
        double r=exp(-6.9078/(t*RATE));
        R->a1[i]=FX(2*r*cos(w)); R->r2[i]=FX(r*r);
        R->gi[i]=FX((1.0-r*r)*(0.5+0.5*((rng>>11)&255)/255.0));
    }
    /* ring adjacency: order 0 -> neighbour-in-index == neighbour-in-pitch (smooth spectral
     * diffusion). order 1 -> even-then-odd interleave so neighbours are far apart in pitch. */
    int *perm=calloc(N,sizeof(int));
    for(int i=0;i<N;i++) perm[i]=i;
    if(order==1){ int j=0; for(int i=0;i<N;i+=2) perm[j++]=i; for(int i=1;i<N;i+=2) perm[j++]=i; }
    for(int p=0;p<N;p++){ int i=perm[p]; R->im1[i]=perm[(p-1+N)%N]; R->ip1[i]=perm[(p+1)%N]; }
    free(perm);
    int dl[NDIFF]={59,89,127,173};
    R->dg=FX(0.45*diffuse); R->diff_on=(diffuse>0.001);
    for(int k=0;k<NDIFF;k++){ R->dlen[k]=dl[k]; R->dbuf[k]=calloc(dl[k],4); R->dpos[k]=0; }
    int dA[3]={241,151,97}, dB[3]={199,317,113};
    R->cg=FX(0.5); R->widthf=FX(width>1?1:width<0?0:width);
    for(int k=0;k<3;k++){ R->clenA[k]=dA[k]; R->cbufA[k]=calloc(dA[k],4); R->cposA[k]=0;
                          R->clenB[k]=dB[k]; R->cbufB[k]=calloc(dB[k],4); R->cposB[k]=0; }
    double wetg=wet*12.0/sqrt((double)N);
    R->wetg_k=(int64_t)llround(wetg*(double)((int64_t)1<<WSHIFT));
    return R;
}
static void mesh_free(mesh*R){ if(!R)return;
    free(R->s1);free(R->s2);free(R->a1);free(R->r2);free(R->gi);
    free(R->y);free(R->rhs);free(R->im1);free(R->ip1);
    for(int k=0;k<NDIFF;k++)free(R->dbuf[k]);
    for(int k=0;k<3;k++){free(R->cbufA[k]);free(R->cbufB[k]);} free(R); }

/* the shared implicit kernel: nsweep Gauss-Seidel/SOR sweeps of (1+2kc)v - kc(v[-1]+v[+1]) = b
 * on the cyclic ring, warm-started in v[] (persists across samples). Walks i forward so v[i-1]
 * is THIS sweep's freshly written value -> a same-sample sequential recurrence around the ring
 * (FPGA-pipelined; a GPU cannot close N*nsweep dependent steps at 48 kHz). recip pre-inverts
 * the constant diagonal so the step is pure multiply-add (no per-sample division). int64
 * intermediates so strong kc can't overflow before the recip scale; YMAX clamps the state. */
static void mesh_solve(mesh*R,fx *v,const fx *b){
    int N=R->N; int64_t kc=R->kc, recip=R->recip, omega=R->omega;
    for(int sw=0; sw<R->nsweep; sw++){
        for(int i=0;i<N;i++){
            int64_t nb  = (int64_t)v[R->im1[i]] + (int64_t)v[R->ip1[i]];
            int64_t acc = (int64_t)b[i] + fmul64(kc, nb);              /* b + kc*(neighbours) */
            int64_t gs  = fmul64(recip, acc);                         /* /(1+2kc)            */
            int64_t vn  = (int64_t)v[i] + fmul64(omega, gs - (int64_t)v[i]); /* SOR           */
            if(vn> YMAX){ vn= YMAX; R->clampn++; }
            if(vn<-YMAX){ vn=-YMAX; R->clampn++; }
            v[i]=(fx)vn;
        }
    }
}

/* one stereo wet sample. cmode selects which quantity the per-sample implicit solve couples. */
static void mesh_process(mesh*R,fx x,double*outL,double*outR){
    fx xin=x-R->xm2; R->xm2=R->xm1; R->xm1=x;              /* bandpass source */
    fx d=xin; if(R->diff_on) for(int k=0;k<NDIFF;k++) d=ap_proc(R->dbuf[k],R->dlen[k],&R->dpos[k],R->dg,d);
    int N=R->N; fx *y=R->y, *rhs=R->rhs;
    int64_t sum=0; double maxabs=0;

    if(R->cmode==0){
        /* STATE-COUPLED: build rhs = previous resonator outputs s1, solve (I-kc*L) y = s1
         * (y is the warm-started running estimate of the diffused state), then drive each
         * resonator with a1*y. The implicit diagonal scales a1 (frequency), NOT r2 (decay):
         * per-mode poles stay at exactly r -> full t60 for every spatial mode while M^{-1}
         * transports energy between nearby-pitch resonators (the traveling self-retune). */
        for(int i=0;i<N;i++) rhs[i]=R->s1[i];
        mesh_solve(R,y,rhs);                               /* y ~= (I-kc*L)^{-1} s1 */
        for(int i=0;i<N;i++){
            int64_t yi = fmul64((int64_t)R->a1[i],(int64_t)y[i])
                       - fmul64((int64_t)R->r2[i],(int64_t)R->s2[i])
                       + (int64_t)fmul(R->gi[i],d);
            if(yi> YMAX) yi= YMAX; if(yi<-YMAX) yi=-YMAX;
            R->s2[i]=R->s1[i]; R->s1[i]=(fx)yi;
            sum += yi;
            double m=fabs((double)yi/ONE); if(m>maxabs)maxabs=m;
        }
    } else if(R->cmode==2){
        /* ADVECTION-COUPLED: rhs = the CLEAN resonator output (full t60 + tuning), solve the
         * skew-symmetric cyclic system (I - kc*D) y = rhs, (D y)[i] = y[i+1]-y[i-1]. Diagonal
         * is exactly 1 (D has no diagonal) so the recip is unused and the resonator passes
         * through unscaled; the skew off-diagonal advects energy around the ring. Walks i
         * forward so y[i-1] is this sweep's fresh value -> same-sample ring recurrence. */
        for(int i=0;i<N;i++)
            rhs[i] = fmul(R->a1[i],R->s1[i]) - fmul(R->r2[i],R->s2[i]) + fmul(R->gi[i],d);
        int64_t kc=R->kc, omega=R->omega;
        for(int sw=0; sw<R->nsweep; sw++){
            for(int i=0;i<N;i++){
                int64_t adv = (int64_t)y[R->ip1[i]] - (int64_t)y[R->im1[i]]; /* (D y)[i] */
                int64_t gs  = (int64_t)rhs[i] + fmul64(kc, adv);            /* diag 1: y=rhs+kc*Dy */
                int64_t yn  = (int64_t)y[i] + fmul64(omega, gs - (int64_t)y[i]);
                if(yn> YMAX){ yn= YMAX; R->clampn++; }
                if(yn<-YMAX){ yn=-YMAX; R->clampn++; }
                y[i]=(fx)yn;
            }
        }
        for(int i=0;i<N;i++){ fx yi=y[i]; R->s2[i]=R->s1[i]; R->s1[i]=yi; sum+=yi;
            double m=fabs((double)yi/ONE); if(m>maxabs)maxabs=m; }
    } else {
        /* OUTPUT-COUPLED (the literal brief): rhs = a1 s1 - r2 s2 + gi d, solve (I-kc*L) y = rhs,
         * feed y back. Stable at any kc but the (1+2kc) diagonal divides r2 too -> over-damps. */
        for(int i=0;i<N;i++)
            rhs[i] = fmul(R->a1[i],R->s1[i]) - fmul(R->r2[i],R->s2[i]) + fmul(R->gi[i],d);
        mesh_solve(R,y,rhs);
        for(int i=0;i<N;i++){
            fx yi=y[i]; R->s2[i]=R->s1[i]; R->s1[i]=yi; sum += yi;
            double m=fabs((double)yi/ONE); if(m>maxabs)maxabs=m;
        }
    }

    /* convergence residual of the per-sample solve (operator depends on cmode) */
    { int64_t kc=R->kc; double maxres=0;
      for(int i=0;i<N;i++){
        int64_t lhs;
        if(R->cmode==2){ int64_t adv=(int64_t)y[R->ip1[i]]-(int64_t)y[R->im1[i]];
                         lhs=(int64_t)y[i]-fmul64(kc,adv); }            /* (I-kcD)y row */
        else { int64_t nb=(int64_t)y[R->im1[i]]+(int64_t)y[R->ip1[i]];
               lhs=(int64_t)y[i]+fmul64(kc,(int64_t)2*y[i]-nb); }       /* (I-kcL)y row */
        double res=fabs((double)(lhs-(int64_t)rhs[i])/ONE); if(res>maxres)maxres=res;
      }
      double denom=maxabs>1e-9?maxabs:1e-9; double rr=maxres/denom;
      if(rr>R->resid_peak)R->resid_peak=rr;
    }
    if(maxabs>R->istate_peak)R->istate_peak=maxabs;

    fx wf=wet_scale(sum,R->wetg_k);
    fx la=wf; for(int k=0;k<3;k++) la=ap_proc(R->cbufA[k],R->clenA[k],&R->cposA[k],R->cg,la);
    fx rb=wf; for(int k=0;k<3;k++) rb=ap_proc(R->cbufB[k],R->clenB[k],&R->cposB[k],R->cg,rb);
    fx l=wf+fmul(R->widthf,la-wf), r=wf+fmul(R->widthf,rb-wf);
    *outL=(double)l/ONE; *outR=(double)r/ONE;
}

int main(int argc,char**argv){
    if(argc<3){fprintf(stderr,"usage: %s in.wav out.wav [N t60 wet hidamp diffuse width kc nsweep omega order]\n",argv[0]);return 1;}
    int    N      = argc>3 ?atoi(argv[3]):384;
    double t60    = argc>4 ?atof(argv[4]):14.0;
    double wet    = argc>5 ?atof(argv[5]):0.42;
    double hidamp = argc>6 ?atof(argv[6]):0.5;
    double diffuse= argc>7 ?atof(argv[7]):0.7;
    double width  = argc>8 ?atof(argv[8]):0.9;
    double kc     = argc>9 ?atof(argv[9]):0.2;
    int    nsweep = argc>10?atoi(argv[10]):14;   /* mode B needs enough sweeps to stay converged */
    double omega  = argc>11?atof(argv[11]):1.0;
    int    order  = argc>12?atoi(argv[12]):0;
    int    cmode  = argc>13?atoi(argv[13]):0;

    long n; float*dry=wav_read(argv[1],&n); if(!dry){fprintf(stderr,"read fail\n");return 1;}
    long tail=(long)(t60*1.8*RATE), total=n+tail;
    mesh*R=mesh_init(N,t60,wet,hidamp,diffuse,width,kc,nsweep,omega,order,cmode);
    short*out=malloc(sizeof(short)*total*2); double peak=0,drymix=1.0-wet; if(drymix<0)drymix=0;
    /* tail-decay probe: RMS in 0.25s windows; also find the -60dB tail crossing */
    double WL=0.25; int W=(int)(WL*RATE), nw=(int)(total/W)+1;
    double*wrms=calloc(nw,sizeof(double)); int*wc=calloc(nw,sizeof(int));
    long clipn=0, nann=0;
    for(long t=0;t<total;t++){
        double xd = t<n?dry[t]:0.0; double l,r; mesh_process(R,FX(xd),&l,&r);
        if(isnan(l)||isnan(r)){ nann++; l=r=0; }
        double oL=softceil(xd*drymix+l), oR=softceil(xd*drymix+r);
        if(fabs(oL)>peak)peak=fabs(oL); if(fabs(oR)>peak)peak=fabs(oR);
        if(fabs(oL)>=0.999||fabs(oR)>=0.999)clipn++;
        double m=0.5*(l*l+r*r); int wi=(int)(t/W); if(wi<nw){wrms[wi]+=m;wc[wi]++;}
        out[t*2]=(short)lrint(oL*32767.0); out[t*2+1]=(short)lrint(oR*32767.0);
    }
    wav_write_stereo(argv[2],out,total);
    /* tail length to -60 dB: peak window RMS, then last window above peak*1e-3 (=-60dB) */
    double peakw=0; int peakwi=0;
    for(int i=0;i<nw;i++){ if(wc[i]){double rms=sqrt(wrms[i]/wc[i]); if(rms>peakw){peakw=rms;peakwi=i;}} }
    double t60_tail=0;
    for(int i=peakwi;i<nw;i++){ if(wc[i]){double rms=sqrt(wrms[i]/wc[i]); if(rms>peakw*1e-3) t60_tail=(i+1)*WL; } }
    double lastw=0; for(int i=nw-1;i>=0;i--){ if(wc[i]){lastw=sqrt(wrms[i]/wc[i]);break;} }
    fprintf(stderr,"imesh: N=%d t60=%.1f kc=%.3f nsweep=%d omega=%.2f order=%d cmode=%s -> %s\n",
            N,t60,kc,nsweep,omega,order,
            cmode==0?"state-coupled(L)":cmode==2?"advection-coupled(D)":"output-coupled(literal L)",argv[2]);
    fprintf(stderr,"     len=%.1fs out_peak=%.3f clip=%.3f%% NaN=%ld  internal_peak=%.3f clamp=%ld resid=%.2e\n",
            total/RATE,peak,100.0*clipn/(total*2),nann,R->istate_peak,R->clampn,R->resid_peak);
    fprintf(stderr,"     tail(-60dB)=%.2fs  tail/peak_rms=%.5f (%s)\n",
            t60_tail, peakw>0?lastw/peakw:0.0,
            (peakw>0&&lastw/peakw<0.01)?"DECAYS":(peakw>0&&lastw/peakw<0.4)?"slow-decay":"SUSTAINS/grows");
    mesh_free(R); free(dry); free(out); free(wrms); free(wc); return 0;
}
