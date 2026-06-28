/* meanfield.c — GLOBAL MEAN-FIELD coupled resonator reverb. Pure fixed-point (Q26), FPGA-ready.
 *
 * Adapted from src/reverb.c / reverb_engine_core.h. KEEPS: WAV I/O, the input diffusion
 * (4 Schroeder allpass), the Q26 fmul primitive, the stereo phase-decorrelation taps.
 * REPLACES: the INDEPENDENT bandpass bank with a GLOBAL MEAN-FIELD COUPLED bank.
 *
 * THE COUPLING (the only-Kria flex):
 *   Every resonator feeds a small fraction of the GLOBAL SUM of all resonators back into
 *   its own input, WITHIN THE SAME SAMPLE (a delay-free global recurrence). Per sample:
 *
 *     P_i   = a1_i*s1_i - r2_i*s2_i + gi_i*d           (uncoupled prediction, past+input)
 *     Mbar  = (1/N) * sum_i P_i                         (mean field, this sample)
 *     y_i   = P_i + (Mbar* - Pbar)                      (inject the SAME-SAMPLE field)
 *   where Mbar* solves the scalar fixed point   M = Pbar + Kc * sat(M).
 *
 *   The injected field depends on the sum of ALL resonators' CURRENT outputs, and each
 *   current output depends on the field -> a same-sample algebraic loop. Because the
 *   coupling is the global *mean* (rank-1), the loop collapses to ONE scalar fixed point,
 *   solved by a fixed, unrolled niter sweep (the fabric datapath: a fixed-depth pipeline
 *   that closes the loop inside one audio sample). A GPU cannot close a per-sample global
 *   reduction->broadcast->reduce loop at 48 kHz; fabric can.
 *
 *   sat() is a smooth C1 saturator on the fed-back field. It is what tames the runaway:
 *   near 0 its slope is 1 (the loop is alive, near-critical, blooming), and as the field
 *   grows its slope falls to 0 (the global loop self-limits) -> bounded edge-of-chaos.
 *   It is ALSO what makes the loop genuinely only-Kria: with sat active the fixed point is
 *   NOT closed-form, so it needs the iterated same-sample solve (no GPU two-pass shortcut).
 *
 *   cc -O3 -Wall explore/meanfield.c -o explore/meanfield -lm
 *   meanfield in.wav out.wav [N t60 wet hidamp diffuse width  Kc niter satL]
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define FB  26
#define ONE ((int64_t)1<<FB)
#define WSHIFT 55
typedef int32_t fx;
static inline fx fmul(fx a,fx b){ return (fx)(((int64_t)a*(int64_t)b)>>FB); }
static inline fx FX(double x){ return (fx)llround(x*(double)ONE); }
static inline fx wet_scale(int64_t sum,int64_t wetg_k){
    __int128 p=(__int128)sum*(__int128)wetg_k;
    int64_t r = (p>=0)? (int64_t)(( p+((__int128)1<<(WSHIFT-1)))>>WSHIFT)
                      :-(int64_t)((-p+((__int128)1<<(WSHIFT-1)))>>WSHIFT);
    return (fx)r;
}
#define RATE 48000.0
#define NDIFF 4
#define YMAX ((fx)(16*ONE))            /* per-resonator saturating clamp: prevents int32 wrap */

typedef struct {
    int   N, shiftN;
    fx   *s1,*s2,*a1,*r2,*gi,*p;        /* per-resonator state + coeffs + P_i scratch */
    /* mean-field coupling */
    fx    Kc;                           /* loop coefficient (Q26) */
    int   niter;                        /* unrolled same-sample fixed-point iterations */
    fx    Ginv;                         /* 1/(1-Kc) linear seed (Q26, clamped) */
    fx    satL, inv2L;                  /* saturator knee + precomputed 1/(2*satL) */
    long  clampcnt;                     /* diag: per-resonator clamp events */
    double satenergy, fieldenergy; long fieldn; /* diag: how active is the saturator */
    /* stereo decorrelation */
    fx   *cbufA[3],*cbufB[3]; int clenA[3],clenB[3],cposA[3],cposB[3]; fx cg,widthf;
    /* input diffusion */
    fx   *dbuf[NDIFF]; int dlen[NDIFF],dpos[NDIFF]; fx dg; int diff_on;
    fx    xm1,xm2;
    double  wetg; int64_t wetg_k;
} mfreverb;

/* smooth C1 saturator, pure integer Q26. |x|<=L: identity (slope 1). L<|x|<2L: quadratic
 * knee g=L+e-e^2/(2L). |x|>=2L: flat at 1.5L (slope 0). Monotonic, bounded. */
static inline fx satf(const mfreverb*R,fx x){
    fx ax = x<0?-x:x;
    if(ax<=R->satL) return x;
    fx e = ax - R->satL; if(e>R->satL) e=R->satL;
    fx term = fmul(fmul(e,e),R->inv2L);
    fx y = R->satL + e - term;
    return x<0?-y:y;
}

static inline fx ap_proc(fx *buf,int len,int *pos,fx g,fx x){
    int r=*pos; fx wd=buf[r];
    fx w=x+fmul(g,wd);
    fx y=wd-fmul(g,w);
    buf[r]=w; *pos=(r+1>=len)?0:r+1;
    return y;
}

static mfreverb* mf_init(int N,double t60,double wet,double hidamp,double diffuse,double width,
                         double Kc,int niter,double satL){
    /* snap N to a power of two so the mean (1/N) is an exact shift (fabric-friendly) */
    int sh=0; while((1<<(sh+1))<=N) sh++; N=1<<sh;
    mfreverb*R=calloc(1,sizeof*R); R->N=N; R->shiftN=sh;
    R->s1=calloc(N,4);R->s2=calloc(N,4);R->a1=calloc(N,4);R->r2=calloc(N,4);R->gi=calloc(N,4);R->p=calloc(N,4);
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
    /* mean-field knobs */
    R->Kc=FX(Kc); R->niter=niter<1?1:niter;
    R->satL=FX(satL); if(R->satL<1) R->satL=1;
    R->inv2L=(fx)((__int128)ONE*ONE/(2*(int64_t)R->satL));
    fx omk = (fx)(ONE - R->Kc);                          /* 1-Kc */
    if(omk < (fx)(ONE/64)) R->Ginv = FX(64.0);           /* Kc>=~0.984: clamp seed, sat dominates */
    else R->Ginv = (fx)((__int128)ONE*ONE/(int64_t)omk);
    /* input diffusion */
    int dl[NDIFF]={59,89,127,173};
    R->dg=FX(0.45*diffuse); R->diff_on=(diffuse>0.001);
    for(int k=0;k<NDIFF;k++){ R->dlen[k]=dl[k]; R->dbuf[k]=calloc(dl[k],4); R->dpos[k]=0; }
    int dA[3]={241,151,97}, dB[3]={199,317,113};
    R->cg=FX(0.5); R->widthf=FX(width>1?1:width<0?0:width);
    for(int k=0;k<3;k++){ R->clenA[k]=dA[k]; R->cbufA[k]=calloc(dA[k],4); R->cposA[k]=0;
                          R->clenB[k]=dB[k]; R->cbufB[k]=calloc(dB[k],4); R->cposB[k]=0; }
    R->wetg=wet*12.0/sqrt((double)N);
    R->wetg_k=(int64_t)llround(R->wetg*(double)((int64_t)1<<WSHIFT));
    return R;
}
static void mf_free(mfreverb*R){ if(!R)return;
    free(R->s1);free(R->s2);free(R->a1);free(R->r2);free(R->gi);free(R->p);
    for(int k=0;k<NDIFF;k++)free(R->dbuf[k]);
    for(int k=0;k<3;k++){free(R->cbufA[k]);free(R->cbufB[k]);} free(R); }

static void mf_process(mfreverb*R,fx x,double*outL,double*outR){
    fx xin=x-R->xm2; R->xm2=R->xm1; R->xm1=x;
    fx d=xin; if(R->diff_on) for(int k=0;k<NDIFF;k++) d=ap_proc(R->dbuf[k],R->dlen[k],&R->dpos[k],R->dg,d);

    /* ---- pass 1: uncoupled predictions + their sum ---- */
    int64_t psum=0;
    for(int i=0;i<R->N;i++){
        fx Pi=fmul(R->a1[i],R->s1[i])-fmul(R->r2[i],R->s2[i])+fmul(R->gi[i],d);
        R->p[i]=Pi; psum+=Pi;
    }
    fx Pbar=(fx)(psum>>R->shiftN);                       /* mean field, this sample (exact /N) */

    /* ---- same-sample scalar fixed point: M = Pbar + Kc*sat(M) ---- */
    fx M=fmul(Pbar,R->Ginv);                             /* linear closed-form seed */
    for(int it=0; it<R->niter; it++) M = Pbar + fmul(R->Kc, satf(R,M));
    fx inj = M - Pbar;                                   /* the SAME-SAMPLE field each res. gets */
    /* diag */
    { fx sm=satf(R,M); double dm=(double)M/ONE, ds=(double)sm/ONE;
      R->fieldenergy+=dm*dm; R->satenergy+=(dm-ds)*(dm-ds); R->fieldn++; }

    /* ---- pass 2: inject, advance state, real wet sum ---- */
    int64_t ysum=0;
    for(int i=0;i<R->N;i++){
        fx y=R->p[i]+inj;
        if(y>YMAX){y=YMAX;R->clampcnt++;} else if(y<-YMAX){y=-YMAX;R->clampcnt++;}
        R->s2[i]=R->s1[i]; R->s1[i]=y; ysum+=y;
    }
    fx wf=wet_scale(ysum,R->wetg_k);
    fx la=wf; for(int k=0;k<3;k++) la=ap_proc(R->cbufA[k],R->clenA[k],&R->cposA[k],R->cg,la);
    fx rb=wf; for(int k=0;k<3;k++) rb=ap_proc(R->cbufB[k],R->clenB[k],&R->cposB[k],R->cg,rb);
    fx l=wf+fmul(R->widthf,la-wf), r=wf+fmul(R->widthf,rb-wf);
    *outL=(double)l/ONE; *outR=(double)r/ONE;
}

/* ---- WAV io (16-bit mono read; stereo write) ---- */
static float* wav_read(const char*p,long*nout){
    FILE*f=fopen(p,"rb"); if(!f){perror(p);return NULL;}
    char id[4]; unsigned u; int ch=1,bits=16; float*buf=NULL;
    if(fread(id,1,4,f)!=4){fclose(f);return NULL;} if(fread(&u,4,1,f)!=1){fclose(f);return NULL;} if(fread(id,1,4,f)!=4){fclose(f);return NULL;}
    while(fread(id,1,4,f)==4){ if(fread(&u,4,1,f)!=1)break;
        if(!memcmp(id,"fmt ",4)){ unsigned char fmt[40]; long want=u<40?u:40; if(fread(fmt,1,want,f)!=(size_t)want)break;
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

int main(int argc,char**argv){
    if(argc<3){fprintf(stderr,"usage: %s in.wav out.wav [N t60 wet hidamp diffuse width Kc niter satL]\n",argv[0]);return 1;}
    int    N      = argc>3?atoi(argv[3]):512;
    double t60    = argc>4?atof(argv[4]):2.5;
    double wet    = argc>5?atof(argv[5]):0.45;
    double hidamp = argc>6?atof(argv[6]):0.45;
    double diffuse= argc>7?atof(argv[7]):0.7;
    double width  = argc>8?atof(argv[8]):0.85;
    double Kc     = argc>9?atof(argv[9]):0.9;
    int    niter  = argc>10?atoi(argv[10]):6;
    double satL   = argc>11?atof(argv[11]):0.5;
    double tailsec= argc>12?atof(argv[12]):0.0;     /* override tail length (s); 0 = auto */

    long n; float*dry=wav_read(argv[1],&n); if(!dry){fprintf(stderr,"read fail\n");return 1;}
    long tail=(long)((tailsec>0?tailsec:t60*2.0)*RATE), total=n+tail;
    mfreverb*R=mf_init(N,t60,wet,hidamp,diffuse,width,Kc,niter,satL);
    short*out=malloc(sizeof(short)*total*2); double peak=0,rms=0; double drymix=1.0-wet;
    int naninf=0; double lastloud=0;
    for(long t=0;t<total;t++){
        double xd = t<n?dry[t]:0.0; fx xq=FX(xd); double l,r; mf_process(R,xq,&l,&r);
        if(!(l==l)||!(r==r)) naninf++;
        double oL=softceil(xd*drymix+l), oR=softceil(xd*drymix+r);
        if(fabs(oL)>peak)peak=fabs(oL); if(fabs(oR)>peak)peak=fabs(oR);
        rms += oL*oL+oR*oR;
        if(fabs(oL)>0.01||fabs(oR)>0.01) lastloud=(double)t/RATE;
        out[t*2]=(short)lrint(oL*32767.0); out[t*2+1]=(short)lrint(oR*32767.0);
    }
    wav_write_stereo(argv[2],out,total);
    rms=sqrt(rms/(2.0*total));
    double satfrac = R->fieldn? R->satenergy/(R->fieldenergy+1e-12):0;
    fprintf(stderr,"meanfield: N=%d t60=%.1f wet=%.2f Kc=%.3f niter=%d satL=%.2f -> %s\n"
                   "  %.1fs peak=%.3f rms=%.4f tail_ends=%.1fs naninf=%d clamp=%ld satactive=%.1f%%\n",
            R->N,t60,wet,Kc,niter,satL,argv[2],
            total/RATE,peak,rms,lastloud,naninf,R->clampcnt,100*satfrac);
    mf_free(R); free(dry); free(out); return 0;
}
