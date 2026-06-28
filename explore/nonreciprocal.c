/* nonreciprocal.c — COUPLED-MESH reverb prototype: NON-RECIPROCAL / directional chain.
 *
 * Adapted from src/reverb.c. Keeps: WAV I/O, the Q26 fixed-point primitive (fmul), the
 * input diffusion (4 Schroeder allpass), the stereo decorrelation output. REPLACES the
 * independent bandpass bank with a DIRECTIONALLY-COUPLED chain:
 *
 *     resonator i feeds resonator i+1, but i+1 does NOT feed i back.
 *
 * The bank is ordered low->high frequency (30 Hz .. 18 kHz, log-spaced), so the one-way
 * coupling makes energy flow UP the spectrum and never back down: a sound that enters at
 * one pitch radiates upward through the tail and cannot return the way it came. A space
 * that cannot physically exist (no reciprocity => violates time-reversal symmetry).
 *
 * STABILITY: a STRICTLY-DIRECTIONAL (lower-triangular) coupling matrix has the SAME
 * eigenvalues as the uncoupled diagonal bank (poles = the individual resonator poles),
 * so it is UNCONDITIONALLY BIBO-stable for ANY coupling gain — it can never self-oscillate.
 * It only reshapes the response and the LEVEL. So taming here = managing gain/level, not
 * chasing a runaway. (Closing the chain into a RING is what would create a true cross-
 * feedback cycle; we deliberately do NOT do that — that is a different topology.)
 *
 *   cc -O3 -Wall explore/nonreciprocal.c -o /tmp/nonrecip -lm
 *   nonrecip in.wav out.wav [N t60 wet hidamp diffuse width coup mode]
 *     coup : directional coupling gain (Q26 source, 0..~0.95)
 *     mode : 0 = SAME-SAMPLE forward scan (combinational chain within a sample)
 *            1 = DELAYED forward hop (z^-1 per link => a wave that travels UP the chain
 *                over time, ~1 resonator/sample => an upward-drifting shimmer tail)
 *
 * Pure integer Q26 in the per-sample loop (== FPGA datapath). Float only in WAV scaling.
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
/* ROUND-TO-NEAREST (half away from zero) fixed-point multiply. The golden engine uses a
 * plain >>FB (arithmetic floor) which rounds toward -inf — that asymmetry feeds a granular
 * DC bias into the ultra-high-Q resonator recursion and sustains a LIMIT CYCLE (the tail
 * plateaus and never decays). Symmetric rounding kills it so the tail decays to silence.
 * FPGA-native: this is exactly ap_fixed<..,AP_RND> instead of AP_TRN. */
static inline fx fmul(fx a,fx b){
    int64_t p=(int64_t)a*(int64_t)b;
    return (fx)((p>=0)?((p+((int64_t)1<<(FB-1)))>>FB):-(((-p)+((int64_t)1<<(FB-1)))>>FB));
}
static inline fx FX(double x){ return (fx)llround(x*(double)ONE); }
static inline fx wet_scale(int64_t sum,int64_t wetg_k){
    __int128 p=(__int128)sum*(__int128)wetg_k;
    int64_t r = (p>=0)? (int64_t)(( p+((__int128)1<<(WSHIFT-1)))>>WSHIFT)
                      :-(int64_t)((-p+((__int128)1<<(WSHIFT-1)))>>WSHIFT);
    return (fx)r;
}
#define RATE 48000.0
#define NDIFF 4
/* per-resonator saturation guard: keeps the Q26 int32 datapath from overflowing if the
 * coupling level ever spikes. ±16.0 in Q26 is far above normal operation (~O(1)); when it
 * never fires the path is bit-identical to no-clamp. Reported in the run summary. */
#define SAT ((fx)(16*ONE))
static inline fx satq(fx v){ return v>SAT?SAT:(v<-SAT?-SAT:v); }

typedef struct {
    int   N;
    fx   *s1,*s2,*a1,*r2,*gi;
    fx   *cc;                                     /* per-resonator coupling coeff (PEAK-GAIN-NORMALIZED) */
    fx   *yprev,*ycur;                            /* directional coupling scratch (current outputs) */
    fx    coup; int cmode;                        /* coupling gain + mode (0 same-sample,1 delayed)*/
    int   hopD; fx *hist; long tcount;            /* per-link delay: y_{i-1}[t-hopD] ring (hopD rows x N) */
    long  satcount;                               /* # of saturation events (stability telltale) */
    fx   *cbufA[3],*cbufB[3]; int clenA[3],clenB[3],cposA[3],cposB[3]; fx cg,widthf;
    fx   *dbuf[NDIFF]; int dlen[NDIFF],dpos[NDIFF]; fx dg; int diff_on;
    fx    xm1,xm2;
    double  wetg; int64_t wetg_k;
} reverb;

static inline fx ap_proc(fx *buf,int len,int *pos,fx g,fx x){
    int r=*pos; fx wd=buf[r];
    fx w=x+fmul(g,wd);
    fx y=wd-fmul(g,w);
    buf[r]=w; *pos=(r+1>=len)?0:r+1;
    return y;
}

static reverb* rev_init(int N,double t60,double wet,double hidamp,double diffuse,double width,
                        double coup,int cmode,int hopD){
    reverb*R=calloc(1,sizeof*R); R->N=N;
    R->s1=calloc(N,4);R->s2=calloc(N,4);R->a1=calloc(N,4);R->r2=calloc(N,4);R->gi=calloc(N,4);
    R->cc=calloc(N,4);
    R->yprev=calloc(N,4);R->ycur=calloc(N,4);
    R->coup=FX(coup); R->cmode=cmode; R->satcount=0;
    R->hopD = hopD<1?1:hopD; R->tcount=0;
    R->hist = calloc((size_t)R->hopD*N,4);        /* per-link delay ring: hopD samples x N resonators */
    unsigned rng=99173;
    for(int i=0;i<N;i++){
        rng=rng*1103515245u+12345u; double jit=((rng>>9)/4194304.0-1.0)*0.5;
        double frac=(double)i/(N-1);
        double f=30.0*pow(18000.0/30.0,frac)*pow(2.0,jit/12.0); if(f>0.45*RATE)f=0.45*RATE;
        double w=2*M_PI*f/RATE, t=t60*(1.0-hidamp*frac*0.85); if(t<0.05)t=0.05;
        double r=exp(-6.9078/(t*RATE));
        R->a1[i]=FX(2*r*cos(w)); R->r2[i]=FX(r*r);
        R->gi[i]=FX((1.0-r*r)*(0.5+0.5*((rng>>11)&255)/255.0));
        /* PEAK-GAIN-NORMALIZED directional coupling. Injecting y[i-1] straight into a high-Q
         * resonator gets multiplied by that resonator's RESONANT PEAK gain Gpeak=|H(e^jw0)| ~
         * 1/((1-r)·sin w0) — enormous at low f — and cascaded over hundreds of stages it
         * explodes. Setting cc = coup/Gpeak guarantees every link transfers AT MOST `coup` of
         * its neighbour's spectral energy (its gain at the neighbour's frequency is < its own
         * peak), so a chain of (<1) links is provably BOUNDED and the directional flow survives. */
        double a1d=2*r*cos(w), r2d=r*r;
        double re=1.0-a1d*cos(w)+r2d*cos(2*w), im=a1d*sin(w)-r2d*sin(2*w);
        double gpeak=1.0/sqrt(re*re+im*im);           /* resonator peak magnitude */
        R->cc[i]=FX(coup/gpeak);
    }
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
static void rev_free(reverb*R){ if(!R)return;
    free(R->s1);free(R->s2);free(R->a1);free(R->r2);free(R->gi);free(R->cc);free(R->yprev);free(R->ycur);free(R->hist);
    for(int k=0;k<NDIFF;k++)free(R->dbuf[k]);
    for(int k=0;k<3;k++){free(R->cbufA[k]);free(R->cbufB[k]);} free(R); }

/* one stereo WET sample, NON-RECIPROCAL directional chain. */
static void rev_process(reverb*R,fx x,double*outL,double*outR){
    fx xin=x-R->xm2; R->xm2=R->xm1; R->xm1=x;
    fx d=xin; if(R->diff_on) for(int k=0;k<NDIFF;k++) d=ap_proc(R->dbuf[k],R->dlen[k],&R->dpos[k],R->dg,d);
    int64_t sum=0;
    int N=R->N;
    if(R->cmode==0){
        /* SAME-SAMPLE forward scan: y[i] sees THIS sample's y[i-1]. One combinational sweep
         * i=0..N-1 — feed-forward (no cycle), but a deep same-sample dependency chain. */
        fx prevY=0;
        for(int i=0;i<N;i++){
            fx exc=fmul(R->gi[i],d)+fmul(R->cc[i],prevY);
            fx y=fmul(R->a1[i],R->s1[i])-fmul(R->r2[i],R->s2[i])+exc;
            fx ys=satq(y); if(ys!=y){R->satcount++; y=ys;}
            R->s2[i]=R->s1[i]; R->s1[i]=y;
            prevY=y; sum+=y;
        }
    } else {
        /* DELAYED forward hop: resonator i sees y[i-1] from hopD samples ago => an energy
         * packet walks ONE resonator UP the chain every hopD samples. Over the whole bank
         * (low->high f) that is a TRAVELING WAVE of pitch climbing the spectrum at a musical
         * rate: a tail that rises in brightness as it decays — a space that cannot exist. */
        fx *row = R->hist + (size_t)(R->tcount % R->hopD)*N;   /* currently holds y[t-hopD] */
        fx *yc=R->ycur;
        for(int i=0;i<N;i++){
            fx cin = (i>0)? fmul(R->cc[i],row[i-1]) : 0;       /* y[i-1] delayed by hopD */
            fx exc=fmul(R->gi[i],d)+cin;
            fx y=fmul(R->a1[i],R->s1[i])-fmul(R->r2[i],R->s2[i])+exc;
            fx ys=satq(y); if(ys!=y){R->satcount++; y=ys;}
            R->s2[i]=R->s1[i]; R->s1[i]=y;
            yc[i]=y; sum+=y;
        }
        memcpy(row, yc, (size_t)N*4);             /* commit current outputs into the delay ring */
        R->tcount++;
    }
    fx wf=wet_scale(sum,R->wetg_k);
    fx la=wf; for(int k=0;k<3;k++) la=ap_proc(R->cbufA[k],R->clenA[k],&R->cposA[k],R->cg,la);
    fx rb=wf; for(int k=0;k<3;k++) rb=ap_proc(R->cbufB[k],R->clenB[k],&R->cposB[k],R->cg,rb);
    fx l=wf+fmul(R->widthf,la-wf), r=wf+fmul(R->widthf,rb-wf);
    *outL=(double)l/ONE; *outR=(double)r/ONE;
}

/* ---- WAV io (16-bit mono read; stereo write) — verbatim from src/reverb.c ---- */
static float* wav_read(const char*p,long*nout){
    FILE*f=fopen(p,"rb"); if(!f){perror(p);return NULL;}
    char id[4]; unsigned u; int ch=1,bits=16; float*buf=NULL;
    if(fread(id,1,4,f)!=4){fclose(f);return NULL;} fread(&u,4,1,f); fread(id,1,4,f);
    while(fread(id,1,4,f)==4){ fread(&u,4,1,f);
        if(!memcmp(id,"fmt ",4)){ unsigned char fmt[40]; long want=u<40?u:40; fread(fmt,1,want,f);
            ch=fmt[2]|(fmt[3]<<8); bits=fmt[14]|(fmt[15]<<8); if((long)u>want)fseek(f,u-want,SEEK_CUR);
        } else if(!memcmp(id,"data",4)){ long frames=(long)u/(bits/8)/ch; buf=malloc(sizeof(float)*frames);
            for(long i=0;i<frames;i++){ double a=0; for(int c=0;c<ch;c++){ short v=0; fread(&v,2,1,f); a+=v/32768.0; } buf[i]=(float)(a/ch); }
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
    if(argc<3){fprintf(stderr,"usage: %s in.wav out.wav [N t60 wet hidamp diffuse width coup mode]\n",argv[0]);return 1;}
    int    N      = argc>3?atoi(argv[3]):512;
    double t60    = argc>4?atof(argv[4]):3.0;
    double wet    = argc>5?atof(argv[5]):0.30;
    double hidamp = argc>6?atof(argv[6]):0.45;
    double diffuse= argc>7?atof(argv[7]):0.7;
    double width  = argc>8?atof(argv[8]):0.85;
    double coup   = argc>9?atof(argv[9]):0.5;
    int    mode   = argc>10?atoi(argv[10]):0;
    double tailsec= argc>11?atof(argv[11]):t60*1.8;   /* explicit tail length (s) to probe decay */
    int    hopD   = argc>12?atoi(argv[12]):1;         /* samples per directional hop (delayed relay) */

    long n; float*dry=wav_read(argv[1],&n); if(!dry){fprintf(stderr,"read fail\n");return 1;}
    long tail=(long)(tailsec*RATE), total=n+tail;
    reverb*R=rev_init(N,t60,wet,hidamp,diffuse,width,coup,mode,hopD);
    short*out=malloc(sizeof(short)*total*2); double peak=0,wpeak=0; double drymix=1.0-wet;
    int nan_seen=0;
    for(long t=0;t<total;t++){
        double xd = t<n?dry[t]:0.0; fx x=FX(xd); double l,r; rev_process(R,x,&l,&r);
        if(isnan(l)||isnan(r)||isinf(l)||isinf(r)){nan_seen=1;l=r=0;}
        if(fabs(l)>wpeak)wpeak=fabs(l); if(fabs(r)>wpeak)wpeak=fabs(r);
        double oL=softceil(xd*drymix+l), oR=softceil(xd*drymix+r);
        if(fabs(oL)>peak)peak=fabs(oL); if(fabs(oR)>peak)peak=fabs(oR);
        out[t*2]=(short)lrint(oL*32767.0); out[t*2+1]=(short)lrint(oR*32767.0);
    }
    wav_write_stereo(argv[2],out,total);
    fprintf(stderr,"nonrecip: N=%d t60=%.2f wet=%.2f coup=%.2f mode=%d hopD=%d -> %s (%.1fs outpeak=%.3f wetpeak=%.3f sat=%ld nan=%d)\n",
            N,t60,wet,coup,mode,hopD,argv[2],total/RATE,peak,wpeak,R->satcount,nan_seen);
    rev_free(R); free(dry); free(out); return 0;
}
