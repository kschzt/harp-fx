/* ring_neighbor.c — COUPLED-MESH reverb prototype: nearest-neighbor (ring) coupling.
 *
 * Adapted from src/reverb.c + src/reverb_engine_core.h. KEEPS, byte-for-byte:
 *   - the WAV I/O (16-bit mono read / stereo write) and softceil output stage,
 *   - the Q26 fixed-point primitive  fmul / FX  (== fabric ap_fixed datapath),
 *   - the input diffusion (4 Schroeder allpasses) and the bandpass input source,
 *   - the per-resonator 2-pole bandpass and the stereo decorrelation taps.
 * REPLACES the INDEPENDENT resonator loop with a RING-COUPLED one: the resonators are
 * laid out on a ring (index = log-frequency), and each one feeds a fraction of its output
 * into its two neighbors. Energy travels around the lattice -> a waveguide-mesh-like space.
 *
 * THE STABILITY PROBLEM (why the bank was decoupled before):
 *   a high-Q 2-pole resonator has peak gain G ~ 1/(1-r^2) ~ 1000 for a multi-second t60.
 *   An energy-ADDING neighbor-sum coupling  kc*(y[i-1]+y[i+1])  has a collective mode
 *   (all neighbors in phase, ring-adjacency eigenvalue +2) whose loop gain is ~2*kc*G, so
 *   it self-oscillates once kc > ~1/(2G) ~ 5e-4 -- far too weak to hear. That is the
 *   runaway we are taming.
 * THE FIX (principled, not a band-aid):
 *   use the discrete-Laplacian (diffusive) coupling  kc*(y[i-1] - 2*y[i] + y[i+1]).
 *   The ring Laplacian is negative-semidefinite (eigenvalues 2cos(theta)-2 in [-4,0]), so
 *   y^T (kc L) y <= 0: the coupling can ONLY remove L2 energy from the network, never add
 *   it. Each resonator is already individually lossy (r<1), so resonators-lossy +
 *   coupling-lossy => the whole mesh is passive => BOUNDED for any kc>0. The +kc*y[i+-1]
 *   off-diagonals still carry traveling waves between cells (this IS the wave equation's
 *   spatial term); the -2*kc*y[i] self term is the price -- a little extra damping. So we
 *   keep the requested "each resonator feeds its neighbors" topology AND stay bounded.
 *
 * COUPLING READ (the FPGA-shape lever, --read):
 *   snapshot  : both neighbors are read from the PREVIOUS sample (yprev[]). This is a
 *               symmetric leapfrog FDTD step -- TIME-STEPPED, a GPU could do it (read last
 *               frame's lattice buffer, update all cells in parallel).
 *   inplace   : the backward neighbor y[i-1] is read from THIS sample (already computed in
 *               the same pass) while y[i+1] is from last sample. The forward dependency
 *               0->1->2->...->N-1 is a SAME-SAMPLE sequential recurrence around the ring --
 *               a GPU cannot parallelize the cells within one audio sample. This is the
 *               only-Kria shape (zero-delay wave-front in the forward direction).
 *
 *   cc -O3 -Wall explore/ring_neighbor.c -o /tmp/ring -lm
 *   ring in.wav out.wav [N t60 wet hidamp diffuse width kc form read order]
 *     form : 0=laplacian(stable,default)  1=neighbor-sum(energy-adding, blows up)
 *     read : 0=snapshot(time-stepped)     1=inplace(same-sample recurrence)
 *     order: 0=by-frequency(default)      1=interleaved(neighbors are far in pitch)
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
typedef int32_t fx;
static inline fx fmul(fx a,fx b){ return (fx)(((int64_t)a*(int64_t)b)>>FB); }
static inline fx FX(double x){ return (fx)llround(x*(double)ONE); }
static inline fx wet_scale(int64_t sum,int64_t wetg_k){
    __int128 p=(__int128)sum*(__int128)wetg_k;
    int64_t r = (p>=0)? (int64_t)(( p+((__int128)1<<(WSHIFT-1)))>>WSHIFT)
                      :-(int64_t)((-p+((__int128)1<<(WSHIFT-1)))>>WSHIFT);
    return (fx)r;
}
/* canonical Schroeder allpass: w=x+g*w[n-D]; y=w[n-D]-g*w (used for diffusion + decorr) */
static inline fx ap_proc(fx *buf,int len,int *pos,fx g,fx x){
    int r=*pos; fx wd=buf[r];
    fx w=x+fmul(g,wd);
    fx y=wd-fmul(g,w);
    buf[r]=w; *pos=(r+1>=len)?0:r+1;
    return y;
}

/* ---- WAV io (copied verbatim from src/reverb.c) ---- */
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

/* ---- the ring-coupled resonator mesh ---- */
typedef struct {
    int N;
    fx *s1,*s2,*a1,*r2,*gi;       /* per-resonator state + 2-pole coeffs (as in template) */
    fx *yprev,*ycur;             /* neighbor-output buffers (the coupling state)          */
    int *im1,*ip1;              /* ring neighbor indices (precomputed for any --order)    */
    fx kc;                      /* coupling gain (Q26)                                    */
    int form, read_mode;        /* 0/1 laplacian|sum ; 0/1 snapshot|inplace              */
    /* shared front/back ends (verbatim behavior from the template) */
    fx *dbuf[NDIFF]; int dlen[NDIFF],dpos[NDIFF]; fx dg; int diff_on;
    fx *cbufA[3],*cbufB[3]; int clenA[3],clenB[3],cposA[3],cposB[3]; fx cg,widthf;
    fx xm1,xm2;
    int64_t wetg_k;
    /* diagnostics */
    double istate_peak;         /* max |resonator state| seen (internal boundedness)      */
} mesh;

static mesh* mesh_init(int N,double t60,double wet,double hidamp,double diffuse,double width,
                       double kc,int form,int read_mode,int order){
    mesh*R=calloc(1,sizeof*R); R->N=N; R->form=form; R->read_mode=read_mode;
    R->s1=calloc(N,4);R->s2=calloc(N,4);R->a1=calloc(N,4);R->r2=calloc(N,4);R->gi=calloc(N,4);
    R->yprev=calloc(N,4);R->ycur=calloc(N,4);R->im1=calloc(N,sizeof(int));R->ip1=calloc(N,sizeof(int));
    R->kc=FX(kc);
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
    /* ring adjacency. order 0: neighbor in index == neighbor in pitch (smooth spectral
     * diffusion). order 1: bit-reversal-ish interleave so neighbors are far apart in pitch
     * (more chaotic sloshing). We store the index map; the kernel just follows im1/ip1. */
    int *perm=calloc(N,sizeof(int));
    for(int i=0;i<N;i++) perm[i]=i;
    if(order==1){ /* even indices then odd: a simple far-apart interleave on the ring */
        int j=0; for(int i=0;i<N;i+=2) perm[j++]=i; for(int i=1;i<N;i+=2) perm[j++]=i;
    }
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
    free(R->yprev);free(R->ycur);free(R->im1);free(R->ip1);
    for(int k=0;k<NDIFF;k++)free(R->dbuf[k]);
    for(int k=0;k<3;k++){free(R->cbufA[k]);free(R->cbufB[k]);} free(R); }

/* one stereo wet sample */
static void mesh_process(mesh*R,fx x,double*outL,double*outR){
    fx xin=x-R->xm2; R->xm2=R->xm1; R->xm1=x;              /* bandpass source */
    fx d=xin; if(R->diff_on) for(int k=0;k<NDIFF;k++) d=ap_proc(R->dbuf[k],R->dlen[k],&R->dpos[k],R->dg,d);
    int64_t sum=0;
    int N=R->N; fx kc=R->kc;
    /* ycur starts the sample == yprev; in inplace mode the backward neighbor reads ycur
     * (this-sample if already updated), the forward neighbor + self read yprev. */
    if(R->read_mode) memcpy(R->ycur,R->yprev,(size_t)N*4);
    for(int i=0;i<N;i++){
        fx yb = R->read_mode ? R->ycur[R->im1[i]] : R->yprev[R->im1[i]]; /* backward neighbor */
        fx yf = R->yprev[R->ip1[i]];                                     /* forward  neighbor */
        fx yc = R->yprev[i];                                             /* self (prev sample)*/
        fx couple;
        if(R->form==0) couple = fmul(kc, (yb - (yc<<1) + yf));           /* discrete Laplacian */
        else           couple = fmul(kc, (yb + yf));                     /* energy-adding sum  */
        fx exc = fmul(R->gi[i],d) + couple;
        fx y = fmul(R->a1[i],R->s1[i]) - fmul(R->r2[i],R->s2[i]) + exc;
        R->s2[i]=R->s1[i]; R->s1[i]=y;
        R->ycur[i]=y;
        sum += y;
        double m = fabs((double)y/ONE); if(m>R->istate_peak) R->istate_peak=m;
    }
    { fx*t=R->yprev; R->yprev=R->ycur; R->ycur=t; }        /* swap: ycur becomes next yprev */
    fx wf=wet_scale(sum,R->wetg_k);
    fx la=wf; for(int k=0;k<3;k++) la=ap_proc(R->cbufA[k],R->clenA[k],&R->cposA[k],R->cg,la);
    fx rb=wf; for(int k=0;k<3;k++) rb=ap_proc(R->cbufB[k],R->clenB[k],&R->cposB[k],R->cg,rb);
    fx l=wf+fmul(R->widthf,la-wf), r=wf+fmul(R->widthf,rb-wf);
    *outL=(double)l/ONE; *outR=(double)r/ONE;
}

int main(int argc,char**argv){
    if(argc<3){fprintf(stderr,"usage: %s in.wav out.wav [N t60 wet hidamp diffuse width kc form read order]\n",argv[0]);return 1;}
    int    N      = argc>3?atoi(argv[3]):384;
    double t60    = argc>4?atof(argv[4]):3.0;
    double wet    = argc>5?atof(argv[5]):0.45;
    double hidamp = argc>6?atof(argv[6]):0.45;
    double diffuse= argc>7?atof(argv[7]):0.7;
    double width  = argc>8?atof(argv[8]):0.85;
    double kc     = argc>9?atof(argv[9]):0.02;
    int    form   = argc>10?atoi(argv[10]):0;
    int    read_m = argc>11?atoi(argv[11]):0;
    int    order  = argc>12?atoi(argv[12]):0;

    long n; float*dry=wav_read(argv[1],&n); if(!dry){fprintf(stderr,"read fail\n");return 1;}
    long tail=(long)(t60*1.8*RATE), total=n+tail;
    mesh*R=mesh_init(N,t60,wet,hidamp,diffuse,width,kc,form,read_m,order);
    short*out=malloc(sizeof(short)*total*2); double peak=0,drymix=1.0-wet;
    /* tail-decay probe: RMS in 0.5s windows over the whole render */
    int W=(int)(0.5*RATE), nw=(int)(total/W)+1; double*wrms=calloc(nw,sizeof(double)); int*wc=calloc(nw,sizeof(int));
    long clipn=0;
    for(long t=0;t<total;t++){
        double xd = t<n?dry[t]:0.0; double l,r; mesh_process(R,FX(xd),&l,&r);
        double oL=softceil(xd*drymix+l), oR=softceil(xd*drymix+r);
        if(fabs(oL)>peak)peak=fabs(oL); if(fabs(oR)>peak)peak=fabs(oR);
        if(fabs(oL)>=0.999||fabs(oR)>=0.999)clipn++;
        double m=0.5*(l*l+r*r); int wi=(int)(t/W); if(wi<nw){wrms[wi]+=m;wc[wi]++;}
        out[t*2]=(short)lrint(oL*32767.0); out[t*2+1]=(short)lrint(oR*32767.0);
    }
    wav_write_stereo(argv[2],out,total);
    double peakw=0,lastw=0; for(int i=0;i<nw;i++){ if(wc[i]){double rms=sqrt(wrms[i]/wc[i]); if(rms>peakw)peakw=rms; lastw=rms;} }
    const char*formn=form?"sum":"laplacian"; const char*readn=read_m?"inplace(same-sample)":"snapshot(time-stepped)";
    fprintf(stderr,"ring: N=%d t60=%.1f kc=%.4f form=%s read=%s order=%d -> %s\n",N,t60,kc,formn,readn,order,argv[2]);
    fprintf(stderr,"      len=%.1fs out_peak=%.3f clip=%.3f%% internal_state_peak=%.3f  tail/peak_rms=%.4f (%s)\n",
            total/RATE,peak,100.0*clipn/(total*2),R->istate_peak,
            peakw>0?lastw/peakw:0.0, (peakw>0&&lastw/peakw<0.1)?"DECAYS":(peakw>0&&lastw/peakw<0.5)?"slow-decay":"SUSTAINS/grows");
    mesh_free(R); free(dry); free(out); free(wrms); free(wc); return 0;
}
