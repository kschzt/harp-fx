/* spectral_neighbor.c — COUPLED-MESH reverb prototype: FREQUENCY-NEIGHBOR coupling.
 *
 * Adapted from src/reverb.c. Keeps: WAV I/O, the bandpass input source (x-x[n-2]),
 * the NDIFF Schroeder input diffusion, the Q26 fmul primitive, the stereo
 * decorrelation taps, softceil. REPLACES the independent-resonator loop with a
 * coupled mesh in which each resonator exchanges energy with its nearest neighbours
 * IN FREQUENCY (== nearest in index, since the bank is log-frequency ordered).
 *
 * Mechanism (the "space that can't exist"): each sample, a fraction of resonator i's
 * output is poured into its spectral neighbours i-1 / i+1 via a discrete Laplacian on
 * the frequency axis (heat/diffusion operator). A partial poured in at one pitch bleeds
 * a little energy into the bins ~21 cents above/below, which ring at THEIR pitch and
 * bleed further -> energy climbs/spreads across the spectrum as the tail evolves
 * (a glissando-shimmer; energy spontaneously changing pitch). With an upward bias the
 * bloom drifts brighter over time -> an airy, impossible space.
 *
 * SAME-SAMPLE recurrence (the only-Kria flex): resonators are swept low->high freq and
 * resonator i reads neighbour i-1's JUST-COMPUTED output (this sample, Gauss-Seidel) and
 * i+1's previous output. That sequential dependency chain across the whole spectrum is
 * exactly what a GPU cannot close at audio rate; an FPGA pipelines it. Gauss-Seidel is
 * also strictly more stable than the fully-delayed (Jacobi) scheme.
 *
 * STABILITY: high-Q resonators have a near-zero pole margin ((1-r)^2), so a tiny coupling
 * already blows them up (that's why the first cross-coupling attempt screeched). Two levers
 * tame it: (1) MODERATE per-resonator Q (shorter individual ring) so coupling has headroom
 * and the long tail comes from the network, not from individual rings; (2) a saturating
 * clamp on the injected coupling + a final per-resonator limiter, so any runaway mode is
 * bounded (a soft self-oscillation) rather than NaN/overflow. All integer Q26.
 *
 *   cc -O3 -Wall explore/spectral_neighbor.c -o sn -lm
 *   sn in.wav out.wav [N tres cc bias hidamp wet width injlim diffuse]
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
#define RATE 48000.0
#define NDIFF 4

typedef int32_t fx;
static inline fx fmul(fx a,fx b){ return (fx)(((int64_t)a*(int64_t)b)>>FB); }
static inline fx FX(double x){ return (fx)llround(x*(double)ONE); }
static inline fx fsat(fx v,fx lim){ return v>lim?lim:(v<-lim?-lim:v); }
static inline fx wet_scale(int64_t sum,int64_t wetg_k){
    __int128 p=(__int128)sum*(__int128)wetg_k;
    int64_t r=(p>=0)?(int64_t)(( p+((__int128)1<<(WSHIFT-1)))>>WSHIFT)
                    :-(int64_t)((-p+((__int128)1<<(WSHIFT-1)))>>WSHIFT);
    return (fx)r;
}

typedef struct {
    int   N;
    fx   *s1,*s2,*a1,*r2,*gi;
    fx    cc, gL, gR, gC, injlim, ylim;          /* coupling: gain, L/R/centre weights, limits */
    fx   *cbufA[3],*cbufB[3]; int clenA[3],clenB[3],cposA[3],cposB[3]; fx cg,widthf;
    fx   *dbuf[NDIFF]; int dlen[NDIFF],dpos[NDIFF]; fx dg; int diff_on;
    fx    xm1,xm2;
    int64_t wetg_k;
} mesh;

static inline fx ap_proc(fx*buf,int len,int*pos,fx g,fx x){
    int r=*pos; fx wd=buf[r]; fx w=x+fmul(g,wd); fx y=wd-fmul(g,w);
    buf[r]=w; *pos=(r+1>=len)?0:r+1; return y;
}

/* tres = PER-RESONATOR ring time (s); cc_d = coupling gain; bias in [-1,1] up/down drift */
static mesh* mesh_init(int N,double tres,double cc_d,double bias,double hidamp,
                       double wet,double width,double injlim_d){
    mesh*R=calloc(1,sizeof*R); R->N=N;
    R->s1=calloc(N,4);R->s2=calloc(N,4);R->a1=calloc(N,4);R->r2=calloc(N,4);R->gi=calloc(N,4);
    unsigned rng=99173;
    for(int i=0;i<N;i++){
        rng=rng*1103515245u+12345u; double jit=((rng>>9)/4194304.0-1.0)*0.5;
        double frac=(double)i/(N-1);
        double f=30.0*pow(18000.0/30.0,frac)*pow(2.0,jit/12.0); if(f>0.45*RATE)f=0.45*RATE;
        double w=2*M_PI*f/RATE, t=tres*(1.0-hidamp*frac*0.85); if(t<0.03)t=0.03;
        double r=exp(-6.9078/(t*RATE));
        R->a1[i]=FX(2*r*cos(w)); R->r2[i]=FX(r*r);
        R->gi[i]=FX((1.0-r*r)*(0.5+0.5*((rng>>11)&255)/255.0));
    }
    /* coupling weights: bias>0 pushes energy UP the spectrum (toward i+1). Normalise so
     * gL+gR == 1 (pure diffusion rate set by cc), gC = gL+gR (Laplacian centre). */
    double b=bias>1?1:bias<-1?-1:bias;
    double gl=0.5*(1.0-b), gr=0.5*(1.0+b);
    R->gL=FX(gl); R->gR=FX(gr); R->gC=FX(gl+gr);
    R->cc=FX(cc_d); R->injlim=FX(injlim_d); R->ylim=FX(12.0);   /* ylim: hard overflow guard */
    int dl[NDIFF]={59,89,127,173}; R->dg=FX(0.45*0.7); R->diff_on=1;
    for(int k=0;k<NDIFF;k++){R->dlen[k]=dl[k];R->dbuf[k]=calloc(dl[k],4);R->dpos[k]=0;}
    int dA[3]={241,151,97},dB[3]={199,317,113}; R->cg=FX(0.5);
    R->widthf=FX(width>1?1:width<0?0:width);
    for(int k=0;k<3;k++){R->clenA[k]=dA[k];R->cbufA[k]=calloc(dA[k],4);R->cposA[k]=0;
                         R->clenB[k]=dB[k];R->cbufB[k]=calloc(dB[k],4);R->cposB[k]=0;}
    double wetg=wet*12.0/sqrt((double)N);
    R->wetg_k=(int64_t)llround(wetg*(double)((int64_t)1<<WSHIFT));
    return R;
}
static void mesh_free(mesh*R){ if(!R)return;
    free(R->s1);free(R->s2);free(R->a1);free(R->r2);free(R->gi);
    for(int k=0;k<NDIFF;k++)free(R->dbuf[k]);
    for(int k=0;k<3;k++){free(R->cbufA[k]);free(R->cbufB[k]);} free(R); }

static void mesh_process(mesh*R,fx x,double*outL,double*outR){
    fx xin=x-R->xm2; R->xm2=R->xm1; R->xm1=x;
    fx d=xin; if(R->diff_on) for(int k=0;k<NDIFF;k++) d=ap_proc(R->dbuf[k],R->dlen[k],&R->dpos[k],R->dg,d);
    int64_t sum=0; int N=R->N;
    fx*s1=R->s1,*s2=R->s2,*a1=R->a1,*r2=R->r2,*gi=R->gi;
    fx cc=R->cc,gL=R->gL,gR=R->gR,gC=R->gC,injlim=R->injlim,ylim=R->ylim;
    for(int i=0;i<N;i++){
        fx cen=s1[i];
        fx lft=(i>0)?s1[i-1]:cen;        /* i-1 already updated THIS sample (Gauss-Seidel) */
        fx rgt=(i<N-1)?s1[i+1]:cen;      /* i+1 still previous sample */
        /* biased discrete Laplacian on the frequency axis: diffuse output to neighbours */
        fx lap=fmul(gL,lft)+fmul(gR,rgt)-fmul(gC,cen);
        fx inj=fsat(fmul(cc,lap),injlim);
        fx exc=fmul(gi[i],d)+inj;
        fx y=fmul(a1[i],s1[i])-fmul(r2[i],s2[i])+exc;
        y=fsat(y,ylim);                  /* overflow guard (never touches sane signal) */
        s2[i]=cen; s1[i]=y;
        sum+=y;
    }
    fx wf=wet_scale(sum,R->wetg_k);
    fx la=wf; for(int k=0;k<3;k++) la=ap_proc(R->cbufA[k],R->clenA[k],&R->cposA[k],R->cg,la);
    fx rb=wf; for(int k=0;k<3;k++) rb=ap_proc(R->cbufB[k],R->clenB[k],&R->cposB[k],R->cg,rb);
    fx l=wf+fmul(R->widthf,la-wf), r=wf+fmul(R->widthf,rb-wf);
    *outL=(double)l/ONE; *outR=(double)r/ONE;
}

/* ---- WAV io (16-bit mono read; stereo write) ---- */
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
    if(argc<3){fprintf(stderr,"usage: %s in.wav out.wav [N tres cc bias hidamp wet width injlim]\n",argv[0]);return 1;}
    int    N      = argc>3 ?atoi(argv[3]):384;
    double tres   = argc>4 ?atof(argv[4]):0.45;   /* per-resonator ring time (s)   */
    double cc_d   = argc>5 ?atof(argv[5]):8e-4;   /* coupling gain                  */
    double bias   = argc>6 ?atof(argv[6]):0.0;    /* spectral drift up(+)/down(-)   */
    double hidamp = argc>7 ?atof(argv[7]):0.4;
    double wet    = argc>8 ?atof(argv[8]):0.45;
    double width  = argc>9 ?atof(argv[9]):0.85;
    double injlim = argc>10?atof(argv[10]):0.02;

    long n; float*dry=wav_read(argv[1],&n); if(!dry){fprintf(stderr,"read fail\n");return 1;}
    long tail=(long)(6.0*RATE), total=n+tail;     /* generous tail to hear the bloom */
    mesh*R=mesh_init(N,tres,cc_d,bias,hidamp,wet,width,injlim);
    short*out=malloc(sizeof(short)*total*2); double peak=0,drymix=1.0-wet; int nan=0;
    for(long t=0;t<total;t++){
        double xd=t<n?dry[t]:0.0; fx x=FX(xd); double l,r; mesh_process(R,x,&l,&r);
        if(!(l==l)||!(r==r)) nan=1;
        double oL=softceil(xd*drymix+l), oR=softceil(xd*drymix+r);
        if(fabs(oL)>peak)peak=fabs(oL); if(fabs(oR)>peak)peak=fabs(oR);
        out[t*2]=(short)lrint(oL*32767.0); out[t*2+1]=(short)lrint(oR*32767.0);
    }
    wav_write_stereo(argv[2],out,total);
    fprintf(stderr,"sn: N=%d tres=%.2f cc=%.1e bias=%+.2f wet=%.2f -> %s (%.1fs peak=%.3f%s)\n",
            N,tres,cc_d,bias,wet,argv[2],total/RATE,peak,nan?" NaN!":"");
    mesh_free(R); free(dry); free(out); return 0;
}
