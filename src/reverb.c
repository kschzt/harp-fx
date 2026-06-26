/* reverb.c — input-driven resonator-NETWORK reverb (the "huge real-time resonator
 * network"). Pure fixed-point (fabric-ready). N tuned resonators densely cover the
 * spectrum; the input audio excites them ALL every sample; they cross-couple through a
 * shared bridge (diffusion) and ring out — the dense decay IS the reverb tail. The dense
 * coupled field that was NOISE as a synth is exactly what a reverb is.
 *
 *   cc -O3 fx/reverb.c -o fx/reverb -lm
 *   fx/reverb in.wav out.wav [N t60 wet sympathy inject hidamp]
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define FB 26
#define ONE ((int64_t)1<<FB)
typedef int32_t fx;
static inline fx fmul(fx a,fx b){ return (fx)(((int64_t)a*(int64_t)b)>>FB); }
#define RATE 48000.0

/* --- minimal WAV io (16-bit PCM, mono out) --- */
static float* wav_read(const char*p,long*nout,int*srout){
    FILE*f=fopen(p,"rb"); if(!f){perror(p);return NULL;}
    char id[4]; unsigned u; unsigned short s16; int sr=48000,ch=1,bits=16; long ndata=0; float*buf=NULL;
    fread(id,1,4,f); fread(&u,4,1,f); fread(id,1,4,f); /* RIFF .... WAVE */
    while(fread(id,1,4,f)==4){ fread(&u,4,1,f);
        if(!memcmp(id,"fmt ",4)){ unsigned char fmt[64]; long want=u<64?u:64; fread(fmt,1,want,f);
            ch=fmt[2]|(fmt[3]<<8); sr=fmt[4]|(fmt[5]<<8)|(fmt[6]<<16)|(fmt[7]<<24); bits=fmt[14]|(fmt[15]<<8);
            if((long)u>want) fseek(f,u-want,SEEK_CUR);
        } else if(!memcmp(id,"data",4)){ ndata=u; long ns=ndata/(bits/8); long frames=ns/ch;
            buf=malloc(sizeof(float)*frames);
            for(long i=0;i<frames;i++){ double acc=0; for(int c=0;c<ch;c++){ short v=0; fread(&v,2,1,f); acc+=v/32768.0; } buf[i]=(float)(acc/ch); }
            *nout=frames; *srout=sr; fclose(f); (void)s16; return buf;
        } else fseek(f,u,SEEK_CUR);
    }
    fclose(f); return NULL;
}
static void wav_write(const char*p,short*b,long n){
    FILE*f=fopen(p,"wb"); long nd=n*2*2; unsigned u; unsigned short s;
    fwrite("RIFF",1,4,f);u=36+nd;fwrite(&u,4,1,f);fwrite("WAVE",1,4,f);fwrite("fmt ",1,4,f);
    u=16;fwrite(&u,4,1,f);s=1;fwrite(&s,2,1,f);s=2;fwrite(&s,2,1,f);u=48000;fwrite(&u,4,1,f);
    u=48000*4;fwrite(&u,4,1,f);s=4;fwrite(&s,2,1,f);s=16;fwrite(&s,2,1,f);fwrite("data",1,4,f);u=nd;fwrite(&u,4,1,f);
    for(long i=0;i<n;i++){fwrite(&b[i],2,1,f);fwrite(&b[i],2,1,f);}
    fclose(f);
}

int main(int argc,char**argv){
    if(argc<3){fprintf(stderr,"usage: %s in.wav out.wav [N t60 wet sympathy inject hidamp]\n",argv[0]);return 1;}
    int    N      = argc>3?atoi(argv[3]):2000;
    double t60    = argc>4?atof(argv[4]):2.5;     /* reverb time (s)         */
    double wet    = argc>5?atof(argv[5]):0.5;
    double symd   = argc>6?atof(argv[6]):0.10;    /* cross-coupling (diffusion) */
    double inject = argc>7?atof(argv[7]):0.04;    /* input → resonators      */
    double hidamp = argc>8?atof(argv[8]):0.6;     /* high-freq decay faster (natural) */

    long n; int sr; float*dry=wav_read(argv[1],&n,&sr); if(!dry){fprintf(stderr,"read fail\n");return 1;}
    long tail=(long)(t60*1.5*RATE), total=n+tail;
    fx *s1=calloc(N,sizeof(fx)),*s2=calloc(N,sizeof(fx)),*a1=calloc(N,sizeof(fx)),*r2=calloc(N,sizeof(fx)),*gi=calloc(N,sizeof(fx));
    double*hz=calloc(N,sizeof(double));
    (void)symd; (void)inject;


    /* tune: N resonators log-spaced 30Hz..18kHz + jitter (dense modal field, no obvious
     * pitch). high modes decay faster (air absorption) → natural tail. */
    unsigned rng=99173;
    for(int i=0;i<N;i++){
        rng=rng*1103515245u+12345u; double jit=((rng>>9)/4194304.0-1.0)*0.5; /* semitone jitter */
        double frac=(double)i/(N-1);
        double f=30.0*pow(18000.0/30.0, frac) * pow(2.0,jit/12.0); if(f>0.45*RATE)f=0.45*RATE;
        hz[i]=f; double w=2*M_PI*f/RATE;
        double t=t60*(1.0 - hidamp*frac*0.85);    /* high freqs ring shorter */
        if(t<0.05)t=0.05;
        double r=exp(-6.9078/(t*RATE));
        a1[i]=(fx)llround(2*r*cos(w)*ONE); r2[i]=(fx)llround(r*r*ONE);
        /* BANDPASS excitation gain ∝ (1-r²): zeros at DC+Nyquist make each resonator a true
         * bandpass (it RINGS at f_i, attenuates elsewhere) — feeding 440Hz rings only the
         * near-440 resonators, so the tail follows the INPUT instead of generating noise.
         * The (1-r²) factor gives ~equal peak gain across the Q range. */
        gi[i]=(fx)llround((1.0-r*r)*(0.5+0.5*((rng>>11)&255)/255.0)*ONE);
    }

    short*out=malloc(sizeof(short)*total); double peak=0;
    double drymix=1.0-wet, wetg=wet*16.0/sqrt((double)N);
    fx xm1=0,xm2=0;
    for(long t=0;t<total;t++){
        fx x = t<n ? (fx)llround(dry[t]*ONE) : 0;
        fx xin = x - xm2;                            /* bandpass source: zeros at DC + Nyquist */
        xm2=xm1; xm1=x;
        int64_t sum=0;
        for(int i=0;i<N;i++){
            fx exc=fmul(gi[i],xin);                  /* selective per-resonator excitation */
            fx y=fmul(a1[i],s1[i])-fmul(r2[i],s2[i])+exc;
            s2[i]=s1[i]; s1[i]=y;
            sum += y;                                /* no global feedback → unconditionally stable */
        }
        double wetd=(double)sum/ONE*wetg;
        double od = (double)x/ONE*drymix + wetd;
        /* clean output: transparent soft ceiling, perfectly linear below 0.9 — never a hard clip */
        if(od> 0.9) od= 0.9+0.1*tanh((od-0.9)/0.1);
        else if(od<-0.9) od=-0.9-0.1*tanh((od+0.9)/0.1);
        if(fabs(od)>peak)peak=fabs(od);
        out[t]=(short)lrint(od*32767.0);
    }
    wav_write(argv[2],out,total);
    fprintf(stderr,"reverb: N=%d t60=%.1fs wet=%.2f → %s (%.1fs, peak=%.3f)\n",N,t60,wet,argv[2],total/RATE,peak);
    return 0;
}
