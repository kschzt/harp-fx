/* reverb.c — RME-grade resonator-NETWORK reverb. Pure fixed-point (Q26), fabric-ready.
 *
 * Signal path:  in → [input diffusion: 4 Schroeder allpass] → [N bandpass resonators] →
 *               [decorrelated L/R output taps] → wet (host mixes the dry, §8.8).
 *
 * The bandpass bank is the dense "resonator network" — each resonator selectively rings at
 * the input's frequencies (true bandpass: zeros at DC+Nyquist), no global feedback, so it is
 * unconditionally stable. Diffusion spreads transients in time (smooth, dense early field);
 * the decorrelated taps give a wide stereo tail. No hard clipping anywhere (RME bar).
 *
 *   cc -O3 -Wall src/reverb.c -o reverb -lm
 *   reverb in.wav out.wav [N t60 wet hidamp diffuse width]
 *   reverb --bench N
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

/* The reverb DSP (rev_init/rev_process/rev_free + the Q26 fixed-point kernel) now lives in
 * a shared header so the harp-fx device bridge (device/reverb_engine.c) links the SAME
 * engine — the device must reproduce this tool bit-for-bit (§8.8). This file keeps the
 * standalone `reverb` CLI + the WAV I/O + the 6/6 property tests (tests/run.sh). */
#include "reverb_engine_core.h"

/* ---- WAV io (16-bit, mono read; stereo write) ---- */
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
static inline double softceil(double v){ /* transparent below 0.9, never hard-clips */
    if(v> 0.9)return 0.9+0.1*tanh((v-0.9)/0.1);
    if(v<-0.9)return -0.9-0.1*tanh((v+0.9)/0.1); return v; }

int main(int argc,char**argv){
    if(argc>=3 && !strcmp(argv[1],"--bench")){
        int N=atoi(argv[2]); reverb*R=rev_init(N,2.5,0.4,0.5,0.7,0.8);
        int n=(int)(2*RATE); struct timespec a,b; clock_gettime(CLOCK_MONOTONIC,&a);
        volatile double acc=0;
        for(int i=0;i<n;i++){ double l,r; rev_process(R,FX(i%97==0?0.5:0.0),&l,&r); acc+=l+r; }
        clock_gettime(CLOCK_MONOTONIC,&b);
        double wall=(b.tv_sec-a.tv_sec)+(b.tv_nsec-a.tv_nsec)*1e-9;
        printf("N=%-6d RT=%.2fx %s (%.0f Mres/s)\n",N,2.0/wall,2.0/wall>=1?"REALTIME":"slow",(double)N*RATE/1e6);
        (void)acc; rev_free(R); return 0;
    }
    if(argc<3){fprintf(stderr,"usage: %s in.wav out.wav [N t60 wet hidamp diffuse width] | --bench N\n",argv[0]);return 1;}
    int    N      = argc>3?atoi(argv[3]):3000;
    double t60    = argc>4?atof(argv[4]):2.5;
    double wet    = argc>5?atof(argv[5]):0.45;
    double hidamp = argc>6?atof(argv[6]):0.45;
    double diffuse= argc>7?atof(argv[7]):0.7;
    double width  = argc>8?atof(argv[8]):0.85;

    long n; float*dry=wav_read(argv[1],&n); if(!dry){fprintf(stderr,"read fail\n");return 1;}
    long tail=(long)(t60*1.5*RATE), total=n+tail;
    reverb*R=rev_init(N,t60,wet,hidamp,diffuse,width);
    short*out=malloc(sizeof(short)*total*2); double peak=0; double drymix=1.0-wet;
    for(long t=0;t<total;t++){
        double xd = t<n?dry[t]:0.0; fx x=FX(xd); double l,r; rev_process(R,x,&l,&r);
        double oL=softceil(xd*drymix+l), oR=softceil(xd*drymix+r);
        if(fabs(oL)>peak)peak=fabs(oL); if(fabs(oR)>peak)peak=fabs(oR);
        out[t*2]=(short)lrint(oL*32767.0); out[t*2+1]=(short)lrint(oR*32767.0);
    }
    wav_write_stereo(argv[2],out,total);
    fprintf(stderr,"reverb: N=%d t60=%.1fs wet=%.2f stereo → %s (%.1fs peak=%.3f)\n",N,t60,wet,argv[2],total/RATE,peak);
    rev_free(R); free(dry); free(out); return 0;
}
