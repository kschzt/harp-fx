/* reverb_fabric.c — drive reverb_kernel and process dry audio THROUGH the fabric (HARP §8.8).
 * PS side of the port.
 *
 * Mirrors src/reverb.c's coefficient setup EXACTLY (the rev_init rng + a1/r2/gi/dg/cg/widthf
 * formulas, frozen double->Q26; the wet gain frozen double->Q55), then streams the dry WAV
 * through reverb_kernel block by block and writes the WET-ONLY stereo WAV (per §8.8 the device
 * emits wet only; the host mixes the dry). Because every coefficient here is computed verbatim
 * from src/reverb.c and the kernel's integer recurrence is identical, the output is BIT-EXACT to
 * src/reverb.c — hence to fabric/GOLDEN_wet.wav.
 *
 * Two builds:
 *   software TB :  c++ -O2 reverb_fabric.c reverb_kernel.cpp -o reverb_tb -lm   (calls the kernel
 *                  directly; proves the kernel logic vs src/reverb.c before Vivado)
 *   on the Kria:  cc  -O2 -DFABRIC reverb_fabric.c -o reverb_fabric -lm         (mmaps the kernel's
 *                  AXI-Lite /dev/uio4 + u-dma-buf /dev/udmabuf0, runs ap_start/done)
 *
 *   run: ./reverb_tb        IN.wav OUT.wav [N t60 wet hidamp diffuse width]
 *        sudo ./reverb_fabric IN.wav OUT.wav [N t60 wet hidamp diffuse width]
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "reverb_defs.h"

#ifdef FABRIC
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
/* s_axi_ctrl byte-offsets — VERIFIED against xreverb_kernel_hw.h (M2 csynth output).
 * (cf,coef,xin,yout,n,do_init) = 4 ptr + 2 int. */
#define AP_CTRL  0x00
#define R_CF     0x10   /* 64-bit DDR addr of rev_coeffs */
#define R_COEF   0x1c   /* 64-bit DDR addr of a1|r2|gi   */
#define R_XIN    0x28   /* 64-bit DDR addr of dry block  */
#define R_YOUT   0x34   /* 64-bit DDR addr of wet block  */
#define R_N      0x40   /* 32-bit sample count           */
#define R_INIT   0x48   /* 32-bit do_init flag           */
/* DDR layout inside the udmabuf (generous alignment) */
#define OFF_CF   0x00000
#define OFF_COEF 0x00040
#define OFF_XIN  0x10000
#define OFF_YOUT 0x20000
static uint64_t read_phys(const char*p){FILE*f=fopen(p,"r");if(!f)return 0;unsigned long long v=0;if(fscanf(f,"%llx",&v)!=1)v=0;fclose(f);return v;}
#else
extern void reverb_kernel(const struct rev_coeffs*, const fx*, const fx*, fx*, int, int);
#endif

static inline fx FX(double x){ return (fx)llround(x*(double)ONE); }

/* softceil — transparent below 0.9, tanh soft-knee above; never hard-clips. Byte-identical
 * to src/reverb.c (the golden output recipe). The Q26->int16 cast in the host-post loop MUST
 * go through this: a >full-scale wet sample cast straight to short wraps mod 2^16 into an
 * opposite-polarity full-scale click (RME bar: never hard-clip). Transparent for the golden
 * input (wet peak 0.374 << 0.9) so GOLDEN_wet.wav is unchanged. */
static inline double softceil(double v){
    if(v> 0.9)return 0.9+0.1*tanh((v-0.9)/0.1);
    if(v<-0.9)return -0.9-0.1*tanh((v+0.9)/0.1); return v; }

/* ---- WAV io (16-bit, mono read; stereo write) — verbatim from src/reverb.c ---- */
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

int main(int argc,char**argv){
    setbuf(stderr,NULL);
    if(argc<3){fprintf(stderr,"usage: %s IN.wav OUT.wav [N t60 wet hidamp diffuse width]\n",argv[0]);return 1;}
    int    N      = argc>3?atoi(argv[3]):3000;
    double t60    = argc>4?atof(argv[4]):2.5;
    double wet    = argc>5?atof(argv[5]):0.45;
    double hidamp = argc>6?atof(argv[6]):0.45;
    double diffuse= argc>7?atof(argv[7]):0.7;
    double width  = argc>8?atof(argv[8]):0.85;
    if(N>RESN){fprintf(stderr,"N=%d exceeds RESN=%d (rebuild kernel)\n",N,RESN);return 1;}

    long n; float*dry=wav_read(argv[1],&n); if(!dry){fprintf(stderr,"read fail\n");return 1;}
    long tail=(long)(t60*1.5*RATE), total=n+tail;
    short*out=malloc(sizeof(short)*total*2);

    /* ---- coefficient setup: double -> Q26/Q55, frozen once (verbatim src/reverb.c rev_init) ---- */
    static fx a1[RESN], r2[RESN], gi[RESN];
    unsigned rng=99173;
    for(int i=0;i<N;i++){
        rng=rng*1103515245u+12345u; double jit=((rng>>9)/4194304.0-1.0)*0.5;
        double frac=(double)i/(N-1);
        double f=30.0*pow(18000.0/30.0,frac)*pow(2.0,jit/12.0); if(f>0.45*RATE)f=0.45*RATE;
        double w=2*M_PI*f/RATE, t=t60*(1.0-hidamp*frac*0.85); if(t<0.05)t=0.05;
        double r=exp(-6.9078/(t*RATE));
        a1[i]=FX(2*r*cos(w)); r2[i]=FX(r*r);
        gi[i]=FX((1.0-r*r)*(0.5+0.5*((rng>>11)&255)/255.0));
    }
    int32_t dg=FX(0.45*diffuse), diff_on=(diffuse>0.001);
    int32_t cg=FX(0.5), widthf=FX(width>1?1:width<0?0:width);
    double wetg=wet*12.0/sqrt((double)N);
    int64_t wetg_k=(int64_t)llround(wetg*(double)((int64_t)1<<WSHIFT));

    /* ---- buffers: coeffs (once), dry block in, wet block out ---- */
#ifdef FABRIC
    int ufd=open("/dev/uio4",O_RDWR|O_SYNC); if(ufd<0){perror("/dev/uio4");return 1;}
    volatile uint32_t*creg=mmap(0,0x10000,PROT_READ|PROT_WRITE,MAP_SHARED,ufd,0);
    if(creg==MAP_FAILED){perror("mmap uio");return 1;}
    uint64_t phys=read_phys("/sys/class/u-dma-buf/udmabuf0/phys_addr");
    if(!phys){fprintf(stderr,"no udmabuf0 phys_addr\n");return 1;}
    int dfd=open("/dev/udmabuf0",O_RDWR|O_SYNC); if(dfd<0){perror("/dev/udmabuf0");return 1;}
    uint8_t*dbuf=mmap(0,16*1024*1024,PROT_READ|PROT_WRITE,MAP_SHARED,dfd,0);
    if(dbuf==MAP_FAILED){perror("mmap udmabuf");return 1;}
    /* volatile: the u-dma-buf O_SYNC mapping is DEVICE memory on aarch64 — memset() and
     * -O2 NEON-vectorized struct stores SIGBUS. Every store MUST be aligned + scalar. */
    volatile struct rev_coeffs *CF =(volatile struct rev_coeffs*)(dbuf+OFF_CF);
    volatile fx                *COE=(volatile fx*)               (dbuf+OFF_COEF);
    volatile fx                *XIN=(volatile fx*)               (dbuf+OFF_XIN);
    volatile fx                *YOU=(volatile fx*)               (dbuf+OFF_YOUT);
    uint64_t pa_cf=phys+OFF_CF, pa_co=phys+OFF_COEF, pa_xi=phys+OFF_XIN, pa_yo=phys+OFF_YOUT;
    creg[R_CF  /4]=(uint32_t)pa_cf; creg[R_CF  /4+1]=(uint32_t)(pa_cf>>32);
    creg[R_COEF/4]=(uint32_t)pa_co; creg[R_COEF/4+1]=(uint32_t)(pa_co>>32);
    creg[R_XIN /4]=(uint32_t)pa_xi; creg[R_XIN /4+1]=(uint32_t)(pa_xi>>32);
    creg[R_YOUT/4]=(uint32_t)pa_yo; creg[R_YOUT/4+1]=(uint32_t)(pa_yo>>32);
    /* write coeffs to DDR with aligned scalar stores (NO memset on Device mem) */
    for(int i=0;i<N;i++){ COE[i]=a1[i]; COE[RESN+i]=r2[i]; COE[2*RESN+i]=gi[i]; }
    CF->n_res=N; CF->diff_on=diff_on; CF->dg=dg; CF->cg=cg; CF->widthf=widthf; CF->_pad=0;
    CF->wetg_k=wetg_k;        /* single aligned 8-byte store (Device-mem safe, not memset) */
    fprintf(stderr,"ctrl@0xa0000000 udmabuf phys=0x%llx\n",(unsigned long long)phys);
#else
    static struct rev_coeffs CFv; static fx COEv[3*RESN]; static fx XINv[BS]; static fx YOUv[2*BS];
    CFv.n_res=N; CFv.diff_on=diff_on; CFv.dg=dg; CFv.cg=cg; CFv.widthf=widthf; CFv._pad=0; CFv.wetg_k=wetg_k;
    for(int i=0;i<N;i++){ COEv[i]=a1[i]; COEv[RESN+i]=r2[i]; COEv[2*RESN+i]=gi[i]; }
    struct rev_coeffs*CF=&CFv; fx*COE=COEv; fx*XIN=XINv; fx*YOU=YOUv;
#endif

    fprintf(stderr,"reverb_fabric: N=%d t60=%.1f wet=%.2f -> %s (%.1fs, %s)\n",
            N,t60,wet,argv[2],total/RATE,
#ifdef FABRIC
            "FABRIC"
#else
            "software TB"
#endif
    );

    double peak=0;
    for(long pos=0;pos<total;pos+=BS){
        int bs=(int)(total-pos<BS?total-pos:BS);
        /* fill the dry block (float->Q26 on the host; pad the t60 tail with zeros) */
        for(int t=0;t<bs;t++){ long idx=pos+t; double xd=idx<n?dry[idx]:0.0; XIN[t]=FX(xd); }
        int do_init=(pos==0)?1:0;
        /* ---- run the kernel for this block ---- */
#ifdef FABRIC
        creg[R_INIT/4]=(uint32_t)do_init;
        creg[R_N   /4]=(uint32_t)bs;
        creg[AP_CTRL/4]=1;                       /* ap_start */
        long guard=0; while(!(creg[AP_CTRL/4]&0x2)){ if(++guard>2000000000){fprintf(stderr,"HANG @blk %ld\n",pos);return 2;} }
        if(pos<3*BS) fprintf(stderr,"blk %ld ok (bs=%d init=%d)\n",pos,bs,do_init);
#else
        reverb_kernel(CF,COE,XIN,YOU,bs,do_init);
#endif
        /* ---- host post: Q26 -> int16 wet-only stereo (== golden recipe) ---- */
        for(int t=0;t<bs;t++){
            double oL=softceil((double)YOU[t*2]/ONE), oR=softceil((double)YOU[t*2+1]/ONE);
            if(fabs(oL)>peak)peak=fabs(oL);
            if(fabs(oR)>peak)peak=fabs(oR);
            out[(pos+t)*2]  =(short)lrint(oL*32767.0);
            out[(pos+t)*2+1]=(short)lrint(oR*32767.0);
        }
    }
    wav_write_stereo(argv[2],out,total);
    fprintf(stderr,"wrote %s (%.1fs peak=%.3f)\n",argv[2],total/RATE,peak);
    free(dry); free(out); return 0;
}
