/* rev_fabric.c — drive the KR260 reverb_kernel from harp-fx-deviced (HARP §8.8 audio.fx).
 *
 * The PS side of the fabric port, lifted VERBATIM from fabric/reverb_fabric.c's `#ifdef FABRIC`
 * path (the standalone, hardware-verified driver) and wrapped as an open/load/block module so
 * device/reverb_engine.c can swap the per-block DSP from rev_process() to the PL.
 *
 * Built ONLY when -DHARPFX_FABRIC is set (the KR260 target). Linux-only (sys/mman + /dev/uio4).
 */
#include "rev_fabric.h"

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/mman.h>
#include <unistd.h>

/* s_axi_ctrl byte-offsets — VERIFIED against xreverb_kernel_hw.h (csynth output).
 * (cf,coef,xin,yout,n,do_init) = 4 ptr + 2 int. Identical to fabric/reverb_fabric.c. */
#define AP_CTRL  0x00
#define R_CF     0x10   /* 64-bit DDR addr of rev_coeffs */
#define R_COEF   0x1c   /* 64-bit DDR addr of a1|r2|gi   */
#define R_XIN    0x28   /* 64-bit DDR addr of dry block  */
#define R_YOUT   0x34   /* 64-bit DDR addr of wet block  */
#define R_N      0x40   /* 32-bit sample count           */
#define R_INIT   0x48   /* 32-bit do_init flag           */
/* DDR layout inside the udmabuf (generous alignment) — identical to the standalone tool. */
#define OFF_CF   0x00000
#define OFF_COEF 0x00040
#define OFF_XIN  0x10000
#define OFF_YOUT 0x20000

static uint64_t read_phys(const char *p) {
    FILE *f = fopen(p, "r");
    if (!f) return 0;
    unsigned long long v = 0;
    if (fscanf(f, "%llx", &v) != 1) v = 0;
    fclose(f);
    return v;
}

/* mmapped state — the u-dma-buf O_SYNC mapping is DEVICE memory on aarch64, so every store
 * MUST be an aligned scalar through these `volatile` pointers (no memset / no NEON). */
static volatile uint32_t          *g_creg;       /* AXI-Lite control regs (/dev/uio4)  */
static volatile struct rev_coeffs *g_CF;         /* scalar coeffs in DDR               */
static volatile fx                *g_COE;        /* a1|r2|gi packed in DDR             */
static volatile fx                *g_XIN;        /* dry block in DDR                   */
static volatile fx                *g_YOU;        /* wet stereo block in DDR            */
static int                         g_open;       /* 1 once mmaps + DDR pointers ready  */

int rev_fabric_open(void) {
    if (g_open) return 0;
    int ufd = open("/dev/uio4", O_RDWR | O_SYNC);
    if (ufd < 0) { perror("rev_fabric: /dev/uio4"); return -1; }
    volatile uint32_t *creg = mmap(0, 0x10000, PROT_READ | PROT_WRITE, MAP_SHARED, ufd, 0);
    if (creg == MAP_FAILED) { perror("rev_fabric: mmap uio4"); return -1; }
    uint64_t phys = read_phys("/sys/class/u-dma-buf/udmabuf0/phys_addr");
    if (!phys) { fprintf(stderr, "rev_fabric: no udmabuf0 phys_addr\n"); return -1; }
    int dfd = open("/dev/udmabuf0", O_RDWR | O_SYNC);
    if (dfd < 0) { perror("rev_fabric: /dev/udmabuf0"); return -1; }
    uint8_t *dbuf = mmap(0, 16 * 1024 * 1024, PROT_READ | PROT_WRITE, MAP_SHARED, dfd, 0);
    if (dbuf == MAP_FAILED) { perror("rev_fabric: mmap udmabuf0"); return -1; }

    g_creg = creg;
    g_CF  = (volatile struct rev_coeffs *)(dbuf + OFF_CF);
    g_COE = (volatile fx *)              (dbuf + OFF_COEF);
    g_XIN = (volatile fx *)              (dbuf + OFF_XIN);
    g_YOU = (volatile fx *)              (dbuf + OFF_YOUT);

    uint64_t pa_cf = phys + OFF_CF, pa_co = phys + OFF_COEF, pa_xi = phys + OFF_XIN, pa_yo = phys + OFF_YOUT;
    g_creg[R_CF   / 4] = (uint32_t)pa_cf; g_creg[R_CF   / 4 + 1] = (uint32_t)(pa_cf >> 32);
    g_creg[R_COEF / 4] = (uint32_t)pa_co; g_creg[R_COEF / 4 + 1] = (uint32_t)(pa_co >> 32);
    g_creg[R_XIN  / 4] = (uint32_t)pa_xi; g_creg[R_XIN  / 4 + 1] = (uint32_t)(pa_xi >> 32);
    g_creg[R_YOUT / 4] = (uint32_t)pa_yo; g_creg[R_YOUT / 4 + 1] = (uint32_t)(pa_yo >> 32);
    fprintf(stderr, "rev_fabric: ctrl mmap ok, udmabuf phys=0x%llx (DDR pointers programmed)\n",
            (unsigned long long)phys);
    g_open = 1;
    return 0;
}

void rev_fabric_load_coeffs(const reverb *R) {
    if (!g_open || !R) return;
    int N = R->N;
    /* a1|r2|gi packed at COE[0..N), COE[RESN..RESN+N), COE[2*RESN..2*RESN+N) — aligned scalar
     * stores (NO memset on Device memory). These are rev_init's frozen Q26 coeffs verbatim. */
    for (int i = 0; i < N; i++) {
        g_COE[i]           = R->a1[i];
        g_COE[RESN + i]    = R->r2[i];
        g_COE[2 * RESN + i] = R->gi[i];
    }
    g_CF->n_res  = N;
    g_CF->diff_on = R->diff_on;
    g_CF->dg     = R->dg;
    g_CF->cg     = R->cg;
    g_CF->widthf = R->widthf;
    g_CF->_pad   = 0;
    g_CF->wetg_k = R->wetg_k;   /* single aligned 8-byte store (Device-mem safe, not memset) */
    fprintf(stderr, "rev_fabric: coeffs loaded (N=%d diff_on=%d)\n", N, R->diff_on);
}

void rev_fabric_block(const fx *xin, int bs, int do_init, fx *yout_stereo) {
    if (!g_open) { for (int i = 0; i < 2 * bs; i++) yout_stereo[i] = 0; return; }
    for (int t = 0; t < bs; t++) g_XIN[t] = xin[t];   /* aligned scalar stores to Device mem */
    g_creg[R_INIT / 4] = (uint32_t)do_init;
    g_creg[R_N    / 4] = (uint32_t)bs;
    g_creg[AP_CTRL / 4] = 1;                            /* ap_start */
    long guard = 0;
    while (!(g_creg[AP_CTRL / 4] & 0x2)) {              /* spin on ap_done */
        if (++guard > 2000000000L) { fprintf(stderr, "rev_fabric: HANG (bs=%d)\n", bs); break; }
    }
    for (int t = 0; t < 2 * bs; t++) yout_stereo[t] = g_YOU[t];
    static int blkno = 0;
    if (blkno < 3) { fprintf(stderr, "rev_fabric: blk %d ok (bs=%d init=%d)\n", blkno, bs, do_init); blkno++; }
}
