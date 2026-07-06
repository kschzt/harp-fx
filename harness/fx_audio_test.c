/* fx_audio_test — drive dry audio through harp-fx-deviced as a §8.8 audio.fx EFFECT and
 * prove the WET return is bit-exact to the src/reverb.c reverb oracle.
 *
 *   fx_audio_test HOST:CTL_PORT DRY.wav DEVWET.wav [HP_PORT]
 *
 * The §8.8 path, end to end:
 *   1. dial the framed control link + core.hello; assert the device advertises `audio.fx`.
 *   2. listen on a host-paced TCP audio port (HP_PORT).
 *   3. audio.start { rate, nsamples, in-slots=[0], out-slots=[0,1], clock=host-paced,
 *      host-paced-port=HP_PORT } — the device connects back on that port (offline bounce).
 *   4. accept the connect-back; pace DRY through it LOCKSTEP: send one H→D pacing frame
 *      (mono dry on in-slot 0), read one D→H frame (stereo WET on out-slots 0,1). Pad the
 *      t60 tail with zero-input frames so the reverb rings out.
 *   5. audio.stop.
 *   6. ORACLE: run the SAME float input through reverb_engine_core.h (rev_init/rev_process)
 *      with the golden params and cast each wet sample to float32 (the device's wire format).
 *   7. compare the device WET stream to the oracle WET stream float-for-float (bit-exact),
 *      write the device WET as a wet-only int16 stereo WAV, and (bonus) md5-diff it against
 *      fabric/GOLDEN_wet.wav (the committed golden, double-quantized).
 */
#include "client.h"
#include "sock_io.h"
#include "harp/audio.h"
#include "harp/cbor.h"
#include "harp/envelope.h"
#include "harp/link.h"

#include <arpa/inet.h>
#include <math.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "reverb_engine_core.h" /* the reverb oracle (rev_init / rev_process) */

#ifndef AUDIO_MAX_NSAMPLES
#define AUDIO_MAX_NSAMPLES 1024 /* mirrors harp device.h (per-frame sample bound) */
#endif

/* ---- golden reverb config (must match device/reverb_engine.c REV_*) ---- */
#define G_N 3000
#define G_T60 2.5
#define G_WET 1.0
#define G_HIDAMP 0.45
#define G_DIFFUSE 0.5
#define G_WIDTH 0.9

/* ---- WAV io (16-bit mono read; wet-only stereo write) — same recipe as src/reverb.c ---- */
static float *wav_read(const char *p, long *nout) {
    FILE *f = fopen(p, "rb");
    if (!f) { perror(p); return NULL; }
    char id[4]; unsigned u; int ch = 1, bits = 16; float *buf = NULL;
    if (fread(id, 1, 4, f) != 4) { fclose(f); return NULL; }
    if (fread(&u, 4, 1, f) != 1) { fclose(f); return NULL; }
    if (fread(id, 1, 4, f) != 4) { fclose(f); return NULL; }
    while (fread(id, 1, 4, f) == 4) {
        if (fread(&u, 4, 1, f) != 1) break;
        if (!memcmp(id, "fmt ", 4)) {
            unsigned char fmt[40]; long want = u < 40 ? u : 40;
            if (fread(fmt, 1, want, f) != (size_t)want) break;
            ch = fmt[2] | (fmt[3] << 8); bits = fmt[14] | (fmt[15] << 8);
            if ((long)u > want) fseek(f, u - want, SEEK_CUR);
        } else if (!memcmp(id, "data", 4)) {
            long frames = (long)u / (bits / 8) / ch; buf = malloc(sizeof(float) * frames);
            for (long i = 0; i < frames; i++) {
                double a = 0;
                for (int c = 0; c < ch; c++) { short v = 0; if (fread(&v, 2, 1, f) != 1) {} a += v / 32768.0; }
                buf[i] = (float)(a / ch);
            }
            *nout = frames; fclose(f); return buf;
        } else fseek(f, u, SEEK_CUR);
    }
    fclose(f); return NULL;
}
static void wav_write_stereo(const char *p, const short *lr, long n) {
    FILE *f = fopen(p, "wb"); long nd = n * 2 * 2; unsigned u; unsigned short s;
    fwrite("RIFF", 1, 4, f); u = 36 + nd; fwrite(&u, 4, 1, f); fwrite("WAVE", 1, 4, f); fwrite("fmt ", 1, 4, f);
    u = 16; fwrite(&u, 4, 1, f); s = 1; fwrite(&s, 2, 1, f); s = 2; fwrite(&s, 2, 1, f); u = 48000; fwrite(&u, 4, 1, f);
    u = 48000 * 4; fwrite(&u, 4, 1, f); s = 4; fwrite(&s, 2, 1, f); s = 16; fwrite(&s, 2, 1, f);
    fwrite("data", 1, 4, f); u = nd; fwrite(&u, 4, 1, f);
    fwrite(lr, 2, n * 2, f); fclose(f);
}

/* softceil — never hard-clips (byte-identical to src/reverb.c / the golden). The device-wet
 * int16 dump below must go through it, else a >full-scale wet sample wraps mod 2^16 into an
 * opposite-polarity click. Float-exact verification (devwet vs orawet) is unaffected. */
static inline double softceil(double v) {
    if (v >  0.9) return  0.9 + 0.1 * tanh((v - 0.9) / 0.1);
    if (v < -0.9) return -0.9 - 0.1 * tanh((v + 0.9) / 0.1); return v;
}

static int read_all(int fd, void *buf, size_t n) {
    uint8_t *p = buf; size_t off = 0;
    while (off < n) {
        ssize_t r = recv(fd, p + off, n - off, 0);
        if (r <= 0) return -1;
        off += (size_t)r;
    }
    return 0;
}
static int write_all(int fd, const void *buf, size_t n) {
    const uint8_t *p = buf; size_t off = 0;
    while (off < n) {
        ssize_t w = send(fd, p + off, n - off, 0);
        if (w <= 0) return -1;
        off += (size_t)w;
    }
    return 0;
}

int main(int argc, char **argv) {
    if (argc < 4) {
        fprintf(stderr, "usage: %s HOST:CTL_PORT DRY.wav DEVWET.wav [HP_PORT]\n", argv[0]);
        return 2;
    }
    const char *hostport = argv[1];
    const char *drypath = argv[2];
    const char *devwetpath = argv[3];
    int hp_port = argc > 4 ? atoi(argv[4]) : 47860;
    uint32_t rate = 48000, nsamples = 256;

    /* 1. control link + hello */
    harp_sockhandle s = harp_sock_dial(hostport);
    if (s == HARP_SOCK_INVALID) { fprintf(stderr, "fx: cannot dial %s\n", hostport); return 1; }
    harp_sock_io tio; harp_sock_io_init(&tio, s);
    harp_link link; harp_link_init(&link);
    harp_client client; harp_client_init(&client, &tio.io, &link, NULL, NULL, NULL);
    harp_client_identity id;
    if (harp_client_hello(&client, "fx-audio-test 0.1", &id) != 0) {
        fprintf(stderr, "fx: hello failed (%s)\n", client.err_code); return 1;
    }
    int has_fx = harp_client_has_cap(&id, "audio.fx");
    fprintf(stderr, "fx: device %s/%s engine=%s caps: audio.fx=%s\n",
            id.vendor, id.product, id.engine_id, has_fx ? "YES" : "no");
    if (!has_fx) { fprintf(stderr, "fx: FAIL device does not advertise audio.fx\n"); return 1; }

    /* 2. listen for the host-paced connect-back */
    int ls = socket(AF_INET, SOCK_STREAM, 0);
    int one = 1; setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in la = {0};
    la.sin_family = AF_INET; la.sin_addr.s_addr = htonl(INADDR_LOOPBACK); la.sin_port = htons((uint16_t)hp_port);
    if (bind(ls, (struct sockaddr *)&la, sizeof la) != 0) { perror("bind hp_port"); return 1; }
    if (listen(ls, 1) != 0) { perror("listen"); return 1; }

    /* 3. audio.start (host-paced TCP, in-slot 0, out-slots 0/1) */
    harp_cbuf req, rsp; harp_cbuf_init(&req); harp_cbuf_init(&rsp);
    harp_client_req_head(&client, &req, "audio.start", true);
    harp_cbor_map(&req, 6);
    harp_cbor_uint(&req, 0); harp_cbor_uint(&req, rate);
    harp_cbor_uint(&req, 1); harp_cbor_uint(&req, nsamples);
    harp_cbor_uint(&req, 3); harp_cbor_array(&req, 1); harp_cbor_uint(&req, 0);          /* in-slots = [0] */
    harp_cbor_uint(&req, 4); harp_cbor_array(&req, 2); harp_cbor_uint(&req, 0); harp_cbor_uint(&req, 1); /* out=[0,1] */
    harp_cbor_uint(&req, 5); harp_cbor_uint(&req, 1);                                    /* host-paced */
    harp_cbor_uint(&req, 7); harp_cbor_uint(&req, (uint64_t)hp_port);                    /* host-paced TCP port */
    harp_env e;
    int rc = harp_client_request(&client, &req, &rsp, &e);
    harp_cbuf_free(&req); harp_cbuf_free(&rsp);
    if (rc != 0) { fprintf(stderr, "fx: audio.start failed (rc=%d %s)\n", rc, client.err_code); return 1; }

    /* 4. accept the device's connect-back */
    int df = accept(ls, NULL, NULL);
    if (df < 0) { perror("accept"); return 1; }
    setsockopt(df, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    struct timeval tv = {15, 0}; setsockopt(df, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    fprintf(stderr, "fx: host-paced data link up; pacing dry → wet\n");

    /* the dry input + tail */
    long n = 0; float *dry = wav_read(drypath, &n);
    if (!dry) { fprintf(stderr, "fx: cannot read %s\n", drypath); return 1; }
    long tail = (long)(G_T60 * 1.5 * RATE), total = n + tail;
    float *devwet = calloc((size_t)total * 2, sizeof(float));

    uint8_t fbuf[HARP_AUDIO_HDR_LEN + AUDIO_MAX_NSAMPLES * 4];
    uint8_t rhdr[HARP_AUDIO_HDR_LEN];
    static float rpay[AUDIO_MAX_NSAMPLES * 2];
    long ssi = 0;
    while (ssi < total) {
        uint32_t blk = (uint32_t)(total - ssi < (long)nsamples ? total - ssi : (long)nsamples);
        /* H→D pacing frame: mono dry on in-slot 0 */
        harp_audio_hdr h = {HARP_AUDIO_FVER, HARP_AUDIO_DIR_H2D, 1, 0, (uint64_t)ssi, (uint16_t)blk, HARP_AUDIO_FMT_F32};
        harp_audio_hdr_encode(&h, fbuf);
        float *pl = (float *)(fbuf + HARP_AUDIO_HDR_LEN);
        for (uint32_t i = 0; i < blk; i++) pl[i] = (ssi + (long)i < n) ? dry[ssi + i] : 0.0f;
        if (write_all(df, fbuf, HARP_AUDIO_HDR_LEN + (size_t)blk * 4) != 0) {
            fprintf(stderr, "fx: send H→D failed at ssi=%ld\n", ssi); return 1;
        }
        /* D→H WET frame: stereo on out-slots 0,1 */
        if (read_all(df, rhdr, HARP_AUDIO_HDR_LEN) != 0) { fprintf(stderr, "fx: no D→H header at ssi=%ld\n", ssi); return 1; }
        harp_audio_hdr rh;
        if (!harp_audio_hdr_decode(rhdr, &rh) || (rh.dirflags & HARP_AUDIO_DIR_H2D) || rh.slots != 2 ||
            rh.nsamples != blk || rh.ts != (uint64_t)ssi) {
            fprintf(stderr, "fx: bad D→H frame at ssi=%ld (slots=%u n=%u ts=%llu)\n",
                    ssi, rh.slots, rh.nsamples, (unsigned long long)rh.ts);
            return 1;
        }
        if (read_all(df, rpay, (size_t)blk * 2 * 4) != 0) { fprintf(stderr, "fx: short D→H payload\n"); return 1; }
        memcpy(&devwet[ssi * 2], rpay, (size_t)blk * 2 * 4);
        ssi += blk;
    }
    fprintf(stderr, "fx: paced %ld samples (%ld dry + %ld tail)\n", total, n, tail);

    /* 5. audio.stop */
    harp_cbuf sreq, srsp; harp_cbuf_init(&sreq); harp_cbuf_init(&srsp);
    harp_client_req_head(&client, &sreq, "audio.stop", false);
    harp_env se; harp_client_request(&client, &sreq, &srsp, &se);
    harp_cbuf_free(&sreq); harp_cbuf_free(&srsp);
    close(df); harp_sock_close(s);

    /* 6. ORACLE: the same float input → reverb → float32 wet (mirror the device's wire) */
    float *orawet = calloc((size_t)total * 2, sizeof(float));
    reverb *R = rev_init(G_N, G_T60, G_WET, G_HIDAMP, G_DIFFUSE, G_WIDTH);
    for (long t = 0; t < total; t++) {
        double xd = t < n ? (double)dry[t] : 0.0;
        double l, r; rev_process(R, FX(xd), &l, &r);
        orawet[t * 2] = (float)l; orawet[t * 2 + 1] = (float)r;
    }
    rev_free(R);

    /* 7a. float-exact device vs oracle */
    long ndiff = 0, firstdiff = -1; double maxabs = 0;
    for (long i = 0; i < total * 2; i++) {
        if (devwet[i] != orawet[i]) {
            if (firstdiff < 0) firstdiff = i;
            ndiff++;
            double d = fabs((double)devwet[i] - (double)orawet[i]);
            if (d > maxabs) maxabs = d;
        }
    }
    /* 7b. write device wet as wet-only int16 stereo WAV */
    short *o16 = malloc(sizeof(short) * total * 2);
    for (long i = 0; i < total * 2; i++) o16[i] = (short)lrint(softceil((double)devwet[i]) * 32767.0);
    wav_write_stereo(devwetpath, o16, total);

    printf("\n=== §8.8 audio.fx reverb bridge verification ===\n");
    printf("samples         : %ld stereo (%ld dry + %ld t60 tail) @ %u Hz, %u-frame blocks\n",
           total, n, tail, rate, nsamples);
    printf("device vs oracle: %s (%ld/%ld float samples differ%s)\n",
           ndiff == 0 ? "BIT-EXACT" : "MISMATCH", ndiff, total * 2,
           ndiff ? "" : " — the device reproduces the reverb exactly");
    if (ndiff) printf("  first diff idx=%ld  max|Δ|=%.3e\n", firstdiff, maxabs);
    printf("wrote           : %s (wet-only int16 stereo)\n", devwetpath);

    free(dry); free(devwet); free(orawet); free(o16);
    return ndiff == 0 ? 0 : 1;
}
