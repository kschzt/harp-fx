/* reverb_engine.c — the engine seam for harp-fx-deviced: a drop-in REPLACEMENT for
 * harp's device/engine.c that turns the reference daemon into a HARP §8.8 `audio.fx`
 * EFFECT device. It provides EXACTLY the symbols the reused daemon (harp-deviced.c) +
 * protocol (session.c + state.c) + panel (panel.c) link against (device.h): g_params[],
 * the timestamped event queue, the engine_* seam, the §14 diagnostic counters, the
 * per-part param access, and the audio thread (free-running + host-paced loops).
 *
 * The DSP — the RME-grade resonator-network reverb (src/reverb_engine_core.h:
 * rev_init/rev_process, pure Q26 fixed-point) — sits behind the render boundary
 * render_output(). engine_is_fx() returns 1, so:
 *   - session.c advertises the `audio.fx` capability in the identity (§6.2/§8.8), and
 *   - engine.c's host_paced_loop DEMUXES the host's H→D input into a->fx_in (planar
 *     float columns) instead of discarding it.
 * render_output reads column 0 (mono in), runs the reverb, and writes the WET stereo
 * (§8.8: the device emits WET ONLY; the host holds the dry). The fx-domain math is the
 * SAME kernel as the standalone `reverb` tool, so the device reproduces it bit-for-bit.
 *
 * NB — vs jetson-synth/device/gpu_bridge.c: that bridge links HARP_ROOT/device/audio_loop.c
 * for the audio threads, because its harp checkout had the daemon plumbing split out of
 * engine.c. THIS harp tree (feat/audio-in-transport) keeps host_paced_loop / audio_thread /
 * audio_stop INSIDE engine.c (fused with the synth voice pool), so this engine-replacement
 * carries its own, effect-simplified copy of that plumbing (no note voices). The reused harp
 * daemon, protocol and §8.8 demux seam are otherwise 100% unmodified.
 */
#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#ifdef _WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  include <winsock2.h>
#else
#  include <sys/socket.h>
#endif

#include "device.h"
#include "fence_wait.h" /* §8.3.1 pure fence-wait predicate (host-unit-tested) */
#include "harp/link.h"  /* harp_write_all (free-running D→H emit) */
#include "harp/plat.h"  /* harp_now_ns: the §8.3.1 real-time fence bound */

#include "reverb_engine_core.h" /* rev_init / rev_process / rev_free + the Q26 kernel */
#ifdef HARPFX_FABRIC
#  include "rev_fabric.h" /* KR260 PL backend: rev_fabric_open/load_coeffs/block (one ap_ctrl_hs) */
#endif

/* audio_rtp_emit / audio_open_tcp_paced / audio_rtp_close live in the reused harp-deviced.c
 * (and are declared in device.h); we just call them from the copied audio thread. */

/* ---- the GOLDEN reverb config (== fabric/GOLDEN_wet.wav; reproduces it bit-exact) ----
 * v1 ships a FIXED reverb so the §8.8 render is deterministic and bit-exact to the oracle.
 * The g_params bank below DECLARES the controls (so the VST/Electra show them) with defaults
 * that mirror this config; wiring live param→DSP re-tuning is a follow-up (it must stay
 * deterministic per stream — see render_output). */
#define REV_N       3000
#define REV_T60     2.5
#define REV_WET     1.0
#define REV_HIDAMP  0.45
#define REV_DIFFUSE 0.5
#define REV_WIDTH   0.9

static reverb        *g_rev;            /* the live resonator bank (one D→H stream) */
static pthread_mutex_t g_rev_mu = PTHREAD_MUTEX_INITIALIZER;
#ifdef HARPFX_FABRIC
/* On the KR260 the per-sample recurrence runs in the PL. g_rev is still built (rev_init owns
 * the coefficient math — ONE source of truth) but only its FROZEN coeffs are shipped to the
 * fabric; the per-block DSP is rev_fabric_block, not rev_process. g_fabric_need_init makes the
 * first render block after audio.start carry do_init=1 (kernel zeroes BRAM state + caches coeffs). */
static _Atomic int g_fabric_need_init;
#endif
/* (re)build the reverb for a fresh stream — a clean tail every audio.start. The render
 * thread is the only caller once streaming, but audio.start/stop can race it, so guard. */
static void reverb_reset(void) {
    pthread_mutex_lock(&g_rev_mu);
    if (g_rev) rev_free(g_rev);
    g_rev = rev_init(REV_N, REV_T60, REV_WET, REV_HIDAMP, REV_DIFFUSE, REV_WIDTH);
#ifdef HARPFX_FABRIC
    /* open the fabric once (idempotent), push this stream's frozen coeffs to DDR, and arm the
     * one-shot init flag so the next render block resets the PL's resonator/allpass state. */
    if (rev_fabric_open() == 0) rev_fabric_load_coeffs(g_rev);
    else fprintf(stderr, "harp-fx-deviced: FABRIC OPEN FAILED — wet will be silence\n");
    atomic_store(&g_fabric_need_init, 1);
#endif
    pthread_mutex_unlock(&g_rev_mu);
}

/* ---------------- the param bank (NPARAMS, default 12) ----------------
 * Slots 1..4 are the reverb's user controls; 5..12 are reserved ("—"). The values are
 * normalized 0..1 like every HARP param. compute_param_map_hash() (state.c) asserts at boot
 * that engine_part_param_get(0, id) == g_params[id].def for every id, so g_pval below MUST
 * mirror these defaults exactly. */
dev_param g_params[NPARAMS] = {
    {1,  "Size",     0, NULL, 0.62f}, /* room/decay (→ t60); 0.62 ≈ the golden 2.5 s */
    {2,  "Wet",      0, NULL, 1.00f}, /* wet send level (§8.8 wet-only return)        */
    {3,  "Diffuse",  0, NULL, 0.50f}, /* input diffusion density                      */
    {4,  "Width",    0, NULL, 0.90f}, /* stereo decorrelation width                   */
    {5,  "—",        0, NULL, 0.0f},
    {6,  "—",        0, NULL, 0.0f},
    {7,  "—",        0, NULL, 0.0f},
    {8,  "—",        0, NULL, 0.0f},
    {9,  "—",        0, NULL, 0.0f},
    {10, "—",        0, NULL, 0.0f},
    {11, "—",        0, NULL, 0.0f},
    {12, "—",        0, NULL, 0.0f},
};

/* part-agnostic param values (the reverb is not multitimbral; all 16 "parts" share one
 * bank). MUST equal g_params[i].def in id order — see the boot assert above. */
static _Atomic float g_pval[NPARAMS] = {0.62f, 1.00f, 0.50f, 0.90f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

_Atomic float g_meter_peak[METER_NSLOTS];
_Atomic float g_meter_rms[METER_NSLOTS];

/* §14 diagnostic counters the protocol (session.c / panel.c) reads. The refdev defines
 * these in engine.c; this engine-replacement defines them here. Kept at zero on the FX
 * path (no note faults, no real-time deadline misses on the offline bounce). */
_Atomic int      g_touch_pending;
_Atomic uint64_t g_evq_drops, g_evt_late, g_ramp_late, g_fence_waits, g_fence_timeouts;
/* §14.2 FunctionFS transport-error counter: defined in device/ffs_link.c (NOT linked on the
 * Ethernet/TCP harp-fx device), but session.c's emit_counters references it under #ifdef
 * __linux__ (the Kria build). Define it here (stays 0 — no FFS) so the Linux link resolves;
 * harmless/unused on macOS. Mirrors jetson-synth/device/gpu_bridge.c. */
_Atomic uint64_t g_usb_errors;
_Atomic uint32_t g_evt_consumed;

/* ---- the live event queue (the symbols session.c pushes into) ----
 * An effect has no note voices, so notes are ignored; PARAM_SET still applies to the bank
 * (so a host's live knob echoes back), drained at the render position. */
static dev_event       g_evq[DEV_EVQ_CAP];
static size_t          g_evq_n;
static pthread_mutex_t g_evq_mu = PTHREAD_MUTEX_INITIALIZER;

void evq_push(dev_event ev) {
    pthread_mutex_lock(&g_evq_mu);
    if (g_evq_n < DEV_EVQ_CAP) g_evq[g_evq_n++] = ev; else CTR_INC(g_evq_drops);
    pthread_mutex_unlock(&g_evq_mu);
}
bool evq_push_batch(const dev_event *evs, size_t count) {
    pthread_mutex_lock(&g_evq_mu);
    bool ok = (g_evq_n + count <= DEV_EVQ_CAP);
    if (ok) { memcpy(&g_evq[g_evq_n], evs, count * sizeof *evs); g_evq_n += count; }
    pthread_mutex_unlock(&g_evq_mu);
    return ok;
}
bool evq_full(void) { return g_evq_n >= DEV_EVQ_CAP; }
void evq_reset_for_new_stream(void) {
    pthread_mutex_lock(&g_evq_mu); g_evq_n = 0; pthread_mutex_unlock(&g_evq_mu);
}

/* ---- the engine seam (device.h) ---- */
int param_index(uint32_t id) { for (int i = 0; i < NPARAMS; i++) if (g_params[i].id == id) return i; return -1; }
int engine_is_fx(void) { return 1; } /* §8.8: this device PROCESSES host audio (effect) */

/* part-agnostic: the reverb has one bank; `part` is ignored (snapshots store the same
 * values for all 16 parts, which load idempotently). */
float engine_part_param_get(int part, uint32_t id) { (void)part; int i = param_index(id); return i < 0 ? 0.0f : atomic_load(&g_pval[i]); }
void  engine_part_param_put(int part, uint32_t id, float v) { (void)part; int i = param_index(id); if (i >= 0) atomic_store(&g_pval[i], v); }

void engine_all_notes_off(void) { /* effect: no notes to silence */ }
void engine_note_off_if(uint32_t note) { (void)note; }
void engine_meters_reset(void) { for (int i = 0; i < METER_NSLOTS; i++) { atomic_store(&g_meter_peak[i], 0); atomic_store(&g_meter_rms[i], 0); } }

/* drain queued events at render position `now`. Notes are ignored (effect); PARAM_SET and a
 * batch txn-commit apply to the bank. RAMP/MOD/TRANSPORT: v1 ignores. */
static void evq_drain(uint64_t now) {
    (void)now;
    pthread_mutex_lock(&g_evq_mu);
    for (size_t r = 0; r < g_evq_n; r++) {
        dev_event *e = &g_evq[r];
        if (e->kind == DEV_EV_PARAM_SET) { int i = param_index(e->a); if (i >= 0) atomic_store(&g_pval[i], e->v); }
    }
    g_evq_n = 0;
    pthread_mutex_unlock(&g_evq_mu);
}

#define TAU_D 6.283185307179586

/* ---- M3 FABRIC backend (WIRED, -DHARPFX_FABRIC) ------------------------------------------
 * On the KR260 the per-sample resonator recurrence runs in the PL, not rev_process(). The
 * driver (device/rev_fabric.c, lifted from fabric/reverb_fabric.c's FABRIC path) is split as:
 *   - rev_fabric_open()        — mmap /dev/uio4 + /dev/udmabuf0, program the DDR pointers (once).
 *   - rev_fabric_load_coeffs() — push rev_init's frozen Q26 coeffs to DDR at reverb_reset() (once).
 *   - rev_fabric_block()       — per render block: fill XIN, ap_start, spin ap_done, read YOUT.
 * render_output() below selects it under #ifdef HARPFX_FABRIC. The kernel's Q26 recurrence is
 * identical to rev_process(), so the wet stays BIT-EXACT to the software path (hence the oracle
 * and the standalone reverb_fabric tool). The SOFTWARE backend (default build) is unchanged.
 * ------------------------------------------------------------------------------------------ */

/* ---- render_output: the render seam the audio loops call (host-paced bounce AND free-run).
 * §8.8: read the host's mono input column (a->fx_in col 0, a->fx_in_n samples), run the
 * resonator-network reverb per sample, and write the WET stereo interleaved into `out`
 * (the host mixes its own dry). Deterministic in (input, reverb state) — no wall clock —
 * which is what makes it bit-exact to the src/reverb.c oracle. Returns 2 (stereo). ---- */
static uint16_t render_output(audio_state *a, float *out, uint32_t n, float rate, uint64_t pos) {
    evq_drain(pos); /* apply any queued param changes at this position */
    if (a->tone_hz > 0.0) { /* §8.x SINAD measurement tone (pos-based, exact) */
        for (uint32_t i = 0; i < n; i++) {
            float s = 0.5f * (float)sin(TAU_D * a->tone_hz * (double)(pos + i) / (double)rate);
            out[2 * i] = out[2 * i + 1] = s;
        }
        return 2;
    }
    const float *in = a->fx_in;           /* §8.8 column 0 = the mono input (NULL on free-run) */
    uint32_t     nin = a->fx_in_n;         /* valid input samples this block (0 ⇒ silence/tail) */
    double peak = 0.0, sq = 0.0;
    pthread_mutex_lock(&g_rev_mu);
    if (!g_rev) { pthread_mutex_unlock(&g_rev_mu); reverb_reset(); pthread_mutex_lock(&g_rev_mu); }
#ifdef HARPFX_FABRIC
    /* ---- M3 FABRIC backend: the per-sample recurrence runs in the KR260 PL ----------------
     * Convert the mono input block to Q26 (silence past nin ⇒ the reverb tail rings), run ONE
     * kernel block (diffusion + resonators + stereo decorrelation + wet gain, all in fabric),
     * and read the wet stereo back. The kernel's Q26 math is identical to rev_process(), so the
     * wet is BIT-EXACT to the software path below (hence to the standalone reverb_fabric tool).
     * The mutex serializes against reverb_reset()'s rev_fabric_load_coeffs (same DDR). ---- */
    static fx xin_q[AUDIO_MAX_NSAMPLES];
    static fx wet_q[2 * AUDIO_MAX_NSAMPLES];
    for (uint32_t i = 0; i < n; i++) {
        double xd = (in && i < nin) ? (double)in[i] : 0.0;
        xin_q[i] = FX(xd);
    }
    int do_init = atomic_exchange(&g_fabric_need_init, 0); /* 1 only on the first block of a stream */
    rev_fabric_block(xin_q, (int)n, do_init, wet_q);
    for (uint32_t i = 0; i < n; i++) {
        float lf = (float)((double)wet_q[2 * i]     / ONE); /* Q26 → float32, same recipe as rev_process */
        float rf = (float)((double)wet_q[2 * i + 1] / ONE);
        out[2 * i] = lf;      /* §8.8 WET ONLY */
        out[2 * i + 1] = rf;
        double al = fabs((double)lf), ar = fabs((double)rf);
        if (al > peak) peak = al;
        if (ar > peak) peak = ar;
        sq += (double)lf * lf + (double)rf * rf;
    }
#else
    reverb *R = g_rev;
    for (uint32_t i = 0; i < n; i++) {
        double xd = (in && i < nin) ? (double)in[i] : 0.0; /* no input ⇒ feed silence, tail rings */
        double l, r;
        rev_process(R, FX(xd), &l, &r);
        float lf = (float)l, rf = (float)r;
        out[2 * i] = lf;      /* §8.8 WET ONLY */
        out[2 * i + 1] = rf;
        double al = fabs((double)lf), ar = fabs((double)rf);
        if (al > peak) peak = al;
        if (ar > peak) peak = ar;
        sq += (double)lf * lf + (double)rf * rf;
    }
#endif
    pthread_mutex_unlock(&g_rev_mu);
    /* §9.9 main-mix meter fold (read-only of `out`; never feeds back into the render). */
    atomic_store(&g_meter_peak[METER_MAIN_IX], (float)peak);
    atomic_store(&g_meter_rms[METER_MAIN_IX], n ? (float)sqrt(sq / (2.0 * n)) : 0.0f);
    return 2;
}

/* ============================================================================
 *  Audio thread plumbing — copied from harp's device/engine.c (host_paced_loop /
 *  audio_thread / audio_stop), with the synth voice-pool resets removed (an effect
 *  has no notes). The §8.8 H→D demux into a->fx_in is engine.c's verbatim logic.
 * ========================================================================== */

#ifdef _WIN32
static ssize_t hp_read(audio_state *a, void *buf, size_t n) {
    for (;;) {
        int r = recv((SOCKET)a->out_fd, (char *)buf, (int)(n > 0x7fffffff ? 0x7fffffff : n), 0);
        if (r >= 0) return r;
        if (WSAGetLastError() == WSAETIMEDOUT &&
            atomic_load_explicit(&a->running, memory_order_relaxed))
            continue;
        return -1;
    }
}
static bool hp_write_all(int fd, const void *buf, size_t n) {
    const char *p = (const char *)buf;
    size_t off = 0;
    while (off < n) {
        int w = send((SOCKET)fd, p + off, (int)(n - off > 0x7fffffff ? 0x7fffffff : n - off), 0);
        if (w <= 0) return false;
        off += (size_t)w;
    }
    return true;
}
#else
#define hp_read(a, buf, n)       read((a)->out_fd, (buf), (n))
#define hp_write_all(fd, buf, n) harp_write_all((fd), (buf), (n))
#endif

static void host_paced_loop(device *d) {
    audio_state *a = &d->audio;
    uint8_t frame[HARP_AUDIO_HDR_LEN + AUDIO_MAX_NSAMPLES * 34 * 4];
    float samples[AUDIO_MAX_NSAMPLES * 34];
    float lpb_col[AUDIO_MAX_NSAMPLES];
    /* §8.8 audio.fx: planar input columns, allocated once per stream (engine_is_fx()==1). */
    if (engine_is_fx() && a->n_in_slots > 0 && !a->fx_in)
        a->fx_in = calloc((size_t)a->n_in_slots * AUDIO_MAX_NSAMPLES, sizeof(float));
    uint8_t rbuf[16384];
    size_t rlen = 0, rpos = 0;
    uint64_t expect_ssi = 0;

    while (atomic_load_explicit(&a->running, memory_order_relaxed)) {
        uint8_t hdr[HARP_AUDIO_HDR_LEN];
        size_t need = sizeof hdr, got = 0;
        while (got < need) {
            if (rpos < rlen) {
                size_t take = rlen - rpos;
                if (take > need - got) take = need - got;
                memcpy(hdr + got, rbuf + rpos, take);
                rpos += take; got += take;
                continue;
            }
            ssize_t r = hp_read(a, rbuf, sizeof rbuf);
            if (r <= 0) {
                fprintf(stderr, "harp-fx-deviced: pacing read ended: %s\n", r == 0 ? "EOF" : strerror(errno));
                return;
            }
            rlen = (size_t)r; rpos = 0;
        }
        harp_audio_hdr h;
        if (!harp_audio_hdr_decode(hdr, &h) || !(h.dirflags & HARP_AUDIO_DIR_H2D)) {
            CTR_INC(d->frame_errors);
            fprintf(stderr, "harp-fx-deviced: malformed pacing frame (%02x %02x ...)\n", hdr[0], hdr[1]);
            return;
        }
        if (h.dirflags & HARP_AUDIO_FENCE) {
            uint8_t fb[HARP_AUDIO_FENCE_LEN];
            size_t fgot = 0;
            while (fgot < sizeof fb) {
                if (rpos < rlen) {
                    size_t take = rlen - rpos;
                    if (take > sizeof fb - fgot) take = sizeof fb - fgot;
                    memcpy(fb + fgot, rbuf + rpos, take);
                    rpos += take; fgot += take;
                    continue;
                }
                ssize_t r = hp_read(a, rbuf, sizeof rbuf);
                if (r <= 0) return;
                rlen = (size_t)r; rpos = 0;
            }
            uint32_t want = (uint32_t)fb[0] | ((uint32_t)fb[1] << 8) |
                            ((uint32_t)fb[2] << 16) | ((uint32_t)fb[3] << 24);
            if ((int32_t)(want - atomic_load_explicit(&g_evt_consumed, memory_order_acquire)) > 0) {
                CTR_INC(g_fence_waits);
                struct timespec fts = {0, 50000};
                bool offline = d->audio.offline;
                uint64_t deadline = harp_now_ns() + 5000000ull;
                while (harp_fence_keep_waiting(
                           (int32_t)(want - atomic_load_explicit(&g_evt_consumed, memory_order_acquire)),
                           atomic_load_explicit(&a->running, memory_order_relaxed),
                           offline, harp_now_ns(), deadline))
                    nanosleep(&fts, NULL);
                if (harp_fence_count_timeout(
                        (int32_t)(want - atomic_load_explicit(&g_evt_consumed, memory_order_acquire)),
                        atomic_load_explicit(&a->running, memory_order_relaxed), offline))
                    CTR_INC(g_fence_timeouts);
            }
        }
        bool lpb = atomic_load_explicit(&a->loopback_on, memory_order_acquire);
        int in_col = -1;
        if (lpb)
            for (uint8_t c = 0; c < a->n_in_slots; c++)
                if (a->in_slots[c] == a->loopback_in_slot) { in_col = c; break; }
        bool keep = lpb && in_col >= 0 && h.slots > 0 && (size_t)in_col < h.slots &&
                    h.nsamples <= AUDIO_MAX_NSAMPLES;
        /* §8.8: demux ALL host input columns into a->fx_in (planar) for the effect. */
        bool fxmode = engine_is_fx() && a->fx_in && a->n_in_slots > 0 && h.slots > 0 &&
                      h.nsamples <= AUDIO_MAX_NSAMPLES;
        size_t fxcols = fxmode ? (a->n_in_slots <= h.slots ? a->n_in_slots : h.slots) : 0;
        size_t stride = (size_t)h.slots * 4;
        size_t base = (size_t)in_col * 4;
        size_t pcur = 0;
        size_t skip = harp_audio_payload_len(&h);
        while (skip) {
            if (rpos < rlen) {
                size_t take = rlen - rpos;
                if (take > skip) take = skip;
                if (keep || fxmode) {
                    for (size_t i = 0; i < take; i++) {
                        size_t off = pcur + i, inrow = off % stride, smp = off / stride;
                        if (keep && inrow >= base && inrow < base + 4)
                            ((uint8_t *)lpb_col)[smp * 4 + (inrow - base)] = rbuf[rpos + i];
                        if (fxmode) {
                            size_t col = inrow >> 2;
                            if (col < fxcols)
                                ((uint8_t *)(a->fx_in + col * AUDIO_MAX_NSAMPLES))
                                    [smp * 4 + (inrow & 3)] = rbuf[rpos + i];
                        }
                    }
                }
                pcur += take; rpos += take; skip -= take;
                continue;
            }
            ssize_t r = hp_read(a, rbuf, sizeof rbuf);
            if (r <= 0) return;
            rlen = (size_t)r; rpos = 0;
        }
        a->fx_in_n = (uint16_t)(fxmode ? h.nsamples : 0);
        uint32_t n = h.nsamples;
        if (n > AUDIO_MAX_NSAMPLES) { CTR_INC(d->frame_errors); return; }
        if (h.ts < expect_ssi) { CTR_INC(d->audio_late_frames); continue; }
        uint16_t slots = render_output(a, samples, n, (float)a->rate, h.ts);
        if (keep) {
            int out_col = -1;
            for (uint16_t c = 0; c < slots; c++)
                if (a->out_slots[c] == a->loopback_out_slot) { out_col = c; break; }
            if (out_col >= 0)
                for (uint32_t s = 0; s < n; s++)
                    samples[(size_t)s * slots + out_col] = lpb_col[s];
        }
        harp_audio_hdr out = {HARP_AUDIO_FVER, 0, slots, h.epoch, h.ts, (uint16_t)n, HARP_AUDIO_FMT_F32};
        harp_audio_hdr_encode(&out, frame);
        size_t payload = (size_t)n * slots * 4;
        memcpy(frame + HARP_AUDIO_HDR_LEN, samples, payload);
        if (!hp_write_all(a->fd, frame, HARP_AUDIO_HDR_LEN + payload)) return;
        expect_ssi = h.ts + n;
    }
}

void *audio_thread(void *arg) {
    device *d = arg;
    audio_state *a = &d->audio;
    /* §8.3-over-§8.7: host-paced deterministic render over TCP — connect back on THIS thread. */
    if (a->host_paced_port > 0) {
        int s = audio_open_tcp_paced(d->rtp_peer_ip, a->host_paced_port);
        if (s < 0) {
            fprintf(stderr, "harp-fx-deviced: host-paced TCP connect-back to :%d failed\n", a->host_paced_port);
            return NULL;
        }
        a->host_paced_sock = s;
        a->fd = s;
        a->out_fd = s;
#ifdef _WIN32
        { DWORD tmo = 200; setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tmo, sizeof tmo); }
#endif
    }
    reverb_reset(); /* a clean resonator bank for this stream (effect analogue of voices_cold) */
    fprintf(stderr, "harp-fx-deviced: audio thread up: mode=%u fd=%d out_fd=%d\n", a->mode, a->fd, a->out_fd);
    if (a->mode == 1) {
        host_paced_loop(d);
        fprintf(stderr, "harp-fx-deviced: host-paced loop exited\n");
        return NULL;
    }
    /* free-running §8.7 path (no host input — the reverb renders its tail/silence). */
    uint8_t frame[HARP_AUDIO_HDR_LEN + AUDIO_MAX_NSAMPLES * 34 * 4];
    float samples[AUDIO_MAX_NSAMPLES * 34];
    uint64_t msc = 0;
    uint64_t period_ns = (uint64_t)a->nsamples * 1000000000ull / a->rate;
    bool discont = false;
    bool primed = (a->rtp_prebuffer == 0);
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    while (atomic_load_explicit(&a->running, memory_order_relaxed)) {
        a->fx_in_n = 0; /* free-running: no host input this block */
        uint16_t slots = render_output(a, samples, a->nsamples, (float)a->rate, msc);
        harp_audio_hdr h = {HARP_AUDIO_FVER, discont ? HARP_AUDIO_DISCONT : 0, slots,
                            a->epoch, msc, (uint16_t)a->nsamples, HARP_AUDIO_FMT_F32};
        discont = false;
        harp_audio_hdr_encode(&h, frame);
        size_t payload = (size_t)a->nsamples * slots * 4;
        memcpy(frame + HARP_AUDIO_HDR_LEN, samples, payload);
        if (a->fd >= 0 && !harp_write_all(a->fd, frame, HARP_AUDIO_HDR_LEN + payload))
            break;
        audio_rtp_emit(a, samples, payload, msc);
        msc += a->nsamples;
        a->msc_final = msc;
        if (!primed) {
            if (msc < a->rtp_prebuffer) continue;
            primed = true;
            clock_gettime(CLOCK_MONOTONIC, &next);
        }
        int trim_ppb = atomic_load_explicit(&a->rate_trim_ppb, memory_order_relaxed);
        period_ns = (uint64_t)((double)a->nsamples * 1000000000.0 /
                               ((double)a->rate * (1.0 + (double)trim_ppb * 1e-9)));
        next.tv_nsec += (long)(period_ns % 1000000000ull);
        next.tv_sec += (time_t)(period_ns / 1000000000ull);
        if (next.tv_nsec >= 1000000000L) { next.tv_nsec -= 1000000000L; next.tv_sec++; }
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        int64_t behind_ns = (int64_t)(now.tv_sec - next.tv_sec) * 1000000000ll + (now.tv_nsec - next.tv_nsec);
        if (behind_ns > 50 * 1000000ll) {
            next = now;
            CTR_INC(d->audio_overruns);
            a->reanchors++;
            discont = true;
        } else {
#ifdef __linux__
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
#else
            struct timespec rel = {0, (long)(-behind_ns)};
            if (behind_ns < 0) nanosleep(&rel, NULL);
#endif
        }
    }
    return NULL;
}

void audio_stop(device *d) {
    meter_pump_stop(d);
    if (!atomic_load_explicit(&d->audio.thread_live, memory_order_relaxed)) return;
    d->audio.running = false;
    if (d->audio.host_paced_sock >= 0)
#ifdef _WIN32
        shutdown(d->audio.host_paced_sock, SD_BOTH);
#else
        shutdown(d->audio.host_paced_sock, SHUT_RDWR);
#endif
    pthread_cancel(d->audio.thread);
    pthread_join(d->audio.thread, NULL);
    atomic_store_explicit(&d->audio.thread_live, false, memory_order_relaxed);
    audio_rtp_close(&d->audio);
    if (d->audio.host_paced_sock >= 0) {
#ifdef _WIN32
        closesocket(d->audio.host_paced_sock);
#else
        close(d->audio.host_paced_sock);
#endif
        d->audio.host_paced_sock = -1;
    }
    d->audio.host_paced_port = 0;
    fprintf(stderr, "harp-fx-deviced: audio stream stopped (%llu reanchors)\n",
            (unsigned long long)d->audio.reanchors);
}

/* ---- USB-FFS gadget transport: unused on the Ethernet/TCP harp-fx device (stubbed).
 * On Linux (Kria) the harp-deviced.c --ffs path references these; our CMake excludes the
 * real ffs.c, so these definitions satisfy the link. On macOS they are never referenced. */
int harp_ffs_serve(const char *ffs_dir, const char *gadget_path, void (*session)(void *ud, harp_io *io), void *ud) {
    (void)ffs_dir; (void)gadget_path; (void)session; (void)ud; return -1;
}
int harp_ffs_audio_in_fd(void) { return -1; }
int harp_ffs_audio_out_fd(void) { return -1; }
