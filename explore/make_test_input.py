#!/usr/bin/env python3
"""make_test_input.py — sparse, transient-rich DRY probe for the harp-fx reverb.

Replaces demos/dry.wav (too placid/sustained) with material that EXPOSES the tail:
  - isolated percussive bursts in silence      -> hear the pure impulse-response tail
  - a sparse FM-mallet phrase with big gaps     -> tail blooms in the holes; distinct
                                                   pitches reveal the bandpass self-retuning
  - one sustained chord stab, then long silence -> hear the freeze/bloom decay

mono / 16-bit / 48 kHz  (the reverb CLI reads 16-bit mono; we match dry.wav).
"""
import numpy as np

SR   = 48000
DUR  = 13.0
N    = int(SR * DUR)
t    = np.arange(N) / SR
buf  = np.zeros(N, np.float64)
rng  = np.random.default_rng(7)

def at(sec):
    return int(sec * SR)

def add(sig, start):
    i = at(start)
    j = min(N, i + len(sig))
    buf[i:j] += sig[: j - i]

def env_exp(n, tau, attack=0.002):
    """fast linear attack, exponential decay (tau seconds)."""
    e = np.exp(-np.arange(n) / (tau * SR))
    a = int(attack * SR)
    if a > 0:
        e[:a] *= np.linspace(0, 1, a)
    return e

# ---- 1. isolated percussive transients (pure IR probes) ---------------------
# Short decaying noise bursts with different spectral tilt so the frequency-
# dependent decay of the tail is audible. Big silent gaps after each.
def click(tau, tilt, amp):
    n = int(0.06 * SR)
    noise = rng.standard_normal(n)
    # one-pole tilt: tilt>0 -> brighten (diff), tilt<0 -> darken (integrate)
    if tilt > 0:
        noise = np.diff(noise, prepend=0.0)
    elif tilt < 0:
        noise = np.cumsum(noise) / 30.0
    sig = noise * env_exp(n, tau)
    sig /= (np.max(np.abs(sig)) + 1e-9)
    return sig * amp

add(click(0.010, +1, 0.80), 0.30)   # bright tick   -> exposes HF tail
add(click(0.020,  0, 0.75), 1.30)   # full-band clap
add(click(0.045, -1, 0.78), 2.30)   # low thump     -> exposes LF tail decay

# ---- 2. sparse FM-mallet phrase, with gaps ----------------------------------
# Marimba/bell-ish: sine fundamental + fast inharmonic attack partial + onset
# click. Pentatonic, deliberately sparse so the tail blooms between notes.
def mallet(freq, amp, body=0.45):
    n = int(1.2 * SR)
    tt = np.arange(n) / SR
    e_body = env_exp(n, body)
    # inharmonic FM strike: index decays fast -> metallic attack that settles to pitch
    idx = 3.0 * np.exp(-tt / 0.05)
    car = np.sin(2 * np.pi * freq * tt + idx * np.sin(2 * np.pi * freq * 3.5 * tt))
    fund = car * e_body
    strike = np.sin(2 * np.pi * freq * 4.0 * tt) * env_exp(n, 0.03) * 0.4
    onset = rng.standard_normal(n) * env_exp(n, 0.004) * 0.15
    sig = (fund + strike + onset)
    sig /= (np.max(np.abs(sig)) + 1e-9)
    return sig * amp

# pentatonic (C major pent) across two octaves
C5, D5, E5, G5, A5, C6 = 523.25, 587.33, 659.25, 783.99, 880.0, 1046.5
phrase = [
    (3.20, E5, 0.62),
    (4.05, G5, 0.58),
    (5.00, A5, 0.60),
    (6.10, C6, 0.55),
    (7.25, G5, 0.58),
    (8.40, C5, 0.62),   # low note, long gap after -> longest tail bloom
]
for start, f, a in phrase:
    add(mallet(f, a), start)

# ---- 3. one sustained chord stab, then long silence -------------------------
# Cmaj9, sharp attack, sustained body, then a release cut -> freeze/bloom tail.
def chord(freqs, amp, start, hold=1.6):
    n = int((hold + 0.4) * SR)
    tt = np.arange(n) / SR
    body = np.minimum(1.0, np.exp(-(tt - hold) / 0.18))   # flat, then release
    body[: int(0.004 * SR)] *= np.linspace(0, 1, int(0.004 * SR))
    sig = np.zeros(n)
    for k, f in enumerate(freqs):
        sig += np.sin(2 * np.pi * f * tt + 0.3 * k) * (0.9 ** k)
    sig *= body
    sig /= (np.max(np.abs(sig)) + 1e-9)
    add(sig * amp, start)

C4, E4, G4, B4, D5 = 261.63, 329.63, 392.0, 493.88, 587.33
chord([C4, E4, G4, B4, D5], 0.70, 9.30, hold=1.4)
# dry ends ~11.1s; ~1.9s trailing silence lets the dry-only audition show the tail.

# ---- normalize to a safe peak (leave headroom for the reverb's wet sum) ------
peak = np.max(np.abs(buf))
buf *= (0.80 / peak)

pcm = np.clip(np.round(buf * 32767.0), -32768, 32767).astype('<i2')

import wave
out = "/Users/jak/src/harp-fx/explore/test_input.wav"
w = wave.open(out, "wb")
w.setnchannels(1)
w.setsampwidth(2)
w.setframerate(SR)
w.writeframes(pcm.tobytes())
w.close()
print("wrote", out, "frames", len(pcm), "sec", len(pcm) / SR, "peak", float(np.max(np.abs(buf))))
