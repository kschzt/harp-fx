# explore/ — coupled-mesh "impossible space" reverbs (the only-Kria DSP search)

Prototypes toward a reverb that genuinely *needs* the Kria: the resonator bank from
`../src/reverb.c`, but with the resonators **coupled into a mesh** — a same-sample
cross-feedback loop a GPU can't close at 48 kHz, tuned past anything physical.

All prototypes are standalone, pure **Q26 integer fixed-point** (deterministic == the FPGA
datapath), read a dry WAV, write a wet WAV. WAVs are gitignored; regenerate by building +
running each `.c` on `../demos/dry.wav` or `explore/test_input.wav`
(`python3 make_test_input.py` → a sparse, transient-rich probe that actually reveals a tail).

## The coupling topologies tried (round 1)
| source | character | verdict |
|---|---|---|
| `ring_neighbor.c` | lush dark hall whose tail **slides in pitch** ("a room retuning itself") | **gorgeous**, but its stable form is a 1-sample-delay snapshot = GPU-doable; tail a bit short |
| `meanfield.c` | the whole space **freezes into an infinite bloom** | viscerally impossible, but rank-1 closed-form (GPU-*hostile*, not -*impossible*) + dark/static |
| `nonreciprocal.c` | directional "**spectral comet**" — energy chirps up & exits, tail brightens as it fades | great *layer*; feed-forward triangular = GPU-doable; short tail |
| `spectral_neighbor.c` | most only-Kria-*shaped* (true non-rank-1 same-sample mesh) | **screeches** the instant the coupling is audible — no stable audible regime |

Round-1 tension: stable+musical couplings were GPU-doable; the genuinely GPU-impossible mesh
was unstable. The crack ↓ resolves it.

## The crack — `implicit_mesh.c` (the deliverable)
Make the coupling **implicit**: each sample *solve* `(I − kc·L)·y = rhs` on the frequency ring
(L = cyclic discrete Laplacian) instead of stepping it, via **fixed-point Gauss–Seidel sweeps**
(diagonal pre-inverted to a Q26 reciprocal → no per-sample division; cyclic wrap via the ring map).

- **Stable (the screech is solved):** `(I − kc·L)` is SPD/diagonally-dominant → contractive →
  bounded. Clean to **kc=8**, where the explicit version saturates/clips 36 % at kc≥0.20.
- **Airtight only-Kria (confirmed 3 ways):** kc=0 → no retune (coupling is load-bearing);
  re-ordering the ring changes the sound (genuinely topological, *not* rank-1 mean-field);
  it's a sequential GS sweep — `y[i]` reads the just-written `y[i-1]` — ~`N·nsweep` (384·14 ≈
  **5.4k dependent Q26 MACs/sample**) wavefronting around the ring inside one audio sample.
  An FPGA pipelines the carried recurrence; a GPU can't close it at 48 kHz.
- **Sound:** on transient material it's a **bright, lush, dense bloom that audibly self-retunes**
  (rough 0.63 = the ring_neighbor lushness; wob ~78 c pitch-slide), ~3× longer than the explicit
  original. **Winner config:** `cmode=0` (state-coupled: solve `(I−kc·L)u=s1`, drive with `a1·u`),
  `N=384 t60=18 wet=0.50 hidamp=0.35 diffuse=0.7 width=0.9 kc=0.20 nsweep=14`.
  Alternatives: `kc=0.20 hidamp=0.25` (vaster/darker), `kc=0.10 hidamp=0.15 nsweep=24` (gentler/brighter).

### Honest caveats (not yet solved)
1. **The long sustain tail goes dark/thin/quiet** — strong Laplacian *diffusion is dissipative*,
   so "more coupling = bigger space" is backwards for the tail. The lush brightness lives in the
   **bloom** on each transient, not the far tail. It's a bright self-retuning *bloom-hall*, not a
   bright-forever cathedral. A brighter tail needs a **structural** add (HF re-injection / shimmer),
   not a knob.
2. **Level is knife-edge in kc** (the cmode-0 detune): kc=0.10 and 0.20 are robust loud points;
   between them detunes ~6× quieter. Tame before exposing a user kc knob.

## OPEN — the load-bearing question before investing further
**Is the implicit GS sweep real-time on the XCK26?** Budget ≈ 142.86 MHz / 48 kHz ≈ **2975 cycles/sample**.
The solver is a *sequential carried recurrence* (unlike the independent bank that hit 2.9× real-time
at unroll ×4), and 384·14 ≈ 5376 dependent steps is already ~1.8× over budget *at a hypothetical II=1* —
and the carry may force II>1. Likely rescuable (red-black ring ordering → break the carry → II=1 +
unroll; fewer sweeps/resonators), but **unconfirmed**. Next step: an **HLS C-synthesis feasibility check**
(write the GS kernel, read achievable II + throughput + resources) *before* perfecting the sound or
rebuilding the lost Vivado scaffolding. The fabric is the deliverable; this C is golden/scaffolding.
