# harp-fx

Audio **effects** engines for [HARP](https://github.com/kschzt/harp) devices — the
processor side of the protocol's new audio-in (host→device) path. Sepends on the public `harp` library/spec.

The headline engine: a **huge real-time resonator-network reverb** for the Kria FPGA. A
dense field of thousands of coupled tuned resonators, excited by the *input* audio, ringing
out as a lush diffuse tail. It's a class of reverb whose scale + single-sample feedback is
real-time-impossible on a CPU and block-latency-wrong on a GPU — the one audio job where the
FPGA genuinely earns its place (see the project notes: synthesis didn't need the FPGA; a big
real-time resonator FX does).

## Layout
- `src/` — fixed-point DSP (Q-format, fabric-ready: pure integer, deterministic == FPGA ap_fixed)
- `demos/` — rendered before/after audio
- (later) the HLS kernel + Kria deployment, and the harp audio-in FX device engine

## Status
Prototype: de-risking the **sound** in software first (the lesson from the synth work —
nail the sound before the fabric port).
