#!/usr/bin/env python3
# timeprofile.py — per-window RMS(dB) + spectral centroid over time, to SEE the
# directional energy flow (does cent drift UP the tail?) and confirm the tail DECAYS.
import sys, wave, numpy as np
w = wave.open(sys.argv[1],'rb'); sr=w.getframerate(); n=w.getnframes(); ch=w.getnchannels()
d = np.frombuffer(w.readframes(n),dtype=np.int16).astype(np.float64)/32768.0
if ch>1: d=d.reshape(-1,ch).mean(axis=1)
win=int(0.5*sr); freqs=np.fft.rfftfreq(win,1.0/sr)
print(f"{sys.argv[1].split('/')[-1]:20s}  t(s)  RMS(dB)  centroid(Hz)")
prev=None
for i in range(0,len(d)-win,win):
    seg=d[i:i+win]; rms=np.sqrt((seg**2).mean())
    db=20*np.log10(rms+1e-12)
    S=np.abs(np.fft.rfft(seg*np.hanning(win)))+1e-12
    cent=(freqs*S).sum()/S.sum()
    arrow=''
    if prev is not None: arrow = '↑' if cent>prev+30 else ('↓' if cent<prev-30 else '·')
    print(f"   {' '*16} {i/sr:5.1f}  {db:7.1f}  {cent:8.0f}  {arrow}")
    prev=cent
