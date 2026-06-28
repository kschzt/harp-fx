import sys, wave, numpy as np
w=wave.open(sys.argv[1],'rb'); sr=w.getframerate(); n=w.getnframes(); ch=w.getnchannels()
d=np.frombuffer(w.readframes(n),dtype=np.int16).astype(np.float64)/32768.0
if ch>1: d=d.reshape(-1,ch).mean(axis=1)
W=int(0.025*sr); fr=np.fft.rfftfreq(W,1.0/sr)
T=float(sys.argv[2]) if len(sys.argv)>2 else 0.6
print(f"{sys.argv[1].split('/')[-1]:16s} t(ms) RMS(dB) cent(Hz)")
prev=None
for i in range(0,min(int(T*sr),len(d)-W),W):
    seg=d[i:i+W]; rms=np.sqrt((seg**2).mean()); db=20*np.log10(rms+1e-12)
    S=np.abs(np.fft.rfft(seg*np.hanning(W)))+1e-12; c=(fr*S).sum()/S.sum()
    a='' if prev is None else ('UP' if c>prev+40 else ('dn' if c<prev-40 else '..'))
    print(f"{' '*16} {i/sr*1000:5.0f} {db:7.1f} {c:7.0f} {a}"); prev=c
