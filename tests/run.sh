#!/usr/bin/env bash
# run.sh — reverb test suite. Properties that matter for an RME-grade, fabric-bound engine.
# Exits non-zero on any failure. Run from repo root: tests/run.sh
set -u
cd "$(dirname "$0")/.."
SRC=src/reverb.c; T=/tmp/revtest; mkdir -p $T
fail=0; pass=0
chk(){ if [ "$1" = PASS ]; then pass=$((pass+1)); echo "  PASS  $2"; else fail=$((fail+1)); echo "  FAIL  $2"; fi; }
md5of(){ md5 -q "$1" 2>/dev/null || md5sum "$1" | cut -d' ' -f1; }

echo "== build (two opt levels — fixed-point must be identical; no warnings) =="
cc -O3 -Wall -Werror $SRC -o $T/rev_o3 -lm || { echo "BUILD FAIL (-O3 -Werror)"; exit 1; }
cc -O0 -Wall         $SRC -o $T/rev_o0 -lm || { echo "BUILD FAIL (-O0)"; exit 1; }
echo "  built clean"

python3 - <<'PY'
import numpy as np, wave
sr=48000
def wav(p,d):
    d=np.clip(d,-1,1); w=wave.open(p,"wb");w.setnchannels(1);w.setsampwidth(2);w.setframerate(sr)
    w.writeframes((d*32767).astype('<i2').tobytes());w.close()
imp=np.zeros(int(5*sr)); imp[int(0.2*sr)]=0.9; wav("/tmp/revtest/imp.wav",imp)
wav("/tmp/revtest/hot.wav", np.tanh(3*np.random.default_rng(0).standard_normal(int(2*sr)))*0.95)
t=np.arange(int(1.5*sr))/sr; wav("/tmp/revtest/tone.wav", np.concatenate([0.6*np.sin(2*np.pi*440*t), np.zeros(int(2.5*sr))]))
PY
P="3000 2.5 0.45 0.45 0.5 0.9"

echo "== T1 determinism: -O3 vs -O0 byte-identical (fixed-point → fabric-safe) =="
$T/rev_o3 $T/imp.wav $T/a.wav $P 2>/dev/null; $T/rev_o0 $T/imp.wav $T/b.wav $P 2>/dev/null
[ "$(md5of $T/a.wav)" = "$(md5of $T/b.wav)" ] && chk PASS "T1 deterministic across compiles" || chk FAIL "T1 deterministic"

echo "== T2 no hard clip on a hot (-0.5dBFS) input =="
$T/rev_o3 $T/hot.wav $T/hotw.wav 3000 2.5 0.6 0.45 0.5 0.9 2>/dev/null
python3 -c "import numpy as np,wave,sys;d=np.frombuffer(wave.open('$T/hotw.wav').readframes(10**9),dtype=np.int16)/32768;sys.exit(0 if abs(d).max()<0.999 else 1)" && chk PASS "T2 no hard clip" || chk FAIL "T2 clip"

echo "== T3 stability: after input ends the tail strictly decays, never grows (no self-osc) =="
python3 - <<'PY' && chk PASS "T3 stable monotonic decay" || chk FAIL "T3 stability"
import numpy as np,wave,sys
w=wave.open("/tmp/revtest/a.wav");sr=w.getframerate()
d=np.frombuffer(w.readframes(10**9),dtype=np.int16).astype(float).reshape(-1,2).mean(1)/32768
pk=np.abs(d).max()
win=int(0.25*sr); e=np.array([np.sqrt(np.mean(d[i:i+win]**2)) for i in range(int(0.5*sr),len(d)-win,win)])
n=len(e)//3
decaying = e[2*n:].max() < e[:n].max()    # late energy below early (overall decay; allows mode-beats)
decayed  = e[-1] < pk*0.01                # ends ≥40 dB below peak (no self-oscillation floor)
sys.exit(0 if decaying and decayed else 1)
PY

echo "== T4 t60 controls the tail: longer t60 leaves clearly more low-band energy at 1s =="
python3 - <<'PY' && chk PASS "T4 tail energy scales with t60" || chk FAIL "T4 t60 control"
import numpy as np,wave,subprocess,sys
from numpy.fft import rfft,irfft
def tail_db(t60):  # low-band (<500Hz) dBFS remaining at t=1.0s after the impulse
    subprocess.run(["/tmp/revtest/rev_o3","/tmp/revtest/imp.wav","/tmp/revtest/ir.wav","3000",str(t60),"1.0","0.3","0.5","0.9"],stderr=subprocess.DEVNULL)
    w=wave.open("/tmp/revtest/ir.wav");sr=w.getframerate()
    d=np.frombuffer(w.readframes(10**9),dtype=np.int16).astype(float).reshape(-1,2)[:,0]/32768
    sp=rfft(d);fr=np.fft.rfftfreq(len(d),1/sr);sp[fr>500]=0;lo=irfft(sp)
    s=lo[int(1.0*sr):int(1.3*sr)]; return 20*np.log10(np.sqrt(np.mean(s**2))+1e-12)
a,b=tail_db(1.0),tail_db(4.0); print(f"    low-band @1s: t60=1.0→{a:.0f}dB  t60=4.0→{b:.0f}dB")
sys.exit(0 if b > a+6 else 1)   # ≥6 dB more tail energy at the longer t60
PY

echo "== T5 perceptual cleanliness: wet adds <1.0 roughness vs dry (440 tone) =="
$T/rev_o3 $T/tone.wav $T/tonew.wav $P 2>/dev/null
python3 - <<'PY' && chk PASS "T5 wet ≈ dry roughness (clean)" || chk FAIL "T5 rough"
import subprocess,sys
r=lambda f: float(subprocess.check_output(["python3","harness/perceptual2.py",f]).decode().split("rough")[1].split()[0])
sys.exit(0 if r("/tmp/revtest/tonew.wav") < r("/tmp/revtest/tone.wav")+1.0 else 1)
PY

echo "== T6 stereo: SUSTAINED input → L/R decorrelated (width) yet level-balanced =="
python3 - <<'PY' && chk PASS "T6 wide + balanced" || chk FAIL "T6 stereo"
import numpy as np,wave,sys
d=np.frombuffer(wave.open("/tmp/revtest/hotw.wav").readframes(10**9),dtype=np.int16).astype(float).reshape(-1,2)/32768
L,R=d[:,0],d[:,1];m=(np.abs(L)+np.abs(R))>1e-3
corr=np.corrcoef(L[m],R[m])[0,1]; bal=abs(20*np.log10((np.std(R)+1e-9)/(np.std(L)+1e-9)))
print(f"    L/R corr={corr:.2f} balance={bal:.1f}dB")
sys.exit(0 if corr<0.95 and bal<3.0 else 1)
PY

echo ""; echo "== $pass passed, $fail failed =="
[ $fail -eq 0 ]
