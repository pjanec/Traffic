#!/usr/bin/env python3
import json, math, sys
BOXLEN=5.0; DT=0.05
PATH=sys.argv[1] if len(sys.argv)>1 else 'artifacts/igbridge/trace.jsonl'
em={}
for line in open(PATH):
    r=json.loads(line)
    if r['id'].startswith('v') and r['k'] in ('new','upd'):
        em.setdefault(r['id'],[]).append((r['t'],r['x'],r['y'],r['h']))
def sdel(a,b): return ((b-a+540)%360)-180
def analyze(rows):
    rows=sorted(rows)
    if len(rows)<10: return None
    turn=[abs(sdel(rows[i-1][3],rows[i][3]))/DT>3 for i in range(1,len(rows))]
    if sum(turn)<6: return None
    # rear bumper path
    rb=[(x-BOXLEN*0.5*math.sin(h*math.pi/180), y-BOXLEN*0.5*math.cos(h*math.pi/180)) for _,x,y,h in rows]
    # smoothed reference = centered 7-pt moving average
    W=3; ref=[]
    for i in range(len(rb)):
        lo=max(0,i-W); hi=min(len(rb),i+W+1)
        ref.append((sum(p[0] for p in rb[lo:hi])/(hi-lo), sum(p[1] for p in rb[lo:hi])/(hi-lo)))
    # wiggle amplitude = deviation of rb from ref, over turning samples (metres)
    dev=[]
    tm=turn+[turn[-1]]
    for i in range(len(rb)):
        if i<len(tm) and tm[i]:
            dev.append(math.hypot(rb[i][0]-ref[i][0], rb[i][1]-ref[i][1]))
    if not dev: return None
    # reversal count (accel sign flips, >0.3 m/s^2)
    ax=[(rb[i+1][0]-2*rb[i][0]+rb[i-1][0])/DT/DT for i in range(1,len(rb)-1)]
    ay=[(rb[i+1][1]-2*rb[i][1]+rb[i-1][1])/DT/DT for i in range(1,len(rb)-1)]
    rev=0
    for i in range(1,len(ax)):
        j=i+1
        if j>=len(rows) or not (j-1<len(turn) and turn[j-1]): continue
        hr=rows[j][3]*math.pi/180; px=math.cos(hr); py=-math.sin(hr)
        a0=ax[i-1]*px+ay[i-1]*py; a1=ax[i]*px+ay[i]*py
        if a0*a1<0 and abs(a1)>0.3 and abs(a0)>0.3: rev+=1
    return max(dev), rev
amps=[]; revs=[]
for vid,rows in em.items():
    m=analyze(rows)
    if m: amps.append(m[0]*100); revs.append(m[1])  # cm
amps.sort(); revs.sort(); n=len(amps)
print(f"turners={n}  wiggle-amplitude(cm): median={amps[n//2]:.1f} p90={amps[int(0.9*n)]:.1f} max={amps[-1]:.1f}  |  reversals: median={revs[n//2]} p90={revs[int(0.9*n)]}")
