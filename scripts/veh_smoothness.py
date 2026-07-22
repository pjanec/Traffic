#!/usr/bin/env python3
# Rear-bumper smoothness gate for a single instrumented vehicle (debug_<id>.csv from the Host).
# Measures what the eye sees in 3D: the drawn REAR bumper's high-frequency wiggle + jerk during turns.
import csv, math, sys
path = sys.argv[1] if len(sys.argv)>1 else 'artifacts/igbridge/debug_v2.csv'
rows=list(csv.DictReader(open(path)))
def col(n): return [float(r[n]) for r in rows]
t=col('t'); cH=col('cH')
rX=col('rearX'); rY=col('rearY')
def sdel(a,b): return ((b-a+540)%360)-180
DT=0.05
# turn windows: |dcH/dt| sustained above 3 deg/s
turning=[abs(sdel(cH[i-1],cH[i]))/DT>3 for i in range(1,len(t))]
# rear-bumper metrics over TURN samples only
def rear_metrics():
    # acceleration (2nd diff) of rear position, and jerk (3rd diff); count accel-direction reversals
    ax=[(rX[i+1]-2*rX[i]+rX[i-1])/DT/DT for i in range(1,len(t)-1)]
    ay=[(rY[i+1]-2*rY[i]+rY[i-1])/DT/DT for i in range(1,len(t)-1)]
    jx=[(ax[i]-ax[i-1])/DT for i in range(1,len(ax))]
    jy=[(ay[i]-ay[i-1])/DT for i in range(1,len(ay))]
    # restrict to turning samples
    tm=turning[1:len(t)-1]
    accmag=[math.hypot(ax[i],ay[i]) for i in range(len(ax)) if i<len(tm) and tm[i]]
    jerkmag=[math.hypot(jx[i],jy[i]) for i in range(len(jx)) if i<len(tm)-1 and tm[i]]
    # wiggle = sign reversals of the rear-accel component perpendicular to heading, amplitude>0.3 m/s^2
    rev=0
    for i in range(1,len(ax)):
        if i>=len(tm) or not tm[i]: continue
        hr=cH[i+1]*math.pi/180; px=math.cos(hr); py=-math.sin(hr)  # a perp axis
        p0=ax[i-1]*px+ay[i-1]*py; p1=ax[i]*px+ay[i]*py
        if p0*p1<0 and abs(p1)>0.3 and abs(p0)>0.3: rev+=1
    import statistics as st
    return dict(turn_samples=len(accmag),
                rear_accel_rms=round(st.pstdev(accmag) if len(accmag)>1 else 0,2),
                rear_accel_max=round(max(accmag) if accmag else 0,2),
                rear_jerk_rms=round(st.pstdev(jerkmag) if len(jerkmag)>1 else 0,1),
                rear_jerk_max=round(max(jerkmag) if jerkmag else 0,1),
                rear_accel_reversals=rev)
# heading smoothness over turn
yr=[sdel(cH[i-1],cH[i])/DT for i in range(1,len(t))]
yacc=[(yr[i]-yr[i-1])/DT for i in range(1,len(yr))]
yrev=sum(1 for i in range(1,len(yacc)) if yacc[i-1]*yacc[i]<0 and abs(yacc[i])>30 and abs(yacc[i-1])>30 and (i<len(turning) and turning[i]))
m=rear_metrics(); m['heading_yawaccel_reversals']=yrev
print(path.split('/')[-1], m)
