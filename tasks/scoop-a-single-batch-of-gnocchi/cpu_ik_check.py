import json,pathlib,numpy as np
from collections import Counter
from yam_arena import ik,station,task_file
from yam_arena.demo import ArmPlan
P=pathlib.Path(__file__).resolve().parent;t=task_file.load(P/'task.py');out={}
for arm,wps in t.demo().items():
    plan=ArmPlan(wps,station.BASES[arm]);bad=[];limits=True
    for k in range(round(plan.duration*20)+5):
        act,label=plan.action7((k+1)/20)
        pos,rot,_,_=plan.target((k+1)/20);actual,R=ik.tcp_pose(act[:6],station.BASES[arm])
        pe=np.linalg.norm(actual-pos);re=np.linalg.norm(ik._rot_error(rot,R))
        if pe>.0005 or re>np.deg2rad(.1):bad.append({'step':k,'label':label,'position_mm':pe*1000,'rotation_deg':np.rad2deg(re)})
        limits=limits and bool(np.all(act[:6]>=ik.LOWER)&np.all(act[:6]<=ik.UPPER))
    out[arm]={'worst_position_mm':plan.worst_ik_mm,'worst_rotation_deg':plan.worst_ik_deg,'limits':limits,'failed_steps':len(bad),'failed_labels':dict(Counter(r['label'] for r in bad)),'worst20':sorted(bad,key=lambda r:r['position_mm'],reverse=True)[:20],'seconds':plan.duration}
(P/'cpu-ik.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out))
