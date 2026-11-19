#!/usr/bin/env python3
"""Collect completed physical load-path diagnostics; never count task validation."""
import argparse,ast,json,re,statistics
from pathlib import Path

def collect(wrapper):
    text=wrapper.read_text()
    if 'run status: completed' not in text or 'run_trial: video ' not in text:
        raise ValueError('Trial or rendering is incomplete')
    folder=Path(text.split(' -> ',1)[1].splitlines()[0])
    rows=[]
    for line in (folder/'run.log').read_text(errors='replace').splitlines():
        if line.startswith('BALANCE_COUPON '):
            _,step,payload=line.split(' ',2)
            rows.append({'step':int(step),**ast.literal_eval(payload)})
    windows={}
    for name,start,end in [('zero1',26,38),('load1',66,78),
                           ('zero2',106,118),('load2',146,158)]:
        values=[r['spring_force_n']/9.81*1000 for r in rows if start<=r['step']<=end]
        if len(values)!=7: raise ValueError(f'Incomplete window {name}: {len(values)}')
        windows[name]={'mean_equivalent_g':statistics.mean(values),
                       'min_g':min(values),'max_g':max(values),
                       'sd_g':statistics.pstdev(values)}
    readings=[windows[f'load{i}']['mean_equivalent_g']-windows[f'zero{i}']['mean_equivalent_g'] for i in (1,2)]
    return {'folder':str(folder),'rows':rows,'windows':windows,
            'known_load_equivalent_g':100.,'incremental_readings_g':readings,
            'errors_g':[v-100. for v in readings],
            'frames':int(re.search(r'run_trial: (\d+) steps recorded',text).group(1)),
            'counts_as_validation':False,'sheet_reviewed':False,
            'scope':'Known external forces on pan, not placed calibration mass or robot recovery. Means of seven sparse samples, not a validated display/filter. Inferred mechanical geometry and stiffness.'}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('wrapper',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    if a.output.exists(): raise SystemExit('Refusing to overwrite evidence')
    result=collect(a.wrapper);a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('rows','windows')}))
