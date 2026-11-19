#!/usr/bin/env python3
"""Collect completed score-disabled support diagnostics; visual review stays separate."""
import argparse,json,re
from pathlib import Path
from collect_connector_probe import literal

def collect(meta_path):
 meta=json.loads(meta_path.read_text()); wrapper=Path(meta['runner_log']).read_text()
 if 'run_trial: video ' not in wrapper or 'run status: completed' not in wrapper:
  raise ValueError('Incomplete trial/render; preserve live process')
 if not re.search(r'task_score: 0(?:\.0)?\s',wrapper):
  raise ValueError('Expected diagnostic zero')
 folder=Path(wrapper.split(' -> ',1)[1].splitlines()[0]); rows=[]; contacts=[]; aborts=[]
 for line in (folder/'run.log').read_text(errors='replace').splitlines():
  if line.startswith('DRIVER_SUPPORT_STATE ') or line.startswith('DRIVER_BENCH_CONTACT '):
   name,step,value=line.split(' ',2); row={'step':int(step),'state':literal(value)}
   (rows if name=='DRIVER_SUPPORT_STATE' else contacts).append(row)
  elif line.startswith('SUPPORT_ABORT '): aborts.append(literal(line.split(' ',1)[1]))
 if not rows or not contacts: raise ValueError('Missing support/contact records')
 def applied(r): return r['state'].get('applied_axial_n',r['state'].get('applied_down_n',0.))
 loaded=[r for r in contacts if applied(r)>=24.9]
 def vertical_reaction(record):
  return sum(f*n[2] for c in record['state']['contacts'] for f,n in zip(c['normal_force_n'],c['normals_world']))
 initial=next((r['state']['block_pos'] for r in rows if 'block_pos' in r['state']),None)
 block_drift=max((sum((v-a)**2 for v,a in zip(r['state']['block_pos'],initial))**.5*1000 for r in rows),default=None) if initial else None
 result={'source':meta['source'],'source_sha256':meta['source_sha256'],'folder':str(folder),
  'score':0,'counts_as_validation':False,'sheet_reviewed':False,
  'frames':int(re.search(r'run_trial: (\d+) steps recorded',wrapper).group(1)),
  'aborts':aborts,'states':rows,'contacts':contacts,
  'summary':{'peak_applied_load_n':max(map(applied,contacts)),
   'full_load_sample_count':len(loaded),
   'full_load_vertical_reaction_n':[min(map(vertical_reaction,loaded)),max(map(vertical_reaction,loaded))] if loaded else None,
   'maximum_block_drift_mm':block_drift,
   'maximum_block_tilt_deg':max((r['state'].get('block_tilt_deg',0) for r in rows),default=0) if initial else None,
   'last_state':rows[-1]['state']},
  'scope':'Sparse contact/pose samples; not a complete repair or proof of real tool friction. Contact reaction must be interpreted with the visible pose.'}
 return result

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('metadata',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
 if a.output.exists(): raise SystemExit('Refusing to overwrite existing evidence')
 result=collect(a.metadata); a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n'); print(json.dumps(result['summary']))
