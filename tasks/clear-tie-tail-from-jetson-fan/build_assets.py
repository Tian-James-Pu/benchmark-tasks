"""Download manufacturer CAD into a private cache and tessellate it.

Run with a Python environment containing cadquery and numpy. CAD is downloaded
for reference and scene integration; it is not redistributed in this package.
"""
from pathlib import Path
import urllib.request,zipfile,io,hashlib,json
import numpy as np
import cadquery as cq
from asset_integrity import verify_source
CACHE=Path.home()/'.cache'/'jetson-fan-recovery'
CACHE.mkdir(parents=True,exist_ok=True)
SOURCES={
 'fan':('https://www.delta-fan.com/Download/3D/AFB0405MA-A.stp','fan.stp'),
 'jetson':('https://developer.nvidia.com/jetson-nano-2gb-developer-kit-3d-cad-step-model','jetson.zip')}
def obtain(name):
 url,file=SOURCES[name];dest=CACHE/file
 if not dest.exists():dest.write_bytes(urllib.request.urlopen(url).read())
 if file.endswith('.zip'):
  archive=zipfile.ZipFile(dest);member=next(n for n in archive.namelist() if n.lower().endswith(('.stp','.step')))
  path=CACHE/'jetson.stp'
  if not path.exists():path.write_bytes(archive.read(member))
 else:path=dest
 verify_source(name,path)
 return path
if __name__=='__main__':
 arrays={};meta={}
 for key in SOURCES:
  path=obtain(key);parts=cq.importers.importStep(str(path)).solids().vals()
  meta[key]={'source_url':SOURCES[key][0],'step_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'solid_count':len(parts),'solids':[]}
  for i,part in enumerate(parts):
   vs,fs=part.tessellate(.08 if key=='fan' else .15,.15)
   v=np.array([[x.x,x.y,x.z] for x in vs])/1000
   f=np.array(fs,dtype=np.int32)
   # Delta source +Y fan axis maps to world +Z; right-handed rotation.
   if key=='fan':v=v[:,[0,2,1]]*np.array([1,-1,1])
   else:v-=np.array([.044,.039,0])
   arrays[f'{key}_{i}_v']=v.astype(np.float32);arrays[f'{key}_{i}_f']=f
   meta[key]['solids'].append({'index':i,'vertices':len(v),'faces':len(f),'volume_m3':part.Volume()*1e-9,'bounds':[v.min(0).tolist(),v.max(0).tolist()]})
  print(key,len(parts),flush=True)
 np.savez_compressed(CACHE/'geometry.npz',**arrays)
 (CACHE/'sources.json').write_text(json.dumps(meta,indent=2)+'\n')
 print(CACHE/'geometry.npz')
