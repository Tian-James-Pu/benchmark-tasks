"""Prepare the manufacturer wire guard, retaining its original dimensions."""
from pathlib import Path
import urllib.request, hashlib, json
import numpy as np
import cadquery as cq
from asset_integrity import verify_source

CACHE = Path.home()/'.cache'/'jetson-fan-recovery'
URL = 'https://orionfans.com/3d/G40-2.stp'
if __name__ == '__main__':
    CACHE.mkdir(parents=True, exist_ok=True)
    source = CACHE/'orion-G40-2.stp'
    if not source.exists():
        source.write_bytes(urllib.request.urlopen(URL).read())
    verify_source('guard',source)
    solids = cq.importers.importStep(str(source)).solids().vals()
    arrays = {}
    for i, solid in enumerate(solids):
        vertices, faces = solid.tessellate(.06, .12)
        vertices = np.array([[v.x,v.y,v.z] for v in vertices])/1000.
        vertices = vertices[:,[0,2,1]]*np.array([1,-1,1])
        # Source ears point along the diagonal basis; the drawing specifies
        # a square 32 mm mount pattern matching the 40 mm fan.
        c = np.sqrt(.5)
        vertices = vertices @ np.array([[c,-c,0],[c,c,0],[0,0,1]]).T
        arrays[f'guard_{i}_v'] = vertices.astype(np.float32)
        arrays[f'guard_{i}_f'] = np.asarray(faces, dtype=np.int32)
    np.savez_compressed(CACHE/'guard.npz', **arrays)
    (CACHE/'guard-source.json').write_text(json.dumps({
        'url':URL, 'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
        'solid_count':len(solids), 'tessellation_mm':.06,
        'drawing':'https://orionfans.com/productFiles/datasheet/G40.pdf'
    }, indent=2)+'\n')
