"""Interpret measured native normal contacts; no simulator mutation."""
import numpy as np
from recipe_scoring import plate_supported
SUPPORT_N=.002

def normalize_signed_rows(raw_rows,min_force=1e-5):
    """Convert native signed scalar/normal pairs to magnitude and acting normal.

    Opposing PhysX sensor reports can share a geometric normal and have opposite
    scalar signs. The physical force on the reporting actor is scalar * normal.
    Zero-force contact is not load-bearing and is excluded from this scorer.
    """
    result={}
    for name,contacts in raw_rows.items():
        result[name]=[]
        for actor,force,normal,point in contacts:
            if not np.isfinite(force) or not np.isfinite(normal).all() or not np.isfinite(point).all():
                raise RuntimeError('Nonfinite native contact; scoring unavailable')
            if abs(force)>min_force:
                result[name].append((actor,abs(force),np.asarray(normal)*(1 if force>0 else -1),point))
    return result

def is_robot_actor(path):
    #Actual station USD names are LeftArm/RightArm; scene dictionary keys differ.
    return any(f"/{name}/" in path for name in ("LeftArm","RightArm","left_robot","right_robot"))

def classify(rows,poses,spoon_pos,spoon_R,plate_pos,plate_R,bowl_x,bowl_y):
    n=len(poses);edges=[];plate_roots=[];spoon_roots=[];all_spoon_roots=[];anchored_roots=[]
    total_up=np.zeros(n);robot_touch=np.zeros(n,bool)
    for i in range(n):
        by_actor={}
        for actor,force,normal,point in rows[f'gnocchi_{i:02}']:
            up=max(0.,force*float(normal[2]));total_up[i]+=up
            by_actor[actor]=by_actor.get(actor,0.)+up
            if is_robot_actor(actor) and force>SUPPORT_N:robot_touch[i]=True
            if actor.endswith('/spoon') and up>SUPPORT_N:
                all_spoon_roots.append(i)
                q=(point-spoon_pos)@spoon_R
                if (q[0]/bowl_x)**2+(q[1]/bowl_y)**2<=1.05 and -.005<q[2]<.027:spoon_roots.append(i)
        for actor,up in by_actor.items():
            if up<=SUPPORT_N:continue
            other=actor.rsplit('/',1)[-1]
            if other!='spoon' and not other.startswith('gnocchi_'):anchored_roots.append(i)
            if other=='plate':
                q=(poses[i,:3]-plate_pos)@plate_R
                if np.linalg.norm(q[:2])<.100 and -.002<q[2]<.08:plate_roots.append(i)
            elif other.startswith('gnocchi_'):
                lower=int(other.rsplit('_',1)[-1])
                if not 0<=lower<n:raise ValueError('Unknown food contact actor')
                if poses[lower,2]<poses[i,2]:edges.append((lower,i))
    onplate=np.array(plate_supported(n,plate_roots,edges))
    bowl=np.array(plate_supported(n,spoon_roots,edges))
    tool=np.array(plate_supported(n,all_spoon_roots,edges))
    anchored=np.array(plate_supported(n,anchored_roots,edges))
    robot_supported=np.array(plate_supported(n,[int(i) for i in np.flatnonzero(robot_touch)],edges))
    return {'plate':onplate,'spoon':bowl&(~anchored)&(~robot_supported),'tool':tool,'up_n':total_up,'robot':robot_supported,'edges':edges}

def symmetric_rows(raw_rows,sensor_paths):
    """PhysX raw events may appear on only one actor's sensor.

    Input rows must first be normalized by normalize_signed_rows. Each acting
    normal then acts on its reporting sensor. Give the other sensed
    actor the opposite reaction, deduplicating a pair if both sensors report it.
    This reconstructs Newton's third-law pair, not an inferred pose contact.
    """
    by_path={path:name for name,path in sensor_paths.items()}
    result={name:[] for name in sensor_paths};seen=set()
    for name,contacts in raw_rows.items():
        source=sensor_paths[name]
        for other,force,normal,point in contacts:
            pair=tuple(sorted((source,other)))
            canonical=normal if source==pair[0] else -normal
            key=(pair,tuple(np.round(point,6)),tuple(np.round(canonical,6)),round(force,6))
            if key in seen:continue
            seen.add(key)
            result[name].append((other,force,normal,point))
            if other in by_path:result[by_path[other]].append((source,force,-normal,point))
    return result
