"""Observe native mass/inertia, contact offsets, and actual filtered contacts."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from probe_chain_trace import *
from dataclasses import replace

_chain_build=CHAIN.build
def reported_chain():
    from pxr import Sdf,Usd,UsdPhysics,PhysxSchema
    layer=Sdf.Layer.CreateAnonymous('.usda');layer.ImportFromString(_chain_build())
    stage=Usd.Stage.Open(layer)
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
    return layer.ExportToString()
CHAIN=replace(CHAIN,build=reported_chain)
OBJECTS=[CHAIN,DRIVE]

class NativeAudit:
    def __init__(self):self.count=0;self.contacts=None
    def physics(self,env,dt):
        if self.contacts is None:
            import omni.usd
            stage=omni.usd.get_context().get_stage()
            view=trace.view
            masses=m._np(view.get_masses()).reshape(-1)
            inertias=m._np(view.get_inertias()).reshape(-1,3,3)
            offsets=m._np(view.get_contact_offsets())
            paths=list(view.prim_paths)
            native=[]
            for i,path in enumerate(paths):
                prim=stage.GetPrimAtPath(path)
                expected_mass=float(prim.GetAttribute('physics:mass').Get())
                expected_inertia=np.array(prim.GetAttribute('physics:diagonalInertia').Get())
                native.append(dict(body=path.rsplit('/',1)[-1],mass=float(masses[i]),expected_mass=expected_mass,
                    inertia=inertias[i].tolist(),expected_inertia=expected_inertia.tolist(),
                    mass_ratio=float(masses[i]/expected_mass),
                    inertia_diagonal_ratio=(np.diag(inertias[i])/expected_inertia).tolist()))
            print('CHAIN_NATIVE',dict(bodies=native,positive_contact_offsets_m=sorted(set(float(x) for x in offsets.ravel() if x>0))),flush=True)
            self.sensors=paths
            self.filters=paths+[f'/World/envs/env_0/drive/{name}' for name in ('crank','rear','guide','tension','cage')]
            self.contacts=trace.sim.create_rigid_contact_view(paths,[self.filters]*len(paths),max_contact_data_count=65536)
        if self.count in (0,1,2,4,8,16):
            raw=self.contacts.get_contact_data(dt)
            forces,points,normals,separations,counts,starts=[m._np(x) for x in raw]
            rows=[]
            for i,j in np.argwhere(counts>0):
                start=int(starts[i,j]);count=int(counts[i,j]);end=start+count
                rows.append(dict(body=self.sensors[i].rsplit('/',1)[-1],other=self.filters[j].rsplit('/',1)[-1],
                                 points=count,min_separation_m=float(separations[start:end].min()),
                                 max_normal_force_magnitude_n=float(np.abs(forces[start:end]).max())))
            rows.sort(key=lambda x:x['min_separation_m'])
            print('CHAIN_NATIVE_CONTACTS',self.count,dict(contact_points=int(counts.sum()),
                  storage_capacity=len(forces),pair_count=len(rows),
                  internal_chain_pairs=[r for r in rows if r['other'] not in ('crank','rear','guide','tension','cage')],
                  deepest_pairs=rows[:20]),flush=True)
        self.count+=1
native_audit=NativeAudit()
on_physics_step(native_audit.physics)
