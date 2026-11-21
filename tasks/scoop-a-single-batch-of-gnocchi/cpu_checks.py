"""Physical invariants and contact-adapter adversaries; no claimed native validation."""
import json,math,pathlib
import numpy as np
import batch_physics as b
from heat_model import Heat,circle_overlap
from contact_rules import classify,symmetric_rows
checks=[]
def check(label,ok):
    assert ok,label
    checks.append(label)
pose=np.zeros((b.N,7));pose[:,6]=1.;pose[:,:3]=[.34,.03,.80]
v=np.zeros((b.N,6));raw=np.full(b.N,80.);cooked=np.full(b.N,150.)
f,t,w=b.water_forces(pose,v,raw,.85,[.34,.03],.12,.75)
check('Raw food sinks under gravity',np.all((f[:,2]>0)&(f[:,2]<b.MASS*b.GRAVITY)))
f,t,w=b.water_forces(pose,v,cooked,.85,[.34,.03],.12,.75)
check('Cooked density proxy produces positive buoyancy',np.all(f[:,2]>b.MASS*b.GRAVITY))
pp=np.array([.34,.03,.75,0,0,0,1.]);pv=np.zeros(6)
f1,t,w=b.water_forces(pose,v,cooked,.85,pp[:2],.12,.75,pp,pv)
pp[:2]+=[.4,.1]
f2,t,w=b.water_forces(pose,v,cooked,.85,pp[:2],.12,.75,pp,pv)
check('Translated loose vessel leaves no ghost water force',np.all(f2==0)&np.all(w==0))
pose[:,:2]+=[.4,.1]
f3,t,w=b.water_forces(pose,v,cooked,.85,pp[:2],.12,.75,pp,pv)
check('Water force follows the actual vessel',np.allclose(f1,f3))
heat=Heat(1.8)
for _ in range(12000):heat.advance(.01,0.)
check('Residual water heat remains valid after burnerOFF',95<heat.temp_c<100)
for _ in range(120000):heat.advance(.01,0.)
check('Water eventually cools below cooking threshold',heat.temp_c<95)
heat=Heat(1.8)
for _ in range(22000):heat.advance(.01,math.radians(150))
check('Nominal simmer stays hot and evaporation is bounded',heat.temp_c==100 and .0<heat.evaporated<.03)
heat2=Heat(1.8,loss_scale=2.)
for _ in range(22000):heat2.advance(.01,math.radians(150))
check('Twice estimated heat loss still maintains cooking heat',heat2.temp_c>=95)
check('Disjoint burner and pan receive no powered heat',circle_overlap(.098,.0725,.2)==0)
check('Centered smaller burner has full geometric overlap',abs(circle_overlap(.098,.0725,0)-math.pi*.0725**2)<1e-12)
#Synthetic contacts exercise only their interpretation; no synthetic data enters native trials.
poses=np.array([[0,0,.01,0,0,0,1],[0,0,.025,0,0,0,1]],float);I=np.eye(3);zero=np.zeros(3)
def row(actor,normal=(0,0,1),force=.034,point=(0,0,.005)):return ('/World/envs/env_0/'+actor,force,np.array(normal,float),np.array(point,float))
def classify2(a=(),bb=()):return classify({'gnocchi_00':list(a),'gnocchi_01':list(bb)},poses,zero,I,zero,I,.0475,.03556)
x=classify2([row('gnocchi_01',(1,0,0))])
check('Incidental lateral food contact is not upward support',x['up_n'][0]==0)
x=classify2([row('spoon'),row('spoon',(0,0,-1))])
check('Opposing normals cannot cancel into unsupported float',x['up_n'][0]>.03)
x=classify2([row('plate')],[row('gnocchi_00')])
check('Food stack rooted on real plate qualifies support',x['plate'].all())
x=classify2([row('spoon')],[row('gnocchi_00')])
check('Ordinary stacked spoonful qualifies actual spoon support',x['spoon'].all() and not x['plate'].any())
x=classify2([row('spoon'),row('pan')],[row('gnocchi_00')])
check('Pan-anchored food stack is not dry spoon retrieval',not x['spoon'].any())
x=classify2([row('spoon',point=(.14,0,.01))])
check('Food touching spoon handle is not bowl retrieval',not x['spoon'].any())
x=classify2([row('spoon'),row('LeftArm/Geometry/base/gripper',(1,0,0))])
check('Gripper contact cannot become valid spoon retrieval',not x['spoon'].any())
x=classify2([row('LeftArm/Geometry/base/gripper',(1,0,0))],[row('gnocchi_00'),row('plate')])
check('Gripper support propagates through lower food despite plate contact',x['robot'].all())
paths={'gnocchi_00':'/World/envs/env_0/gnocchi_00','plate':'/World/envs/env_0/plate'}
one={'gnocchi_00':[],'plate':[row('gnocchi_00',(0,0,-1))]}
sym=symmetric_rows(one,paths)
check('One-sided native plate event reconstructs positive food support',len(sym['gnocchi_00'])==1 and sym['gnocchi_00'][0][2][2]>0)
both={'gnocchi_00':[row('plate')],'plate':[row('gnocchi_00',(0,0,-1))]}
sym=symmetric_rows(both,paths)
check('Two-sided reporting does not double the same contact',len(sym['gnocchi_00'])==1 and len(sym['plate'])==1)
r={'scope':'CPU model invariants and synthetic contact interpretation, not hardware measurements or native validation','checks':checks,'count':len(checks),'thermal_loss_sensitivity':{'nominal_final_c':heat.temp_c,'twice_loss_final_c':heat2.temp_c},'batch_mass_kg':b.N*b.MASS}
pathlib.Path(__file__).with_name('cpu-model-checks.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r))
