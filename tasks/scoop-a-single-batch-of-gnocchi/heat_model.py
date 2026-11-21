"""Lumped hot-water energy model; estimated transfer/loss parameters, not measured hardware.

Water stores heat after OFF. Cooking eligibility uses >=95C as an explicit operational
hot-water check, observable with an inexpensive immersion thermometer in a real trial.
"""
import math
WATER_CP=4180.  # J/(kg K), representative liquid-water value; not a fitted parameter.
STEEL_CP=500.  # J/(kg K), representative stainless-steel approximation.
LATENT=2.26e6  # J/kg near atmospheric boiling.
ROOM_C=20.;BOIL_C=100.;HOT_C=95.
PAN_DRY_MASS=1.1
LOSS_W_PER_K=2.0  # Estimated whole-vessel loss coefficient; sensitivity must be disclosed/tested.
EFFICIENCY=.75  # Estimated heat delivered to water/pan from a solid plate, not measured.

def circle_overlap(r1,r2,d):
    if d>=r1+r2:return 0.
    if d<=abs(r1-r2):return math.pi*min(r1,r2)**2
    a=math.acos(max(-1.,min(1.,(d*d+r1*r1-r2*r2)/(2*d*r1))))
    b=math.acos(max(-1.,min(1.,(d*d+r2*r2-r1*r1)/(2*d*r2))))
    h=math.sqrt(max(0.,(-d+r1+r2)*(d+r1-r2)*(d-r1+r2)*(d+r1+r2)))
    return r1*r1*a+r2*r2*b-.5*h

class Heat:
    def __init__(self,water_mass,loss_scale=1.):
        self.water_mass=float(water_mass);self.temp_c=100.;self.loss_scale=float(loss_scale)
        self.evaporated=0.;self.input_w=0.;self.loss_w=0.
    def advance(self,dt,dial_radians,overlap=1.,rear_radians=0.,rear_overlap=0.):
        #Front1000W zone: explicit linear six-level approximation, not OEM duty-cycle calibration.
        setting=max(0.,min(1.,dial_radians/math.radians(300)))
        rear=max(0.,min(1.,rear_radians/math.radians(300)))
        self.input_w=EFFICIENCY*(1000.*setting*max(0.,min(1.,overlap))+1500.*rear*max(0.,min(1.,rear_overlap)))
        self.loss_w=LOSS_W_PER_K*self.loss_scale*max(0.,self.temp_c-ROOM_C)
        c=max(.05,self.water_mass)*WATER_CP+PAN_DRY_MASS*STEEL_CP
        energy=(self.input_w-self.loss_w)*dt
        if self.temp_c>=BOIL_C and energy>0:
            dm=min(self.water_mass,energy/LATENT);self.water_mass-=dm;self.evaporated+=dm
        else:self.temp_c=min(BOIL_C,max(ROOM_C,self.temp_c+energy/c))
        return self.temp_c>=HOT_C and self.water_mass>.05
