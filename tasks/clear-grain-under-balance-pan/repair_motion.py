"""Fixed pan removal, grain sweep, reseating, and 100 g verification path."""
from hardware import TOP
from motion import demo as original_demo
from yam_arena.demo import Waypoint as W

def demo():
    O=-.04695;G=-.0014;x=.391;z=TOP+.064
    original=original_demo()
    left=original['left'][:9]
    left += [
        W(1.5,(x,.005,z+.018),O,'approach outside loadcell support',0.),
        W(1.,(x,.005,z+.018),G,'close above case',0.),
        W(2.,(x,.005,z),G,'lower edge above case',0.),
        W(3.,(x,.04,z),G,'sweep exposed grain clear',0.),
        W(1.5,(x,.04,z+.018),G,'withdraw vertically',0.),
        W(1.,(x,.04,z+.018),O,'open clear of case',0.)]
    for w in original['left'][9:-1]:
        p=w.pos
        if p[:2]==(.23,.18):p=(.228509,.180923,p[2])
        elif p[:2]==(.3491,.0162):p=(.35,.016,p[2])
        left.append(W(w.duration,p,w.finger,w.label,w.yaw))
    left.append(W(37.,(.23,.18,TOP+.18),O,'observe verification',0.))
    right=original['right']
    right[0]=W(42.,right[0].pos,right[0].finger,'wait for physical repair',0.)
    return {'left':left,'right':right}
