"""Fixed waypoint demonstration from the published starting layout."""
from yam_arena.demo import Waypoint as W
TOP=.7475;O=-.04695;H=-.0345

def demo():
 left=[W(1.5,(.3504,.0162,TOP+.11),O,'hover pan',0.),
 W(2.,(.3504,.0162,TOP+.068),O,'approach pan rim',0.),
 W(1.,(.3504,.0162,TOP+.068),H,'grip pan rim',0.),
 W(2.,(.3504,.0162,TOP+.11),H,'lift loose pan',0.),
 W(2.,(.23,.18,TOP+.11),H,'move pan to bench',0.),
 W(2.,(.23,.18,TOP+.0155),H,'lower pan onto rim',0.),
 W(1.,(.23,.18,TOP+.0155),O,'release pan',0.),
 W(2.,(.23,.18,TOP+.18),O,'clear pan',0.),
 W(2.,(.23,.18,TOP+.18),O,'observe',0.),
 W(1.5,(.23,.18,TOP+.07),O,'approach loose cover',0.),
 W(1.5,(.23,.18,TOP+.0155),O,'lower to cover',0.),
 W(1.,(.23,.18,TOP+.0155),H,'grip original rim',0.),
 W(1.5,(.23,.18,TOP+.11),H,'lift cover',0.),
 W(2.,(.3491,.0162,TOP+.11),H,'return cover',0.),
 W(2.,(.3491,.0162,TOP+.068),H,'seat on sensor support',0.),
 W(1.,(.3491,.0162,TOP+.068),O,'release cover',0.),
 W(1.5,(.23,.18,TOP+.18),O,'clear scale',0.),
 W(40.,(.23,.18,TOP+.18),O,'observe verification',0.)]
 right=[W(32.,(.20,-.25,TOP+.22),O,'wait for recovery',0.),
 W(1.5,(.23,-.18,TOP+.10),O,'approach check weight',0.),
 W(1.5,(.23,-.18,TOP+.025),O,'lower to body',0.),
 W(1.,(.23,-.18,TOP+.025),-.010,'grip 100g weight',0.),
 W(1.5,(.23,-.18,TOP+.12),-.010,'lift check weight',0.),
 W(2.,(.35,.016,TOP+.10),-.010,'carry over pan',0.),
 W(2.,(.35,.016,TOP+.0825),-.010,'lower onto pan',0.),
 W(1.,(.35,.016,TOP+.0825),O,'release check weight',0.),
 W(1.5,(.20,-.25,TOP+.22),O,'clear check weight',0.),
 W(4.,(.20,-.25,TOP+.22),O,'observe loaded indication',0.),
 W(1.5,(.35,.016,TOP+.10),O,'approach loaded weight',0.),
 W(1.5,(.35,.016,TOP+.0825),O,'lower to weight',0.),
 W(1.,(.35,.016,TOP+.0825),-.010,'grip check weight',0.),
 W(1.5,(.35,.016,TOP+.10),-.010,'lift weight off pan',0.),
 W(2.,(.23,-.18,TOP+.12),-.010,'return weight to bench',0.),
 W(2.,(.23,-.18,TOP+.025),-.010,'lower weight',0.),
 W(1.,(.23,-.18,TOP+.025),O,'release weight',0.),
 W(1.5,(.20,-.25,TOP+.22),O,'clear weight',0.),
 W(4.,(.20,-.25,TOP+.22),O,'observe unloaded indication',0.)]
 return {'left':left,'right':right}
