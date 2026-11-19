"""Unmodified full-scale workshop stationery, outside the repair path."""
from simready import SimReady
from crutch import TOP

def objects():
    return [
        SimReady('blue_marker',pos=(.60,-.50,TOP),size=(.109,.008,.007),mass=.008,
                 package='stationery_marker_thin_blue_a01',usd='sm_stationery_marker_thin_blue_a01_01.usd'),
        SimReady('slotted_screwdriver',pos=(.65,-.35,TOP),size=(.148,.027,.026),mass=.03647007420659065,
                 package='handTool_screwdriver_slotted_4mm_a01',usd='sm_handTool_screwdriver_slotted_4mm_a01_01.usd')]
