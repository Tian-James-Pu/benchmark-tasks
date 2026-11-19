# Caster contact development gate

This is not a validated benchmark task. Every outcome scores zero. The short probe tests native approach, original-size retention and an experimental deformable hair-mat aggregate; full clearing, reassembly and functional testing are not demonstrated.

Run the task with the native YAM scripted policy at20Hz for180steps. Only authorized physical GPUs2and3 may be used. Runtime and seed behavior remain unmeasured.

The iRobot i7 model table specifies340×340×94mm and3.4kg: https://homesupport.irobot.com/articles/en_US/Knowledge/163. The manufacturer documents removal, axle cleaning, reassembly and free-spin checking: https://homesupport.irobot.com/articles/en_US/Knowledge/2450. Its recycling manual p20 records25.4g and provides a ruler photograph: https://global.irobot.com/sfsites/c/cms/delivery/media/MC2MHJ2WE64VDNNKDJA3SWN5Q3VA. The original caster photograph is https://www.irobot.com/en_US/roomba-accessories/4624869.html. Wheel26×18mm, housing50mm, axle3mm and internal clearance are disclosed reconstructions, not OEM dimensions.

Compliance is an engineering approximation: ABS2GPa with4×0.6×4mm beam sectors gives a derived spring constant. The original grooved stem, removable axle and open fork are physical contacts, with no episode-triggered joint removal or attachment. The aggregate hair torus uses density200kg/m³, modulus50kPa, Poisson ratio0.2 and friction0.4/0.3 as uncalibrated illustrative estimates. It does not establish a faithful jam until its loaded mechanics and removal behavior have been checked. Hair entanglement, plastic damage and fracture remain missing. No oversized grip features or changed arm force limits are used.

`mechkit.py` is copied from the repository standard kit, originally credited to `heat-shrink-lug`; its native collision offset is reduced to50µm to resolve actual millimetre hardware. The native deformable hierarchy follows NVIDIA PhysX volume-deformable API examples.
