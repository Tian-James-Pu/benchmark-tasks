# Damaged robot-cover screw prototype

This folder is a parked, unfinished, zero-score hardware diagnostic. It does not demonstrate screw removal. The bounded native trials did not establish a two-jaw head clamp, so they do not measure removal torque capacity.

The hardware reference is Mitsubishi Electric's RH-2FRH2515 robot. Its service manual BFP-A3888A, printed page 4-47 (PDF 59), specifies four stainless M4x6 hex-socket button screws for the cable cover and a fixing torque of 1.39–1.89 Nm. The robot stays upright on its bolted base. The specification manual BFP-A3886A, printed page 2-23 (PDF 36), supplies the 180x140 mm base, 135/115 mm links and 550 mm maximum height. Cover dimensions and nearby housing contours are disclosed photo estimates.

The commercial tool reference is ENGINEER PZ-58. Catalogue page 4 gives a closed160×54 mm envelope,15 mm handle thickness,28 mm jaw length,8.5 mm steel-body thickness,6 mm tip thickness and130 g mass. Its top jaws grip head diameters3–9.5 mm. The handle outline follows the dimensioned plan view. The three curved longitudinal teeth and recessed side flanks follow the manufacturer's mechanism illustrations and patent US8656812B2 figures3,4 and8; their internal dimensions are scaled drawing estimates, not factory CAD. The prototype keeps separate plier halves and their passive pivot, native robot grasping, and physical tool/head contacts. There is no attachment, pose snapping or scripted bite. Spring rate and contact friction are estimates; rigid contacts omit plastic indentation.

The diagnostic tests pickup, head engagement and torque transfer. The screw has a100 Nm/rad elastic torsion fixture; its deflection reports reaction torque. Manufacturer1.39–1.89 Nm is installation torque, retained as a conservative capability target rather than measured breakaway. Threads, full extraction and replacement-gauge verification are not yet implemented. A successful pickup or clamp cannot establish removal feasibility. Scripted control uses simulator poses and joints for feedback; objects remain under native contact dynamics.

Sources:

- https://dl.mitsubishielectric.com/dl/fa/document/manual/robot/bfp-a3888/bfp-a3888a.pdf
- https://dl.mitsubishielectric.com/dl/fa/document/manual/robot/bfp-a3886/bfp-a3886a.pdf
- https://www.engineertools-jp.com/product-page/pz-58-screw-removal-pliers-gt
- https://www.engineertools-jp.com/_files/ugd/104650_3b3ff41c6db944fd9f689350ad540cdc.pdf
- https://www.accu.co.uk/socket-button-screws/8127-SSB-M4-6-A2
- https://www.nejisaurus.engineer.jp/special-page-nejisaurus
- https://patents.google.com/patent/US8656812B2/en

Run the final bounded diagnostic with `inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=probe_two_handles_posehold.py -E object_jitter=0 --instruction 'Remove the damaged screw from the robot cable cover.' --scorer task_score --max-steps 720`. It always scores zero. The source-corrected geometry and increased controller pose compensation still produced one-sided head contact followed by jaws closing off the head. The guard prevented the torque stroke. The later tool drop follows the guard relaxing finger squeeze, rather than establishing a separate pre-abort grasp failure. No validated demos or model evaluation exist.

The local mechkit helper is copied from the open-source heat-shrink-lug task. Task-critical mechanism geometry is reconstructed because a welded visual plier asset cannot provide the actual pivot and extraction jaws.
