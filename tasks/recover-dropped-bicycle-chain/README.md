# Inward-dropped bicycle chain — mechanism prototype

This folder is under construction. There is no final task, success scorer,
demonstration or validation result. The target remains recovery of an intact
chain from between the chainring and frame, followed by actual chain-driven
transmission and released retention.

The proposed drivetrain uses a full-scale 425 mm chainstay reference, 68 mm
bottom bracket shell, compatible Shimano Altus components and a KMC Z8.3 chain
with 12.7 mm pitch. The frame-side gap, all chainrings, front and rear cages,
native pedal/crank grasp surfaces and rear freewheel must remain represented.
The drive-side-up support pose changes the assembly's placement, not its scale
or access clearance. Angled wrist reach is promising; solid clearance and
actual chain manipulation remain untested.

The current offline geometry contains a 106-link closed chain, sized from the
largest 42/30-tooth combination plus two links as specified in Shimano
SI-6NMRA-002. Every adjacent pin is 12.7 mm apart. The USD builder contains 106
alternating plate bodies and 106 independently rotating rollers; joint targets
remain inside one reference so the final closing joint is retained.

Plate shape, roller tolerances, lateral pin response and cage centers are still
approximations. The 0.362 kg geometry-derived mass is not a measured KMC mass.
The current route is a planar seated-chain baseline. It does not yet include
the real front/rear chainline offset, inward-dropped starting fault, frame,
complete crank, or rear freewheel. A separate planar commissioning fixture has
an estimated passive cage spring and a driven test sprocket. Both initial
physics runs failed during settling; there is no stable transmission result.

The first tooth approximation overlapped rollers and was replaced by the JIS
S-profile equations in Tsubaki's *Drive Chain & Sprockets* catalog, printed page
154 (PDF page 155):
https://tt-net.tsubakimoto.co.jp/lib/catalog/C_CDC_EN/book/data/all_page.pdf
The corrected authored tooth surfaces clear the initial rollers within a
3.34 micrometre polygon discretization bound. The second settling failure
therefore still needs a runtime initialization/contact/constraint diagnosis.
Standard sprocket profiles do not reproduce Shimano shift cuts or lateral
chamfers. No force, joint-limit or chain-size change has been used to hide the
failure. The trace records body poses and joint anchors during observed settling, after native environment creation/reset; the first sample is not guaranteed to precede all initialization physics.

Mechanism code must use a closed pin-connected chain, alternating inner/outer
plates, roller/tooth contacts and passive derailleur tension. An engagement
flag, rigid loop, rope-only substitute or guided chain handler would not test
the intended recovery. All early probes will score zero.

`mechkit.py` is copied from the repository's standard mechanism helper,
originally from the heat-shrink-lug task. `scaled_convex.py` preserves authored
world geometry while avoiding small-convex cooking distortion; it is copied
from the connector prototype. The local mechanism helper adds an optional
contact-offset field for small roller contacts; neither helper changes the
shared runtime. Collider vertices retain eight decimal places before convex
cooking. `physics_hooks.py` supplies task-local physics-rate observation hooks,
copied from the connector prototype. Offline construction uses NumPy, SciPy,
Shapely, trimesh and USD.

The bounded follow-up isolated external contact: the same closed chain falls
coherently and rests on the table with sub-micrometre joint-anchor error when
the external gears and cage are absent. Native masses and inertias match the
authored values. Exact merging of adjacent convex gear triangles reduces the
number of colliders without changing the tooth-profile union or central bore.
It reduces the initial contact count but still ejects the chain during settling.
This candidate is parked, with no successful drivetrain or repair demonstration.
No force, timestep, or clearance sweep is planned under the current submission
priority. Evidence is in `pipeline/pdf-screening/sr023-bicycle-chain/`.
