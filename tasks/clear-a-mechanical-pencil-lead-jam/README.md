# Mechanical-pencil jam recovery — held, unvalidated

The final bounded native core experiment expelled the lead sideways during an
ordinary button-force stroke. The contact representation did not establish a
healthy feed baseline. This folder is held and must not be submitted or counted.

This folder is an unvalidated reconstruction for the Pentel Graphgear1000 PG1019
nominal 0.9 mm family. There is no completed robot demonstration or final scorer.
The component probes are fixed-housing experiments with an explicit diagnostic
button force; they always score zero. They are not task demos.

## Importance and sources

Pentel documents [PG1019 jam clearing](https://www.pentel.co.jp/support/mechanicalpencil/314/):
unscrew the outer cone, unscrew the inner metal tip, clear the lead blockage, refit
both, and check feed with the rear cap installed. Its [Canadian FAQ](https://pentel.ca/faq)
also permits matching crafting wire. These are direct evidence of this specific
user repair, not a measured claim about how often the population performs it.
The [current product](https://pentel.ca/product/pg1019) specifies nominal 0.9 mm
lead and a 4 mm retractable sleeve, and describes general writing as well as drafting.

[JWIMA's mechanism explanation](https://www.jwima.org/knowledge/types/mechanical-pencil/structure/)
supports three jaws, a chuck ring, axial return spring and lead retainer.
[Its FAQ](https://www.jwima.org/knowledge/types/mechanical-pencil/faq/) describes
broken lead blocking the guide pipe or three-way chuck and the ordinary clearing
operation. This is topology evidence, not an OEM sectional drawing of this model.

## Reconstruction limits

The approximately 150 mm length and 9.5 mm grip diameter are disclosed product-photo
and commercial-envelope estimates. Inner jaw profiles, ring dimensions, 1.0 mm
guide bore, spring constants and friction are explicit engineering estimates.
Nominal 0.9 mm does not establish an exact 0.9000 mm bore. The official step photos
show the 0.5 mm variant in the same supported family; their bore dimensions are
not copied. No P209 fixed-sleeve or unrelated patented knock mechanism is used.

The component experiment has three independently compliant jaws, a floating ring,
a native axial feed spring and three compliant lead-retainer sectors. Graphite is
a free rigid body with real contact. The fixed housing and 2.2 N button-axis load
are diagnostic fixtures only. They are not the final robot workspace or calibrated
manufacturer loads. The lead is never attached to a slider or position-updated.
The clip latch, sleeve retraction, removable finite threads and physical broken
fragment remain unimplemented; this experiment cannot establish full restoration.

The later `probe_source_feed.py` corrects the visible internal placement: the
chuck projects ahead of the original black body thread. The exposed metal-tip
envelope is bracketed at 2.5–3 mm rather than the initial 5–6 mm estimate, and the
original thread collar is included in the native access check. The corrected
source core uses a 3 mm maximum tip, a 2.5 mm chuck ring and three rear-rooted
compliant leaves with a conical closing surface. Its 0.01 N m/rad leaf spring is
an engineering approximation for a small slotted brass collet, not a measured
Pentel spring constant. No diagnostic displacement alone establishes repeatable
healthy feed or completed repair.

The copied mechkit helper preserves sub-milligram masses and mesh coordinates at
higher serialization precision. Internal collisions are enabled, with 20 micrometre
contact offsets and concave SDF surfaces. Mesh closure, native mass, contact behavior
and resulting feed must be checked before these estimates are used in a final task.
