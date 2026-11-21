# Bowden-tube recovery — engineering prototype

This is not a submission-ready task. There is no complete task scene, scorer, robot
repair demonstration, or model evaluation yet. The prototype investigates SR-048,
restoring the reaction support of a slipped sheath on a tendon-driven robot hand.

The hardware reference is the [Yeah Hand v1.3 release](https://github.com/opsobot/yeah-hand/releases/tag/v1.3).
Its [bill of materials](https://github.com/opsobot/yeah-hand/blob/494859df54603ad9b80a65a947fb8123e0a3221f/hardware/BOM.md)
specifies 3 mm OD, 1.5 mm ID PTFE tubing and a 0.75 mm braided tendon. The
assembly uses printed tube seats, not threaded metal Bowden adjusters.

The Blender objects named “Bowden Tube” cut approximately 4 mm passages out
of the skeleton. They are Boolean construction tools, not evidence of 4 mm
physical tubing. The smaller tendon passage is approximately 2.88 mm across.
The narrow supporting shoulder, real access, tube compliance and loss of motion
from a displaced tube must be validated before building a benchmark around them.

`bowden.py` computes continuous tension and equal reaction forces from tendon
length. It contains no seating predicate, attachment, pose correction or success
flag. Its material parameters are not calibrated. Correct numerical forces
alone do not establish a faithful cable simulation or a successful repair.

Hardware sources are licensed CERN-OHL-S-2.0; documentation is CC-BY-SA-4.0.
No source CAD mesh is distributed in this folder at this stage. Any later CAD
derivatives must preserve the applicable notices and corresponding sources.

`hardware.py` fetches the released skeleton STL through a pinned SHA-256 archive
and preserves its full scale. `probe_tube_seat.py` is an isolated contact fixture:
a clamped static skeleton and a gravity-disabled rigid 10 mm tube piece receive
up to 3 N of axial load. The piece ejected after unloading, so this test does not
validate tube-seat support. It also extends into a curved passage, which may
confound the result. `probe_tube_seat_short.py` uses a 4 mm piece entirely within
the straight upper passage and logs normal contact forces; it also became unstable under rising load. Both diagnostics always score zero. Tube elasticity, friction and the
complete tendon reaction path remain unvalidated.

The local `mechkit.py` comes from the repository's standard mechanism kit, with
higher geometry output precision and smaller contact offsets for this diagnostic.
The setup is a laboratory contact test, not a proposed benchmark start state.

The unit-calibration probes exposed an unresolved runtime inconsistency: a free
body marked with `disableGravity=true` in both USD and the runtime API falls
under gravity after a nonzero force wakes it. Earlier coupon unload behavior
therefore does not establish a hardware retention failure. Independent 1 kg
cube checks confirm the force scale: support is 9.81 N under gravity and 14.81 N
with an additional 5 N load; 0.1 N produces approximately 0.1 m/s² horizontally.
The `probe_tube_seat_zero_g_*` pair uses explicit zero gravity for the entire
isolated scene and eight velocity solver iterations at 2400/4800 Hz. This is a
numerical geometry diagnostic, not the physical benchmark or a completed repair.

Both explicit zero-gravity coupon runs reached 3 N without the abort condition.
During loading, the tube-tip centre remained within 2.38 micrometres of the seat
plane at 2400 Hz and 0.59 micrometres at 4800 Hz. This supports the geometric
location of the shoulder. Instantaneous contact velocities and post-unload drift
still depend on the time step, so dynamic/material fidelity remains unvalidated.
The full tube now has a compliant prototype. The coupled tendon route, motor spool and moving finger remain unimplemented.

Sections of the released finger meshes identify circular guides at the MCP and
PIP joints, with radii 6.5135 mm and 5.1929 mm. Their centers are measured from
the guide surfaces; the PIP flexible-part object origin is offset from its guide
axis and must not be used as a joint axis. The distal joint instead has a
flexible polymer neck, without a circular guide in these meshes. A full finger
model must represent or explicitly approximate that compliance. These geometry
measurements do not validate the joint stiffness or the repair task.

The loader also exposes the final released `Index Phalanges.stl` and
`Index Joints and Inserts.stl`. These preserve the tendon channels, pad recesses
and other final Boolean cuts absent from the intermediate editable components.
The loader reproduces all 276,566 finger triangles and 8,234 joint/insert
triangles exactly, converting millimetres to metres. It does not repair the
source's disconnected or zero-area artifacts. `index_rigid_parts()` now separates
the base and three rigid phalanges by vertex connectivity, retaining detached
artifacts separately and recording each triangle's original index. An exact
coordinate round-trip and complete, unique triangle partition were checked for
all 276,566 triangles. Point-attached artifacts remain with their source piece;
three resulting meshes are not watertight. These are visual/source partitions,
not validated collision meshes. `index_flexible_parts()` similarly preserves all
8,234 triangles while separating the MCP, PIP and DIP joint pieces and the two
tendon inserts. Their elastic properties remain unspecified; the complete
finger still needs a validated articulation before use in a repair scene.

`physics_hooks.py` updates mechanical forces at every physics step, independently
of the robot's 20 Hz action rate. The first oscillator probe exposed a missing
hook when the embodiment module was already imported; that path is now patched.
The corrected runs record exactly 12, 24 and 48 callbacks per action at 240,
480 and 960 Hz. For a 1 kg, 500 N/m, 2 Ns/m damped oscillator, maximum position
errors against the analytical solution were 9.99, 3.63 and 1.59 mm respectively
over three seconds. Substep velocities match the applied force; the remaining
finite-step energy error requires a smaller step for stiff, lightweight parts.
This validates callback operation, not tendon material properties.

`point_forces.py` clears each owned part's force buffer once and accumulates all
point loads, preserving multiple tendon reactions and their moments. Repeated
calls to the simulator's single-load setter would erase earlier loads on other
bodies of the same articulation. Its pure-couple diagnostic applies two opposite
0.1 N forces 100 mm apart on a 1 kg cube. It records no centre-of-mass drift and
matches the analytical angular velocity within 0.00021 rad/s over one second,
including the measured simulator angular damping of 0.05/s. The first comparison
omitted that damping. This checks point-force accumulation on one body; the following diagnostic
extends that check to multiple bodies.

`probe_tendon_reactions_unlimited.py` checks a tension-only spring/damper line
between two bodies of one free articulation, with zero gravity and no joint
stops. Over three seconds at 960 Hz, it records 2,880 force callbacks, less than
0.82 micrometres of center-of-mass drift, linear momentum below
4.25e-8 kg m/s and angular momentum below 1.41e-8 kg m²/s. Sampled total energy
decreases throughout. This verifies accumulated reactions across bodies; it
does not calibrate tendon stiffness or reproduce the hand's complete force path.


`compliant_tube.py` models the BOM 3 mm OD / 1.5 mm ID hollow tube with two
bending degrees of freedom per junction, conserved segment lengths and a
straight material rest shape. The 24-segment initial route in `extension_route.py`
is measured from sections of the released editable CAD's clearance cutter;
its length is 66.6792 mm. This is a reproducible initial route, not a measurement
of the installed tube's exact cut length. The tube has no seating-triggered
attachment, stiffness change or pose correction. Axial stretch, torsion,
ovalization, plastic kinking, creep and tube self-contact are omitted.

Density 2150 kg/m³ and flexural modulus 496 MPa are generic PTFE references from
[Chemours](https://www.teflon.com/en/products/product-selection-guide), not
calibration measurements of this installed tube. The bending stiffness is EI
per dual segment length. Each numerical carrier receives 0.1% of its segment's
mass, deducted from that segment so the total mass is conserved.

The small-load bending diagnostic uses an eight-segment straight tube, with
56 mm free beyond its fixed first segment, a 0.01 N lateral tip load and zero
gravity. The discrete beam prediction is 0.319848 mm; simulated steady
deflection is 0.329890 mm at 2400 Hz and 0.324235 mm at 4800 Hz. The orthogonal
axis gives 0.329887 mm at 2400 Hz. This establishes axis consistency and reduced
static error at a smaller timestep. Residual post-unload motion remains
step-dependent; this does not validate large bends, contact stability or real
tube material properties. `probe_full_tube_idle.py` tests the complete curved
tube in the source skeleton with normal gravity and no tendon load.


Full-tube contact tests are still failing numerical validation. With its straight
material rest shape, the 24-segment tube becomes nonfinite during reset settling;
adding 0.1 mm clearance at both ends does not fix this. Removing housing contact
keeps the tube finite as it straightens and falls under gravity. Making the
initial curved geometry stress-free also keeps the contact run finite, but shows
velocity jitter and changes the physical material model, so it is only a control.
A 0.4 s preload ramp during reset still fails. These tests do not establish a
physical retention failure or a successful seated baseline.

`centered_tube.py` is a coordinate-representation diagnostic. It places each
body frame at its center of mass while preserving geometry, mass, inertia and
joint anchors. Its offline world-vertex comparison differs by less than
1.2e-10 m and joint anchors by less than 5.4e-11 m. The centered-frame runtime still becomes nonfinite at 2400 Hz. The original geometry also fails at 9600 Hz; the smaller timestep delays failure.


`smooth_extension_route.py` preserves the CAD endpoint guide directions and
smooths the intermediate cutter path with a cubic curve. This is an assumed
initial elastic shape, not a measured installed tube shape. Its sampled exterior
is clear of the source housing, but that mesh is not watertight. The 24-segment
version still becomes nonfinite. An eight-segment version is 0.109 mm shorter
(66.4535 mm) and remains finite in a short 4800 Hz test. The longer passive
energy audit shows it leaves its seat after roughly 1.7 seconds from physics
initialization. Total energy stays below its initial value, but rises between
some samples before escape. This neither proves correct contact nor establishes
a usable seated baseline. The 9600 Hz comparison stays seated for three seconds, so the qualitative
escape-versus-hold result depends on timestep and is not validated.

`tube_energy.py` audits segment and numerical-carrier kinetic energy, gravity
and bending energy from observed body poses and velocities and authored masses,
inertias and spring targets. It adds no force, attachment or state correction.


`redistributed_carrier_tube.py` is a numerical comparison for the very small
intermediate hinge bodies. It assigns each carrier 10% of its cell mass,
adjusting the other body's center of mass and inertia to preserve the pair's
original locked-cell moments. Serialized USD checks preserve all mesh vertices
and joint attributes exactly; pair center-of-mass differences are below 15.1 nm
and relative inertia errors below 1e-6. Stiffness, damping, material, contact
geometry and gravity stay unchanged. This is not exact dynamic equivalence:
the carrier does not follow the second hinge rotation. Fresh beam and contact
checks are required before using this representation in a task.
`carrier_tube_energy.py` uses the redistributed masses and full inertia tensors
for its audit. Eight- and 24-segment contact comparisons have completed; results are below.

The first carrier-redistribution runtime attempt failed during simulator startup
because USD was imported while loading the task. USD construction is now
deferred until after simulator startup; the task-import check leaves `pxr`
unloaded. The failed launches produced no simulation frames.

The 24-segment redistributed-carrier contact test still becomes nonfinite
during reset settling. The eight-segment comparisons completed; results are below.

The eight-segment redistributed-carrier run completed three seconds at 4,800 Hz
with finite states and both ends still seated; the 9,600 Hz comparison also completed. Its fresh straight-beam check gives a mean loaded deflection of
0.323780 mm over control steps 21–30, 1.229% above the discrete prediction.
The same time window with the original carrier gives 0.323752 mm, a 0.00843%
difference. This supports the small-load bending response, not contact
convergence, material calibration, or the complete hand's behavior.

At 9,600 Hz the tube remains seated for three seconds, but its final maximum
joint angle is 4.070 degrees versus 19.425 degrees at 4,800 Hz. The upper
endpoint differs by 0.463 mm between the runs. A 19,200 Hz comparison is running
to check convergence; finite coordinates alone are insufficient validation.


Collision geometry audit: small convex meshes cooked at their original metre
coordinates can differ substantially from the authored surface. A live connector
locking-tooth hull extended 0.554 mm beyond its intended shape; a 0.50 mm probe
became a distorted tetrahedron. Reducing the minimum cooking thickness alone
did not fix this. `scaled_convex.py` instead enlarges collider vertex coordinates
by 1000 and applies an inverse 0.001 child transform. Body dimensions, joints,
mass, inertia and material settings remain unchanged. Offline comparison bounds
world-vertex roundoff below 8 nm across the checked connector and tube colliders.
Fresh contact and retention trials are required; earlier successful mechanical
probes are historical diagnostics and do not validate the corrected task.

After the cooking correction, the eight-segment 4,800 Hz contact control is
finite, and the 24-segment free-relaxation control is finite without skeleton
contact or gravity. The constrained 24-segment model still becomes nonfinite
at 2,400 Hz. Centering its body frames at their centers of mass preserves
authored geometry, mass, inertia and joint settings. That centered test was
later found to have an incorrect runtime starting position (see below).
The corrected 19,200 Hz comparison subsequently completed. None of these controls
demonstrates the complete coupled hand or a successful repair.

The corrected 24-segment 19,200 Hz contact run completed one second with
finite logged states. This establishes neither timestep convergence nor full
hand behavior. A separate D6-joint model removes the artificial intermediate
carrier bodies, retaining the tube geometry, material mass and cell bending
stiffness. It is a new dynamic discretization and needs independent validation.
Its small-load bending result is 0.323783 mm versus a 0.319848 mm discrete
prediction. Free relaxation remains finite but leaves a 1.645-degree spread
between segment directions; momentum and constrained-contact checks are running.
The underlying two-axis articulation capability is documented in the
[Omni Physics joint guide](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/dev_guide/rigid_bodies_articulations/joints.html).

Runtime placement correction: a floating articulation rooted on its container
let PhysX choose a middle segment. Arena then placed that segment at the
requested first-segment pose, shifting the entire centered tube about 29 mm.
`spherical_tube_rooted.py` explicitly roots the articulation on `frame`.
A fresh check of every body center at runtime bounds the initial placement
error to 0.103 micrometres. The corrected free run stays finite for one second
with a final segment-direction spread of 0.689 degrees. The rooted contact
run still becomes nonfinite at 2,400 Hz: fixing placement alone is insufficient.
Neither run validates the complete hand or repair task.

The prototype is parked pending loaded tube and seat measurements. A commercial
size-matched option is BOLA S 1810-23 PTFE tubing: 1.5 mm ID, 3.0 mm OD,
0.75 mm wall, with a manufacturer minimum bend radius of 12 mm at 23 °C
([BOLA catalog, p. 190](https://www.bola.de/static/katalog/bola-en/files/basic-html/page190.html)).
The CAD-derived path has a minimum centerline radius of 18.07 mm; even subtracting
1.5 mm to compare conservatively as an inside radius gives 16.57 mm. This makes
that tube size geometrically plausible, but does not prove that this SKU is used
in the source hand, or validate force-deflection, loaded/cyclic behavior, kink
resistance or seat retention.

The linear beam diagnostic estimates 8.30% outer-fibre strain and 41.18 MPa by
extrapolating a small-strain constitutive law. That stress is not a physical
failure prediction: the model is outside its validity at that strain. No
material softening or curved stress-free shape was introduced to make the
simulation succeed. The constrained 24-segment tube still becomes nonfinite at
2,400 Hz; high-rate component stability does not establish a calibrated
load path. The next gate is a measured force-deflection and seat-retention test
for the selected tube under the source tendon route/load, followed by a
calibrated, timestep-converged full-hand repair and negative-control trial.

The rooted eight-cell, 4,800 Hz control completed three seconds with finite
states, but the tube left its seat and its endpoints moved about 0.3 m. Its
contact sheet was inspected; it is not a seated baseline or a repair demo.
Already-running refinement controls are retained as numerical diagnostics.
Restarting development requires a supported tube constitutive response,
converged contact behavior, the complete source hand and tendon force path,
and successful robot repair and negative-control trials.
