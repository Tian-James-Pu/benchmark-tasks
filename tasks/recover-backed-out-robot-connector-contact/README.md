# Connector contact recovery — feasibility work

Not a submission-ready task. All scores are intentionally disabled. Wire coupon
grips and a full-scale connector contact prototype are being evaluated. A
successful coupon grasp does not validate a connector lock or the complete task.

The probe uses a 70 mm long, 2.4 mm diameter rigid insulated-wire coupon of 1.5 g.
Its purpose is to check ordinary gripper contact at this scale before constructing
the actual harness. No attachment or special gripping tab is used.

The 120 mm articulated wire probe also lifted and released through pad contact.
Its four-segment bending model remains uncalibrated.

`probe_connector.py` adds a free 33472-4001-size housing, separate crimped
terminal, housing locking finger, captive secondary retainer and articulated
lead. It has no terminal-to-housing joint or attachment. The first idle trials
failed because the parts were expelled or became numerically unstable. The
diagnostic variants are not demonstrations of successful repair.

The reconstruction uses the manufacturer's terminal drawing dimensions:
20.45 mm total length, 9.95 mm contact-box length, 2.55 mm width, 2.80 mm body
height and 0.30 mm sheet. The housing envelope measures 20.783 by 24.796 by
37.000 mm. Internal latch geometry, clearances, loads, sealing and variant-specific
retainer positions remain unvalidated. No manufacturer CAD mesh is redistributed.

Sources: [Molex MX150 application guide, revision D7](https://www.molex.com/content/dam/molex/molex-dot-com/products/automated/en-us/applicationspecificationspdf/000/000000/AS-33472-100-001.pdf),
[manufacturer housing drawing](https://www.corsa-technic.com/item_dwg/SD-33472-0001_C2.pdf),
and [manufacturer terminal drawing](https://www.corsa-technic.com/item_dwg/SD-33012-002_D.pdf).
The current guide specifies gripping the receptacle lead at least 20 mm from
the insulation crimp. Direct safe reseating is allowed; extraction is not required
merely to impose an extra step.

`mechkit.py` derives from the mechanism helper distributed with `heat-shrink-lug`.

A task-local reset correction now restores configured articulation joint positions
and velocities before settling. A separate clamped engineering fixture retained a
preseated contact under a ramp to 90 N at 2400 Hz physics, with about 0.10 mm axial
movement. The same fixture was numerically unstable at 240 Hz. This is a component
test, not a robot repair demonstration; no task score is enabled. Complete hardware
fidelity, secondary-retainer operation and the complete repair remain to be validated.

The 90 N fixture result uses a regularized lance joint armature of 1e-7 kg m².
Reducing it to 1e-10 made the loaded test unstable even at 2400 Hz; 1200 Hz also
failed at high load with the larger armature. These numerical limits remain part
of the engineering evidence and must be resolved or justified before submission.

The first secondary-retainer negative test failed: it reached its closed position
with the terminal still 3.7 mm backed out. A revised reinforcement-bar clearance
allows closure over a fully seated contact. The paired partly seated test blocks
closure as described below. This is a physical-geometry issue, not something a
scoring predicate can repair.

The corrected-mass probes use the published 8.180 g housing assembly, three
0.160 g cavity plugs, a 0.391 g contact and 16 g/m wire. Their inertias and the
passive retainer detent are still estimates. These probes have no enabled task
score and are not included in the completed-task count.

With corrected masses and the revised clearance, a 2400 Hz fixture retained the
contact through 90 N with roughly 0.05 mm axial movement. The paired 25 N closing
attempt with a backed-out contact stopped at 1.041 mm of retainer travel, leaving
3.995 mm of contact backout; both component runs completed 161 frames. These use
the regularized joint inertia. A completed 4800 Hz comparison retained the contact
through the same load; its final axial position differed by 9.62 micrometres.
This supports timestep convergence for this fixture, not physical validation of
the added joint inertia.

A completed two-arm trial grasped the wire, pushed the contact into the housing
and released the wire. The contact remained seated at an axial offset of
-0.157 mm at the end of 381 frames. The secondary retainer remained open, so this
is not a complete repair or a full-score task demonstration.

The current retainer-press probe starts with the contact seated and uses an
ordinary 4 mm screwdriver. Its scripted policy observes the simulated tool tip
and housing poses to correct its robot commands; it never writes object poses.
This privileged feedback is for the scripted demonstration only. The retainer
front-plate webs are reduced geometric reconstructions, not measured CAD.

The completed accumulated-feedback press trial reached the target but did not
close the retainer: the housing moved upward by 10.39 mm while sampled retainer
travel peaked at 0.221 mm, then returned open. The ungrasped screwdriver fixture stopped at 4.57 mm after its tool
rotated onto the table. A direct retainer-load fixture reached 5.00 mm and
retained that position after unloading. These are engineering component tests;
the completed robot contact diagnostic found only tool-to-retainer contact.
Its peak sampled normal force was 1.89 N. The housing and holding hand rose
together by 10.39 mm, with only 0.0053 mm change in their relative vertical
position. The two-arm position-feedback trial reached 1.177 mm of retainer travel and
14.21 N peak sampled tool contact force, then the driver rotated in its grip.
The retainer returned open. The first full-pose controller reached a wrist
limit because its requested tool orientation was unreachable from the grasp.
A captured-angle variant aligned successfully but stopped at 0.898 mm after
its slip detector counted remote-tip motion from a small rotation as translation.
The retainer later peaked at 1.083 mm while holding, then reopened on withdrawal.
An independent material-point check found less than 0.42 mm movement at the
grasp over the sampled press. The corrected variant measures translation at the
original grasp point and retains separate rotation and joint-limit stops. It
reached the wrist command limit, peaked at 1.662 mm of retainer travel and
13.31 N sampled normal contact force, then returned open. A forward-grip diagnostic
grips the existing handle 25 mm farther forward and raises the nominal tool
approach by 25 mm to compensate for the shorter tip offset. Its unloaded
waypoints pass the reach check, but its contact trial hit the 12-degree
rotational-slip stop at 12.09 degrees. It reached only 0.036 mm of retainer
travel and returned open; moving onto the tapered handle did not improve the grip.
An original-grip variant tested a working pose 75 mm higher, with more
available wrist travel. It reached 0.756 mm of travel before a 5.109 mm
translation-slip stop, then returned open. The driver axis was about 14 degrees
from the press axis. A new diagnostic tilts the housing 15 degrees toward the
driver. That trial reached 1.989 mm of sampled travel and 17.31 N sampled normal
tool load, then lost the housing from the right grasp and returned open. A
diagnostic now records both hands' contact forces and stops both arms when
either grasp is lost or a joint command approaches its limit.
All variants use ordinary joint commands with unchanged mechanics and actuator
limits. The detent resistance has not been lowered to make a demonstration pass.

A later calibration found an unresolved runtime inconsistency in per-body
gravity exclusion after external forces wake a body. The ungrasped screwdriver
fixture intended to disable gravity; that assumption is now unverified, so its
failure cannot be attributed solely to rotational dynamics. The ordinary robot
press scenes use normal gravity.

The updated front-plate negative control stopped at 1.041 mm under a direct
25 N load while its contact remained 4.00 mm backed out. Its retainer did not
close. This does not substitute for the remaining robot and full-task tests.

The screwdriver is NVIDIA SimReady `handTool_screwdriver_slotted_4mm_a01`,
`sm_handTool_screwdriver_slotted_4mm_a01_01.usd`, at scale 1, revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, under CC BY 4.0.
Source: [NVIDIA SimReady assets](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd).
© NVIDIA, CC BY 4.0. The loader makes the tool a rigid convex body, sets its
published mass, and reduces the collision contact offset to 0.1 mm with zero
rest offset for this small contact probe. `simready.py` is the shared asset loader.
It is evaluated as a pressing tool, not as the manufacturer's narrower pry tool.

`probe_connector_full.py` prepares the combined sequence and return motions.
Its nominal offline reach check passes, but it has not run in the simulator and
has no enabled score. The specimen currently has one service lead and three
cavity plugs; it is not a complete functioning motor harness. Hardware relevance
and representative harness access must be resolved before submission.

`probe_connector_three_lead_idle.py` adds two independently moving contacts,
leaving one cavity plugged, and gives all four cavities a reduced locking
finger. The bottom row mirrors the upper mechanism; these internal profiles
remain approximations. Its 141-frame idle trial was stable: the faulty contact
remained 4.018 mm backed out and the other two stayed seated. The assembly mass
is 15.273 g, including three 120 mm wires. The companion
`probe_connector_three_lead_retention.py` applies up to 10 N to each seated
contact in an explicitly clamped component fixture. Both seated contacts stayed
latched through the 10 N load and returned to within 0.151 mm of their nominal
seat after unloading, with the retainer open throughout. This checks the primary
lances at a modest load; it is not a robot demonstration or the closed-retainer
90 N retention check. `probe_connector_three_lead_reseat.py` tested the robot
reseating motion with all three straight, parallel leads present. It failed:
the primary contact remained 3.991 mm backed out after the motion, while both
neighbors stayed seated. The successful single-lead demonstration therefore
does not establish access in the populated connector.

The [IntC manufacturer specification](https://48ugv.com/storage/Datasheets/IntC%20front%20panel%20module.pdf)
documents a four-position 33472-0401 CAN connector with ground, CAN-minus,
CAN-plus and one unused cavity. This establishes a populated robotics example,
but does not specify the modeled wire gauge, branch length or service access.
The neighboring conductors must remain present in a full repair demonstration;
selective access to the faulty wire has not yet been demonstrated.

The part-number relationship is documented in [Molex change notice
10692736](https://www.mouser.com/PCN/Molex_10692736.pdf), dated 22 June 2015:
the 2-by-2 receptacle family changed from 33472-04xx to 33472-40xx when its
rear mat-seal holes were revised for smaller ISO wire. This supports using
33472-4001 as the successor to the IntC document's 33472-0401. It does not
establish the actual harness branch lengths or service pose. The present
prototype still omits the mat seal's physical constraints and resistance;
the notice does not provide a force curve for calibrating those effects.

`detent.py` evaluates the same two spring wells at the physics rate. Earlier
probes selected the well only at the 20 Hz action rate, which could delay the
transition past the midpoint and require excess force. The 8 kN/m stiffness,
6.2 Ns/m damping, 40 N effort limit and 5 mm travel are unchanged. New seated
and blocked-contact fixtures completed at 2,400 Hz. With 25 N applied and then
removed, the seated-contact fixture closes to 5.000 mm and stays closed; the
backed-out contact blocks travel at 1.041 mm. The well changed once in the
positive case and never in the negative case. These are fixed-housing component
checks, not robot repair demonstrations. The model remains an approximation,
without a measured manufacturer force curve.

The upward robot press with this physics-rate detent reproduced the earlier
grasp failure before the well transition: maximum sampled travel was 1.989 mm,
with no well switch. The right-hand grasp guard stopped both arms when the
housing slipped. A reversed starting layout permits a reachable downward press;
its current trial instead triggers the screwdriver's 5 mm slip guard. Neither
press establishes full closure by the robot.

`articulated_tool.py` preserves the source SimReady tool's bodies, joints,
materials and SDF collisions. It rebases the physical root body so Arena's reset
preserves the requested asset-origin placement. Source circlip pliers remain
stable at idle, with a computed total mass of 237.8 g. Their authored angular
spring value converts to 57.296 Nm/rad per hinge. Squeezing the handles opens
the tips; this is a circlip tool, not a needle-nose plier. Current light and firm
pickup attempts fail to sustain a lift. A reversed starting orientation followed
by a jaws-down turn also fails: contact is lost at the start of the lift. This
tool has not grasped a wire, and its source spring properties are not hardware
calibration. The pickup probe's logged jaw markers use an incorrect longitudinal
offset; they must not be interpreted as measured physical tip clearance. Root
height and finger-contact diagnostics establish the pickup failure independently.

The separate, full-size SimReady long-nose tool has a passive hinge, with no
authored opening spring. It stays open on the bench at a 7 degree initial angle.
The robot can close its handles, lift the closed tool, hold it for three seconds,
lower it and release it. Its root rises about 59 mm during an 82 mm TCP lift, so
this test does not establish a rigid or precision grasp. A new component trial
loads the faulty wire between the tips while the tool remains on the bench,
using a 1.5 degree opening to limit the jaw sweep near the neighboring leads.
All three leads remain present. That trial reseated the contact before the
handles closed, during wire contact with the bench or tool, and then stopped
after the housing rotated 14.1 degrees in the grasp. It does not demonstrate
selective wire pinching. The simplified wire bending, locking finger and omitted
rear mat-seal mechanics need assessment before that shortcut can be interpreted
as a credible repair. No mandatory tool sequence is justified by this trial.
Every component probe continues to report zero task score.

The downward press reached only 0.233 mm of retainer travel before its material
marker moved 5 mm relative to the gripper body. Diagnostic logs show that most
of this movement is across the closing direction. Actual finger poses show
unequal openings: immediately before the stop, a sample with 3.893 mm of
movement relative to the nominal TCP has only 0.288 mm relative to the actual
finger midpoint. The earlier measurement includes movement of the jaw center
relative to the palm. A new probe measures translation relative to the actual
fingers while retaining the 5 mm / 12 degree stops and stock robot settings.
Its press result is still pending.

A tool-only collision comparison picked up the convex screwdriver, but the
authored SDF tool rolled away before the same fixed pickup motion arrived.
Those runs do not establish which collision representation gives better grasp
friction, because they did not produce equivalent starting grasps.

In a fixed-housing diagnostic elevated above the bench, a 0.05 N/s axial ramp
applied directly at the terminal first recorded seating at 0.3375 N. The same
ramp applied through the last wire segment reached 0.4 N, bent the wire and
left the terminal 3.782 mm backed out after unloading. This measures the current
model's load transfer, not the hardware's insertion force or seal friction.
Both probes retain all three leads and report zero task score.


The actual-jaw-center press reaches 2.4023 mm sampled retainer travel, then the
right housing-grasp guard trips at 24.0119 degrees of rotation and 2.6647 mm
translation. The retainer returns to its open state after the arms stop. An
earlier parser missed the differently named BIMANUAL_PRESS_ABORT message and
incorrectly attributed reopening to planned withdrawal. Extending the hold
reproduces the same grasp failure. Neither run is a full connector repair.


The longer-hold trial repeats the same right-housing rotation abort and was
stopped after the controller froze. `probe_connector_press_tpa_centered_housing_grip.py`
shifts pickup 12 mm toward the side flange center and 6 mm lower, with the
subsequent TCP targets adjusted for that grasp. Stock robot geometry, gains,
force caps and the 5 mm / 12 degree slip guard are unchanged. Nominal trajectory
IK is within 0.19 mm and 0.071 degrees; runtime contacts are not yet validated.
The independent bench-reseat feedback trial stops descent when the primary
terminal's observed position reaches the seat, then lifts the housing. Both
remain component diagnostics with scoring disabled.

The first bench feedback threshold (+0.2 mm axial offset) stops with the
locking finger still at about 2.48 degrees. The latch-feedback variant instead
waits for axial offset at most -0.1 mm and finger deflection at most 1 degree
before lifting. These are controller observations, with no object-state writes;
retention remains to be tested. The prepared open/closed retention fixtures
apply a gradual 5 N pull to the terminal root with a clamped housing. They are
mechanical diagnostics, not robot demonstrations or manufacturer calibration.

The latch-feedback bench diagnostic completed: at 11.9 s the primary contact
reached -0.151 mm axial offset and its locking finger returned to 0.050 degrees.
After lifting the housing 35 mm and holding, the contact remained at -0.166 mm
and the finger at 0.091 degrees. Separate clamped-fixture trials retain an
initially seated contact through a gradual 5 N root pull. With the secondary
retainer open, finger deflection reaches 3.754 degrees; closed, it reaches
0.409 degrees. These checks verify the reduced model's contact path, not the
manufacturer's retention rating or a complete robot repair.

`probe_connector_bench_and_press.py` combines three-lead bench reseating and
secondary-retainer pressing, using the centered flange grip and a firm clamp.
It preserves the physical three-lead scene and uses observed poses to control
ordinary robot joint commands. Both nominal arm trajectories pass reach checks
(under 0.19 mm / 0.071 degrees). Its combined runtime is not yet validated, and
its scorer remains disabled. The central 4 mm driver contact remains an
engineering diagnostic; the manufacturer's recommendation to apply even
pressure to both retainer ends is still a tool-contact fidelity question.

The centered-flange press completed 840 control steps without an abort. The
retainer first reached 4.95 mm at 29 s and remained at least 4.999998 mm closed
from 35 to 42 s, when every sampled tool-to-housing contact count was zero
(samples every 0.5 s). Maximum housing grasp drift was 0.723 mm / 0.618 degrees.
The housing remained held and the terminal began seated; this is a component
check, not a completed repair. Both open and closed retainer fixtures retained
an initially seated terminal through a 10 N root pull, with maximum locking
finger deflections 3.754 and 0.409 degrees respectively. The PDF's electrical
continuity-through-three-flexes criterion remains unimplemented.

The combined three-lead trial observed primary seating at 20.95 s. The prepared
`probe_connector_bench_press_release.py` extends the bench phase by two seconds
so the subsequent lift has three seconds, then adds placement, tool release and
four seconds with the hands withdrawn. It is now running. Separate mechanical
flex fixtures apply three lateral endpoint cycles through a force-limited
virtual test-machine spring; these do not measure electrical continuity.

Both three-flex fixtures completed with approximately +/-15 mm actual wire-tip
excursion per cycle, while the virtual spring target moved +/-20 mm. Peak
spring loads were 0.052 N closed and 0.050 N open; neither terminal withdrew.
These flex checks are separate from the 10 N axial pull checks. A matched
4 mm-backed-out pull baseline withdrew a further 2 mm by 0.57625 s, under
a peak applied load of 0.3792 N before the physics-rate cutoff. No electrical continuity or material-damage result is claimed.


The first combined reseat/press sequence lost the tool grasp at 12.05 degrees
before closing the retainer. The longer-lift sequence tests whether lifting
clear of the bench before pressing resolves this failure; no success is assumed.

The manufacturer's preferred continuity-test method uses a 0.50 mm rounded
POGO-72J-4 pin on the terminal's front edge, rather than inside its spring-contact
throat. A passive spring fixture uses ECT's standard 1.53 ozf preload and 4 ozf
force at 4.24 mm travel; interpolation, moving mass and damping are approximations.
Both first fixture comparisons loaded plastic, with no sampled metal contact.
A contact-location diagnostic identified an inner shoulder corner. An analytic
rounded-pin collider comparison keeps the same pin diameter and 0.60 mm opening.
None of these results establishes electrical continuity or completes the task.


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

With corrected colliders, the initially seated contact survives the 10 N
open- and closed-retainer pull diagnostics. The corrected flex comparison
records no missed metal contact in 7,201 samples for the seated contact,
versus 6,748 missed samples for the backed-out contact. However, contact-point
logs show that the seated probe slips past the front edge and loads an internal
wall sideways. These numbers therefore do not validate the preferred probe
test or electrical continuity. A separate fixed-axis fixture diagnostic targets
the observed front sheet edge and reports terminal-local contact coordinates.
The family's 4.02 mm probe-row layout is not verified for this reconstructed
2-by-2 housing. Full repair, correct test contact and damage handling remain
unfinished.

The fixed-axis probe diagnostic now contacts the front face at terminal-local
y = -8.000 mm, with sampled metal force 1.079–1.093 N and no loaded plastic
contacts. The fixture axis is z = 2.45 mm in this reconstruction. This short
stationary result still requires the matched flex and backed-out comparisons;
it does not establish electrical resistance or complete repair.

The subsequent aligned flex trial failed: the seated contact lost 143 of
7,201 samples, with the first loss reported at 4.55 s. The backed-out contact
lost all 7,201 samples. A stationary comparison using the original housing's
full-width front shoulders also makes correct front-face contact; its flex
comparison subsequently completed with no missed or invalid samples out of
7,201, minimum axial metal force 1.049 N, and no loaded plastic contact.
The matched backed-out control also completed: all 7,201 samples lacked
metal contact, with no nonfinite samples. The seated run's 13 logged loaded
contact points during flex remain at terminal-local y = -8.000 mm (within
0.00011 mm), with axial normal magnitude at least 0.99915.
This supports retaining the original shoulders. Neighboring circuits are
being tested separately; the actual repaired specimen still needs this test. These are mechanical contact checks, not electrical resistance
measurements, and the complete repair task remains unvalidated.


The earlier front-edge contact result is insufficient for robust access: the
nominal original housing shoulder masks that edge, and its exposure depended
on slight terminal tilt. The drawing also shows a folded internal contact spring
missing from that hollow-box prototype. The current centered-throat diagnostic
adds a reduced passive spring using the drawing's 0.30 mm C19025 sheet and
contact location. Undimensioned profile dimensions and effective compliance
remain estimates, not manufacturer CAD or a measured force curve.

The alternative POGO-1 spherical probe is 0.64 mm diameter, centered on the
nominal cavity. Its standard preload is 0.62 ozf, with 4 ozf at 4.24 mm travel;
the implied linear spring is 221.624 N/m. The original housing shoulders remain.
Three stationary 120-step controls completed: the seated primary and lower-row
contacts each maintained metal contact through all 7,201 measured physics steps,
with no loaded plastic contact; the 4 mm backed-out primary had no metal contact
in all 7,201 samples. No external wire flex was applied. These are mechanical
probe-access controls, not electrical resistance measurements or completed task
validation. The remaining upper-left seated cavity also completed with metal
contact in all 7,201 samples, minimum metal force 0.974 N and no loaded plastic
contact. All three populated cavities now have stationary access evidence;
the updated folded-spring geometry still needs contact checks during wire flex.

A matched timestep study keeps this geometry and the passive detent unchanged.
The 480 Hz fixed-housing closing coupon reaches 5.000 mm and remains closed
after its 25 N test load is removed. However, its separate three-flex probe
loses metal contact in 695 of 1,441 samples and ejects the primary lead. The
physics-rate cutoff removes the imposed wire load once terminal backout exceeds
2 mm. This failure rules out using the successful lock-position result alone
to justify 480 Hz for the task. The 2400 Hz closing coupon also completes:
sampled retainer travel agrees within 0.011 mm, and both remain at 5.000 mm
after unloading. This supports the fixed closing coupon only. The matched
2400 Hz wire-flex comparison also completed: all 7,201 measured physics
steps retained metal contact, minimum metal force 0.840 N and no loaded
plastic contact. Actual wire-tip excursions are approximately -15.20 to
+15.26 mm in each of the three cycles, rather than the commanded ±20 mm.
The terminal remains seated and the retainer closed after settling. This
separates the unstable 480 Hz result from a stable diagnostic baseline;
the matched 1200 Hz trial finishes without lead ejection but is not equivalent.
Its probe has at most 0.009 mm compression and up to 0.368 mm sampled metal
penetration, while the 2400 Hz probe compresses 1.74–1.77 mm with at most
0.025 mm penetration. The 1200 Hz normal is almost entirely lateral and the
contact location has moved past the beam surface used at 2400 Hz. Therefore
metal-contact counts alone do not justify lowering the physics rate. These
results do not establish the response of a real mat seal or validate a
complete repair.

The block-supported robot press stopped before closing the retainer because its
left wrist reached the unchanged joint-limit guard. A new controller places the
same loose support block 100 mm toward the right; offline kinematics show margin
for the observed tilted screwdriver. Physical closure remains unproven. No robot
force, joint limit, hardware clearance or retainer stiffness was changed.

The selected 33472-4001 drawing, general note 1a (combined PDF page 4),
directs assembly and service to AS-33472-100. Revision D7 explicitly gives
5 mm prelock-to-final-lock travel on pages 13, 15 and 30, including the
receptacle assembly. The model retains this source-supported travel. The
earlier approximate 3.75 mm image inference mixed different section planes;
it is not a basis for changing travel. Exact final-lock recess geometry and
mat-seal forces remain unresolved approximations.

The shifted-right support press also stops at the unchanged left-wrist
joint-limit guard, at 41.90 s. Sampled retainer travel peaks at 0.969 mm
with 7.89 N sampled driver/retainer force, then returns toward open. No
driver/block reaction was measured. The failure is preserved rather than
reported as a completed repair. An offline solid-geometry audit finds slight
gripper interference at the block edge. A fresh controller changes only the
final driver-butt target by 25 mm toward the near edge of the same block;
41 sampled approach poses have at least 5.19 mm gripper/block clearance,
but physical success remains unproven. All hardware and robot limits remain
unchanged.

The first near-edge attempt ended in a native simulator crash after 343 recorded
frames, before the support/press phase. Its partial video and logs are preserved;
it had no mechanical abort or closure observation. An identical frozen retry
is running. This infrastructure failure does not assess the proposed approach.

The current product specification, PS-33472-000 F24, gives a 30 N maximum
terminal-to-housing insertion force and a 90 N minimum retention force. Its
2x2 receptacle exception permits up to 90 N for TPA insertion and extraction.
These limits are not measured force curves and do not calibrate the omitted
mat seal. The selected BOM identifies inherently lubricated silicone but gives
no hole interference or hardness. A downloaded 2016 reference STEP contains a
defeatured exterior solid, not usable internal seal or lance surfaces. Its
overall axial extent must be reconciled with its TPA configuration: drawing
dimension E is a body-length maximum, while G separately specifies prelock
projection. No hardware geometry was changed on the basis of that ambiguity.

A separate in-place retention diagnostic has been prepared but not installed
or simulated. It would load the actual released repaired specimen with balanced
force and moment, record unloading and all terminal displacement, and reject
missing release evidence. It is not a replacement for physical pogo/contact
continuity or a manufacturer qualification test. The complete task remains
unvalidated while seal response, full physical repair and the actual-specimen
load/flex assay are unresolved.

Work is parked after the final edge-supported press completed 1,001 recorded
frames with score zero. Maximum sampled TPA travel was 1.14037 mm, with no
closure; travel returned to 0.03315 mm. The unchanged right-arm joint guard
stopped withdrawal at 46.65 seconds. The final matched 4,800 Hz contact
diagnostic retained all metal samples, but it does not resolve the missing
seal behavior or establish a complete physical repair. No additional component
variants are planned without new actual seal/internal hardware evidence.
