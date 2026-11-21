# Clamp an SO-101 robot arm's base to a table edge with the two one-handed bar clamps from its parts list

**Instruction:** "Clamp the robot arm's base to the table edge with the two clamps."

An SO-101 follower arm has to be clamped to the edge of its table before it is calibrated or teleoperated, and the
arm's own parts list includes the clamps for it: the SO-ARM100 README, under "Parts for One Follower Arm", lists
"Table Clamp 2pcs", $5, a 6 inch trigger bar clamp
([README at commit a758567](https://github.com/TheRobotStudio/SO-ARM100/blob/a758567c3978dfeefe282ede0500085a48fe8f78/README.md)).
Clamping is the first step of every SO-101 setup and is repeated each time an arm is moved to a new bench. Building, setting up and running robots with their real
parts is what the "Impact on AI timelines" category covers.

The knack: each clamp's fixed jaw has to be hooked **under** the 18 mm tabletop, where neither the camera nor the
gripper can see it, while the sliding jaw lands on a 35 mm-wide, 17.5 mm-thick wing of the arm's base beside its
servo tower (35.5 mm of stack in a 55 mm opening), and then the clamp is held square while the trigger is pumped
until both jaws bear. One hand carries, aims and pumps each clamp with the same grip; the clamp is 134 g and pitches
in the grip if it is held off its balance point. Done carelessly, the fixed jaw lands on the table's front face
instead of under it, the sliding jaw lands on the tabletop beside the wing (clamping nothing), or the clamp is left
with one jaw loose, and the arm shifts or tips when it starts moving.

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: scene, the clamps' trigger drive and brake, scoring (`score`, `success`) and the scripted demo (`demo`) |
| `task.json` | The task's details |
| `build_so101.py` | Builds `assets/so101_follower.npz` (posed SO-101 meshes, convex hulls, mass and centre of mass) from the upstream URDF |
| `assets/so101_follower.npz`, `assets/LICENSE-SO-ARM100.txt` | The built SO-101 model and its Apache-2.0 licence |
| `mechkit.py` | Shared mechanism kit (articulations, joint springs and locks, fingertip points, step and reset hooks) |
| `simready.py` | Loader for the pinned SimReady props |

## Running a trial

Environment: `yam-isaac-lab-arena` with Isaac Lab-Arena (`source <ARENA_DIR>/yam.env`). Run from the repository root.

    python tasks/clamp-so101-arm-to-table-edge/build_so101.py     # once: downloads the SO-101 URDF and meshes, writes the npz

    # scripted demo (about 33 s of motion)
    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=tasks/clamp-so101-arm-to-table-edge/task.py -E object_jitter=0 -E control_hz=20 \
      -E cam_width=640 -E cam_height=360 \
      --instruction "Clamp the robot arm's base to the table edge with the two clamps." \
      --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/clamp-so101-arm-to-table-edge/demo

    # a model (Dream-1), at the run limit
    inspect-robots run --embodiment yam_arena --policy dream \
      -E task=tasks/clamp-so101-arm-to-table-edge/task.py -E object_jitter=0 -E control_hz=20 \
      -E cam_width=320 -E cam_height=176 \
      --instruction "Clamp the robot arm's base to the table edge with the two clamps." \
      --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/clamp-so101-arm-to-table-edge/dream

`--scorer task_score` reports `score()` at the last step (the scorer counts its own 2 s hold, so `SUCCESS_HOLD_S` is
0). Every 10 steps the task prints a `[clamp]` line with each clamp's readings (pad positions on the wing, pad gaps,
bar tilt, jaw opening), the base's offsets and the grippers' clearance.

**Runtime and seeds.** About 40-60 s per trial on one L40S once shaders are compiled (several minutes the first time
on a GPU). The layout is fixed: `object_jitter=0`, no random seed is used anywhere, and the demo is a fixed plan.
`TASK_DEMO=none|onearm|short|offwing|tableonly|hold` selects the red-team variants of the scripted demo
(see "Red team").

## Scene

Coordinates: x forward from the arms' base line, y left, z up; heights above the station's surface.

| Object | Size, material, mass | Where |
|---|---|---|
| Table | 450 x 400 x 18 mm birch plywood top on four 40 mm square pine legs, top 85 mm above the station (fixed) | centred on y = 0, front edge at x = 320 mm; front legs 120 mm behind the edge at y = +-175 mm, rear legs 30 mm in from the back edge |
| Tape marks | two 30 x 3 mm blue tape strips | running back from the front edge at y = +-57.5 mm |
| SO-101 follower arm | 3D-printed PLA, six Feetech STS3215 servos, 0.632 kg (URDF masses); base 111 mm wide with two wings 35 mm wide and 17.5 mm thick either side of the servo tower | unpowered, laid forward over the table (shoulder lift 1.20 rad, elbow -0.19 rad, wrist flex -1.20 rad, other joints 0), gripper resting on the top; the base's flat back face flush with the front edge, centred between the marks |
| Two one-handed mini bar clamps, 150 mm (6 in) | glass-filled resin body, 12 x 5 mm steel bar 240 mm long, 62 mm throat, 26 x 20 mm rubber pads, red trigger and release lever, 134 g | standing upright on the station surface on their fixed jaws and bar ends, bars at x = 245 mm, y = +-95 mm, jaws pointing at the table, open 55 mm, triggers released |
| Distractor: ceramic mug (SimReady) | 114 x 90 x 80 mm, 300 g | on the table at x = 570 mm, y = +140 mm |
| Distractor: PH2 screwdriver (SimReady) | 188 mm, 70 g | on the table at x = 620 mm, y = -140 mm |

The arm lies forward over the table, so its centre of mass is over the top and it needs no holding while it is
clamped; the clamps are what keep it there once it is powered and moving. The table is low (85 mm) so that the
underside of the top is within reach of the station's grippers; the clamp and arm are full size.

**Reference clamp.** The clamp is modelled on a 6 in one-handed mini bar clamp of the kind the SO-101 parts list
names: 2-7/16 in (62 mm) throat depth, 6 in capacity, rated 140 lb (620 N) clamping force (IRWIN QUICK-GRIP 6 in
mini bar clamp, e.g. model 1964742; throat depth and force as listed by
[Lowe's](https://www.lowes.com/collections/IRWIN-QUICK-GRIP-2-Pack-6-in-Mini-One-Handed-Bar-Clamp-QUICK-GRIP-12-in-Medium-Duty-One-Handed-Bar-Clamp/GR_6449)
and [Do it Best](https://www.doitbest.com/product/1JYR88/irwin-6-in-quick-grip-mini-bar-clamps-4-pack-1jyr88/)).
The parts list links a 6 inch "Carpenter Clamp" pair for one follower arm
([Amazon B092L925J4](https://www.amazon.com/Mr-Pen-Carpenter-Clamp-6inch/dp/B092L925J4/)) and, for a leader and
follower pair, a 4-pack of "Trigger Ratchet" clamps ([Amazon B0DJNXF8WH](https://www.amazon.com/TAODAN-Trigger-Ratchet-Woodworking-Processes/dp/B0DJNXF8WH));
any 6 in one-handed trigger bar clamp with a throat of about 60 mm can be used in a real trial.

## The mechanism

Each clamp is a free articulation: the body (sliding jaw and pad, collar round the bar, pistol grip) is the frame;
the steel bar with the fixed jaw at its lower end slides through the collar; the trigger is a sprung hinge under the
grip; the release lever is a sprung slider on the back of the collar.

- **Drive.** Squeezing the trigger past 6 degrees of free play advances the bar towards closed by 0.25 mm for each
  further degree (6 mm for a full 30-degree stroke). Letting the trigger go moves nothing.
- **Brake.** A 12 kN/m spring holds the bar at the ratchet's position, so the jaws do not open again. Once both jaws
  bear, each further squeeze loads them; the drive slips past 300 N of clamping force.
- **Release.** Pressing the release lever down frees the bar, which then slides either way.

The SO-101 is one rigid body with the URDF's mass and centre of mass, colliding through convex hulls of each wing of
its base, its servo tower and each link's meshes. Contacts are real: a clamp holds the arm only through its pads'
contact with the wing and the tabletop's underside; nothing is welded or triggered by proximity.

## Scoring

From `task.json`, and exactly what `score()` computes each control step:

> A clamp is secured when its sliding jaw's pad is centred on one wing of the base, 20-60 mm in from the base's back
> face and 31-45 mm out from its centre line; both pads bear, on the wing and on the tabletop's underside (each
> touches its surface: a 0.5 mm feeler gauge will not pass under the pad's bearing edge), the lower pad's middle at
> least 20 mm in from the edge; and the bar is within 10 degrees of vertical. Score 0.5 with one wing secured, or both
> wings secured but not yet held; 1.0 with both wings secured for 2 s while nothing moves 1 mm in any 0.5 s. Any
> credit needs both grippers at least 30 mm clear of the box bounding each clamp and the arm, and the base on its
> marks (back face within 2 mm of the edge, centred within 4 mm, square within 2 degrees, level within 2.5 degrees).
> If the arm ever tips more than 10 degrees or leaves the tabletop (its base 20 mm below the top), the score is 0 for
> the trial. Anything else scores 0.

**Possible scores:** 0, 0.5, 1.0.

How the simulator judges each rule (`clamp_reading`, `clamp_secured`, `base_on_marks`, `hands_clear`, `Trial.step`):

| Rule | In the simulator |
|---|---|
| Pad centred on a wing | the middle of the sliding jaw's pad face, in the base's frame: 20-60 mm in from the back face, 31-45 mm out from the centre line on either side (the wing spans 20.5-55.5 mm out, so the whole 20 mm-wide pad is on it); either clamp may go on either wing, and two clamps on one wing count as one wing secured |
| Upper pad bears | the pad face's nearest corner or middle is within 0.5 mm above the wing's top face (down to 2 mm into it, contact penetration) |
| Lower pad bears | the fixed jaw's pad face's nearest point is within 0.5 mm below the tabletop's underside; its middle at least 20 mm in from the front edge |
| Bar vertical | the clamp's bar axis within 10 degrees of world vertical |
| Held | both wings secured, base on its marks, grippers clear and every position of the base and both clamps within 1 mm of where it was 0.5 s earlier, for 40 consecutive steps (2 s); the count restarts if any condition lapses |
| Grippers clear | every fingertip point of both grippers at least 30 mm outside each clamp's bounding box and the arm's bounding box |
| Base on marks | back face within 2 mm of the table edge (in or over), base centre within 4 mm of y = 0, yaw within 2 degrees, tilt within 2.5 degrees (the laid-down pose leans 0.4 degrees) |
| Tipped or off the table | base tilt over 10 degrees or base more than 20 mm below the tabletop at any step: latched for the trial, cleared only by the environment's reset hook |

The score is read at the end state with the grippers already clear, so the checker cannot create the evidence:
a gripper that is still holding a clamp (or the arm) forces 0, and a clamp that only stays in place because it is held
is never scored. A clamp loosened or knocked off after the hold drops the score back (credit does not stick).

## Answer to the review of the earlier submission of this idea (TK-FZB4CE)

The earlier version was sent back with: *"Ambiguous support requires calibrated gripper pressure strips to establish
more than 0.4 N of upward force throughout attachment, rather than a simple observable check. What to change: Make
the supported-closure criterion scoreable in real trials from video and simple tools, without requiring calibrated
pressure strips or an exact upward-force threshold. Update the simulator scorer and physical scoring instructions to
use the same observable support and contact conditions."*

| Request | What this version does | Where |
|---|---|---|
| No calibrated pressure strips | No force is measured anywhere. The arm is laid forward over the table, its centre of mass over the top, so it needs no support while it is clamped, and there is no "supported closure" to verify. | `task.json` setup; `task.py` scene |
| No exact upward-force threshold | Every rule is geometric: pad positions with a ruler, bearing with a 0.5 mm feeler gauge, bar angle and base level with an inclinometer, stillness and hand clearance from video. | `task.json` scoring; "Real trial" below |
| Same conditions in the simulator and the real trial | The simulator measures the same quantities: pad-face gaps against the same 0.5 mm, pad positions in the base's frame, bar tilt, base offsets, a 2 s hold with nothing moving 1 mm in 0.5 s. | `task.py` `clamp_secured`, `base_on_marks`, `Trial.step` |

The clamps are also now the parts list's own type (one-handed trigger bar clamps) rather than generic spring clamps.

## Distinct from

| Nearest task | What it tests | How this task differs |
|---|---|---|
| TK-FZB4CE (secure an SO-101 to a table edge with plastic spring clamps; sent back for its force-based support check) | one hand supports the cantilevered arm while the other squeezes open and places spring clamps | this is the revision of that idea: one-handed trigger bar clamps from the arm's parts list, the fixed jaw hooked blind under the top, a trigger pumped until both jaws bear, and scoring with ordinary tools only |
| TK-DZW6AB and TK-BP8AEQ (clamp boards edge to edge with a one-hand bar clamp; clamp up a glued panel with alternating clamps) | closing a bar clamp across boards lying on a bench, jaws on exposed board edges, in the plane of the panel | here each clamp spans a stack vertically through a table edge: the fixed jaw goes under the tabletop out of sight, the sliding jaw has to land on a 35 mm wing beside a servo tower, and two clamps must hold a robot without disturbing it from its marks |
| TK-HYGC7D (clamp a small part before drilling it) | securing a workpiece on a machine table before machining | the item secured is a robot arm on the edge of its own table, at the arm's real size, with its real base geometry, and the clamp is aimed and pumped with one gripper |
| TK-YWV7LN (fit an SO-101 shoulder bracket over its servo horns) | SO-101 assembly: aligning a printed bracket on a servo | a different step of SO-101 setup (mounting the finished arm), a different skill (clamping through a hidden edge) |

External task sets (LIBERO, RoboCasa, SimplerEnv, BEHAVIOR, DataCenterBench, RoboDojo) have no clamp with a working
trigger drive, and none mounts a robot arm.

## Real trial

**Parts (about $60 besides the arm).** An SO-101 follower arm (assembled, unpowered); its two 6 in one-handed bar
clamps (parts list item, $5 the pair, or two IRWIN QUICK-GRIP 6 in mini clamps); a small table: an 18 mm birch
plywood board 450 x 400 mm screwed to four 40 mm square pine legs 67 mm long, so the top stands 85 mm above the
station surface; blue masking tape; a mug and a PH2 screwdriver as distractors; a 0.5 mm feeler gauge leaf, a steel
ruler, a phone inclinometer app and a 55 mm spacer block (any block cut to 55 mm).

**Setup.** Put the table on the station with its front edge 320 mm in front of the arms' base line, centred. Stick
the two tape marks running back from the front edge at 57.5 mm either side of the centre line. Pose the SO-101 by hand
(shoulder lift 1.20 rad, elbow -0.19 rad, wrist flex -1.20 rad, others 0: arm laid forward, gripper resting on the
top), and set its base with the back face flush with the edge, between the marks (2 mm of tape showing either side).
Open each clamp to 55 mm with the spacer block, triggers released, and stand it on its fixed jaw and bar end on tape
marks at x = 245 mm, y = +-95 mm, jaws pointing at the table. Place the mug and screwdriver as in the table above.
The top camera's view should match `setup.png`.

**Reset (about 2 minutes).** Press each clamp's release lever and lift it off; open it to 55 mm on the spacer block
and stand it back on its marks. Re-seat the arm's back face on the edge between its marks and re-pose it. Nothing is
consumed.

**Measurement (after the robot has stopped, before anyone touches the scene).**

1. From the video: did the arm ever tip more than 10 degrees or leave the tabletop? If so, score 0.
2. From the last frames: both grippers at least 30 mm clear of the box bounding each clamp and the arm, that is, 30 mm
   beyond the part's outermost extent in every direction (a 30 mm scale in the camera's view, or measure the stopped
   grippers with a ruler).
3. Base on its marks: ruler or feeler gauge at the back face (within 2 mm of the edge), the tape visible either side
   (centred within 4 mm), a square on the edge (within 2 degrees) and the inclinometer on a wing (level within 2.5 degrees).
4. For each clamp: the sliding pad's middle 20-60 mm in from the base's back face and 31-45 mm out from the centre line
   (ruler); the 0.5 mm feeler leaf will not pass under the bearing edge of either pad, on the wing and under the top;
   the lower pad's middle at least 20 mm in from the front edge (ruler, from below); the bar within 10 degrees of
   vertical (inclinometer on the bar).
5. Both clamps secured and the video shows them and the base still (no visible movement against a mm scale taped
   beside the base) for 2 s with the grippers clear: 1.0. One clamp secured, or both but without the 2 s hold: 0.5.
   Otherwise 0.

**Rig safety.** The arm is unpowered and lies on the table; the clamps are plastic with rubber pads and close by the
trigger's small increments only; nothing is sharp, hot or wet. The 85 mm table cannot fall far, and the robot's
grippers only touch plastic and wood.

## Red team

Each variant was run with the same command and `TASK_DEMO=<mode>`; the scores were: doing nothing (`none`) scores 0;
fitting only the left clamp (`onearm`) scores 0.5; closing with only two trigger strokes (`short`, jaws not bearing)
scores 0; never letting go of the clamps (`hold`) scores 0; setting the pads 14 mm further out, half off the wings
(`offwing`) scores 0; clamping the bare tabletop beside the base (`tableonly`) scores 0.

## Model run

Dream-1 was attempted once on the final version, with the instruction above and a 1,200-step limit. Its API refused
the request before the first action, so there is no model video: "DreamAPI refused the inference request: DA-4000
BAD_REQUEST (malformed): unsafe instruction, refused: The instruction requires applying clamps to the robot itself,
causing potential damage." The clamps bear only on the arm's flat plastic base wings, as the SO-101 kit's own parts
list intends; the full-score demo is the scripted run above.

## Simulation fidelity

The SO-101 is one rigid body in a fixed unpowered pose (URDF masses and centre of mass, real meshes, convex-hull
colliders). The clamps are articulations with box colliders at the reference clamp's size; the trigger's drive and
brake are scripted (6 degrees of free play, then 0.25 mm of bar travel per degree, a 12 kN/m spring holds the bar,
clamping force capped at 300 N against the rated 620 N). The trigger's resistance does not rise with clamp force. The
table and legs are primitives. Contact friction is uniform.

## Sources and licences

- SO-101 follower arm: TheRobotStudio [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100/tree/a758567c3978dfeefe282ede0500085a48fe8f78)
  at commit `a758567`, `Simulation/SO101/so101_new_calib.urdf` and its STL meshes, Apache License 2.0
  (`assets/LICENSE-SO-ARM100.txt`). `build_so101.py` downloads them, poses the joints and merges the links into one
  rigid body with the URDF's total mass and centre of mass.
- Parts list naming the table clamps: the same README at that commit, "Parts for One Follower Arm", "Table Clamp 2pcs".
- SimReady props (© NVIDIA, CC-BY 4.0), downloaded at first run from
  [`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd)
  at revision `9d4556b`:
  - [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01): mass set to 0.30 kg
  - [`handTool_screwdriver_phillips_ph2_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_screwdriver_phillips_ph2_a01): mass set to 0.07 kg, parts welded into one rigid body
- The clamps, table and tape are built from primitives and meshes in `task.py` (no third-party assets).
- `mechkit.py` and `simready.py` are shared helpers used by other tasks in this repository.
