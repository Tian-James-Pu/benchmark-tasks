# Fasten a GoPro into a helmet's buckle: line up the five fingers' holes, push the thumb screw through and run it down

A GoPro HERO lies on the table beside a helmet. A GoPro curved adhesive mount is stuck on the helmet's crown, with a mounting buckle
clipped into it, and the buckle's thumb screw has been backed out so that the buckle's three fingers stand open. The robot is told:

> Fasten the GoPro to the helmet's mount with its thumb screw, facing forward and level.

To do it, one hand has to pick the camera up, turn it upright and set its two folding fingers between the buckle's three. It then
has to hold the camera there, with all five 5 mm holes lined up within about a millimetre and the fingers clear of the slots'
floor. Meanwhile the other hand pinches the screw's paddle head, pushes the screw through all five fingers to the captive nut and
turns it four turns, regripping every half turn. The first hand keeps the camera level until the screw is home, and both hands then
leave. If the camera is let go, or held a millimetre low, the camera drops to the slots' floor. The screw's tip then stops on the
camera's first finger. If the camera is let go before the screw is tight, it flops over on the loose screw.

## Run a trial

From the repository root, with the YAM simulator installed (`source ~/IsaacLab-Arena/yam.env`):

    # the scripted demonstration (about 60 s of motion; the episode ends once full credit has been held 2 s)
    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=tasks/pin-gopro-fingers-into-buckle-with-thumbscrew/task.py -E object_jitter=0 -E control_hz=20 \
      -E cam_width=640 -E cam_height=360 \
      --instruction "Fasten the GoPro to the helmet's mount with its thumb screw, facing forward and level." \
      --scorer task_score --max-steps 1400 --store-frames --log-dir ~/yam-eval/gopro-demo

    # a model (Dream-1 shown), up to the run limit
    inspect-robots run --embodiment yam_arena --policy dream \
      -E task=tasks/pin-gopro-fingers-into-buckle-with-thumbscrew/task.py -E object_jitter=0 -E control_hz=20 \
      -E cam_width=320 -E cam_height=176 \
      --instruction "Fasten the GoPro to the helmet's mount with its thumb screw, facing forward and level." \
      --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/gopro-model

`--scorer task_score` reports 1.0 once full credit has been held for 2 s (the episode then ends), and otherwise the last step's
partial credit. The scene is deterministic: with `object_jitter=0` there are no seeds to set, and every trial starts from the same
layout. Runtime: the demo's 1,280 steps took about 12 minutes on a shared GPU (the first run on a GPU adds a few minutes of shader
compilation); a 2,400-step model trial takes about twice that.

## Importance and sources

Action cameras are used on mounts (helmet, handlebar, chest), and their owners fit the camera to a mount every time they move it. GoPro alone sold about 3 million cameras in 2023 ([GoPro Q4 and 2023 results](https://investor.gopro.com/press-releases/press-release-details/2024/GoPro-Announces-Fourth-Quarter-and-2023-Results/default.aspx)).
GoPro's mounting system is two fingers on the camera interleaved with three on the mount, joined by an M5 thumb screw through
holes lined up across all five
([a description of the interface with its dimensions: fingers about 3.0 mm, gaps 3.1 to 3.5 mm, holes 5.0 mm](https://qidi3d.com/ja/blogs/print-lab/gopro-mounts-cyclists)).
GoPro sells the curved adhesive mount, mounting buckles and thumb screws used here as spares
([GoPro Grab Bag](https://gopro.com/en/us/shop/mounts-accessories/grab-bag/AGBAG-002.html), whose page also gives GoPro's helmet
guidance). The camera is the 2024 GoPro HERO: 56.6 x 47.7 x 29.4 mm and 86 g with its built-in mounting fingers
([Adorama](https://www.adorama.com/alc/tiny-but-mighty-gopro-hero-camera), which also suggests a curved mount for helmet shots), $199.99
([Digital Camera World](https://digitalcameraworld.com/news/the-new-gopro-hero-is-now-on-sale-and-i-can-see-it-being-a-viral-option)).

Why it is hard for a robot, in one sentence: the camera's 3 mm fingers sit in 3.3 mm gaps and their holes have to stay within
about a millimetre of the buckle's hole axis while the second hand pushes a 4.9 mm screw through 5.2 mm holes; the camera is free to drop
2 mm to the slots' floor (and does as soon as it is let go). After that the screw needs eight half turns with a regrip after each,
while the camera is still held level. Neither hand can do the other's job, and the order matters.

### Distinct from

| Nearest task | What it tests | Why this is different |
|---|---|---|
| TK-XDB5VZ (accepted): release a camera from a cam-clamped quick-release mount | Opening a cam lever and lifting a camera plate off | No interleaved fingers, no holes to line up, no pin or screw to pass through; it is a release, not a fastening |
| TK-QJ8724 (accepted): fit a U-joint cross through its yoke ears | Pushing one journal through two ears of a shop part and centring it | One part is pushed through a fixed part; here one hand holds a free camera's holes on an axis while the other passes a screw through five layers, then screws it home while the camera is held level |
| TK-DAQ7SB (pending): square a phone on a mini tripod ball head | Holding a button while levelling a phone | No pin alignment, no screw; the levelling there is the whole task, here it is held during the fastening |
| TK-6UQ94P (not accepted, revising): shackle a trailer's towing lug and screw the pin home | A rigging shackle's two ears set over a fixed lug, its pin screwed into one ear | Rigging hardware over a fixed lug; here a free 86 g camera's two fingers are held between three fixed fingers (five 3 mm layers in line) while the other hand passes the screw to a captive nut, then the camera is held level through the regripped turns and its lens direction is scored. An everyday camera-mounting job, not rigging |

Popular task sets (LIBERO, RoboCasa, SimplerEnv, BEHAVIOR, RoboDojo) have no camera mounting and no task that passes a pin
through interleaved parts held by the other hand.

## The scene

All positions are (x forward from the arm bases, y to the left) in cm; heights above the table.

- **Helmet**: a yellow HDPE hard hat (NVIDIA SimReady `ppe_helmet_hardHat_a01`, 29.0 x 20.8 x 17.3 cm), fixed (taped down in a
  real trial), centred at (30, 0), brim to +x; its crown is 17.35 cm up.
- **Curved adhesive mount and buckle** (authored mesh, black plastic) on the crown: base 4.2 x 4.0 x 0.4 cm on a 1.2 mm foam
  pad; buckle body 2.6 x 2.4 x 0.8 cm; three fingers 3.0 mm thick with 3.3 mm gaps, 12 mm wide, rounded (6 mm radius) about 5.2 mm
  holes whose axis runs along y, 2.0 cm above the crown. The slots between them have their floor 8 mm below the axis. The tops of
  the fingers are chamfered 0.5 mm on the faces that meet a gap. The left outer finger carries an M5 steel hex nut (8 mm across
  flats, 4 mm thick) in a hex pocket whose floor is 0.75 mm outside the finger.
- **Thumb screw** (authored mesh): M5 x 0.8 steel thread, 20 mm long, 4.9 mm over the thread, 0.45 mm tip chamfer, under a black
  plastic neck (9 mm across, 22 mm long) and a flat paddle head (18 x 12 x 6 mm): 54 mm in all, about 7 g. These are the dimensions
  of common long GoPro-type thumb screws (GoPro's Grab Bag includes a long one); measure the one you use (see Real trials). At the
  start it is backed out with its tip 0.15 mm inside the right outer finger's hole: the shoulder under its head stands 17.15 mm off
  that finger, and the paddle lies flat, 4.7 to 5.9 cm right of the buckle's centre.
- **Camera** (authored mesh; SimReady has no action camera): GoPro HERO (2024), black, 56.6 wide x 47.7 high x 29.4 deep mm,
  86 g, lens on the front, touchscreen on the back, shutter on top. Its two folding fingers are folded out: 3.0 mm thick, 3.3 mm
  apart, 12 mm wide, with 5.2 mm holes 10 mm below the body. It lies on its right side at the start, lens to +x, top to -y,
  fingers pointing +y, with the fingers' hole axis at (24, 20), 2.83 cm up.
- **Distractors** (SimReady, loose, ignored by the scorer): sunglasses (`accessory_eyeglasses_a01`, 30 g) at (50, 24), turned
  90 degrees; a bottle of drink (`food_beverage_bottledDrink_a01`, 5.3 cm across, 17.5 cm tall, 550 g) at (47, -24); a pair of
  cloth work gloves (`ppe_gloves_cloth_a01`, 120 g) at (58, -2).

## How the simulator judges it

- **The fingers and the screw are real contacts.** The camera is one free rigid body. Each of its fingers collides through 16
  convex pieces round a 5.2 mm hole, so the 4.9 mm shank passes only through a hole that is on the axis. The shank's 0.45 mm tip
  chamfer, with the give of the hand holding the camera, leads a finger in when it is up to about 1 mm off (0.9 mm in one demo run).
  At 2 mm off and more (the camera let down onto the slots' floor), the tip lands on the finger's face and stops there (a jam),
  whatever the pushing hand does. Nothing about the jam is scripted. With the screw through, the camera hangs on it
  and turns freely about it.
- **The buckle's middle finger is fixed; its outer fingers are stiff** (100 kN/m) and can move 0.4 mm inward. They stand at rest
  until the screw is seated, then are driven inward and clamp the camera's fingers (about 25 N each). That stands in for the screw's
  tension bending a real buckle's outer fingers in, and is what holds the camera's tilt afterwards.
- **The thread is code** (each control step): the screw slides freely until its tip reaches the nut (13.95 mm of travel, the
  shoulder then 3.2 mm off the outer finger). From there it cannot be pushed further. Its axial position follows its turn, 0.8 mm
  per turn, through a stiff drive (50 kN/m). Four turns bring the shoulder against the outer finger (seated); the screw's turn is
  then held, as a finger-tight screw stops turning. Turning it back out unthreads it.
- **The scorer** reads the shoulder's standoff from the screw's travel, where the camera's two finger holes are relative to the
  screw's axis, the direction of the camera's lens, how far the camera moves during the hold, and every fingertip point of both grippers.

## Scoring

Read at the end, before anyone touches the scene.

- **0** unless the screw passes through both of the camera's fingers (both finger holes within 1 mm of the screw's axis, and the
  standoff 7.0 mm or less, which puts the tip at least 0.55 mm past the camera's second finger) and every fingertip point of both
  grippers (each finger's tip and 3 cm up it, at its middle and both edges) is at least 3 cm from the camera, the screw and the
  buckle.
- **0.25**: that, with the standoff over 3.0 mm (through the camera, not yet into the nut).
- **0.5**: standoff 3.0 mm or less (threaded at least a quarter turn into the nut).
- **0.75**: standoff 0.5 mm or less (run down).
- **1.0**: run down, with the lens pointing forward within 5 degrees of the helmet's front-to-back line, held for 2 s with the
  grippers clear and the camera still (it moves less than 1 mm and 0.5 degrees in those 2 s). The trial ends there.

Possible scores: 0, 0.25, 0.5, 0.75, 1.0. The camera on the table, an empty buckle with the screw run down, the camera in the buckle
with the screw jammed on its first finger, or a gripper still on the camera at the end all score 0. A camera fastened facing
backwards or tilted past 5 degrees stays at 0.75. There is no penalty rule: dropping the camera, regripping or retrying is allowed,
and only the end state counts.

## The demonstration

`demo()` in `task.py` is a waypoint plan from the published starting layout, played by `yam_scripted`, with one closed loop. The
left hand comes down over the camera, pinches it across its depth at its centre of mass (so that turning it does not make it pivot
between the pads), lifts it and turns it upright while moving it over the helmet, then lowers its fingers between the buckle's.
Two feed-forward offsets come from measuring earlier runs: the arm sags 2 to 3 mm holding the camera, and the front pad bears on
the lens, so the camera sits about 3.5 mm behind the pinch's centre and pitched about 2 degrees. Open loop, the held camera still
ended 1 to 2 mm low or behind. In those runs the screw jammed on its first finger, exactly as the task intends. So, for the last
3 s before the push, the left hand's held position is trimmed each step by the measured offset of the camera's finger holes from
the buckle's axis. It is then frozen once the screw reaches the camera. This reads simulator state, as a scripted demo may (a
person would use their eyes); the evaluated model gets only the camera images. The right hand pinches the paddle (lying flat)
from its outer end, waits for the camera, and pushes the screw 13.95 mm through all five fingers to the nut, twisting it 30
degrees to and fro as it goes, as a person does when a screw catches on a hole's edge. It then turns it in
half turns: turn 180 degrees, let go, turn back open, pinch again, following the screw's 0.4 mm advance. It turns nine half
turns, one more than the thread needs; the last stops at the seat, as a hand turns until the screw stops. Then the right hand
backs off, the left lets go of the camera and backs off, and the camera stays clamped, level.

Results (task_score, GPU, this version): **1.0 and 1.0** in two runs (one on each of two GPUs), each ending at step 1,279 when
full credit had been held 2 s. A third run of the same demo, with a temporary diagnostic log added (since removed), also scored
1.0 at step 1,279. Its log shows the details. The screw reached the camera's first finger with the holes up to 0.9 mm off the axis,
went through, and the hole offset fell to 0.1 mm. The thread caught at about step 445 and the screw seated at about step 1,150,
with the camera pitched about 2 degrees. Both hands were 3 cm clear from about step 1,236. Earlier runs with the left hand open loop show what a 1.5 to 4 mm offset does:
the screw's tip stops on the camera's first finger, 0.6 to 1 mm into its travel, and the trial scores 0.

Red team (each run as a copy of `task.py` with only `demo()` changed; same scene and scorer):

| Variant | What it does | task_score |
|---|---|---|
| Do nothing | Both arms stay at their ready poses for 20 s | **0** |
| One hand at a time | The left lines the camera up in the buckle, lets go and leaves; only then does the right push and turn the screw | **0**: let go, the camera drops to the slots' floor, tips over and falls off the helmet onto the table; the screw is then run down into the empty buckle |
| Careless | Both hands as in the demo, but the left lets go once the screw is at the nut, before it is run down | **0.75**: the screw is run down, but the loose camera has flopped forward and is clamped tilted past 5 degrees |

The cheapest routes are covered by these: a single hand cannot both hold the camera and push the screw, and the screw alone run
into the empty buckle earns nothing. The partial levels need the screw through both of the camera's fingers, which needs the
camera held lined up while the screw goes in.

Model run (Dream-1, the run limit of 2,400 steps, the second command above): **0**. It raised both arms over their bases and held
them there for most of the two minutes, then in the last half minute swung the left arm forward beside the helmet, near the
sunglasses; it never touched the camera, the screw or the buckle.

## Real trials

**Build.** A GoPro HERO (2024), a GoPro curved adhesive mount with a mounting buckle and a long thumb screw (all in GoPro's Grab
Bag), and a hard hat (or any helmet: the curved mount is made for helmet shells). Stick the mount on the crown, front to front, and
let the adhesive cure for 24 h. Tape the helmet to the table at (30, 0), brim away from the robot. Put tape marks for the camera's
outline at its start, and for the three distractors.

**Preparation, each trial.** Unscrew the thumb screw and draw it back until the shoulder under its head is 17 mm from the outer
finger (a ruler; the tip is then just inside that finger's hole), paddle flat. Check that it does not fall out: the tip has 3 mm of
the finger's hole to sit in, and the real screw may droop a few degrees, which the simulation does not model. Fold the camera's
fingers out fully and lay it on its tape marks.

**Measurement.** Before touching anything at the end: photograph the buckle from the head side. With a ruler or feeler gauge,
measure the gap between the shoulder under the screw's head and the outer finger (seated: a 0.5 mm feeler does not enter). Check by
eye that the screw passes through both of the camera's fingers. Put a phone inclinometer app on the camera's top and read its
forward tilt. Check from the video that both grippers were 3 cm clear and the camera still for the last 2 s.

**Reset (about 2 minutes).** Hold the camera, unscrew the screw four turns and draw it back to the 17 mm mark, lift the camera
out, lay it on its marks, and put the distractors back. Nothing is consumed. Check the buckle's fingers for cracks every 20 trials;
spare buckles come in the Grab Bag.

**Safety.** Nothing is sharp, hot or heavy. The thumb screw is turned by hand-sized torques; the YAM's wrist cannot strip it.

## Fidelity: what is simplified

Rigid bodies (PhysX, 240 Hz). The camera, buckle and screw are authored meshes at real size, because SimReady has no action camera
or GoPro mount; the helmet and distractors are SimReady. The holes are 5.2 mm and the shank 4.9 mm (0.15 mm radial clearance; the
interface description above gives 5.0 mm nominal holes, which real M5 screws pass with clearance). The thread is code, not
geometry: no thread friction torque, no cross-threading. The clamp is the outer fingers moving 0.4 mm on stiff slides, not
bending. The screw cannot droop or tilt in its hole at the start. Friction is uniform per part (camera 0.7, buckle and screw 0.4).

## Files, assets and licences

- `task.py`: scene, mechanism, scorer, demo. `task.json`: the task's text.
- `mechkit.py` is the standard copy of the mechanism kit from `tasks/heat-shrink-lug/mechkit.py` in this repository (another
  team's open-source task), unchanged. `demokit.py` is taken unchanged from the same task; only its grip helpers (`down`, `level`)
  are used, to give the open-loop waypoints full gripper orientations. `simready.py` is the SimReady loader adapted from
  `tasks/cutters-off-pegboard/simready.py` in this repository.
- SimReady props, downloaded at first run from the pinned revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd` of
  `nvidia/simready-assets` on Hugging Face, © NVIDIA, CC-BY 4.0
  ([licence](https://creativecommons.org/licenses/by/4.0/)):
  - [ppe_helmet_hardHat_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/ppe_helmet_hardHat_a01): fixed in place (no mass).
  - [accessory_eyeglasses_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/accessory_eyeglasses_a01): its hinged temples welded into one rigid body; mass set to 30 g.
  - [food_beverage_bottledDrink_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/food_beverage_bottledDrink_a01): mass set to 550 g.
  - [ppe_gloves_cloth_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/ppe_gloves_cloth_a01): its two gloves welded into one rigid body; mass set to 120 g.

  Nothing large is committed: the files download at first run.
