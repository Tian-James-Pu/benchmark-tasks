# Slate a take for a phone camera

On the station table a phone on a tabletop tripod films toward the robot. In front of the arms an acrylic film
clapperboard leans back, sticks shut, against a cardboard box, its writing face toward the phone. The instruction is
the slating call itself:

> Slate the take: clap the clapperboard shut once, with it facing the phone camera and fully in its picture.

It names the goal and not the method. What makes it hard is a two-handed hand skill a camera assistant uses at the
start of every take: one hand holds the slate square to the lens and still, the other drives the hinged clapstick
shut in one fast stroke. Here the board cannot stand by itself (it leans 40 deg back on the box and falls back onto it
when let go), and the hinge's tension screw holds the stick wherever it is left, so letting go of an open stick does
not clap it: a hand has to strike it shut. The strike has to be fast (the sticks meet within 50 ms of being 10 deg
apart) and must not jolt the board (its face centre stays within a 1 cm range around the clap).

The task is one task file, [`task.py`](task.py), for the `yam_arena` embodiment: scene, scorer and scripted
demonstration. It uses [`mechkit.py`](mechkit.py) for the hinged clapperboard, [`simready.py`](simready.py) for the
SimReady props and [`demokit.py`](demokit.py) only for its grip orientations in the scripted demo (see "Code and
credits").

## An independent implementation of a lapsed idea

The idea of slating a take with a clapperboard was first submitted by another team in PR #3276. That submission was
not accepted on one point and was not revised within its 48-hour hold, so the hold lapsed. This folder is an
independent implementation: its scene, mechanism, scorer, demonstration and text are written from scratch with a
different kit, a different layout and a scorer built around the review's request. How each point of that review is
addressed:

| Review point on PR #3276 | How this task addresses it |
|---|---|
| "At 60 fps, a 45.8 ms closure can span four sampled frames and fail the three-frame limit solely because of camera timing." | The closing time is defined as a frame count at 240 fps, the same rate as the simulator's physics step, so the simulated scorer and the real check count the same frames: at most 12 frame intervals at 240 fps (50 ms) from the last frame with the stick 10 deg or more open to the first frame with the sticks meeting. A 240 fps frame is 4.2 ms, so the count resolves the 50 ms limit to within one frame (8%) instead of one 60 fps frame (33%). |
| "Use sufficiently high-frame-rate recording, such as 240 fps, and state how frames are counted against the threshold." | The real trial is filmed by the phone's own slow-motion mode at 240 fps (an iPhone 15 records 1080p slo-mo at 120 or 240 fps, [Apple][iphone15]). "Real-trial measurement" below states exactly how frames are counted: the frame index of the last frame with a free-end gap of 5.0 cm or more, the frame index of the first frame with the sticks touching, and their difference against 12. |

Beyond the review, the build also meets the current bar: the hinge side follows the real product (hinge on the
camera's left), every recognisable prop other than the clapperboard and phone rig is a SimReady prop, there are three
SimReady distractors, the violation record is cleared only in the reset hook, scoring has five levels, and the
README gives sources for each claim.

## Importance and sources

- **The operation.** A clapperboard is snapped shut on camera at the start of a take so that picture and separately
  recorded sound can be lined up: editors match the frame in which the sticks close to the sound of the clap
  ([Wikipedia, "Clapperboard"][wiki]). The same article says the clapper loader (second assistant camera) operates
  it, holding it in view of the camera and snapping the sticks shut.
- **The hand skill.** A camera assistant's guide to slating says: "Proper slating technique dictates holding the
  slate with one hand and closing the clapper with the other"; "THE SLATE MUST NOT MOVE WHILE THE CLAPPER IS IN
  MOTION"; editors look for "the first frame where it doesn't show any motion blur"; "Do not let the clapper bounce";
  and "The slate only needs to be visible for about a second after hitting it" ([Art Adams, ProVideo Coalition, 1
  March 2011][pvc], tips 4, 5 and 13). Those are the scored conditions here: board held still, stick closed fast,
  no bounce, slate kept in picture for 1 s after the clap.
- **Why Societal relatability.** The clapperboard snap is one of the most widely recognised film-making gestures, and
  slates are sold cheaply for home and phone video (the modelled board is a 20 USD-class acrylic slate). We do not
  claim that most adults have slated a take; the claim is recognisability and a widely shared hobby and
  small-production practice (inference, no survey cited). We do not claim Impact on jobs: camera assistants are far
  fewer than 500,000 workers.
- **The product.** The board is modelled on the Neewer acrylic director's clapboard (item 66601326), listed at 11.8 x
  9.8 x 1.4 in (30 x 25 x 3.5 cm) and 520 g, supplied with two pens, an eraser, a cloth and an L-shaped hex key
  ([Neewer product page][neewer]). Neewer's 12 x 10 in acrylic clapper board gives the same 30 x 25 cm size with a
  30 x 20 cm writing area ([Neewer 12 x 10 in board][neewer1210]). Another seller's acrylic slate is 30 x 24.5 cm and
  430 g ([Bresser BR-VC1][bresser]). The board and stick thicknesses (5 mm and 8 mm) are our estimates, not listed
  figures; with acrylic at 1.18 g/cm3 they give 0.50 kg, inside the listed range (the listed 520 g may include the
  accessories). That the hex key is for the hinge's tension screw, and that the hinge sits on the left as seen from
  the front with the free end on the right, are inferences from the product listing and photos.
- **The camera.** The phone is sized as an iPhone 15 (147.6 x 71.6 x 7.80 mm, 171 g), whose main camera is a 26 mm
  equivalent and which records slow motion at 240 fps ([Apple tech specs][iphone15]). A 26 mm-equivalent lens on a
  16:9 frame (36 x 20.25 mm of the 36 x 24 mm format) sees 69.4 x 42.6 deg (derived: 2 atan(18/26) and
  2 atan(10.125/26)).
- **Why 50 ms.** At 24 fps a film frame lasts 41.7 ms. A stick that crosses its last 10 deg within about one frame
  gives a sync frame with the sticks clearly apart in the frame before and shut in the frame after; a slower close
  smears the meeting over several frames (inference from the motion-blur advice above, not a published threshold).

## Distinct from

| Nearest task | What it tests | Why this one differs |
|---|---|---|
| TK-PK8X7K (accepted): snap a chalk line | One hand holds a line taut, the other plucks it to snap a mark | A taut line released, scored by the mark; here a hinged stick is struck shut, scored by closing speed with the held board kept still and square to a camera |
| TK-JRVAY3 (accepted): grate cheese holding the grater steady | One hand steadies a tool while the other makes repeated strokes | Repeated slow strokes; here a single fast strike, timed at 240 fps, with the held part's stillness and orientation to a camera scored |
| TK-88C465 (revision pending): hold a nail upright while another arm hammers | Holding a part still while something else strikes it | The striker there is another robot; here the robot itself strikes a light hinged part, and the hold must also keep a picture framed |
| TK-K34A6L (PR #3276, hold lapsed) | The same idea | Not accepted; this is an independent build that meets its review |

We found no clapperboard task in LIBERO, RoboCasa, BEHAVIOR, SimplerEnv, RoboDojo or RoboARC.

## The scene

Positions are in cm from the midpoint between the two arm bases: x forward (away from the arms), y to the robot's left,
z up from the table top.

- **Clapperboard** (loose, 0.50 kg; `slate` in the code). A black acrylic writing board 30.0 x 20.0 x 0.5 cm with
  white printed field lines; a fixed lower clapstick 30.0 x 2.5 x 0.8 cm along its top edge; the upper clapstick, the
  same size, white with black diagonal stripes, on a hinge pin through both sticks 1.2 cm in from the board's end on
  the robot's right (the camera's left), so its free end is on the robot's left, 28.8 cm from the pin. 30 x 25 cm
  overall. Sticks shut, the board stands on its bottom edge with the middle of that edge at x 28.4, y 0, writing face
  toward the phone, leaning back 40 deg from vertical with its back on the box's far top edge (its top edge 19.2 cm
  above the table).
- **Cardboard box** (loose, 1.5 kg; SimReady `box_cardboard_d01`): 26.4 (x) x 34.5 (y) x 16.6 cm tall, from x -12.4
  to 14.0, centred on y 0.
- **Phone on a tabletop tripod** (fixed): a phone the size of an iPhone 15 (14.8 x 7.2 x 0.8 cm) in landscape in a
  clamp on a three-legged tabletop tripod, its main camera lens at x 72, y 0, 13 cm above the table, looking straight
  back along -x toward the robot; 69.4 x 42.6 deg field of view, recording at 240 fps.
- **Tape strip** on the table along the camera's axis: yellow, 22 x 4.8 cm, from x 40 to 62, centred on y 0.
- **Distractors** (loose, ignored by the scorer): a paper coffee cup (SimReady `food_beverage_coffeeCup_a01`, 8.5 cm
  across, 11.7 cm tall, 0.25 kg) at x 52, y 31; a roll of tape (SimReady `tape_clear_a01`, 0.12 kg) at x 66, y 22; a
  notebook (SimReady `misc_notebook_a01`, 22.9 x 17.0 x 1.0 cm, 0.12 kg) at x 53, y -30, turned 23 deg.
- Both arms start at the ready pose, grippers open. With `-E object_jitter=0` the layout is the same in every trial.

### The clapperboard's mechanism

The clapperboard is one articulation (mechkit's `mechanism` with a free frame): the frame is the board and the lower
stick, and the upper stick is a hinge about the board's normal through the pin. 0 deg is shut, which is the joint's
lower limit (the sticks belong to one articulation and do not collide with each other, so the shut stop is the
limit); 50 deg is fully open. The hinge's tension screw is joint friction, so the stick stays where it is left. The
stick's own weight is left out: on the real board the tightened screw holds it at any angle, which is the behaviour
modelled. The board has the inertia of a 30 x 22.5 x 0.5 cm plate of 0.43 kg; the stick that of a 0.071 kg bar.

## Scoring

The scorer reads the stick's opening angle and the board's pose after every physics step, at 240 Hz (Isaac Lab
updates the scene's buffers once per physics step; the task wraps that update). Everything is in `Clap` and `Trial`
in [`task.py`](task.py). The record of claps is cleared only in the reset hook, never because the scene looks like
its start again.

- **A clap** is the stick reaching 0.5 deg or less (the sticks meeting; a 2.5 mm gap at the free end) after it was
  opened 15 deg or more (a 7.5 cm gap). The first clap is the take's sync mark and is the one judged.
- **Facing**: at the first clap, the angle between the board's face normal and the line from the face's centre to the
  lens is 20 deg or less.
- **In picture**: at the first clap, all four outer corners of the clapperboard are inside the 69.4 x 42.6 deg view
  (a pinhole check).
- **Fast**: at most 12 frames at 240 fps (50 ms) from the last frame with the stick 10 deg or more open (a 5.0 cm gap)
  to the first frame shut.
- **Still**: the face centre's positions over the 60 frames before the clap and the 60 after (0.25 s each side) all lie
  within a 1 cm range.
- **Held**: for 1 s after the clap the stick never opens 5 deg or more again (a 2.5 cm gap: no bounce, no second clap)
  and the board stays facing and in the picture.

Possible scores:

| Score | Condition |
|---|---|
| 0 | No clap; or the first clap is not facing or not fully in the picture |
| 0.25 | The first clap faces the phone and is in its picture, but is slow |
| 0.5 | And it is fast, but the board moved 1 cm or more around it |
| 0.75 | And the board was still, but the stick reopened 5 deg or more, or the board left the picture or turned away, within the next 1 s |
| 1.0 | All of it (success): the run ends there |

`--scorer task_score` gives 1.0 on success, otherwise the score at the end of the run. Doing nothing scores 0.

## Real-trial measurement (240 fps)

- **Rig:** the slate on its tape marks, the box on its marks, a phone on a tabletop tripod with its main lens 13 cm
  above the table and 43.6 cm in front of the slate's bottom edge, recording slow motion at 240 fps (1080p), and a
  second phone or webcam overhead. Cost: a 20 USD-class acrylic slate, a cardboard box and a tabletop tripod.
- **Closing time (the frame count):** step through the 240 fps clip around the clap. Frame A is the last frame in
  which the gap between the sticks at the free end is 5.0 cm or more (10 deg; mark 5.0 cm on the lower stick's end
  face with a thin line so it can be read in the picture). Frame B is the first frame in which the sticks touch along
  their length (no visible gap). The closing count is B - A; the clap is fast if B - A is 12 or less. A 240 fps
  frame is 4.17 ms, so the count brackets the true time to within one frame: 12 counted frames means a true time
  between 45.8 and 54.2 ms, the same rule the simulator applies to its own 240 Hz samples.
- **Clap count and bounce:** in the same clip, any reopening of the free end to a 2.5 cm gap or more within 240 frames
  (1 s) after frame B fails the hold.
- **Still:** in the 240 fps clip, the slate's printed corner marks over frames B - 60 to B + 60 (0.25 s each side),
  scaled by the board's known 30 cm width in pixels; their positions all lie within a 1 cm range. The overhead video
  shows movement toward or away from the lens against the tape.
- **Facing:** from the overhead video at frame B, the board's yaw (its top edge against a line square to the tape)
  and its lean (the horizontal offset of the top edge from the bottom edge, over 25 cm); the face is off the axis by
  acos(cos(yaw) cos(lean)), at most 20 deg.
- **In picture:** at frame B all four corners of the clapperboard are visible in the phone's frame.
- **Reset (about a minute):** shut the stick, stand the slate on its bottom-edge marks and lean it back on the box's
  far top edge, check the box on its marks and the phone recording at 240 fps, arms at the ready pose.
- **Safety:** the board is 0.5 kg of acrylic with no sharp edges; the strike lands on the stick, not on a person.
  Pads are not at risk.

## Run and score a trial

Requirements: [yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) on Isaac Lab-Arena / Isaac Sim,
Linux with an NVIDIA RTX GPU. The SimReady props download on the first run (network access needed).

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/slate-a-take-with-clapperboard/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Slate the take: clap the clapperboard shut once, with it facing the phone camera and fully in its picture." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/slate-dream
```

The scripted demonstration (ends on success after about 420 steps):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/slate-a-take-with-clapperboard/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=640 -E cam_height=360 \
  --instruction "Slate the take: clap the clapperboard shut once, with it facing the phone camera and fully in its picture." \
  --scorer task_score --max-steps 700 --store-frames --log-dir ~/yam-eval/slate-demo
```

Videos, one MP4 per camera, next to each run's frames: `inspect-robots video <log>.json`.

`SLATE_DEBUG=1` prints the stick's angle, the board's facing angle, whether it is in the picture, its pose and the
score with its reason every 5 steps, every step after a clap, and each clap with its frame count and the last 40
angle samples. `TASK_DEMO=<mode>` plays a red-team variant of the demo (below).

### Dream-1

In our Dream-1 run (1,200 steps) both arms reached out over the table toward the phone and the far side and never
touched the clapperboard or the stick; there was no clap, so it scored 0.0.

### The scripted demonstration

Open loop from the published starting layout (no simulator state is read). The right hand comes in from the board's
right edge, fingers pointing along the board and 40 deg down it in its plane, and pinches the board across its 5 mm,
4 cm in from the edge and 13 cm up; it stands the board up about its bottom edge, in eight steps, to 2 deg past
upright toward the phone, and holds it there on the table to the end. The left hand pinches the upper stick's last 4 cm
end-on (fingers along the stick, closing across its 8 mm, on its upper 18 mm only so they stay off the lower stick),
opens it 25 deg along its arc, lets go (the hinge holds it open), moves over the stick toward the hinge and closes its
fingers, then strikes the stick shut with the closed fingers' lower side, 10 to 14 cm out from the hinge. The strike
is a single 0.05 s waypoint that swings only the elbow (the arm's strong shoulder-group joint) so the soft wrist joints
do not lag. In our runs the stick went from 10 deg to shut in 10 frames (42 ms), the board's face centre moved 0.5 mm
around the clap, and the board faced the lens within 1.2 deg.

### Red team

Each is a mode of `demo_plan` (`TASK_DEMO=<mode>`), run in the simulator like the demo:

| Mode | What it does | Score |
|---|---|---|
| `none` | Both arms stay away | 0 |
| `onearm` | The left hand alone opens and strikes the stick with the board where it leans on the box | 0 (the clap was fast, 9 frames, but the face was 39 deg off the lens) |
| `letgo` | The board is stood up, then let go before the clap: it falls back onto the box | 0 |
| `slow` | The careless way: the board is held, but the stick is closed again at an ordinary pace while still pinched | 0.25 (in picture and facing, but 122 frames from 10 deg to shut) |
| `snap_pinched` | Holding on instead of striking: the stick is snapped shut in one 0.05 s move while still pinched | 0.5 (fast, 12 frames, but the arm's swing dragged the board: its face centre moved 13.3 mm around the clap) |

One route is allowed and not penalised: propping the board instead of holding it. Set with its bottom edge within
about 4 cm of the box, the board leans back on the box's edge only 10 to 15 deg and faces the lens within 20 deg, so
it could then be clapped with no hand on it. That is a fair way to slate, and it scores like a held board. It still
needs the board moved about 11 cm and set down at a precise lean, then the fast strike. In three open-loop attempts
we did not manage to prop it: each time the bottom edge slid out as the board was tilted back, and it fell back to
its 40 deg lean (score 0).

## Runtime per trial

On one NVIDIA RTX PRO 6000 Blackwell shared with other simulation jobs, the scripted demo at 640 x 360 (about 420
steps, ending on success) took under 2 minutes once the shaders were compiled. Isaac Sim's first start on a new
machine adds a few minutes while it compiles its shaders. A Dream-1 trial of 1,200 steps at 320 x 176 takes longer,
mostly for the model's calls.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic.

## Code and credits

| File | What |
|---|---|
| `task.py` | Scene, clapperboard mechanism, the 240 Hz clap scorer and the scripted demo |
| `mechkit.py` | The mechanism kit (articulations written as USD text, reset and step hooks), copied unchanged from `tasks/heat-shrink-lug/mechkit.py` in this repository (another team's open-source task) |
| `simready.py` | Loads SimReady props, copied from `tasks/cutters-off-pegboard/simready.py` in this repository (another team's open-source task) |
| `demokit.py` | Closed-loop demo runner, copied unchanged from `tasks/heat-shrink-lug/demokit.py`; only its grip-orientation helper is used, so the scripted waypoints can carry a full gripper rotation |

### Assets

The box and the three distractors come from the Hugging Face dataset
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`box_cardboard_d01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/box_cardboard_d01)
- [`food_beverage_coffeeCup_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/food_beverage_coffeeCup_a01)
- [`tape_clear_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/tape_clear_a01)
- [`misc_notebook_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/misc_notebook_a01)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time:
masses are set (box 1.5 kg, cup 0.25 kg, tape 0.12 kg, notebook 0.12 kg), the box collides as a convex hull, and the
invisible grasp-annotation curves are deactivated. The files are not committed; `simready.py` downloads them into
`~/.cache/simready` on the first run.

SimReady has no film clapperboard and no phone or tabletop tripod. The clapperboard is built from boxes and extruded
outlines in `task.py` to the product's dimensions (a plain board and two bars: its real shape). The phone (a plain slab
with a lens), its clamp and the tripod are fixed primitives standing in for the camera rig, which the robot never
touches.

[wiki]: https://en.wikipedia.org/wiki/Clapperboard
[pvc]: https://www.provideocoalition.com/the-secret-art-of-the-slate-25-tips-to-help-you-slate-like-a-pro/
[neewer]: https://neewer.com/products/neewer-acrylic-plastic-directors-film-clapboard-cut-66601326
[neewer1210]: https://neewer.com/products/neewer-acrylic-film-clapper-board-12-x10-plastic-movie-theater-decor-clapboard-66600880
[bresser]: https://www.bresser.com/en/Photostudio/BRESSER-BR-VC1-Clapperboard.html
[iphone15]: https://support.apple.com/en-us/111831
