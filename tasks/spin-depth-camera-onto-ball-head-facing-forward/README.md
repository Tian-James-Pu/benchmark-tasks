# Spin a RealSense depth camera onto a loose ball head's stud, facing the work area

**Instruction:** "Mount the camera so it faces the robot's workspace."

A robot data-collection rig needs a fixed third-person camera. An articulating camera arm is clamped to the desk's
far edge and holds a ball head about 10 cm above the desk. The head's ball is locked by its knob, but its panning
base is loose, so everything above the base swivels freely. On the head's 35 mm platform stands a 1/4"-20 stud with a
knurled lock wheel run down to the platform. A strip of blue tape on the desk runs from under the head towards the
arms: the line the camera must face along. An Intel RealSense D435 stands on the desk to the right of the head,
lenses facing away from the arms.

To mount it, the robot has to set the camera's tripod hole on the stud and turn the camera on. The camera turns but
the stud doesn't, and the stud rides on a free pan base. A camera turned while nobody holds the head turns the head
with it and barely goes on. So one hand steadies the head while the other turns the 75 g camera about three and a half
turns down the stud, in half-turn strokes. The camera has to stop facing along the tape within 5 degrees, half a turn
short of seated, and the lock wheel is then run up against it so the aim holds. Run fully down until it stops, the
camera faces backwards.

## Files

| File | What it is |
|---|---|
| `task.py` | Scene, mechanisms, scorer (`score()`, `success()`) and the scripted demonstration (`demo()`) |
| `task.json` | Task details |
| `mechkit.py` | The mechanism kit, taken unchanged from another team's open-source task in this repository (`tasks/heat-shrink-lug/`) |
| `simready.py` | The SimReady prop loader, unchanged, used by many tasks in this repository |

## Running a trial

    source ~/IsaacLab-Arena/yam.env
    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=tasks/spin-depth-camera-onto-ball-head-facing-forward/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
      --instruction "Mount the camera so it faces the robot's workspace." \
      --scorer task_score --max-steps 1700 --store-frames --log-dir ~/yam-eval/spin-camera

    # a model run: the same command with --policy dream, -E cam_width=320 -E cam_height=176, --max-steps 3000

`--scorer task_score` reports 1.0 once `success()` holds, otherwise the last step's `score()`. The scorer counts its
own one-second hold (`SUCCESS_HOLD_S = 0`). There is no randomness: `object_jitter=0`, one fixed layout, any seed.
The scripted demonstration takes 1,600 control steps (80 s) and about 4 minutes of wall time on one GPU. Each control
step prints a `[spin]` line (every 20 steps) with what the scorer reads; `SPIN_DEBUG=0` turns it off.

## The model

Everything is at full size. Every collider is real contact, and nothing changes state on proximity alone.

- **Head** (`head`, a fixed articulation; origin on the stud's axis at the platform's top face). The frame is the
  arm, its clamp and the head's 40 mm panning base. Link `pan` is everything above the base: the 32 mm socket body
  with its locked ball, the neck, the 35 mm platform and the stud. It turns about the vertical on the loose base, with
  0.002 N m s/rad of drag.
- **Stud**: 1/4"-20 UNC, 6.35 mm major diameter, standing 10 mm above the platform (6 mm proud of the lock wheel).
  Its last millimetre is turned down to 4.4 mm, the lead. The stud's threaded body is a collider of its own (link
  `stud_body`, held by a stiff spring): the first thread, which the camera's bore rests on until the thread catches.
- **Lock wheel** (links `lw_rise` and `lw`): a 40 mm knurled wheel, 4 mm thick, on the stud's thread. Each control
  step, its height follows its turn at 1.27 mm per turn (20 threads per inch). It rises when turned anticlockwise
  seen from above, from the platform to 3 mm up. A 20,000 N/m spring, capped at 8 N, holds it there, so a wheel run
  up against the camera presses it with at most 8 N. Run up hard against the camera (touching it, with its thread
  loaded by at least 0.2 mm of spring), it is held from turning, as a jam nut is; it turns again once the camera is
  backed off it by more than 0.6 mm.
- **Camera** (`camera`, free, 75 g): an aluminium bar 90 x 25 x 25 mm with rounded ends and a glass front with four
  lens windows. Its 1/4"-20 hole is on the axis of its underside, 8 mm deep, and is modelled at the thread's minor
  diameter (a 5.4 mm square bore).
- **The thread catches** when the camera sits on the stud's first thread, with its base no more than 9.5 mm above
  the platform, its hole within 0.6 mm of the stud's axis and its tilt under 5 degrees, and it is turned clockwise
  (seen from above) past the angle where the two thread starts meet. That angle is fixed relative to the head, as on a
  real thread. A PhysX D6 joint between the head and the camera, authored switched off, is then switched on, and the
  stud's threaded-body collider is drawn down out of the bore. The joint keeps the camera on the stud's axis and
  square to it. It adds the thread's running drag, 0.02 N m s/rad between camera and head: ten times the pan base's,
  so an unheld head turns with the camera. It draws the camera down 1.27 mm per clockwise turn. Turned back
  anticlockwise past the catch, the camera comes off the thread. The same switched-on thread joint is used by an
  accepted task in this repository (TK-7JBD35).
- **The thread phase**: from the catch, the camera goes 5 mm down the stud (3.94 turns) before it seats on the lock
  wheel. Half a turn before that, it faces along the tape.

## How the scorer judges each rule

The tracker reads the end state at every control step. A real trial measures the same things with a ruler, feeler
gauges and a photo from above.

| Rule | Simulator | Real trial |
|---|---|---|
| Caught | the thread joint is on | the camera is threaded on: lifting it gently lifts the head |
| Gap ≤ 1.0 mm | camera base no more than 5.0 mm above the platform (the wheel's 4.0 mm plus 1.0 mm) | 5.0 mm feeler stack doesn't pass between platform and camera base |
| Aim within 5° | angle between the camera's lens axis and the tape, seen from above | photo from above, protractor on the tape |
| Head moved ≤ 5° | the pan joint's angle from the start | pan mark against its tape mark on the base (5° is 1.75 mm at the 20 mm base edge) |
| Wheel snug | no more than 0.3 mm between the wheel's top and the camera's base | a 0.3 mm feeler doesn't enter |
| Hands clear | every fingertip point at least 3 cm from the camera and from the head above its base | photo |
| Still, held 1 s | camera moved under 0.5 mm, turned and head turned under 0.5° a step, for 20 steps | 1 s of video |
| Dropped (harm) | the camera ever fell faster than 0.8 m/s (a fall of more than 3 cm) | video |

## Scoring

- **0.0** if the camera has ever been dropped (recorded at every step, cleared only at a reset).
- **1.0**: caught, gap at most 1.0 mm, aim within 5 degrees, head turned no more than 5 degrees on its base, lock
  wheel within 0.3 mm of the camera, hands clear by 3 cm, all still, held 1 s.
- **0.75**: caught, gap at most 1.0 mm, aimed within 5 degrees, without the rest of 1.0 (the wheel not run up, the
  head turned, a hand still on it).
- **0.5**: caught and seated (gap at most 1.0 mm) but aimed more than 5 degrees off, for example run fully down.
- **0.25**: caught but loose (gap more than 1.0 mm).
- **0.0** otherwise: not on the thread.

Possible scores: 0, 0.25, 0.5, 0.75, 1.0. The partial levels need no hands clear, since they are progress. Full credit
does, and it lapses as soon as the state that earned it does.

## Scripted demonstration

`demo()` is open loop (yam_scripted). The left hand comes in level from its side and closes on the head's socket body,
holding the head still on its loose base. The right hand picks the camera off the desk across its 25 mm depth,
carries it over the stud and lowers it until the bore drops over the stud's lead. It then turns it back 30 degrees and
forward again so the bore settles. Next it turns the camera clockwise in seven strokes of half a turn each (let go,
lift clear, turn back, take it again; the grip only fits the camera every half turn). The thread catches 23 degrees
into the first stroke. The seventh stroke is turned 8 degrees further, which takes up the grip's give, and leaves the
camera facing along the tape, 0.64 mm short of seated. The right hand lets go and rolls the lock wheel up with the end
of its closed fingers, pressed 2.5 mm into the wheel's rim beside the camera's face, in four sideways strokes, until
the wheel jams against the camera's base. Then both hands let go and leave.

**Status: not finished.** In the latest scripted run the thread catches 24 degrees into the first stroke. The head stays
within 1.1 degrees, and the camera ends 0.63 mm short of seated, aimed 0.6 degrees off the tape, hands clear:
`task_score` 0.75. The lock-wheel strokes have not yet touched the wheel (its turn stays 0), so the demo doesn't yet
reach 1.0, and no red-team runs have been made.

## Distinct from

| Nearest task | What it tests | Why this is different |
|---|---|---|
| TK for PR #1289, Reattach your detached wrist camera (accepted) | refitting a wrist camera to the robot's own fixed mount | here a free camera is screwed onto a stud that swivels with a loose head, which must be held, and stopped at an aim |
| PR #3453, Square a phone on a mini tripod ball head (pending) | a button-released spring clamp | no thread, no lock wheel, no turning the device itself on |
| PR #190, Release a camera from a cam-clamped quick-release mount (accepted) | removing a camera from a cam clamp | the reverse operation on a different mechanism |
| PR #1405, Thread a spin-on oil filter onto its engine nipple (accepted) | threading onto a fixed nipple, to a torque | here the threaded post swivels freely, the end angle matters, and a lock wheel holds it |

External sets: LIBERO, RoboCasa, BEHAVIOR and SimplerEnv have no camera mounting or threaded fastening onto a swivelling
post with an angular target.

## Importance and sources

Category: Impact on AI timelines. Fixed third-person RGB-D cameras are standard on robot data-collection rigs, and
setting them up and re-aiming them is routine work in building and running those rigs. FurnitureBench's
reproducible real-robot setup, for example, lists three Intel RealSense D435 cameras on Manfrotto variable-friction
arms and super clamps, with the front camera's image going to the policy.

- Intel RealSense D400 Series datasheet (rev. 017, March 2026), table 3-52: D43x 90 x 25 x 25 mm, 75 g nominal;
  table 4-20: one 1/4-20 UNC tripod mounting point: https://www.realsenseai.com/wp-content/uploads/2026/03/RealSense-D400-Series-Datasheet-Mar-2026.pdf
- FurnitureBench (Heo et al., RSS 2023), hardware list with 3x RealSense D435, Manfrotto variable-friction arms and
  super clamps: https://arxiv.org/pdf/2305.12821
- SmallRig Ball Head Mount 2796: 1/4"-20 screw on top, 360 degree rotation, 130 degree tilt, locking knob:
  https://smallrig.com/smallrig-ball-head-mount-2796.html
- Manfrotto 492 micro ball head: 35 mm base, 1/4" top screw, 140 g, 360 degree pan:
  https://www.manfrotto.com/global/tripod-micro-ball-head-492/
- SmallRig 4373 crab-shaped super clamp with ball-head magic arm (desk clamp arm, 1.5 kg load):
  https://smallrig.com/smallrig-Crab-Shaped-Super-Clamp-with-Ballhead-Magic-Arm-4373.html

The modelled head is generic: 35 mm platform, 32 mm socket body, 40 mm pan base. Its lock wheel is a standard
1/4"-20 knurled lock wheel, 40 mm across and 4 mm thick, on the head's stud. The D435's tripod-hole depth isn't
published. The model's 8 mm hole takes the 6 mm the stud stands proud of the wheel, with 2 mm to spare.

## Real trial: preparation, reset, measurement

- **Rig** (about $420): a RealSense D435 (about $300), a SmallRig 4373-type super clamp with ball-head magic arm (about
  $20-40), a mini ball head with a 1/4"-20 stud and a separate pan base (about $20-50), and a 1/4"-20 knurled thumb nut
  as the lock wheel. Clamp the arm to the desk's far edge so the head's platform is 10 cm above the desk, 40 cm in
  front of the line between the arm bases and 16 cm right of their midpoint. Lock the ball upright with its knob and
  tape over the knob, so the knob is not part of the task. Leave the pan base loose. Mark the pan base and the socket
  body with one line of tape.
- **Tape**: a 12 mm blue strip from under the stud straight towards the arms (parallel to the arms' forward axis), 15
  cm long with an arrowhead.
- **Start**: the camera stands on its base on the desk, centred 23 cm in front of the arm bases and 18 cm right of
  their midpoint, lenses facing away from the arms. The lock wheel is run fully down to the platform.
- **Distractors** (not scored): a mug, a roll of clear tape and a Phillips screwdriver, at the positions in `setup`.
- **Reset** (about 2 minutes): unscrew the camera, run the wheel down, line the pan marks up, and stand the camera on
  its tape marks.
- **Measure** at the end, before touching anything: a photo from straight above for aim and pan mark, a feeler stack
  for the camera base's height above the platform, and a 0.3 mm feeler between wheel and camera.
- **Rig safety**: nothing sharp, hot or wet. The camera is light, and the harm rule (a drop) is recorded from video.
  Put a folded towel on the desk under the head for the real trial.

## Assets and licences

- NVIDIA SimReady props (CC-BY 4.0, © NVIDIA), loaded by `simready.py` at the revision pinned there:
  `dishware_mug_a01` (mug, mass set to 0.30 kg), `tape_clear_a01` (tape roll, 0.06 kg),
  `handTool_screwdriver_phillips_ph2_a01` (screwdriver, 0.10 kg, convex colliders). They are downloaded at first run
  from https://huggingface.co/datasets/nvidia/simready-assets. Nothing large is committed.
- The camera, ball head, arm, clamp, lock wheel and tape are built in `task.py` from mechkit meshes at the dimensions
  above. SimReady has no RealSense camera or camera ball head.
