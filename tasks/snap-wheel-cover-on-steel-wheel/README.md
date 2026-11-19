# Snap a plastic wheel cover onto a steel wheel with its valve notch over the valve stem

A bare 13-inch steel wheel lies face up on the table, its rubber valve stem standing up inside the rim at the robot's left. A 13-inch
plastic wheel cover (hubcap) has been set loosely on it, resting on its retaining clips, with its valve notch a quarter turn away from the
stem. The instruction is only the goal: "Put the wheel cover on the wheel."

Doing it right takes three things a person does without thinking:

- **Turn the cover until its notch is over the stem.** Pressed on anywhere else, the cover lands on the stem: it will not clip down on that
  side, and pressing harder crushes or tears the stem.
- **Press it on round its rim** until every clip has snapped in behind the bead seat's lip.
- **Let go.** Full credit needs both grippers clear of the cover.

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: the wheel, stem and cover, the scorer (`score`, `success`) and the scripted `demo()` |
| `mechkit.py` | The mechanism kit (articulations written as USD text, hooks, fingertip points), unchanged, from the open-source kit in `tasks/heat-shrink-lug/` |
| `simready.py` | Loads the SimReady distractors, unchanged |
| `task.json` | The task's details for the board |

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena (`source ~/IsaacLab-Arena/yam.env`).
- Network access the first time the scene is built: `simready.py` downloads the SimReady packages into `~/.cache/simready`.

## Run and score a trial

From the repository root:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/snap-wheel-cover-on-steel-wheel/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Put the wheel cover on the wheel." --scorer task_score --max-steps 2400 \
  --store-frames --log-dir ~/yam-eval/snap-wheel-cover-on-steel-wheel/dream
```

The scripted demo (`demo()`, played open loop by the scripted policy):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/snap-wheel-cover-on-steel-wheel/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=640 -E cam_height=360 \
  --instruction "Put the wheel cover on the wheel." --scorer task_score --max-steps 900 \
  --store-frames --log-dir ~/yam-eval/snap-wheel-cover-on-steel-wheel/demo
```

`task_score` prints the score. Nothing is randomised, so every seed gives the same scene. Runtime: the demo lasts 30 s of simulated time
(600 steps) and reaches full credit at step 575, when the trial stops; on a shared GPU a 900-step trial took several minutes of wall time,
and a 2,400-step trial takes proportionally longer.

## The scene

- **Wheel** (`wheel`, a fixed mechanism): a 13 x 4.5J steel wheel, drawn as a revolved 3 mm steel section (flange, bead seats, drop
  centre, centre disc with four stud holes and four vent holes). It collides through rings of 40 convex segments: the flange top, where
  the cover's lip seats; the outboard bead seat's inner face, the lip the clips ride over (2.2 cm tall); the groove behind it, where the
  clips catch; and the well's shoulder.
- **Valve stem**: the wheel's one link, a rubber TR414-class snap-in stem on a hinge at its base about the rim's tangent, with a spring for
  the rubber's bending stiffness (0.5 N m/rad).
- **Cover** (`cover`, a free mechanism, 0.45 kg): a dished ABS cover 36.0 cm across with a 3.35 cm valve notch and eight through slots
  between eight spokes, colliding through convex pieces (the notch and the slots are real gaps). Its eight retaining clips are links:
  each a sprung slider with a ramped head. Pressed down, the head's ramp meets the bead seat's lip and cams the clip in; further down the
  head clears the lip and springs out into the groove, where its flat top catches under the lip.
- **Distractors**: a cleaning cloth, a slotted screwdriver and a pair of light-duty gloves, loose, outside the demo's paths, ignored by the
  scorer.

## Scoring

Each control step (20 Hz), the scorer reads the cover's pose, its clips' joints, the stem's angle and the grippers' fingertip points:

- a clip is **engaged** when the cover's lip beside it is within 1.0 mm of the flange and the clip has snapped back out to within 1.5 mm
  of its free position;
- the stem is **in the notch** when its axis passes within 1.1 cm of the notch's centre line;
- **hands clear** means every fingertip point of both grippers (each finger's tip and 3 cm up it) is at least 3 cm from the cover.

| Outcome | Score |
|---|---|
| Stem in the notch, all eight clips engaged, hands clear, held 2 s | 1.0 (kept from then on) |
| Otherwise at the end, stem in the notch | 0.2 + 0.075 per engaged clip (0.2 to 0.8) |
| Stem not in the notch | 0 |
| Stem bent more than 15 degrees from upright at any time | 0 for the whole trial |

The bent-stem record is cleared only by the kit's reset hook, never by the scene looking right again.

In a real trial: the lip-to-flange gap is checked with a 1 mm feeler gauge beside each clip, an engaged clip does not lift when the
cover's edge beside it is pulled up by hand with light force, the stem's position in the notch is read with a ruler, and a bent stem
with a protractor on a photo.

## The demo

Both arms straddle two opposite spokes with open fingers down through the slots, grip them, lift the cover 1.2 cm clear of the rim and
turn it a quarter turn clockwise (seen from above), so the notch comes round over the stem. They set it back on the rim, let go, then
press it down over its clips in mirrored pairs, starting beside the stem, and move away. Turning the cover while it rests on the rim, by
pushing in a slot, fails in simulation as it does by hand: the cover slides off-centre, the clips on one side drop off the lip, and it
cocks and jams.

| Run | Score |
|---|---|
| Scripted demo, run 1 | 1.0 (full credit held from step 575) |
| Scripted demo, run 2 | 1.0 (full credit held from step 575) |
| Red team: do nothing | 0 (the stem is never in the notch) |
| Red team: press the cover on where it lies, without turning it | 0 (it lands on the stem, two clips engage, it sits cocked 2 degrees, the stem is not in the notch) |
| Red team: one arm only (lift one side by a spoke and turn, then press round on its own side) | 0 (the turn brought the notch to the stem, 0.2 at that point; pressing one side alone then pushed the cover 15 cm off centre) |

In its Dream-1 run (2,400 steps) the model raised both grippers to either side of the cover and held them there; it never turned or pressed the cover, so the stem was never in the notch and the run scored 0.

In the careless run the stem carried the cover's load along its axis and bent less than 1 degree, so the score is 0 by the notch rule,
not the bent-stem rule. The simulated stem is a sprung hinge and is not crushed; a real one pressed on like this is.

## Fidelity

Rigid PhysX bodies at 240 Hz. No SimReady wheel or hubcap exists, so both are custom meshes and the wheel is fixed to the table. The ABS
cover is rigid, where a real one flexes. Its steel retaining ring is modelled as eight separate sprung clips, 1,500 N/m each, softer
than a real ring: about 10 N per clip seats it in simulation, where a person uses palm blows. The stem is a sprung hinge. Friction is
uniform; there is no click or sound, and no force sensing.

A 15-inch wheel was considered first. Lying flat, its cover rests 2.5 to 4 cm higher than a 13-inch one (its rim is wider), which puts
the press and grip points above what these arms reach with a top-down grip (about 0.95 to 0.97 m above the floor near the robot, 0.94 m
on the far side). 13-inch steel wheels and covers are a current, common size, so the task uses one at real scale.

## Assets

The distractors are from NVIDIA's SimReady library,
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`cleaning_cloth_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/cleaning_cloth_a01)
- [`handTool_screwdriver_slotted_65mm_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_screwdriver_slotted_65mm_a01)
- [`ppe_gloves_lightDuty_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/ppe_gloves_lightDuty_a01)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time: masses set (cloth
50 g, screwdriver 120 g, gloves 60 g); any joints welded into one rigid body. `simready.py` downloads them into `~/.cache/simready` the
first time the scene is built. Nothing large is committed.

`mechkit.py` is the open-source mechanism kit shared by several tasks in this repository (copied unchanged from
`tasks/heat-shrink-lug/`).

## Importance and sources

Steel wheels with clip-on plastic covers are standard on base trims of small cars, and covers come off for every tyre change, rotation
and puncture repair. Refitting one is a short, everyday job that needs two different skills in order: an alignment the cover's own shape
dictates (the notch must meet the stem, or pressing on breaks things), then a snap fit seated all round. Fitting guides say the same:

- iFixit, "2002 Nissan Altima Hubcap Replacement": align the retention ring's divot with the tyre-valve cutout, align the hubcap's
  cutout with the tyre valve, then push in all the sides until it secures:
  https://www.ifixit.com/Guide/2002+Nissan+Altima+Hubcap+Replacement/172847
- Ring Automotive, 13" Vegas wheel trims (RWT1333): a current 13-inch high-impact ABS wheel trim held by an adjustable retaining ring, the
  kind of cover modelled here: https://ringautomotive.com/en/product/RWT1333
- Tesla, Model 3 service documentation, "Removing and Installing Wheel Covers": a manufacturer's procedure for a clip-on cover, which
  starts "Align the wheel cover with the valve stem", then presses it until secured, and warns to make sure it is fully secured before
  driving so it does not fall off:
  https://service.tesla.com/docs/Public/diy/model3/en_us/GUID-377C88DB-7FBE-4320-BC1A-0A01AF48005C.html
