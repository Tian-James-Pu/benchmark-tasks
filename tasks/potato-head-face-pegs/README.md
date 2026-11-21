# Give a classic Mr. Potato Head his face: every peg pushed home, and him still on his feet

> **DRAFT, NOT VALIDATED, DO NOT SUBMIT.** The closed-loop demo does not reach 1.0 yet. The best CPU run scored 0.25: the mouth and
> nose seat, but the eyes and ears do not. The fit and shape numbers in this text are stale. `task.py` now has a broad-bottomed
> body, 12 mm bores, 4,000 N/m pads at 0.5 mm interference and friction 0.3/0.25 (about 2 to 4 N to push home), not the 8 N described below.

**Instruction:** *Give Mr. Potato Head his eyes, nose, mouth and both ears, each pushed all the way in, and keep him upright the whole time.*

Mr. Potato Head stands on the table on his shoes, facing the arms, with no face. His eyes and mouth stand on their lower edges in front
of him, his nose lies on its side, and his two ears lie flat on the table, one at each side. Each part has one round peg, and each peg
goes into its own round hole: eyes, nose and mouth down the middle of his front, an ear on each side. The pegs are a tight
(interference) fit, so a part only goes home to its collar under a firm straight push, while the hollow figure is light enough to
scoot across the table or fall over under a fraction of that push. The robot has to brace him with one hand while the other pushes
each peg straight in, then let go of everything with him still standing.

## Importance and sources

**Category: Societal relatability.** Giving Mr. Potato Head his face is one of the most familiar toy jobs there is, something most
adults did themselves as children or have done with their own children:

- Mr. Potato Head has been on sale since 1952; sales of the toy and related products passed 100 million units by 2012
  ([Wikipedia, "Mr. Potato Head"](https://en.wikipedia.org/wiki/Mr._Potato_Head), citing the maker). It was inducted into the National
  Toy Hall of Fame in 2000 ([The Strong National Museum of Play](https://www.museumofplay.org/toys/mr-potato-head/)).
- The product is current and cheap: Hasbro's classic Mr. Potato Head, item F3244, for ages 2 and up, listed as a large potato body,
  a base with feet and 11 accessories, among them a smile, a nose and ears
  ([Hasbro consumer care page](https://consumercare.hasbro.com/product/potato-head-mr-potato-head-classic-toy-for-kids-ages-2-and-up-includes-13-parts-and-pieces-to-create-funny-faces/C1D52A99-0713-431C-8018-EE82EE39AC2A)),
  about $10 at retail.
- The holes are round, so a part goes in at any rotation: Hasbro replaced the holes with flat slots in the 1970s
  ([Wikipedia](https://en.wikipedia.org/wiki/Mr._Potato_Head)) and "reintroduced the toy's round holes in the 1980s"; the body was
  doubled in size in 1975 and the parts enlarged for child safety
  ([The Takeout, "The Spudly History Of Mr. Potato Head"](https://thetakeout.com/a-brief-history-of-mr-potato-head-1846175691)).

**The knack.** Each peg is an interference fit that needs about 8 N to push home (our estimate for a well-fitting new toy: firm for a
toddler, easy for an adult), but the hollow 120 g figure slides on a table at about 1 N and, pushed at face height 8 to 11 cm up on a
6 cm-long base, tips over at well under 1 N. So one hand braces the body from behind (or from the far side, for an ear) at the height of
the hole while the other pushes the part straight along the hole's axis. Pushed one-handed, the figure scoots away or falls; pushed at
an angle, a peg binds on the hole's pads and stops partway, its collar standing proud. This is the everyday two-handed act of holding
a light object steady against your own push, applied to five parts on three axes (the ears go in sideways). People do it in their
hands; on a table the robot braces with the other gripper, or lifts him upright in one hand, which the scoring also allows.

**What is not claimed.** The 8 N fit force and the masses are estimates (README "Fidelity"), not measurements of a particular toy.

### Distinct from

| Nearest | What it tests | Difference here |
|---|---|---|
| Push a peg home past a hidden obstruction without forcing it (#219, accepted) | one peg into a fixed hole, feeling for an obstruction | the holes are in a light free-standing toy that must be braced against each push, five parts on three axes |
| Hold-and-press peg (#1063, rejected) | pressing a peg while holding a part | a real, recognisable toy at full size; the held object is the receiving body, not the peg |
| Stack a baby's ring-stacker toy (#3106, rejected) | dropping rings over a post | interference-fit pegs pushed home sideways into a free-standing light body |
| Garnish a glass with a lime wedge without tipping it (#2625, rejected) | pushing onto a light object without upsetting it | a firm (8 N) push that the object cannot resist on its own, so the second hand must brace |
| Peg insertion in benchmark sets (ManiSkill PegInsertionSide, robosuite NutAssembly, Factory/IndustReal) | a peg into a fixed or heavy socket | the socket is a 120 g toy that slides and tips under the push; nothing here is fixed |

## Run and score

From the repository root, in the Python environment that has `yam_arena` and `inspect-robots`:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/potato-head-face-pegs/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Give Mr. Potato Head his eyes, nose, mouth and both ears, each pushed all the way in, and keep him upright the whole time." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/potato-head-face-pegs
```

The scripted demonstration is closed loop (it reads the parts' and the body's poses, never the scorer), so it runs through
`demokit.py` from the task folder:

```bash
cd tasks/potato-head-face-pegs
python demokit.py task.py ~/yam-eval/potato-head-face-pegs/demo --mode full                       # writes full/demo.mp4, setup.png, result.json
python demokit.py task.py ~/yam-eval/potato-head-face-pegs/redteam --no-record --mode none,onearm,started
python task.py                                                                                     # offline: shell clear of every bore, seat depths
```

`result.json` holds `task_score`. The layout is fixed (`object_jitter=0`); there are no seeds or random choices.
RUNTIME_PLACEHOLDER

## How the simulator models it

All in `task.py`. Nothing is triggered by proximity: every state change is contact and friction.

- **Body** (a free articulation, 120 g in all): the potato is an ellipsoid 72 mm deep, 88 mm wide and 116 mm tall, its underside 16 mm
  up, sitting into the base with feet (one 62 x 72 x 20 mm block for collision; two blue shoes as seen). It is hollow, as the toy is:
  its collision shape is a 3 mm shell of 177 convex patches on a latitude-longitude grid, laid out so that each scored hole is at the
  middle of a patch. Round each hole the patch is ten wedges cut along the hole's axis, leaving a clean 12 mm opening.
- **The five holes:** eyes, nose and mouth on his front centre line, their axes level (along x), at 106, 81 and 56 mm up; one ear hole
  on each side at 106 mm, axes along y. Behind each hole a rigid sleeve with a 10.8 mm bore runs from 2 to 16 mm in.
- **The interference fit:** in each sleeve four pads (up, down and either side), each 4 mm wide and 9 mm long with a 2 mm lead-in ramp,
  ride on radial sliders with an 8,000 N/m spring. At rest each stands 0.6 mm into the 10 mm peg's path, so a peg going in squeezes all
  four out by 0.6 mm (4.8 N each), and that load through contact friction (0.5 static, 0.4 sliding, plastic on plastic) is the fit:
  about 8 N to keep a peg moving, about 10 N to start it. The same grip holds a seated part. The pads are hidden; they stand for the
  plastic's own springiness, which a rigid shell lacks. A peg pushed in at an angle loads two pads harder than the others and binds.
- **Parts** (free rigid bodies, convex hulls): each has a 10 mm peg 16 mm long with a 1.5 mm chamfer at its tip, behind a 2.5 mm
  collar 15 mm across (14 mm on the ears). Eyes: a white plate 56 x 26 x 10 mm with two eyeballs, 10 g. Mouth: red lips 50 x 22 x 11 mm
  with teeth, 9 g. Nose: an orange bulb 24 mm long and 21 mm across, 6 g. Ears: pink, flat, 28 mm out from the collar, 38 mm tall and
  14 mm thick, 6 g each, the peg in the ear's plane.
- **Where a collar seats:** the body is curved, so a flat collar meets it at the rim's most outward point, up to 3 mm in front of the
  hole's centre on the surface (eyes 2.6 mm, nose 0.3 mm, mouth 1.2 mm, ears 3.0 mm, computed from the ellipsoid by `seat_depth`).
  "Seated" means the collar face is within 1 mm of that, the gap a feeler card checks at the collar's closest point.
- **Distractors:** a plastic cup, an apple and a marker (SimReady), loose, away from the work.

## Scoring

The tracker in `task.py` reads, every control step, the body's pose, each part's pose, and the arms' fingertip points.

- **Fell:** if the body ever tilts more than 45 degrees from upright, the trial scores 0. This record is cleared only at a reset
  (mechkit's `on_reset`), never by him being stood back up. It covers both being knocked over and being laid down on his back to use the
  table as the brace, which the instruction rules out.
- **Standing** (needed for any score at the end): within 10 degrees of upright, and the underside of his feet within 3 mm of the table.
  He may be lifted during the trial (held upright), as long as he ends standing.
- **Seated** (per part): its peg in its own hole (eyes top front, nose middle front, mouth bottom front, either ear in either side hole,
  each hole taking one part), the collar face no more than 1 mm out from where it meets the body, the peg within 1.5 mm of the hole's
  axis and within 6 degrees of square to it. Any rotation about the peg counts: the holes are round and the toy allows it.
- **1.0:** all five seated, standing, nothing moving (no origin moved more than 0.5 mm in a 0.05 s step), and every fingertip point at
  least 3 cm from the figure (sampled over the body, the feet and each part), all held for 1 s. Once earned, the credit lasts only while
  all five stay seated and he stays standing.
- **0.5:** three to five parts seated, without the rest of 1.0 (for example, hands still on him).
- **0.25:** one or two parts seated.
- **0:** none seated, or he fell at any time, or he is not standing.

Possible scores: 0, 0.25, 0.5, 1.0.

## Real trial: preparation, reset and measurement

- **Toy:** a new Hasbro classic Mr. Potato Head (F3244), body on its base with feet, with the eyes, nose, smile and both ears; the
  other parts stay in the box. Before trials, check the fit: pressing a part home on a kitchen scale should read roughly 0.5 to 1.2 kg
  (5 to 12 N); note the value.
- **Layout:** tape marks for the base (centred 27 cm in front of the arms' midpoint, facing them) and for each part at the positions in
  `task.json` "setup". Stand the eyes and the smile on their lower edges facing the arms; lay the nose on its side, peg pointing away;
  lay each ear flat, peg pointing at the body.
- **Reset (about 1 minute):** pull the five parts out (a firm straight pull), put them back on their marks, stand the body on its mark.
  Nothing is consumed.
- **Measurement, before anyone touches him:** he is standing if his shoes are flat on the table and he leans less than 10 degrees (a
  photo from the side). For each part, slide a 1 mm feeler card (or a 1 mm plastic card) at the collar's closest point to the body: a
  seated part's collar does not admit it. A part in the wrong hole does not count. He has "fallen" if the video shows him tipped past
  45 degrees at any time. Hands clear: the grippers at least 3 cm from him in the last second (video frame).
- **Rig safety:** the forces are small (under 15 N) and nothing is sharp, hot or wet.

## Fidelity

- **Sizes and masses are estimates.** Hasbro publishes no dimensions for the body or parts. We sized them from retail listings (the
  boxed set is about 15 x 10 x 19 cm) and product photos: a body about 13 cm tall overall, holes about 1 cm across, toddler-size
  parts of 6 to 10 g. A real toy may differ by several millimetres; the task's difficulty does not hinge on any one dimension.
- **The fit force is an estimate.** About 8 N to push a part home (10 N to start it) is our estimate for a new toy, not a measurement.
  The real-trial preparation above records the actual value with a kitchen scale. No force is scored.
- **Simplified shapes.** The body is an ellipsoid, not a sculpted potato; each part is a few convex pieces. The shell is rigid: a real
  polyethylene body flexes a little round a hole, which the pads stand for.
- **Only what is scored is modelled.** The five scored holes have bores and pads; the hat hole and the two arm holes are painted on.
  The storage door in his back is not shown.
- **Friction** is plastic on plastic, 0.5 static and 0.4 sliding, for the body, its pads and the parts (an estimate); the base slides on
  the table at about 1 N.
- **The robot side** is the stock simulated YAM station: rubber-like fingertip friction and soft position-controlled joints, which
  is why the demo's push runs its command ahead of the part (the arm's give is what pushes).

## Assets and licences

The body, base with feet and parts are authored meshes (SimReady has no Mr. Potato Head). The three distractors are NVIDIA SimReady
props, downloaded at first run from the pinned revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd` of `nvidia/simready-assets` on
Hugging Face, © NVIDIA, CC-BY 4.0:

- [dishware_plastic_cup_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_plastic_cup_a01): mass set to 0.02 kg, convex collider.
- [food_produce_apple_b01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/food_produce_apple_b01): mass set to 0.18 kg, convex collider.
- [stationery_marker_thin_blue_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/stationery_marker_thin_blue_a01): mass set to 0.01 kg, convex collider.

Nothing large is committed. `simready.py` is the loader from `tasks/cutters-off-pegboard/` (unchanged), `mechkit.py` the mechanism kit
from `tasks/heat-shrink-lug/` (unchanged, with its credit line) and `demokit.py` the closed-loop demo runner from `tasks/heat-shrink-lug/`
(unchanged): other teams' open-source files in this repository. "Mr. Potato Head" is a Hasbro trademark, named here only to identify
the real toy the task reproduces.

## Validation

VALIDATION_PLACEHOLDER
