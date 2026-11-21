# Move cookies from a sheet pan to a cooling rack with a turner

Four cookies lie on a quarter sheet pan. The robot picks up a solid turner (a flat kitchen spatula) by its handle, gets
the blade under each cookie in turn, carries it level over the pan's rim, leaves it flat on a wire cooling rack and
puts the turner down. No gripper may touch a cookie, no cookie may fall or flip, and the pan stays where it is.

The cookies in this task, simulated and real, are **non-perishable replicas** of a medium home-baked chocolate chip
cookie: a printed PETG disc of the same size, mass and underside shape, checked before every session (see "Real-world
setup"). Nothing is baked, eaten or thrown away, and the rig can be reset in about a minute.

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: the scene, the scorer (`Trial`, `score`, `success`) and the scripted open-loop `demo()` |
| `task.json` | The task's details for the board |
| `cookie_replica.scad` | The cookie replica for real trials (OpenSCAD; export to STL and print) |
| `mechkit.py` | Plain-text USD parts (meshes, colliders, free bodies) and the per-step and reset hooks |
| `simready.py` | Loads NVIDIA SimReady props (the distractors), downloading them once from Hugging Face; also used to fetch the turner's mesh |
| `demokit.py` | Only its grip helpers are used: they let a `Waypoint` carry a tilted gripper rotation |

## The knack

A cookie is a 10 mm disc lying flat on bare aluminium. A blade pushed at it from above its underside shoves it across
the pan; the blade has to be pressed onto the pan floor, nose down, so that its 1 mm edge passes under the cookie's
rounded rim, then pushed on while being levelled so the cookie rides up onto it. The pan's 28 mm rim limits how flat
the turner can be held near the edge (the neck and handle must clear the rim), so the cookies near the robot need a
steeper attack than the far ones. Once on the blade, the cookie is carried level (it slides off a blade tipped past
about 19 degrees), set on the rack and left there: the blade is tipped nose down onto the wires so the cookie's
leading edge rests on them, then drawn back so the cookie slides off the blade onto the grid instead of falling.
Grabbing a cookie with a gripper, tipping it off from a height, or tipping the pan to pour the cookies all fail.

## Run and score a trial

Needs `yam_arena` (yam-isaac-lab-arena) and Inspect Robots. From the repository root:

```bash
source ~/IsaacLab-Arena/yam.env
INSTR="Move the cookies from the sheet pan onto the cooling rack with the turner, without touching them or moving the pan, then put the turner down."

# the scripted demo (yam_scripted plays demo())
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/move-cookies-to-cooling-rack-with-turner/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=640 -E cam_height=360 --instruction "$INSTR" --scorer task_score --max-steps 2400 \
  --store-frames --log-dir ~/yam-eval/cookies-demo

# a model: the same command with --policy dream (and -E cam_width=320 -E cam_height=176)
```

The run prints `task_score` at the end. `COOKIE_DEBUG=1` prints, twice a second, each cookie's status and pose, the
turner's pose, the pan's shift and the grippers' clearances.

- **Runtime per trial:** the demo's plan is 94 s; every full-credit condition holds from about 89 s of simulated
  time and the episode ends after 1 s of that, at step 1,805 of the 2,400 allowed (about 90 s). Wall time was about
  14 minutes with seven trials sharing the GPUs; a lone trial is faster.
- **Seeds:** the layout is fixed (`-E object_jitter=0`) and the demo is open loop, so the seed changes nothing.

The first run downloads the SimReady packages (about 60 MB) into `~/.cache/simready`.

## Scoring

Each of the four cookies ends in one of three states, judged by `Trial` in `task.py` once per control step (20 Hz):

| State | Points | Meaning (simulation) | Real trial: how to tell |
|---|---|---|---|
| GOOD | +0.2 | not spoiled; on the rack with its whole outline inside the rack's frame; underside within 3 mm of the grid's top; tilted at most 8 degrees; not over the turner's blade; linear speed under 0.01 m/s and angular speed under 0.5 rad/s | calibrated side video and top/side photos against a ruler |
| SPOILED | -0.1 | At any time: fingertip within 5 mm of the cookie surface, inverted (flat underside up), or dropped. A drop is any point downward faster than 0.45 m/s (relative to the blade while supported on it); centre downward faster than 0.05 m/s while support gap exceeds 13 mm; or lowest point at a new rest more than 13 mm below its prior resting reference. During continuously observed support on the same turner blade, move that prior reference with the blade before comparing heights; clear this blade-relative reference whenever support is lost. Surfaces: tabletop, pan, rack, blade or another cookie; rest is within 4 mm. At the end, unsupported on the tabletop, under the rack, or elsewhere off pan/rack is spoiled. | Calibrated overhead and side video at 240 fps with a vertical ruler: measure fingertip clearance, motion against supports and inversion. The rigid replica does not reproduce fracture; these are task-defined handling proxies. |
| LEFT | 0 | anything else: still on the pan, on the turner, on another cookie, leaning on a rim | photo |

**Score = 0.2 x GOOD - 0.1 x SPOILED**, never below 0, so four GOOD cookies give 0.8. The score becomes **1.0** once all
four are GOOD and, for 1 s together, the pan is within 30 mm and 10 degrees of its starting place and not tilted more
than 10 degrees, a turner collider point is within 4 mm of the tabletop inside the finite table footprint, no point is more than 4 mm below the table, tilt is under 10 degrees, linear speed under 0.01 m/s and angular speed under 0.5 rad/s, and every finger
point is at least 5 cm from the cookies, the rack and the turner; the trial ends there. (The code measures to boxes that
enclose the turner's blade, neck and handle, to the rack's outline and to each cookie's underside centre less its radius,
so it never finds a finger farther away than it really is.) Before that the score is
re-read every step: full credit is never remembered, so a cookie touched or a turner picked up again takes it away.
**The whole trial scores 0** if the pan is ever lifted more than 20 mm or tilted more than 10 degrees: the pan has just come out of the
oven, and pouring the cookies from it is not the job. Doing nothing scores 0.

Possible scores: 0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8 and 1.0.

What the scorer cannot be fooled by:

- **Touch, drop and flip are history.** They are recorded in `Trial` and cleared only in `Trial.reset`, which mechkit's
  `on_reset` hook calls when the simulator resets the scene. Nothing that happens later (a cookie that is picked up by
  hand and then set down perfectly) clears them.
- **"On the rack" means resting on the wires**: the cookie's underside within 3 mm of the grid's top and still, and its
  centre not over the turner's blade. A cookie still on a blade lying on the wires is not GOOD; neither is one held up
  by a gripper (it is SPOILED by the touch anyway).
- **The last 0.2 needs the turner put down and both grippers clear**, checked with mechkit's `fingertips()`: 12 points
  on each gripper's fingers. `success()` is true only at a step where every full-credit condition holds, and the
  episode ends only after 1 s of such steps in a row (`SUCCESS_HOLD_S`).
- **The pan rule is permanent**: once tipped or lifted, the trial scores 0 whatever follows.

## Real-world setup

All products are ordinary kitchenware; nothing is custom except the printed cookie replicas.

| Object | Product | Size, mass | Price |
|---|---|---|---|
| Sheet pan | Nordic Ware Naturals Baker's Quarter Sheet, uncoated aluminium | 12.1 x 8.8 x 1.1 in inside (307 x 224 x 28 mm); simulated outside 33.6 x 25.3 cm over the rolled rim and 0.45 kg (estimates: weigh and measure yours) | about $14 |
| Cooling rack | Choice 8 1/2 x 12 in footed wire cooling rack for a quarter sheet pan (WebstaurantStore item 407CRKQRTRCH), chrome-plated | 305 x 216 mm; simulated grid: 2 mm wires 12.7 mm apart on 6 cross wires, top 19.5 mm up (not in the listing: measure yours and record it) | $5.49 |
| Turner | OXO Good Grips 13 in stainless steel flexible solid turner (model 1050062), or any solid turner of similar size | 13 in overall, 4 x 4 in blade; simulated: 330 mm, 85 x 91 mm blade, 100 g | $14.99 |
| Cookies | four replicas printed from `cookie_replica.scad` in PETG | 57 mm across, 10 mm thick, 16 +/- 1 g | under $1 of filament |
| Distractors | a wooden rolling pin, a mixing bowl, a salt shaker | as in `task.json` | |

**The cookie replica.** The size and mass are those of a medium chocolate chip cookie made from a recipe with butter in
USDA FoodData Central (2 1/4 in, 16 g). The replica's underside is flat to a 49 mm circle and rounds up to the rim over
the outer 4 mm, the way a baked cookie's edge lifts off the sheet; it is the same profile as the simulated cookie
(`COOKIE_PROFILE` in `task.py`). Print it flat side down on a smooth bed, 0.2 mm layers, with the infill set so that
it weighs 16 +/- 1 g (about 35 % gyroid; solid would be 28 g). Print six and keep two as spares.

**Checks before each session** (a few minutes, ordinary tools):

1. Size: 56 to 58 mm across (ruler, two diameters at right angles), 9.5 to 10.5 mm thick (calipers or ruler).
2. Mass: 15 to 17 g (kitchen scale).
3. Sliding: lay each replica on the bare pan, raise one end of the pan slowly, and read the angle at which it starts
   to slide with a phone inclinometer. It must be between 22 and 29 degrees (the simulation's cookie on the pan
   slides at 25 degrees). Wipe the pan and the replica with a dry cloth first; if a replica slides too easily, roughen
   its underside lightly with 400-grit paper; if too hard, wipe it with a damp cloth and dry it.

The rigid replica cannot reproduce cookie fracture, bending or adhesion. Touch, fall and inversion are explicit task-scoring
proxies for careful handling; they do not predict whether a particular recipe would break. In the real trial, judge these
conditions from calibrated overhead and side video against a ruler. For a drop, use the same three criteria as the scorer:
point speed over 0.45 m/s (relative to the blade while supported on it), downward centre motion over 0.05 m/s while more
than 13 mm above a supporting surface, or a new resting height more than 13 mm below its prior resting reference. During continuously observed support on the same turner blade, move that prior reference with the blade before comparing heights; clear this blade-relative reference whenever support is lost. A
supporting surface is the pan, rack, turner blade, table or another cookie; rest means within 4 mm. Inversion means the
flat underside faces up.

**Layout.** Mark with tape: the pan's outline, the four cookie spots on the pan's floor (the positions in `task.json`),
the rack's outline and the turner's outline. Positions are measured from the midpoint between the two arm bases.

**Reset.** Put the replicas back on their spots, the pan and the rack on their outlines and the turner on its outline,
blade away from the robot. About one minute. Nothing is consumed.

**Measurement.** Record side and overhead views at 240 fps with a vertical ruler in the scene. Use these calibrated videos
to assess fingertip clearance, drops, inversion and support heights against the numerical thresholds in the scoring table.
Take top and side photos before resetting to record final cookie placement, tilt, pan position and turner rest.

**Rig safety.** Nothing is hot, sharp or heavy: the pan is at room temperature, the turner's blade edge is about
1 mm thick and not sharpened, and the heaviest object weighs 0.45 kg. The replicas are dry plastic, so nothing greases
the gripper pads.

## How the simulation works

- **Scene.** Free rigid bodies in PhysX at 240 Hz. The pan is a 1.5 mm floor with four flared walls (each wall five
  convex pieces) and a rolled bead, at the catalogue size. The rack is a frame, 6 cross wires, 16 long wires and 4
  feet, at the catalogue size; the grid's height and wire spacing are estimates. The cookies are convex discs with
  the replica's profile. The turner shows NVIDIA SimReady's `kitchenware_spatula_i01` mesh (a 330 mm solid turner),
  with its blade thinned to 1 mm in the mesh and in the collider; its colliders are simple convex pieces: the blade (its
  front 8 mm bevelled to 0.35 mm), the arched neck in three pieces so nothing fills in under the arch, and the
  19.5 x 19 mm handle. The prop's blade has three long slots; the simulated blade is solid (its collider is one plate),
  and that plate is drawn over the slots too, so the turner looks and behaves like the solid OXO blade used in real trials.
- **Friction** (static, the pair's mean): cookie on the pan 0.48, cookie on the blade 0.35, cookie on the wires 0.53,
  turner handle on the gripper pads higher (a separate grip material on the handle). These are estimates, not
  measurements; the replica's sliding check above ties the real one to the simulated pan value.
- **Masses:** pan 0.45 kg (catalogue), rack 0.28 kg (shipping weight 0.62 lb, less packing: estimate), turner 100 g
  (shipping weight 0.25 lb: estimate) with its centre of mass 6 cm behind the blade's heel (estimate: a steel blade and
  a lighter plastic handle), cookies 16 g.
- **What is simplified:** the rigid replica cannot reproduce cookie fracture, bending or adhesion; touch, drop and inversion are operational task proxies, not fracture predictions; the blade does not flex (the real OXO blade is flexible; the simulation uses a rigid blade, so flex and its effect on contact are unmodeled); the pan is
  not hot; uniform friction per pair.

## The scripted demo

`demo()` drives the right arm alone, open loop, from the published starting layout (it reads no simulator state). It
grips the turner's handle across its width, lifts it, and for each cookie, near ones first: lowers the blade over the
pan nose down (12 to 16 degrees for the near row, so the neck clears the rim; 8 degrees for the far row) until its
edge presses on the floor 12 mm short of the cookie, pushes it in under the cookie while flattening to 8 degrees (the
cookie is shoved 1 to 2 cm and then rides up onto the blade), lifts it, levels it over the rim, carries it to the rack,
sets the blade on the wires, tips it 20 degrees nose down about its edge (the edge rests on the wires, pressed no
more than 1 mm: pressing harder drags the loose 0.28 kg rack along the table), and draws it back 4.5 cm, leaving the
cookie on the grid. It then lifts the turner 13 cm above the wires before moving sideways, because the handle can pitch
between the pads and let the blade hang nose down. After the fourth cookie it lays the turner back on its outline,
opens and moves clear.

The left arm is not needed: the pan is heavy enough not to move under the blade's light push. A two-handed route
(steadying the pan with the other gripper) is allowed and scores the same.

**Results.** Two runs of the final `task.py` with the command above both scored **1.0** (`task_score: 1`), each
ending at step 1,805 once every full-credit condition had held for 1 s: four cookies resting flat on the wires, none
touched, dropped or inverted, the pan unmoved, the turner within 1 cm of its marked footprint and the gripper clear. The fastest
downward speed of any point on any cookie during the demo was 0.35 m/s (the drop limit is 0.45 m/s).

## Red team

Each variant is `task.py` with another `demo()` (the files `redteam_<name>.py` were run with the same command and then
deleted):

| Variant | What it does | Score | Why |
|---|---|---|---|
| none | the arms stay at their ready poses | 0 | all four cookies still on the pan (LEFT) |
| hand | the careless way: pinch a cookie by its rim with the gripper, no turner, and set it on the rack | 0 | the cookie ends flat on the wires, but the touch spoiled it at once and nothing clears that; the other three stay on the pan |
| drop | the full demo, but each cookie is left by yanking the blade back from under it 4 cm above the wires | 0 | three are dropped (point speeds up to 0.77 m/s, or a rest more than 13 mm lower than the last) and SPOILED; the fourth slid off the nose-down blade with its rim already on the wires (peak 0.33 m/s), the intended way, and is GOOD: 0.2 - 0.3, floored at 0 |
| fast | the full demo, but each cookie is carried 12 cm above the wires and brought down 10 cm in 0.28 s | 0 | the sudden stop pitches the turner in the grip and flings each cookie off the blade (tilts of 45 degrees and more): all four dropped |
| pour | lift the pan's near rim to tip the cookies off (run on the previous revision of the scorer; the pan rule is unchanged) | 0 | the pan is lifted more than 20 mm or tilted more than 10 degrees: the whole trial scores 0 |
| hold | the full demo, but the turner is never put down: it is held above the rack at the end | 0.8 | four GOOD cookies give 0.8; the last 0.2 needs the turner down and the gripper 5 cm clear |

A one-arm variant is not separate: the demo itself uses one arm, which the task allows (a second gripper steadying
the pan or rack is allowed too and scores the same).

## Importance and sources

Claimed category: **Societal relatability**. Each claim is followed by its source; where a statement is our own
inference or estimate, it says so.

- **The real operation.** Cookie recipes end with this step. The Original Nestlé Toll House Chocolate Chip Cookies
  recipe: "Cool on baking sheets for 2 minutes; remove to wire racks to cool completely"
  ([recipe as reproduced by CBS News](https://www.cbsnews.com/news/chocolate-chip-cookies/)). The tool for it is a
  turner (a flat solid spatula), sold for exactly this kind of lift (see the turner below). *Inference:* that most
  adults have moved cookies or similar baked items from a sheet with a spatula is our judgement, not a survey figure.
- **Why it is a robot skill, not a classification.** The difficulty is physical: a 1 mm blade has to pass under a
  10 mm, 16 g disc lying on bare aluminium without shoving it away, the 28 mm pan wall limits the attack angle near
  the robot, and the cookie has to be carried level (it slides off past about 19 degrees in the simulation) and left
  on 2 mm wires without a fall. Nothing is hidden or chosen; every step is contact with a tool.
- **Paid work: background only, not claimed.** Bakers (SOC 51-3011): 262,400 employed in 2025, from the BLS
  employment projections shown on [O*NET OnLine 51-3011.00](https://www.onetonline.org/link/details/51-3011.00).
  Its task statements include "Place dough in pans, molds, or on sheets, and bake in production ovens or on grills"
  and "Roll, knead, cut, or shape dough to form sweet rolls, pie crusts, tarts, cookies, or other products"; none
  names moving baked goods to racks, and 262,400 is below the jobs category's bar, so the jobs category is not claimed.
- **Cookie size and mass.** USDA FoodData Central, "Cookies, chocolate chip, prepared from recipe, made with butter"
  (FDC ID 172808): measure "1 cookie, medium (2-1/4" dia)" = 16.0 g
  ([FoodData Central record 172808](https://fdc.nal.usda.gov/portal-data/external/172808), field `foodMeasures`).
  *Estimate:* the 10 mm thickness (16 g in a 57 mm disc at about 0.7 g/cm3) and the rounded underside rim.
- **Sheet pan.** Nordic Ware Naturals Baker's Quarter Sheet, uncoated aluminium, 12.1 in inside length on the
  [maker's product page](https://www.nordicware.com/products/naturals-bakers-quarter-sheet/); 12.1 x 8.8 x 1.1 in in
  the title of a [Walmart listing](https://www.walmart.com/ip/265980133) ("Nordic Ware Naturals Aluminum Quarter
  Sheets, 3 Pack, Silver, 12.1" x 8.8" x 1.1""). *Estimates:* the 0.45 kg mass, the 1.5 mm floor and the 10 mm
  wall flare (which set the simulated outside size, 33.6 x 25.3 cm over the rolled rim); measure the real pan and record it.
- **Cooling rack.** Choice 8 1/2 x 12 in chrome-plated footed wire cooling rack for a quarter size sheet pan, item
  407CRKQRTRCH, $5.49, 0.62 lb shipping weight
  ([WebstaurantStore](https://www.webstaurantstore.com/choice-8-1-2-x-12-chrome-plated-footed-wire-cooling-rack-pan-grate-for-quarter-size-sheet-pan/407CRKQRTRCH.html)).
  *Estimates:* the listing gives no height or wire spacing; 19.5 mm and 12.7 mm are ours, and 0.28 kg is the
  shipping weight less packing.
- **Turner.** OXO Good Grips 13 in stainless steel flexible solid turner, model 1050062: 4 x 4 in blade, 13 in
  overall, 0.25 lb shipping weight, $14.99
  ([WebstaurantStore](https://www.webstaurantstore.com/oxo-1050062-good-grips-13-stainless-steel-flexible-solid-spatula-turner/2971050062.html)).
  The simulated turner is SimReady's prop, not a model of this product; its size (330 mm, 85 x 91 mm blade) is close.

### Distinct from

| Nearest task | What it tests | Why this is different |
|---|---|---|
| TK-3AWYMC (accepted): slide a quarter sheet pan onto a toaster oven's rails | moving the pan itself into rails | here the pan must not move; the skill is a tool getting under loose items on it |
| TK-MP8KLR (pending): press spritz cookies onto a baking sheet | operating a cookie press | no tool transfer of finished items |
| TK-4NVEUZ / TK-QLUVBN (closed): take a burger patty's temperature while holding it with a turner | holding food still with a turner while probing | here the turner must get under and lift fragile items and release them elsewhere |

Food turning (flipping a pancake or patty with a spatula) appears in other task sets; nothing here is turned over, and
a flipped cookie is spoiled.

## Assets and credits

The turner's mesh and the distractors come from the Hugging Face dataset
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`kitchenware_spatula_i01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/kitchenware_spatula_i01) (the turner)
- [`kitchenware_rollingPin_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/kitchenware_rollingPin_a01)
- [`kitchenware_bowl_b01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/kitchenware_bowl_b01)
- [`kitchenware_shaker_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/kitchenware_shaker_a01)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time: the
turner's mesh is read from its USD file and shown on a body with our own colliders, its blade thinned from about 2.5 mm
to 1 mm and its faces coloured steel (blade and neck) and black (handle); masses are set (turner 100 g, rolling pin
0.35 kg, bowl 0.30 kg, shaker 0.12 kg, whose two bodies are welded into one); the rolling pin's spin is damped
(2 /s). Nothing large is committed: the files download at the first run.

`mechkit.py` and `demokit.py` are taken unchanged from another team's open-source task in this repository,
`tasks/heat-shrink-lug/`; `simready.py` from `tasks/cutters-off-pegboard/`. The pan, rack and cookies are built
here from simple meshes: SimReady has no sheet pan or wire cooling rack (its nearest prop, `kitchenware_tray_a01`, is a
267 x 450 x 71 mm serving tray), so these two plain structural parts are primitives at the products' catalogue sizes.
