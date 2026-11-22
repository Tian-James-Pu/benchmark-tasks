# Roll a filled glazed planter across a patio, onto a taped spot

A 14-inch glazed ceramic planter, filled with potting mix, stands on a porcelain patio. It has to go 24 cm to the right,
onto a square of painter's tape. Filled, it weighs 18.5 kg. That is too heavy to carry comfortably, and dragging it scrapes
the paving: paving guides say to lift planters, not drag them. People without a dolly move a pot like this by rolling it.
They tip it a little onto the edge of its foot, roll it along that edge a hand's width at a time, and stand it back up.

The knack: the pot wants to fall back upright, with about 7.8 N m at 22 degrees of tilt, and it goes over at 37 degrees. So
both hands hold the rim against that pull, between about 15 degrees (enough for the heel to roll) and the 32-degree limit,
while turning it like a steering wheel so that the heel rolls instead of skidding. Then they stand it up, let go, take a new hold and go again. Lift it, drag it, or let it tilt past
32 degrees, and the trial scores 0.

Instruction: **"Move the planter onto the taped square on the patio."** The instruction names the goal, not the method.

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: scene, scoring (`score`, `success`, the per-step tracker `Trial`) and the closed-loop demo `demo_script` |
| `mechkit.py` | Builds the planter and the pavers as USD meshes and convex colliders, and runs the per-step and reset hooks |
| `demokit.py` | Runs `demo_script` on the harness's own embodiment and arm plans, and records the video |
| `simready.py` | Loads the SimReady distractors, downloading them once from Hugging Face |
| `task.json` | The task's details for the board |

## Run a trial

Environment: the `yam_arena` simulator package and Inspect Robots. From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --policy dream --scorer task_score --max-steps 3000 \
    --instruction "Move the planter onto the taped square on the patio."
```

The scene is the same for every seed: there is no randomisation, and `object_jitter=0`. A trial is at most 3,000 control
steps (150 s at 20 Hz). The scripted demo takes 1,345 steps (67 s), so the limit is about 2.2 times the demo.
Runtime: about 35 s of wall time per 1,000 steps without cameras (CPU physics, `--fast`). With the three cameras at
640 x 360 on a shared RTX PRO 6000 GPU it ran at 1.5 to 3 steps per second, depending on how busy the GPU was.

Model run: Dream-1 scored 0.0 over the full 3,000 steps; it reached for and touched the rim but never tipped or moved the
planter.

## The demo

From this folder:

```bash
python demokit.py task.py OUT                                    # the full-score run, with video and setup.png
python demokit.py task.py OUT_checks --fast --mode none,onearm,tip,hold,drag,shove,lift   # the failure checks
```

The demo is a scripted reference, not a policy. It reads the planter's pose from the simulator (the planter is in full view
of the cameras) and never reads the scorer. Each of its five strokes goes like this:

1. **Hold.** Both hands take the rolled lip, one 30 degrees either side of the bearing that faces the arms, set 15 degrees
   round ahead of it. Each hand comes from above and outside, pointing in toward the pot's axis 50 degrees from straight down.
   Its fingers close across the roll's 3.2 cm section on a line 50 degrees below level: one finger on the inside top of the
   roll, the other under its outer bulge, about 45 N each. A pull toward the arms then bears on the inner finger, so the
   hold depends less on friction. A pinch closing almost vertically on the roll slipped off its round at this mass.
2. **Tip.** The hands tip the pot 22 degrees toward the arms, about the edge of its heel. The heel stays where it is: the
   pot turns about it.
3. **Roll.** The hands turn the pot 30 degrees clockwise (seen from above) about its own tilted axis, moving round the rim
   like hands on a steering wheel. Meanwhile its centre travels to the right by the heel's radius times that angle, about
   5.5 cm, so the heel rolls along the paver. Each stroke is aimed at the cross from where the pot now stands, within 12
   degrees of straight right, to take out drift.
4. **Stand it up.** The hands stand the pot upright, let go and back off along their approach.

The fifth stroke is shortened to finish on the cross. The hands then move away, and the pot must stand still, hands clear,
for 1 s.

Closed loop: every step, each hand's command is shifted until the lip point it holds is where the planned pose puts it. This
takes up the arms' sag under the load (2 to 3 cm), up to 5 cm.

Results (`task_score` from the harness):

| Run | Physics | Score | Steps | Notes |
|---|---|---|---|---|
| final A | GPU, cameras, video | 1.0 | 1,345 | the submitted video and screenshot; no blank frames |
| final B | GPU, cameras, video | 1.0 | 1,345 | a second fresh run; no blank frames |
| check | CPU (`--fast`) | 1.0 | 1,319 | the foot slid 5.2 cm in all, against the 10 cm limit; most tilt 23.0 degrees; foot centre 0.5 cm from the cross |
| check at 19.5 kg | CPU (`--fast`) | 1.0 | 1,322 | the same script with the mass raised to 19.5 kg (a heavier pot or wetter mix) |

Failure checks (`--mode`, CPU physics, 18.5 kg):

| Mode | What the script does | Score |
|---|---|---|
| `none` | nothing | 0.0 |
| `onearm` | the same strokes with the right hand alone | 0.0 (one hand doesn't tip it: the pot never leaves its foot) |
| `tip` | the same strokes, tilted 34 degrees instead of 22 | 0.0 (it reaches the cross, but passed 32 degrees) |
| `hold` | the full demo, but stops at the cross still holding the rim | 0.0 (hands not clear) |
| `drag` | holds the lip and slides the pot upright toward the cross | 0.0 (the arms move it about 2 cm: not 10 cm closer) |
| `shove` | pushes the pot's side low down with a closed hand | 0.0 (it doesn't move) |
| `lift` | tries to lift it 8 cm | 0.0 (it doesn't leave the pavers) |

At 18.5 kg the arms can't lift or drag the pot, so the last three never trip the lift and drag rules. Those rules were also
checked offline, by feeding the tracker hand-made trajectories (no simulator). A pot dragged upright 24 cm to the cross scores 0
(slid 24 cm). A pot lifted 2 cm and carried there scores 0 (lifted). A pot tipped to 34 degrees and set back scores 0. An
ideal roll of 30 degrees on the heel counts 0.0 cm of slide, or 0.3 cm if the travel is off by 6%. Standing at the cross for 21
steps scores 1.0, and for 10 steps 0.5. Standing 6, 13 and 18 cm from the cross scores 0.5, 0.25 and 0. At the cross with a
fingertip 3 cm from the lip, tilted 3 degrees, or moving 2 cm/s, it scores 0. A pot that reached 1.0 and was then pushed 10 cm
away drops to 0.25. A pot that was dragged and then stood at the cross scores 0.

## Scoring

The tracker (`Trial.step`) runs after every control step's physics; `score()` returns its level, and `success()` is true
while the level is 1.0. Its harm record is cleared only by the reset hook, never because the pot looks right again.

**Harm: 0 for the rest of the trial**
- **Tipped:** the pot's axis ever more than 32 degrees from vertical. With its centre of mass 15 cm up the axis, its balance
  point on the heel is 37 degrees, and past it the pot goes over. 32 degrees leaves 5 degrees of margin. Rolling needs 15 to
  25 degrees (the demo uses 22).
- **Lifted:** the lowest point of its foot ever more than 1 cm above the pavers. Tilting onto the heel keeps one point down,
  so it doesn't count; the whole pot off the ground does.
- **Dragged:** its foot slid more than 10 cm in all. Every step the tracker finds the foot point touching the pavers (the
  lowest heel point on the side the pot leans to, or the whole foot ring when it stands flat). It measures how far that same
  point of the pot moved across the pavers since the last step, and adds that up. Rolling and pivoting leave the touching
  point where it is, so they add almost nothing (the demo adds about 1 cm per stroke of skid). Dragging moves it the full
  distance. 10 cm is under half the trip, so a pot dragged most of the way is out.

**Standing:** axis within 2 degrees of vertical; foot centre within 3 mm of the paver tops; moving under 1 cm/s and turning
under 3 degrees/s; every fingertip point (12 on each gripper) at least 5 cm from the pot's outline, its open top counted as
closed at the rim.

**Levels:** 1.0 once it has stood with its foot centre within 3 cm of the tape cross for 1 s (20 steps running). 0.5 standing
within 8 cm of the cross (including while that 1 s runs). 0.25 standing at least 10 cm closer to the cross than its start
(24 cm away). Otherwise 0. Possible scores: 0, 0.25, 0.5, 1.0. Credit doesn't stick: if the pot is moved after reaching a
level, the level follows where it now stands.

## Scale and mass

Nothing is rescaled. The planter is modelled at the product's listed size: 35.6 cm across the lip and 30.5 cm tall. Its mass
is the real filled pot's:

- the pot: 22.2 lb (10.1 kg), from the retailer's listing for the Large size of this series (see Sources). We could not
  load the white listing's specification table; one search result quoted 27.8 lb for it. For a real trial, weigh the filled
  pot and bring it to 18.5 kg by how wet the mix is. The demo also scored 1.0 at 19.5 kg in CPU physics.
- the mix: 15 L of potting mix fills the pot to 5 cm below the rim (from the modelled inside profile: 15.2 L with a 13 mm
  wall). A gardening guide gives "each 10 liters of moist mix ≈ 5–7 kg", so 7.5 to 10.5 kg. The model takes 8.4 kg
  (0.56 kg/L), a watered-and-drained mix.
- together 18.5 kg, centre of mass 15 cm up the axis (pot shell about 15.5 cm, mix about 14.4 cm).

The real load is what the demo carries. Tipping from upright takes about 18 N m about the heel, about 63 N at the rim
between the two hands. Holding at 22 degrees takes about 7.8 N m. The YAM arms manage this with their shoulder joints (27 N m
limit) and a hold that relies less on friction. They can't lift the pot or drag it. Many people could lift 18.5 kg, but a
36 cm glazed pot is awkward to carry and its rim is no handle, which is why people roll it.

## Importance and sources

Category: **Societal relatability.** Container plants on patios are common, and moving a full pot to follow the sun, make
room or bring it in is a chore most people with a patio, balcony or porch have done. Garden Research's National Gardening
Survey (2023 edition) reports: "Participation in lawn and gardening activities remained high in 2022, with a 5-year high of 80%
of households taking part." The difficulty is physical and visible in a 10-second clip: a heavy round pot held tipped on its
edge and rolled with both hands, under load, through several dependent strokes.

Why rolling, and why the rules:
- **Don't drag:** "Lift heavy garden furniture, planters, barbecues and other equipment rather than dragging them across the
  slabs" (Paving Slabs UK, "What Should You Avoid with Porcelain Paving?"). Dragging 18.5 kg on an unglazed ceramic
  foot can mark the slabs and chip the foot.
- **Too heavy to lift casually:** filled pots weigh far more than empty ones ("each 10 liters of moist mix ≈ 5–7 kg",
  Gardening Beyond, "How To Move Heavy Pots Around The Garden"). The same guide warns: "Never carry by the rim alone. Old clay
  rims snap under point pressure." Rolling doesn't carry the pot by its rim. The foot carries its weight, and the hands only
  hold the tilt (about 30 N between them at 22 degrees). Lifting is scored 0.
- **Don't let it go over:** an 18.5 kg glazed pot that tips past its balance point falls on its side. It spills its mix and
  can crack, and a falling pot can land on a foot.

Sources (checked 21 November 2026):
- At Home, "Rolled White Ceramic Outdoor Planter, Large", item 125032510:
  https://www.athome.com/rolled-white-ceramic-outdoor-planter-large/125032510.html (14 x 12 in, white glazed, rolled rim).
  The same series' "Rolled Aqua Ceramic Outdoor Planter, Large", item 125032560, lists 14 x 14 x 12 in and 22.2 lb:
  https://www.athome.com/rolled-aqua-ceramic-outdoor-planter-large/125032560.html. Prices vary by store; any 14-inch,
  rolled-rim glazed planter of similar weight serves.
- Paving Slabs UK, "What Should You Avoid with Porcelain Paving?":
  https://www.pavingslabsuk.co.uk/blogs/porcelain-paving-slabs/what-avoid-with-porcelain-paving
- Gardening Beyond, "How To Move Heavy Pots Around The Garden | Safe Moves": https://gardeningbeyond.com/?p=31816
- Garden Research, National Gardening Survey 2023 Edition: https://gardenresearch.com/view/national-gardening-survey-2023-edition/

## Distinct from

| Nearest | What it tests | How this task differs |
|---|---|---|
| TK-HYNFZE, "Get a heavy carton up onto a low platform by setting its front edge on the step and pushing" (accepted) | tipping a box once so its edge rests on a step, then pushing it up | No step and no pushing. A round pot is held tipped on a curved heel, near a tipping point, and rolled through five strokes with the hands moving round its rim. It never slides. |
| TK-49K7DX, "Turn a small steel drum on its side and roll it into an angled bay" (not accepted: a 1.2 kg miniature) | laying a drum down and rolling it on its side | Full size and full mass, 18.5 kg. The pot is never laid down: it rolls on the edge of its foot, tipped 15 to 25 degrees, and stands up at the spot. |
| TK-DLNFPH, "Walk a washing machine into its space, corner by corner" | walking a box by pivoting on alternate corners (closed without a decision) | A round foot has no corners to pivot on. The hands keep the pot tipped and steer it as it rolls. |

The board also has planter tasks that place or hang a pot (hang a planter by its ring, seat a flowerpot in a cord hanger,
double-pot two plants), and tasks that move heavy things with a hand truck. None rolls a heavy pot on its edge. We found
nothing like it in LIBERO, RoboCasa, SimplerEnv or RoboTwin, which are tabletop or kitchen pick-and-place and articulation
tasks. BEHAVIOR-1K moves objects with an abstracted grasp, so a move there is pick-and-place. RoboARC's standard tasks are
puzzles, knots and assemblies.

## Real trial: setup, reset, measurement

**Parts** (roughly $150 to $250 in all, an estimate): one 14-inch rolled-rim glazed ceramic planter (the At Home Large above or
equivalent); two 60 x 60 x 2 cm porcelain patio pavers (about 16 kg each); a 2 cu ft bag of potting mix; 24 mm blue
painter's tape; a bathroom scale; a steel rule; a phone with an inclinometer app.

**Setup.** Use a sturdy table rated for at least 60 kg: pavers plus pot come to about 51 kg. Lay the pavers side by side
across it, edges 16 cm in front of the arm bases, with a 5 mm gap filled with grout or sand. Fill the pot with mix to 5 cm
below the rim, water it and let it drain, and adjust until it weighs 18.5 +/- 0.5 kg. Tape the 40 cm square (inner edges)
and the 8 cm cross on the right paver, the cross at (48, -12) cm. Draw a 20 cm pencil circle at (48, 12) cm on the left paver
for the start. Stand the pot in that circle. Tape the phone inside the rim on the far side from the arms, logging tilt. Set the
distractors as listed in `task.json`. Film from above and from the side (the side camera looking along y), at 30 fps or more.

**Measurement**, read at the end before anyone touches the scene:
- Foot centre: lay the steel rule flat on the paver from the foot's edge to the square's inner tape edge, in front, behind,
  left and right. The offset along x is (back gap - front gap) / 2, along y (left gap - right gap) / 2, and the distance
  from the cross is the root of the sum of their squares. Centred, every gap is 10 cm.
- Standing: the phone reads within 2 degrees of level; the pot is still, and every gripper finger at least 5 cm from it, on
  video for 1 s.
- Tipped: the phone's logged maximum tilt over 32 degrees.
- Lifted: the side video shows daylight more than 1 cm under the whole foot (a 1 cm block beside it gives the scale).
- Dragged: before the trial, put a short radial strip of contrasting tape on the foot's edge every 30 degrees. In the
  overhead and side video, the touching part of the foot should stay put on the paver while it touches. Add up how far it
  moves while touching, frame to frame. Over 10 cm is a drag.

**Reset** (about 2 minutes): roll the pot back into its start circle the same way, or with a plant dolly. Check its weight
and mix level, and re-set any distractor that moved.

**Rig safety.** Nothing is sharp, hot or wet: the glazed rim is rounded and the mix is drained. The pot could go over if an
arm lets it tilt past its balance point. Keep people out of the space beside and in front of the pavers. Put a foam mat on
the floor in front of the table, and have the operator ready at the e-stop. The arms can't lift the pot, so it can't be swung.

## Model details

- **Planter** (one free rigid body): the lathe profile of a 14-inch rolled-rim pot, estimated from the listing's dimensions
  and photos. Measure your pot and adjust `BODY`/`ROLL` if it differs. The glazed body up to the soil surface is 36 convex
  wedges. The rolled lip and the wall above the soil are 32 segments, each two convex hulls, so the open top above the soil
  stays open and the roll can be gripped. The foot ring is 20 cm across with a rounded heel. Mass 18.5 kg, centre of mass set
  15 cm up the axis. Friction of its foot on the pavers: 0.7 static, 0.6 dynamic (ceramic on porcelain). The potting mix is
  part of the rigid body: it can't spill or shift.
- **Pavers** (fixed): two visual slabs and a darker joint over one flush box collider. The tape is visual only (0.6 mm).
- **Distractors:** SimReady sunglasses and a mug on the far corners of the pavers, and a water bottle on the table beyond them.
  The scorer ignores them, and the demo doesn't touch them.
- **Not modelled:** breakage, chipping and scratching (these are scored from the pot's pose), the mix's movement, and any
  flex of the rim.

## Assets and credits

The distractors come from the Hugging Face dataset
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`accessory_eyeglasses_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/accessory_eyeglasses_a01) (sunglasses)
- [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01) (mug)
- [`food_beverage_bottledWater_b01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/food_beverage_bottledWater_b01) (bottle of water)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time: each is
made a loose rigid body with its mass set (sunglasses 30 g, mug 350 g, bottle 550 g), and the mug's triangle-mesh collider
falls back to a convex hull (PhysX does this for moving bodies). Files download at first run; nothing large is committed.

The planter, the pavers and the tape are meshes generated by `task.py` with `mechkit.py`. There are no textures and no
other third-party assets. `mechkit.py` and `demokit.py` are unchanged copies of the open-source kit used by the
`heat-shrink-lug` task in this repository. `simready.py` is adapted from `tasks/cutters-off-pegboard/simready.py` in this
repository, and its loader is unchanged.
