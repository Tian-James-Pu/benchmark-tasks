# Flip an empty party cup off the table edge with one flick of its rim so it turns over and lands mouth down

An empty Solo 16 oz red party cup stands mouth up at the edge of a table, its rim 1.5 cm out over the edge, exactly as
it is set up for the party game Flip Cup. The robot has to flick the overhanging rim upward so that the 11.8 g cup turns
half a turn in the air and lands mouth down on the table.

Instruction: *"Flip the empty cup off the table edge so it lands mouth down on the table."*

The knack is a narrow window of impulse. The cup gets one strike from below, lasting a few hundredths of a second, and
then flies on its own. In our trials a cup flicked with the finger moving at about 1.5 m/s left the table spinning at
14-17 rad/s and had to be turned 140 degrees or more by the time its rim met the table, or it fell back onto its side.
A little more and it went over past mouth down, or rolled off the edge. A little less and it only tipped over. Holding
the cup and turning it over by hand scores nothing: the cup has to turn on its own. Nothing can be corrected once the
finger has left it.

Files: [`task.py`](task.py) (scene, scorer, scripted demonstration), [`task.json`](task.json),
[`mechkit.py`](mechkit.py) (builds the cup and the tabletop), [`simready.py`](simready.py) (loads the distractor props),
[`demokit.py`](demokit.py) (runs the closed-loop demonstration script).

## Importance and sources

- **Most adults know this motion.** Flip Cup is one of the best-known party games. Wikipedia describes the set-up used
  here: "the cup is placed open side up at the edge of the table" and the player flips it "by flicking or lifting the
  bottom of the cup until it flips and lands face down on the table"
  ([Wikipedia: Flip cup](https://en.wikipedia.org/wiki/Flip_cup)). The same flick also turns over a cup to dry or to
  stack. The cup in the task is empty: the skill is the flick, and nothing is drunk.
- **The real product.** Solo P16R 16 oz red plastic cold cup, high-impact polystyrene: top diameter 3.876 in
  (9.85 cm), base diameter 2.53 in (6.43 cm), height 4.73 in (12.0 cm), sold 1,000 a case
  ([distributor listing, Dart/Solo P16R](https://www.imperialbrady.com/catalog/product-detail/solo-3-9-in-16-oz-red-hips-cold-party-cup-disposable-3-9x4-7x2-5-in-1000-case?id=22341)).
  The case weight is listed as 26.08 lb for 1,000 cups ([GoFoodservice, Solo P16R](https://www.gofoodservice.com/p/solo-p16r)),
  which is 11.8 g a cup including its sleeves and carton. We use 11.8 g: weigh the cup for a real trial (an empty
  16 oz party cup weighs 10-12 g). Retail packs of 50 cost about $10.
- **Why it is hard for a robot.** Most manipulation benchmarks are quasi-static: the robot holds an object or pushes it,
  and can correct as it goes. Here the robot touches the object once, for a few hundredths of a second, and the
  outcome is decided by the impulse it gave. A model has to pick a strike speed and direction inside a narrow window
  and pull its hand out of the cup's way, from video alone.

## Distinct from

| Nearest task | What it tests | Why this one differs |
|---|---|---|
| PR #2746, pull-back toy car through a gate (accepted) | storing energy in a toy's spring and releasing it cleanly | the toy's own spring drives it along the floor; here the robot's strike is the only energy, and the object flies and turns in the air |
| PR #3080, flick a prize wheel to stop on JACKPOT (pending) | an impulse to a wheel on a fixed axle, coasting to a stop | one degree of freedom on a bearing; the cup is a free body that has to leave the table, turn half a turn and land on its rim |
| PR #2722, Twister spinner flick (closed) | a flick of an arrow on a pin, at least two turns | again a pivoted arrow, and "at least" a number of turns; this task has both a lower and an upper limit (half a turn, not a whole one) |
| PR #3045, toss diced potato in a sauté pan (pending) | tossing many small pieces with a pan | the pan is held throughout and many pieces are scored by count; here one object is struck once, bare-handed, and scored by its landing orientation |

LIBERO, RoboCasa, SimplerEnv, BEHAVIOR and RoboDojo have no task in which an object is thrown or flicked into a target
orientation; their cup tasks are pick-and-place, pouring or stacking.

## The scene

All coordinates are in metres from the midpoint between the two arm bases: x forward, y to the robot's left, z up. The
station table top is at z = 0.7475.

| Object | Model | Size | Mass | Where |
|---|---|---|---|---|
| Tabletop | fixed box, melamine laminate on chipboard (mechkit `fixed`) | 48 x 46 x 4 cm | fixed | x 0.12 to 0.60, y -0.05 to 0.41, top at z 0.7875 |
| Cup | Solo P16R, high-impact polystyrene (mechkit `free`, built in `task.py`) | 9.85 cm across the rim, 6.43 cm across the base, 12.0 cm tall | 11.8 g | base centre at x 0.330, y -0.0158, standing mouth up on the board |
| Soda can (distractor) | SimReady `food_beverage_can_b01` | 6.7 cm across, 9.2 cm tall | 380 g | x 0.53, y 0.02, on the board |
| Beer bottle (distractor) | SimReady `food_beverage_beerBottle_a01` | 6.0 cm across, 22.2 cm tall | 200 g | x 0.53, y 0.12, on the board |
| Napkin (distractor) | SimReady `cleaning_cloth_a01` | 26 x 23 cm | 50 g | centred at x 0.46, y 0.28, flat on the board |

The board's edge facing the right arm (the line y = -0.05) is the table edge. The cup's rim stands 1.5 cm past it (its
outermost point is at y = -0.065), and its base (6.43 cm across) stands 2.1 mm inside the edge, so the cup stands on
the board but part of its rim hangs over open air. The board stands in for the party table: it is thick enough to give a
real edge with nothing under the overhanging rim. The distractors stand on the far side of the board, outside the
cup's flight; they do not take part in scoring.

The cup (mechkit `free`, one rigid body):
- wall: 24 flat panels following the taper from the base (radius 32.1 mm) up to just under the rim (radius 46.7 mm,
  11.6 cm up), each a convex hull 3 mm thick;
- rim: the rolled bead as 24 convex segments, 4 mm tall, standing 2.5 mm out from the wall (outer radius 49.2 mm);
- base: a disc 1.5 mm thick;
- visual: the red outside and white inside of the real cup, turned on a lathe.

PhysX spreads the 11.8 g over these colliders, so most of it is in the wall, as in the real thin-walled cup, and its
centre of mass sits a little above half height. The cup's depenetration speed is capped at 1 m/s so a fast finger can't
launch it out of an overlap (mechkit sets that cap for links, not free parts; `task.py` adds it when it writes the file,
with no change to `mechkit.py`). Friction: cup 0.45 static and 0.40 dynamic, board 0.40 and 0.35, restitution 0.

## Scoring

The scorer (class `Trial` in `task.py`) runs every control step (20 Hz) on the cup's pose and on 24 sample points on the
grippers' fingers (mechkit `fingertips`). It never reads the robot's commands.

- **Touch:** a finger touches the cup when any finger sample point is within 12 mm of the cup's outer surface (the
  wall's taper, the rim and the base). The points sit 4.5 mm behind the finger pads, so this is a gap of about 7.5 mm.
- **Handled** (recorded): a touch while the cup's axis is more than 45 degrees from upright. A flick leaves the cup
  long before that. Carrying, turning or righting the cup by hand all have to touch it further over.
- **Fell** (recorded): any point of the cup's base or rim circle more than 2 cm below the tabletop: it went off the
  table.
- **Turn:** the largest angle between the cup's axis and straight up, over the whole trial.
- **Resting:** the lowest point of the cup within 5 mm of the tabletop, the midpoint of its axis over the board, moving
  under 2 cm/s and turning under 0.3 rad/s, and every finger sample point 5 cm or more from it.
- **Mouth down:** the axis within 15 degrees of straight down.

Score, every step: handled or fell -> 0 for the rest of the trial; otherwise, if the cup is resting: mouth down for 1 s
in a row -> 1.0 (success: the trial ends after another `SUCCESS_HOLD_S` = 1 s of it); not mouth down, but the turn has
reached 135 degrees -> 0.5; the turn has reached 100 degrees (past lying on its side, which for this taper is 82
degrees) -> 0.25; anything else, including a cup not resting yet, standing where it was or only tipped onto its side ->
0. The score reported is the harness's `task_score`: 1.0 if success held, otherwise the last step's score.

Possible scores: 0, 0.25, 0.5, 1.0.

The recorded `handled` and `fell` flags and the largest turn are cleared only in mechkit's `on_reset` hook, never
because the cup looks right again.

## A real trial: setup, measurement, reset

**You need:** a table with a straight edge (any party or kitchen table), a pack of Solo P16R 16 oz cups (about $10 for
50), a ruler, tape, a phone filming at 60 fps or more from the side, and the same three distractors (a full can of
soda, an empty beer bottle, a cloth napkin) standing at least 15 cm from the cup.

**Preparation (each trial):** draw the cup's base circle (6.43 cm across) on a piece of tape on the table, with its edge
2.1 mm inside the table edge, so that the rim stands 1.5 cm past the edge. Weigh the cup (10-12 g) and stand it mouth
up on the circle. Check with the ruler that the rim stands 1.5 cm +- 2 mm past the edge.

**Measurement (from the video and the scene once still):**
- Watch the video frame by frame: if a finger touches the cup in any frame in which the cup leans more than 45 degrees,
  the trial scores 0. If the cup goes off the table, 0.
- Find the frame in which the cup is turned furthest from upright and read the angle (an on-screen protractor or a
  phone app): 100 degrees or more, 135 degrees or more.
- When everything is still, with the hands at least 5 cm from the cup: is the cup mouth down (its base up, flat on its
  rim, within 15 degrees of vertical) on the table? 1.0 if so, otherwise 0.5 or 0.25 by the largest turn, otherwise 0.

**Reset (under a minute):** stand the cup back on its circle. A cup with a creased rim is replaced (they cost 20 cents).

**Rig safety:** the cup weighs 12 g and the finger only strikes upward past the table edge; nothing can be broken or
spilled. Keep the soda can and bottle at the far side of the table, as in the scene.

## Run and score a trial

Install the simulator and Inspect Robots (the program's walkthrough), then from the repository root:

```bash
inspect-robots run --embodiment yam_arena --policy <your policy> \
  -E task=tasks/flip-cup-land-mouth-down/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Flip the empty cup off the table edge so it lands mouth down on the table." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/flip-cup-land-mouth-down
```

`task_score` in the log is the score above. The run needs network access the first time, to download the three
SimReady distractor props into `~/.cache/simready`.

### The scripted demo

The demonstration is `demo_script` in `task.py`, run by `demokit.py` on the harness's own embodiment and arm plans:

```bash
cd tasks/flip-cup-land-mouth-down
python demokit.py task.py ~/yam-eval/flip-cup-land-mouth-down/demo          # writes demo/full/demo.mp4, setup.png, result.json
```

The right gripper, closed and level, pointing forward (+x), comes up beside the table edge until its finger stands
under the overhanging part of the rim, 3 cm below it. It then flicks: its six joint targets are set for 0.1 s to a
point far up and slightly inward (+y) from the finger, so that the joints run at their torque limits and the finger meets
the underside of the rim at about 1.5 m/s. Then the targets jump to a point below and away from the edge, so the hand
drops out of the turning cup's way. The cup does not move before the strike, so the flick is played open loop. The
strike values are in `task.py` (`ZF`, `XOFF`, `FY`, `FZ`).

Results: `task_score` 1.0 in each of two fresh runs on the GPU, rendered at 640 x 360 (and in two more runs before the
beer bottle was moved away from the left arm's start pose). In each run the finger leaves the cup at step 82, the cup
leaves the table spinning at about 16 rad/s, reaches mouth down about 0.25 s later and settles 1.5 cm in from where it
stood; success holds from step 111 and the trial ends at step 130 (6.5 s).

The window is narrow. In a sweep of the strike on the GPU (target lift 0.30 to 0.40 m, inward push 0.02 to 0.10 m,
finger start 3 or 6 cm below the rim, finger 2 or 4 mm further along the rim; run before the final scene tidy-up), 9 of 27 settings scored 1.0 and the
rest scored 0.5, 0.25 or 0: the cup lay on its side, turned past mouth down, or went over the edge. Moving the finger
2 mm along the rim was enough to change the result.

### Red-team runs

`demo_script` has failure modes, run the same way with `--mode`:

| Mode | What it does | `task_score` |
|---|---|---|
| `none` | nothing | 0.0: the cup still stands at the edge |
| `push` | tips the cup over slowly with the finger, as a hand pushing it over would | 0.0: it lies on its side (largest turn 82 degrees) |
| `swat` | a fast sideways hit at the rim with no lift | 0.0: the cup slides and rolls off the far side of the table (recorded as fell) |
| `hard` | the flick with too much lift (0.60 m target) and more inward push | 0.0: the cup only tipped over onto its side (largest turn 87 degrees) |

Carrying the cup over by hand was not run as a mode. Any route that holds the cup while turning it touches it past
45 degrees and scores 0 by the handled rule.

One arm: the task is already done with one arm (Flip Cup forbids using two hands), so a separate one-arm run is not
meaningful; the left arm has no part in the demonstration.

## Model run

Dream-1 was attempted once on the final version, with the instruction above and a 1,200-step limit. Its API refused
the request before the first action, so there is no model video: "DreamAPI refused the inference request: DA-4000 BAD_REQUEST (malformed): unsafe instruction, refused: Flipping or pushing objects off the table onto the floor is unsafe." The cup in this task stays on the table:
leaving the table scores 0.

## What is modelled and what is not

- The cup is rigid. The real cup's 0.5 mm wall flexes a little under a fingernail; the strike in the simulator is a
  rigid finger pad on a rigid rim.
- Collider walls are 3 mm thick (thicker than the real wall), so a finger moving at 1.5 m/s can't pass into the wall
  within one 1/240 s physics step. The mass is spread over the colliders as a shell, so the cup's inertia is close to
  the real one's.
- No air drag (it slows a 12 g cup a little in a 0.2 s flight) and no restitution (a real polystyrene cup bounces very
  little).
- Friction values are estimates for polystyrene on melamine; PhysX averages the two surfaces.
- Touch is judged from sample points on the fingers within about 7.5 mm, not from contact forces.
- The tabletop is a fixed 4 cm board on the station table, standing in for a party table's edge.

## Runtime per trial

The demonstration ends at step 130 (6.5 s of simulated time), 1 s after success first holds. A model run is limited to
1,200 steps (60 s). A trial takes about 1-2 minutes of wall time on one GPU, plus shader compilation on a GPU's first
run.

## Seeds

The scene has no randomness: `object_jitter=0`, and the cup starts in the same place every trial. The physics is
deterministic for the same commands on the same GPU.

## Assets and licences

- `mechkit.py` is copied unchanged from `tasks/heat-shrink-lug/mechkit.py` in this repository (another team's
  open-source task). `simready.py` is copied from `tasks/cutters-off-pegboard/simready.py` and `demokit.py` from
  `tasks/heat-shrink-lug/demokit.py` (likewise).
- The cup and the tabletop are built in `task.py` from the dimensions above. SimReady's only plastic cup
  (`dishware_plastic_cup_a01`, 6.4 cm across and 10 cm tall) is a smaller tumbler with a different taper, and the
  flip depends on the cup's real size, taper and mass, so the Solo cup is modelled from its specification instead.
- Distractors, from NVIDIA's SimReady library (`nvidia/simready-assets` on Hugging Face, revision
  `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`), © NVIDIA, CC-BY 4.0. They are downloaded at first run and nothing is
  committed. At load time each gets the mass in the scene table above (its authored inertia scaled to match), and the
  can and bottle collide as convex hulls.
  - [`food_beverage_can_b01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/food_beverage_can_b01)
    (`sm_food_beverage_can_b01_01.usd`)
  - [`food_beverage_beerBottle_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/food_beverage_beerBottle_a01)
    (`sm_food_beverage_beerBottle_a01_01.usd`)
  - [`cleaning_cloth_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/cleaning_cloth_a01)
    (`sm_cleaning_cloth_a01_01.usd`)
