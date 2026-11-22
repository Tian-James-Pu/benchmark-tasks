# Whip an index card out from under a quarter so the coin drops into the glass

An index card lies across the mouth of a juice glass with a quarter on top, centred over the glass. The instruction is
the classroom challenge as it is usually set: **Get the quarter into the glass without touching the quarter or lifting
the card.**

The only way to do it is the inertia trick. The card has to leave fast. Paper grips a coin with a friction coefficient of
about 0.35, so the card can give the coin at most about 3.4 m/s² of acceleration. A card pulled or pushed gently takes the
coin with it and dumps it beside the glass. A card snatched away at several times that acceleration, leaving at
roughly 1 m/s or more, slips out from under the coin before friction has moved it more than a centimetre or two. The
coin then drops straight into the glass. In between, the coin is dragged onto the rim and falls outside. Most adults
have tried this at school or at a café table. For a robot it is a narrow speed window on a 2 g sheet of paper. It has to
be hit open loop in a tenth of a second, with the grip on the card firm and the card kept flat. Any lift tilts the coin
or drags the glass.

## Run a trial

From the repository root, in the Python environment that has `yam_arena` and `inspect-robots`:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/flick-the-card-so-the-coin-drops-in-the-glass/task.py \
  -E scene=station -E object_jitter=0 -E control_hz=20 \
  -E cam_width=640 -E cam_height=360 \
  --instruction "Get the quarter into the glass without touching the quarter or lifting the card." \
  --scorer task_score --max-steps 600 --store-frames \
  --log-dir ~/yam-eval/flick-card-demo
```

For a model trial use the model's policy (for Dream-1, `--policy dream`), `-E cam_width=320 -E cam_height=176` and
`--max-steps 1200` (the run limit: 60 s at 20 Hz). The trial prints `task_score`. The layout is fixed
(`object_jitter=0`) and nothing is randomised, so there is no seed to set. The scripted demo is open loop and reads
nothing from the scene or the scorer.

Runtime: the scripted demo succeeds and the trial ends after 209 control steps (10.5 simulated seconds). On one
L40S-class GPU shared with other jobs that takes about a minute once shaders are compiled, and several minutes on a
GPU's first run. A model trial of 1,200 steps is 60 simulated seconds.

## The scene

Positions are (x forward from the arm bases, y to the left) in cm. Heights are above the table top.

| Object | What it is | Size, mass, material | Where |
|---|---|---|---|
| Glass | 8 oz clear glass juice tumbler (NVIDIA SimReady `dishware_glass_8oz_a01`) | 6.6 cm across the rim, 10.0 cm tall, about 3 mm wall, 4 mm base; 150 g; soda-lime glass | standing on a tape mark centred at (36.0, -15.0) |
| Card | Oxford ruled index card, 3 x 5 in (item 31 / 31EE) | 12.7 x 7.62 cm, 2.0 g; paper (0.3 mm thick in the simulation, 0.2 mm real) | flat across the rim, ruled side up, centred over the glass, long side pointing away from the robot; it overhangs the rim by 3.0 cm towards and away from the robot and 0.5 cm at each side |
| Coin | US quarter | 24.26 mm across, 1.75 mm thick, 5.67 g; copper-nickel clad | flat on the card's centre, over the glass's axis |
| Notebook (distractor) | SimReady `misc_notebook_a01` | 22.9 x 17 x 1 cm, 120 g | centred at (20, 27), turned 90 degrees |
| Mug (distractor) | SimReady `dishware_mug_a01` | 11.4 x 9 x 8 cm, 300 g; ceramic | at (43, 17), turned 200 degrees |
| Apple (distractor) | SimReady `food_produce_apple_b01` | 7 cm, 180 g | at (50, -33) |

Both grippers start open at their ready poses. The distractors are loose. The scorer ignores them, except that none of
them may touch the coin.

How the simulator models it (`task.py`):

- The glass is drawn with the SimReady mesh, read from the pinned package when the scene is built. It collides as a 4 mm
  base disc and 24 wall panels (two convex pieces each) that follow the mesh's measured inside and outside radii: 2.55 to
  3.11 cm at 4 mm up, 2.85 to 3.23 cm at 2 cm, 3.01 to 3.31 cm at the rim, 10.03 cm up. So the rim is flat, the card
  lies on it, and the coin can fall inside. The SimReady loader's own collision for this prop is a signed distance
  field, and a 0.3 mm card sank 2.8 mm into its rim and dragged the glass along with it. The glass is a free 150 g
  body with its centre of mass 3 cm up, so a careless pull can slide or tip it.
- The card is one rigid 2 g box. A real card flexes. The demo keeps it flat, so this matters only for strategies that
  bend the card.
- The coin is one rigid body with the United States Mint's mass and size and the inertia of a uniform disc. It has
  speculative continuous collision detection, so a coin falling at over 1 m/s cannot pass through the 3 mm glass wall
  in one 1/240 s physics step.
- Friction (static, dynamic; PhysX averages the values of the two parts in contact): coin 0.35/0.30, card 0.40/0.35,
  glass 0.40/0.40. So paper on coin is about 0.37 static and 0.33 sliding. Restitution is 0.
- Physics runs at 240 Hz and control at 20 Hz.

## Scoring

The scorer (`Trial` in `task.py`) runs once per control step, after that step's physics. It reads the coin, card and
glass poses, the distractors' bounding boxes and the finger sample points (each finger's tip and a point 3 cm up it, at
its middle and both edges).

**Harm.** Each of these is recorded the step it happens and scores 0 for the rest of the trial. Records are cleared
only when the scene resets.

- A finger point comes within 2.5 cm of the coin's centre: the coin was touched. The rim is 1.21 cm from the centre and
  the points sit about 5 mm inside the finger.
- The coin's centre rises more than 1.0 cm above where it started: the coin, or the card under it, was lifted.
- The glass's axis tilts more than 10 degrees from vertical, its base centre moves more than 2.0 cm from its mark, or its
  base lifts more than 1.0 cm off the table.
- A distractor's bounding box comes within 5 mm of the coin's rim.

**States.**

- *Card clear*: seen from above, no part of the card is within 3.3 cm of the glass's axis, so none of it is over the
  mouth.
- *Coin in the glass*: its centre is within 3.0 cm of the glass's axis and between the glass's base and 1.0 cm below
  the rim.
- *Hands clear*: every finger point is at least 5 cm from the glass (its 3.3 cm radius and 10 cm height) and from the
  coin.
- *Still*: the coin and the glass move under 1 cm/s and turn under 0.5 rad/s.

**Score.**

- **1.0**: coin in the glass, card clear, hands clear, everything still, no harm, held for 20 consecutive control steps
  (1 s). It then stays 1.0 for the rest of the trial unless a harm follows.
- **0.5**: no harm, card clear, and the coin's centre within 6.0 cm of the glass's axis, seen from above. That covers a
  coin that landed on the rim or bounced off it, and a coin that is in the glass while the full conditions do not yet
  hold.
- **0.25**: no harm, card clear, and the coin farther away. The card carried it off.
- **0**: any harm, or the card still over the mouth (doing nothing).

Possible scores: 0, 0.25, 0.5, 1.0. The episode ends 2 s after the 1.0 condition is first met (`SUCCESS_HOLD_S`), so the
video shows the hand away from the glass.

## The demo

One arm (the right) does it, open loop, from waypoints in `task.py` (`yank_plan`). The left arm holds its ready pose.

1. The gripper comes in from above and behind the card. It points forward and 15 degrees down, with its jaws one above
   and one below the card. It moves forward until its fingertips are 1.2 cm short of the glass's wall, so the pads close
   on the 3 cm of card that overhang the rim on the robot's side.
2. It pinches the card and holds still for 0.5 s.
3. It whips the card out sideways (towards -y), 23 cm in 0.1 s of commanded motion. The arm lags its targets. Measured
   over one control step, the card reached 1.57 m/s. The card slips out from under the coin in about two control steps,
   with the coin dragged about 1.5 cm. The coin drops into the glass and settles on its base, 1.3 cm from the axis.
   The card leaves the pads and drops to the table beside the glass.
4. The hand opens, rises 8 cm and waits there, 19 cm from the glass.

Two fresh runs of the final source scored **1.0** each, ending at step 209. The coin lies on the glass's base and the
glass never moves. The simulation is deterministic for this fixed layout. While building, the same pinch with the card
whipped out over 0.3 s instead of 0.1 s reached a similar peak speed but lower acceleration, and the coin was carried
9 cm off (see the red team).

## Model run

Dream-1 ran the full 1,200-step run limit and scored 0: the card stayed over the glass's mouth for the whole run, and late in the run the right arm reached towards the apple instead.

## Red team

Each variant is a copy of `task.py` with a different `demo()`, run once through the same scorer and then deleted.

| Variant | What it does | Score |
|---|---|---|
| Do nothing | Both arms stay at their ready poses | 0 (the card never leaves the mouth) |
| Careless, medium pull | The demo's pinch, then the same 23 cm sideways pull over 0.3 s instead of 0.1 s | 0.25: friction carried the coin off with the card. It fell to the table 9.3 cm from the glass's axis |
| Careless, slow pull | The same pull over 1.0 s | 0: the coin rode along on the card. As the card's free end left the rim it pivoted in the pinch and the coin rose 1.5 cm ("coin lifted"). Then card and coin fell to the table 13.6 cm from the glass |
| Cheapest shortcut: lift and tip | Pinch the card's near end, raise it 4 cm to tip the coin off, slide sideways | 0: coin lifted with the card |
| Cheapest shortcut: carry the coin | Down grip on the coin itself to lift it off the card | 0: the coin was touched (and lifted) |

One-armed play is the intended solution here. A second arm may steady the glass, but nothing requires it.

## Distinct from

| Nearest task | What it tests | How this differs |
|---|---|---|
| #3075 (pending): slide the top card off a deck by fingertip friction | Deliberately slow, friction-driven sliding of one card off another | The opposite regime. Here friction is what must be beaten: a slow pull is exactly the failure. The skill is a short, very fast acceleration of a held sheet so that a loose object left on it does not follow. |
| #2722 (rejected): spin a Twister spinner with one flick | A flick that sets a light arrow coasting to a target | That flick is aimed at the object that moves. Here the card is only the means, and success is judged on the object that must not move with it. |
| #2746: release a pull-back toy car | Storing spring energy, then releasing it | No stored energy here. It is an inertia contrast between two loose objects in contact, decided by the arm's own speed. |

Also checked, by title and instruction across the board's accepted, pending and rejected PRs (`card`, `index card`,
`coin`, `quarter`, `inertia`, `tablecloth`, `flick`, `yank`): #1592 pours and sorts coins, #2268 posts quarters into
a piggy bank and #2785 feeds a gumball machine. All three grasp and place the coin. #2364 scratches a gift card with a
coin. Nothing on the board removes a support from under an object by speed. None of LIBERO, RoboCasa, SimplerEnv,
BEHAVIOR, DataCenterBench or RoboDojo has a task where the success condition is that an object was *not* carried
along by fast motion under it. Their tasks are quasi-static grasp, place, push and articulate.

## Importance and sources

- The activity is a standard school science demonstration of inertia. Science World (British Columbia) sets it as a
  classroom challenge in exactly this task's words, "get the penny into the cup without touching the penny or
  lifting the card", solved by flicking the card swiftly:
  https://www.scienceworld.ca/resource/playing-cards-and-pennies/
- The University of Regina's CMSTE "Coin in Cup" quickstart, written for Grade 7 Force and Motion and Physics 30, uses
  a recipe card, a coin and a drinking glass. It says the slower the card moves, the more the coin moves with it. That
  is the speed window this task scores:
  https://www.cmste.uregina.ca/Quickstarts/coinincup.html
- Quarter specification (5.67 g, 24.26 mm, 1.75 mm), as published by the United States Mint and summarised at
  https://en.wikipedia.org/wiki/Quarter_(United_States_coin) . The Mint's own page refused automated fetching while
  this was written.
- Index card: Oxford ruled 3 x 5 in cards, item 31 / 31EE, sold in packs of 100 for about $2 to $7. The listing gives an
  8 mil (0.2 mm) sheet:
  https://www.zoro.com/tops-business-forms-oxford-ruled-index-cards-3-x-5-white-100pk-31/i/G010771614/
- Glass: an 8 oz juice tumbler. For scale, Libbey's 8 oz Governor Clinton glass (618HT) is listed at 2 5/8 in (6.7 cm)
  across the top and 3 5/8 in (9.2 cm) tall:
  https://www.webstaurantstore.com/libbey-618ht-governor-clinton-8-oz-beverage-glass-case/551618.html . The simulated
  glass is the SimReady 8 oz glass: 6.6 cm across, 0.8 cm taller. Any 8 oz tumbler of about this mouth works for the
  real trial. The 150 g mass is an estimate for a glass of this size, not a measured figure.

Evidence limits: these sources establish that the activity is common and that a slow card carries the coin. The
1 m/s figure in this README comes from the friction estimate above and from the simulation. It was not measured on
real hardware.

## Real trials

**Rig.** A Libbey-type 8 oz glass tumbler on the bench, on a 7 cm circle of masking tape at (36, -15) cm. One Oxford
3 x 5 in ruled index card, laid flat across the rim, ruled side up, long side pointing away from the robot. Two tape
ticks on the bench, 3.0 cm in front of and behind the rim, mark the card's ends. A US quarter goes in a 24 mm circle
drawn in pencil at the card's centre. Lay a folded towel on the bench 20 to 40 cm to the robot's right of the glass, so
that a whipped card and a knocked glass land soft. A tumbler of tempered glass, or a clear polycarbonate tumbler of the
same size, keeps the glass from breaking if it is knocked over. The forces involved are tiny: a 2 g card, a 6 g coin
and a 150 g glass.

**Measurement**, read from a phone video at 60 fps from the side and one from above, plus a ruler:

- Coin in the glass and card clear of the mouth: look from above.
- Coin within 6 cm of the glass's axis: measure from the tape circle's centre.
- Harm: a finger touching the coin or a distractor touching it (video); the coin rising more than 1 cm (side video,
  against a ruler taped behind the glass); the glass leaving its tape circle by more than 2 cm, tipping visibly or
  lifting.
- Hold: the hand at least 5 cm from the glass and everything still for 1 s.

**Reset, about one minute.** Tip the quarter out of the glass. Stand the glass on its circle. Lay the card back between
its ticks (use a fresh card from the pack if it is creased: cards cost a few cents). Put the quarter in its pencil
circle and the distractors on their marks. Nothing else is consumed.

## Assets and code

- **SimReady props**, from the Hugging Face dataset
  [`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision
  `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, © NVIDIA, licensed
  [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/):
  - [`dishware_glass_8oz_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_glass_8oz_a01):
    the glass. Its mesh is used unchanged as the visual. The colliders are replaced by a base and wall panels (see
    above), the mass is set to 150 g and the material is a plain glass look.
  - [`misc_notebook_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/misc_notebook_a01)
    (notebook, mass set to 120 g)
  - [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01)
    (mug, 300 g)
  - [`food_produce_apple_b01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/food_produce_apple_b01)
    (apple, 180 g)

  The files download at first run and are not committed.
- **Card and coin**: SimReady has no index card or coin, so both are built in code: a box with printed rules, and a
  revolved coin profile.
- **Kit code**: `mechkit.py` and `simready.py` are taken unchanged from other teams' open-source tasks in this
  repository (`mechkit.py` as in `tasks/heat-shrink-lug/`, `simready.py` from `tasks/cutters-off-pegboard/`).
  `coinkit.py` (the quarter's specification, profile and free-body helper) is taken unchanged from
  `tasks/spin-a-quarter-on-its-edge/`.
