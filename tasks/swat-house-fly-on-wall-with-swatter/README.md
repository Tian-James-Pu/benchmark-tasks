# Swat a fly on the wall: line the swatter up out of its reach, then flick the head flat onto it in a tenth of a second

A house fly has landed on a wall. A plastic fly swatter hangs on a peg at the wall's end. The instruction gives only the
goal: **"Kill the fly on the wall with the fly swatter."**

Most adults have done this, and most have also missed: the fly sees the swatter coming and is gone. What makes it hard
for a robot is the same thing that makes it hard for a person:

- **The fly reacts to anything that looms.** Here, once any part of the swatter or of either hand comes within 15 cm of
  the fly, it flies off 0.1 s later. The robot has to line the swatter up from further away (the demo holds the head
  about 18 cm off), and then close the last 15 cm in about a tenth of a second: about 1.5 m/s at the head.
- **The head has to land flat.** The fly is killed only if the mesh head lies flat on the wall over it (within 10
  degrees). An edge, a corner or a tilted head misses. The swatter's flexible neck is what lets the head slap flat while
  the hand and handle stay well off the wall, as in a real swat.
- **The swatter must be taken from its peg first, and put down afterwards.** It hangs by its handle's hole; it has to be
  slid off the peg, turned in the hand from hanging to level with the head upright, and, after the swat, laid down
  and let go.
- **The wall is in the way.** The gripper is about as wide as its open fingers, so the hand cannot get within about
  10 cm of the wall: the swatter's length and its neck have to do the reaching.

A slow, careful approach, the natural default for a robot, always fails here: the fly leaves before the head arrives.

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: wall, peg, swatter and fly, the scorer (`score`, `success`) and the closed-loop scripted demo (`demo_script`) |
| `mechkit.py` | The mechanism kit (articulations written as USD text, step/reset hooks, fingertip points) |
| `simready.py` | Loads the SimReady distractors, downloading them once from Hugging Face |
| `demokit.py` | Runs a closed-loop scripted demo on the harness's own embodiment, unchanged |
| `task.json` | The task's details for the board |

## Run a trial

From the repository root, after `source <ARENA_DIR>/yam.env`:

```bash
# a model (Dream-1 shown), 60 s at 20 Hz
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/swat-house-fly-on-wall-with-swatter/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Kill the fly on the wall with the fly swatter." --scorer task_score --max-steps 1200
```

The scripted demo is closed loop (it reads where the swatter and the fly are), so it runs through `demokit.py` from
this folder, in the simulator's Python environment:

```bash
cd tasks/swat-house-fly-on-wall-with-swatter
python demokit.py task.py ~/yam-eval/swat-house-fly-on-wall-with-swatter/demo --mode full     # full/demo.mp4, setup.png, result.json (task_score)
python demokit.py task.py ~/yam-eval/swat-house-fly-on-wall-with-swatter/redteam --fast --mode none,slow,close,edge,poke,hold
```

Nothing is randomised: every seed gives the same scene, with the fly in the same place. The demo earns full credit
at about step 460 (23 s of simulated time). On a shared GPU a recorded run went at about 0.7 to 2 steps a second, so a
1,200-step trial takes 10 to 30 minutes of wall time; the `--fast` debug mode (no cameras, CPU physics) runs at about
45 steps a second.

## The scene

Positions are (x forward from the arm bases, y to the left) in cm; heights are above the table top.

| Object | What it is | Where |
|---|---|---|
| Wall section (fixed) | White-painted MDF panel, 60 x 40 x 1.2 cm, on a 10 x 60 x 1.8 cm foot, with a trim strip along its top | Face at x 42, facing the arms, from y 3 to 63 |
| Peg (fixed) | 5 mm steel peg, 5 cm long | Out of the panel's left end face (y 3) towards -y, x 42.6, 44 cm up |
| Fly swatter (loose) | Smart Savers HA063 plastic swatter: flat handle 1.5 cm x 4 mm with a 10 mm hanging hole 1.1 cm from its end; 12.7 x 11.9 cm mesh head, 2 mm thick, on a flexible neck; 42.8 cm long as modelled; about 30 g (18 g handle, 12 g head) | Hanging by its hole on the peg (hole centre at x 42.6, y 0, 43.8 cm up), head down, flat faces towards +-y |
| House fly stand-in (fixed) | Black body 7 x 4 x 3 mm with two grey wings | On the panel's face, centre at y 49, 14 cm up |
| Mug (distractor) | SimReady `dishware_mug_a01`, ceramic, 11.4 x 9 x 8 cm, 300 g | (30, -30), turned 138 degrees |
| Notebook (distractor) | SimReady `misc_notebook_a01`, paper, 22.9 x 17 x 1 cm, 250 g | (20, -12), turned 100 degrees |
| Candle (distractor) | SimReady `decor_candle_a01`, candle in a jar, 7.4 cm across, 7.5 cm tall, 350 g | (55, -22) |

Both grippers start open at their ready poses. The fly's spot was chosen so that it is more than 15 cm from both hands
at their ready poses (the nearest fingertip point is 18 cm away) and within the left arm's reach with the swatter. The
distractors are on the right, away from everything the task needs; the scorer ignores them.

**How the swatter is modelled.** A plastic swatter is not rigid: its neck and handle bend, which is why a person can
slap the head flat on a wall while the hand stays well off it. Here the swatter is two rigid bodies, the handle and
the head, joined at the neck by a hinge with a 0.15 N m/rad spring and +-40 degree stops. The handle meets the wall at
an angle of about 20 degrees in the demo and the head folds flat against the wall on contact. Bending in the other
directions (twist, side bend) is not modelled. The gripper's housing is about as wide as its open fingers, so a hand
holding the handle cannot get closer than about 10 cm to the wall: the swatter's length and neck do the reaching.

**How the fly is modelled.** The fly is not a physical body (it weighs about 12 mg). It is a visual marker on the wall;
code decides whether it is swatted or flies off, and hides it when it flies off.

## Scoring

Checked at every control step (20 Hz) by a tracker in `task.py` that reads the swatter's pose, its neck angle and the
fingertip points of both grippers.

- **Near:** any point of the swatter (a 7 x 7 grid over the head, 32 points along the handle's edges) or of either
  gripper's fingers (each finger's tip and a point 3 cm up it, at its middle and both edges) within 15 cm of the fly's
  centre.
- **Swat:** the head lies flat on the wall over the fly: the head's plane within 10 degrees of the wall's; the fly's
  centre at least 4 mm inside the head's outline; and the head's face within 6.5 mm of the wall at the fly's position
  (the fly is 3 mm high, so a head that close is touching it).
- **Escape:** the fly flies off at the second check after the first check that finds something near (0.1 s later),
  unless a swat has happened by then (the swat test runs first at each check). Once flown off it does not come back.

| Score | When |
|---|---|
| 1.0 | Swatted, and then the swatter lies on the table (every point of it less than 4 cm above the table top), let go (every fingertip point more than 5 cm from it) and still (under 2 cm/s and 0.2 rad/s) for 1 s. It stays 1.0 from then on. |
| 0.75 | Swatted, but the swatter not laid down and let go that way |
| 0.5 | Not swatted, but after the fly flew off the head lay flat over its spot by the swat test (a slap that was right but too slow or started too close) |
| 0.25 | The fly flew off at the approach of a swatter taken off its peg (the hole's centre more than 5 cm from where it hung when something first came near), with no later flat slap |
| 0 | Anything else: nothing done, or the fly scared off by a hand, or by the swatter still on its peg |

Whether the fly was swatted or flew off is recorded when it happens and cleared only by a reset. Possible scores: 0, 0.25, 0.5, 0.75, 1.0.

The swat and the fly's escape are both final states that remain visible: a swatted fly stays on the wall, an escaped
one is gone. In a real trial the swat is read from an ink mark on the head (below), so the scorer never has to judge
a fast motion by eye.

## The demo

`demo_script(ctx)` in `task.py`, left arm only:

1. Close on the hanging swatter's handle 3.5 cm below the hole, the gripper level and pointing at the panel's end, the
   fingers on the handle's flat faces. Slide it off the peg (6.5 cm along the peg, 2 mm up) and draw it back.
2. Turn the gripper to point along -y and then, step by step while lowering it, to point straight down. The hanging
   swatter swings out level towards +y with the head upright and the mesh facing the wall.
3. Read where the fly is and line up: hand 9.3 cm off the wall, the handle drawn back 15 degrees from the wall, the
   head about 18 cm from the fly (outside the 15 cm). Hold still for 0.6 s.
4. Flick the wrist round. The wrist joint is soft (it lags a command with a time constant of about 0.15 s), so the
   flick is commanded 45 degrees past the wall in a single step and taken back two steps later. In instrumented CPU
   runs of the same script the head's nearest point went from about 18 cm to 9 to 10 cm to 1 to 2 cm from the fly in
   successive 0.05 s steps, and the swat registered at the next check (step 331), the head within 2 degrees of flat:
   about 1.6 m/s over the last 15 cm.
5. Roll the swatter onto its face towards the robot, lower it to 7 cm above the table, let go, and move the hand away.

The recorded runs (GPU, 640 x 360) and the failure checks are in the table below.

## Model run

Dream-1 was attempted once on the final version, with the instruction above and a 1,200-step limit. Its API refused
the request before the first action (the arms held their ready poses), so nothing was scored and there is no model video: "DreamAPI refused the inference request: DA-4000 BAD_REQUEST (malformed): unsafe instruction, refused: Striking or swatting living creatures such as animals or insects is unsafe." The fly here is a 7 mm stand-in on a panel, not a living insect; in the real rig it is a felt pad on a solenoid.

## Validation

| Run | Mode | Score | What happened |
|---|---|---|---|
| GPU, recorded 1 (the submitted video and screenshot) | full | 1.0 | Swatted, swatter laid down and let go; full credit reached at step 463 (23.15 s) |
| GPU, recorded 2 | full | 1.0 | Same: full credit reached at step 463 (23.15 s); the simulation is deterministic for this scene |
| CPU `--fast` | none | 0 | Nothing moves; the fly stays |
| CPU `--fast` | slow (turn onto the fly over 2.4 s) | 0.5 | The fly flew off as the head came within 15 cm; the head then lay flat over its spot |
| CPU `--fast` | close (line up with the head 12 degrees from the wall, within 15 cm) | 0.5 | The fly flew off while the hand held still; the flick then landed flat on the empty spot |
| CPU `--fast` | edge (the gripper rolled 80 degrees, so the head comes in edge-on) | 0.25 | The fly flew off as the swatter came near; no flat slap |
| CPU `--fast` | poke (no swatter: a quick jab at the fly with the closed fingers) | 0 | The fly flew off at the hand's approach; the gripper's housing stopped against the wall with the fingertips 7 cm short of it |
| CPU `--fast` | hold (the full swat, swatter kept in hand) | 0.75 | Swatted, never laid down |

While building the demo, with the fly moved to four other spots on the wall ((y, height) = (46, 12.5), (46, 15.7),
(51.9, 12.6) and (51.9, 12.3) cm), the same sequence also scored 1.0 in CPU runs. An earlier version that drew the
swatter 25 degrees back (head 22 cm off) instead of 15 missed the fly at (51.9, 12.6) by 0.9 mm: the head was still
6 mm off the wall when the fly left. The flick is only just fast enough, which is the point of the task.

## Distinct from

| Nearest board PR | What it tests | Why this is different |
|---|---|---|
| #2458 (rejected): catch a spider under a glass | A slow trap lowered over a crawling target | The fly here escapes on approach, so success depends on approach speed inside a 0.1 s window, and the tool face must land flat; a slow trap is exactly what fails here |
| #2157 (rejected): reflex-hammer knee tap | A brisk tap on a fixed target with a hand tool | The knee never moves or escapes; here the target leaves if the tool is seen coming, so the robot must line up out of range and close the last 15 cm in 0.1 s, landing a flexible head flat |
| #1523 (rejected): whac-a-mole | Pressing a button within a reaction window | A button press with the hand, with no tool, no approach-speed window and no face angle; here the tool is a flexible 43 cm swatter taken off a peg, swung by the wrist |

We know of no task in LIBERO, RoboCasa, SimplerEnv, BEHAVIOR, RoboTwin or DataCenterBench that swats anything or has a
target that escapes when approached; their dynamic tasks are throws, pours and pushes. Other dynamic tasks on the
board (pushing a swing in phase, ripping a ripcord, clapping a clapperboard) have targets that wait for the robot.

## Importance and sources

- **Relatable:** swatting a fly is one of the most common household chores, done by hand with a cheap tool sold in
  every hardware store. The swatter here is a [Smart Savers HA063 plastic fly swatter 3-pack
  (Do it Best)](https://www.doitbest.com/product/633615/smart-savers-5-in-x-4-7-in-plastic-fly-swatter-3-pack-633615/):
  the listing gives a 5 x 4.7 in head, 16.75 in overall, plastic. Its mass is not listed; 30 g is our estimate, and
  the handle and neck thickness are estimates from typical plastic swatters.
- **Why speed matters:** flies escape a looming object with a jump planned from what they see. In Card and Dickinson,
  "Visually mediated motor planning in the escape response of Drosophila", Current Biology 18, 1300-1307 (2008)
  ([PDF](https://faculty.fiu.edu/~theobald/wp-content/uploads/2016/01/Card-and-Dickinson-2008-Visually-Mediated-Motor-Planning-in-the-Escape-Res.pdf)),
  fruit flies faced with a disk falling towards them took off 215 +- 42 ms after the stimulus began, and started
  positioning their legs for the jump about 200 ms before takeoff. The 0.1 s window after the swatter comes within
  15 cm is a simplified, deliberately strict stand-in for that reaction, not a measured house-fly value.

## Real trials

**Rig.** A 60 x 40 cm sheet of 12 mm MDF, painted white, screwed to a 10 x 60 cm MDF foot and clamped to the table at the
positions above; a 5 mm steel peg (or a straight screw-in tool peg) screwed into its left end 44 cm up. The fly is an
electromechanical stand-in, about $30 in parts:

- an 8 mm hole through the panel at the fly's spot (y 49, 14 cm up); behind it a small 12 V push-pull solenoid
  (10 mm stroke) carrying a 3 mm rod whose tip, a 7 mm black felt pad cut to a fly's outline, stands 3 mm proud of the
  wall when the solenoid is out and pulls back behind the face when it fires;
- a VL53L1X time-of-flight distance sensor set flush in the panel 2 cm beside the hole, looking straight out, and an
  Arduino Nano that fires the solenoid 100 ms after the sensor first reads under 15 cm, and lights an LED that stays on
  until the reset button is pressed;
- a 14 mm felt ring, flush with the wall, round the hole.

The time-of-flight sensor sees a cone (27 degrees) straight out from the wall, not a sphere round the fly: a hand
coming in along the wall from the side is seen later than in the simulator. State this when comparing results.

**Preparation, each trial (under 2 minutes):** wipe the swatter's head clean; ink the fly pad and the felt ring with a
washable ink pad (one colour for the pad, another for the ring); press the reset button (the pad comes out, the LED
goes off); hang the swatter on the peg by its hole, head down, flat faces towards the panel's end; put the distractors on
their tape marks; start a phone video at 240 fps from beside the table.

**Measurement (after the trial, before anyone touches the scene):**

- *Swat:* a pad-coloured fly mark on the mesh face of the head. The pad is out only until the fly escapes, so a mark
  proves the head reached it in time. A mark on the rim or edge only is not a swat (not flat).
- *Late flat slap (0.5):* no fly mark, but a ring-coloured mark on the mesh face.
- *Escape on a taken swatter (0.25):* the LED is on, and the video frame in which it lights shows the swatter off its
  peg.
- *Laid down (1.0):* the swatter lies on the table (a ruler: no part more than 4 cm up), the gripper at least 5 cm from
  it, nothing moving.

**Rig safety.** The swatter is 30 g of plastic and the panel is clamped; nothing is consumed or sharp. The ink is
washable and kept off the grippers' pads (the fly pad is only 7 mm).

## Assets and code

- **Distractors:** from the Hugging Face dataset
  [`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision
  `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:
  - [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01) (mug, 300 g)
  - [`misc_notebook_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/misc_notebook_a01) (notebook, 250 g)
  - [`decor_candle_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/decor_candle_a01) (candle, 350 g)

  They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). The only change at load time
  is the masses, which are set as listed. The files download at first run and are not committed.
- **Task objects:** SimReady has no fly swatter, wall section, peg or insect, so these are custom primitives
  (`mechkit.py` meshes and box colliders), with the dimensions above.
- **Kit code:** `mechkit.py` is the shared mechanism kit from `tasks/heat-shrink-lug/` in this repository (another
  team's open-source kit), unchanged apart from a credit line in its docstring. `simready.py` is the SimReady loader
  from `tasks/cutters-off-pegboard/`, unchanged apart from its docstring. `demokit.py` is the closed-loop demo runner
  from `tasks/heat-shrink-lug/`, unchanged.
