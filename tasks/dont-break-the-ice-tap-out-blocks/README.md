# Tap the marked blocks out of a Don't Break the Ice tray

A Hasbro Don't Break the Ice game (C2093) stands assembled on a table in front of the arms: the blue ice-tray frame on
its four legs, 32 small plastic ice blocks and the large centre block with Phillip the Penguin standing on it. Six of the
small blocks carry a red sticker dot. The game's plastic mallet lies on the table to the right. The instruction:

> Using the game's mallet, knock out only the six ice blocks marked with red dots, without letting Phillip fall through.

Nothing sits under the ice: every block is held up only by friction against its neighbours and the frame. A brisk tap at
the centre of a block knocks that block out on its own. A slow press, or a blow that lands on the line between two blocks,
carries the neighbours down with it; the sheet gives way and blocks around it drop, Phillip's among them when the block
is next to his. Three of the six marked blocks touch Phillip's block. The robot has to pick up a light mallet, hold its
head upright, aim each blow at a block's centre within about 6 mm, and strike briskly and stop, six times, while keeping
the trailing handle and its own fingers off the ice and off Phillip.

## Importance and sources

**Category: Societal relatability.** Don't Break the Ice was first sold by Schaper Toys in 1968, passed to Milton Bradley
(Hasbro) in 1986 and is still made ([Wikipedia](https://en.wikipedia.org/wiki/Don%27t_Break_the_Ice)). It is one of the
standard preschool games: most adults have played it as children or with their own children, and a 10 s clip of it is
recognisable at once. Hasbro's current rules ([instructions page](https://instructions.hasbro.com/en-us/instruction/Don-t-Break-the-Ice-Game),
rule sheet C2093) say to "place all of the ice blocks into the tray in rows", that "the last row of blocks may need to be
squeezed in", and on each turn to "carefully tap on that ice block using your mallet. You must keep tapping that same block
until it falls through the bottom of the ice tray". The object is to "knock out the blocks of ice, but be careful - don't
break the ice that makes Phillip fall through!" The box lists 2 mallets, the ice tray frame, 4 ice tray legs, 1 large ice
block, 32 small ice blocks and the Phillip figure ([Hasbro consumer care](https://consumercare.hasbro.com/en-ca/product/don-t-break-the-ice-game/7DF4542A-5056-9047-F5FA-87295ED41F5C)).
It costs $19.99 ([Target](https://www.target.com/p/don-t-break-the-ice-game/-/A-52119448)).

**The knack, with numbers.** Each block is held by static friction against its neighbours; in the simulation each holds
up to 4 N before slipping. A blow that drives a block 4 mm deeper than its deepest stuck neighbour within one control step
(a tap faster than about 0.1 m/s at the face) pops it out alone. A press slower than that drags the neighbours to 0.8 of
the block's depth, and at 8 mm the sheet breaks: the block falls and takes every touching neighbour more than 5.5 mm down,
and any block then left without support on two opposite sides follows. The mallet head is 24 mm across on a 36 mm cell,
so a blow more than about 6 mm off centre lands on a neighbour too. Measured in the simulator (CPU, `strokeT` modes):
strokes averaging 0.12 m/s took out all six marked blocks alone; at 0.07 m/s five came out; at 0.04 m/s and 0.02 m/s the
hand, stopping 2 mm into the block as the demo does, freed none. A steady 1 cm/s press (`careless`) brought down the
sheet around each block and Phillip with it.

**Why it is a robot problem.** It is tool use with a timing window, not pick-and-place: a 20 g mallet held 6 cm from its
head, a target that has to be hit squarely, a stroke that has to be fast and also stopped short (the hand is 1.8 cm above
the face, over the neighbouring blocks), six times in a row, with the right order and path so that the trailing handle
never sweeps across the 6 cm figure in the middle.

## Distinct from

| Nearest task | What it tests | How this differs |
|---|---|---|
| #2522 Jenga push-and-stack (accepted) | a slow, steady push that slides one block out of a loaded tower | here a slow push is exactly what fails; the block has to be struck, with a tool, briskly and squarely |
| #1736 Kerplunk (rejected) | pulling straws out so that marbles do not fall | pulling sticks sideways by hand; no struck tool, no rate dependence |
| #1482 Knock a spent espresso puck out of the portafilter (accepted) | one firm knock of a held portafilter on a knock box | there the held object is knocked against a fixed bar and any hard knock works; here a held tool has to hit one of 33 identical, mutually supporting blocks, and a hard but slow or off-centre blow fails |
| #2746 pull-back toy car (accepted) | storing and releasing spring energy cleanly | a release, not an aimed strike |

None of LIBERO, RoboCasa, SimplerEnv, BEHAVIOR, DataCenterBench or RoboDojo has a task where a held tool must strike one
friction-held block free from among its neighbours.

## Run and score

The demonstration is closed loop (it reads where the mallet head sits in the hand), so it runs through `demokit.py` from
this folder, in the simulator's Python environment (`source ~/IsaacLab-Arena/yam.env`):

```
python demokit.py task.py ~/yam-eval/dont-break-the-ice-tap-out-blocks/demo --mode full          # writes full/demo.mp4, setup.png, result.json (task_score)
python demokit.py task.py ~/yam-eval/dont-break-the-ice-tap-out-blocks/redteam --no-record --mode none,careless,fingers,offcentre
python demokit.py task.py ~/yam-eval/dont-break-the-ice-tap-out-blocks/cal --fast --mode stroke0.6,stroke1.0,stroke2.0   # slower strokes (CPU)
```

A model is run with the harness's ordinary command:

```
inspect-robots run --embodiment yam_arena --policy dream -E task=tasks/dont-break-the-ice-tap-out-blocks/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Using the game's mallet, knock out only the six ice blocks marked with red dots, without letting Phillip fall through." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/dont-break-the-ice-tap-out-blocks/model
```

`--scorer task_score` reports the scorer's last value; full credit is reached by the scorer's own 1 s hold. The demo takes
about 26 s (about 510 steps); the run limit is 1,200 steps. There is no randomness in the layout: `object_jitter=0`, any
seed. A demo run takes about 40 s of wall time on a free GPU once shaders are compiled.

## How the simulator models it

- **The ice sheet** is one mechanism (`mechkit`): a fixed frame (rim and legs) and 33 links, one per block, each on a
  vertical slide from its seat (0) down to the table (-46 mm). Each link has its real mass and gravity, and a spring of
  2,000 N/m capped at 4 N toward a target: while the block's faces stick it is held where the law puts it, and pressed
  harder than 4 N it gives way. Links do not collide with each other or the frame (only with the mallet, the hands and
  Phillip's collider), so all block-to-block interaction is the law below.
- **The law**, run once per control step after physics (`Trial.step`), reading only the blocks' depths:
  1. a block more than 4 mm deeper than its deepest stuck neighbour has slipped out: its spring is released and it falls;
  2. otherwise a block more than 8 mm down breaks the sheet: it falls, with every stuck neighbour more than 5.5 mm down
     (and theirs, in turn);
  3. a block without support on both of two opposite sides (a stuck neighbour or the frame on its -x and +x sides, or on
     its -y and +y sides) falls;
  4. every stuck block's spring target is set to 0.8 of its deepest stuck neighbour's depth: the faces drag their
     neighbours, and the sheet sags together under a slow press.
  The coupling acts one control step late, which is what makes the result rate dependent: a fast blow outruns the
  neighbours, a slow one does not. The numbers are calibrated to give the behaviour the game is known for, not measured.
- **Phillip** is part of the large block's link (visual figure and a 3.8 x 6.2 cm collider); he falls through with it.
- **The mallet** is one free rigid body: cylinder head and flat box handle, 20 g, with its centre of mass from the
  colliders. It lies with its head face down and its handle's end on the table, so it starts tilted about 9.5 degrees.
- **The tray** is welded to the world, standing for a tray whose legs are taped to the table.

## Scale and mass

| Part | Size used | Source and confidence |
|---|---|---|
| Tray frame | 24.4 x 24.4 cm outside, 6 x 6 cells at 3.6 cm, ice underside 5.0 cm above the table | the box is about 26.7 x 26.7 x 6.4 cm (Target lists 10.6 x 10.5 x 2.5 in) and holds the frame; 6 x 6 cells from the rules' contents (32 small + one 2 x 2 large block); exact frame and leg dimensions not published, estimated |
| Small block | 3.55 x 3.55 x 3.5 cm, 4 g | estimated from the cell pitch; thin-walled hollow plastic |
| Large block + Phillip | 7.15 x 7.15 x 3.5 cm, 16 g; figure 3.8 x 6.2 cm, 10 g | estimated |
| Mallet | head 2.4 x 5.0 cm, handle 11 cm, 20 g | estimated; the whole game ships at 1.35 lb (612 g) with its box (Target) |

A real trial should measure the actual game and use those numbers; the law's thresholds are in millimetres of block travel
and do not depend on these estimates.

## Scoring

Read every control step from the scene, and in a real trial from the end state and the video:

- A block is **out** once it has dropped through the tray (its top below the tray's underside; in a real trial, it is on
  the table under the tray).
- **0 for the whole trial** if Phillip falls through (his large block ever sinks 1 cm or more), or if a gripper finger ever
  touches an ice block or Phillip (in simulation: a fingertip point inside a block's box, 2 mm margin sideways, 3 mm below
  its top; in a real trial, from the video). These are recorded and cleared only at a reset.
- **0** if more than 2 unmarked blocks are out.
- Otherwise **1.0** if all 6 marked blocks are out, no unmarked block is out, every fingertip and the mallet's head are at
  least 3 cm from the ice and Phillip, and no block in the tray moves (under 5 mm/s), all continuously for 1 s;
  **0.75** if all 6 marked blocks are out; **0.5** if 4 or 5; **0.25** if 2 or 3; **0** if fewer than 2.

Possible scores: 0, 0.25, 0.5, 0.75, 1.0. Unmarked blocks that drop are allowed in the game's own rules, so one or two
cost only full credit; more than two, which is what a slow press or a careless sweep causes, scores 0.

## Real trial: preparation, reset and measurement

- **Preparation.** Assemble the game as its rules say (tray upside down, blocks pressed in in rows, the last row squeezed
  in, the large block in the four central cells, then turned over onto its legs). Stick 12 mm red dots on the six blocks
  named in `setup`. Put the tray's centre at (27, -10) cm with its sides square to the arms, on tape marks, its legs held
  with double-sided tape. Stand Phillip in his slot. Lay the mallet on its tape mark, face down, handle towards the arms.
  Check before each trial that every block's top is flush with the rim (a straightedge across the rim) and that Phillip
  stands upright.
- **Reset** (about 2 minutes): lift the tray off its tape, turn it over, press the fallen blocks back in from below, turn it
  back, re-check the straightedge, stand Phillip, and put the mallet back.
- **Measurement.** Count the blocks lying under the tray and note which carry dots; check that Phillip's block is still in
  the tray; check the video for any finger touching a block or Phillip, and for the last second with the hand and mallet
  head 3 cm or more clear and nothing moving.
- **Rig safety.** All parts are light plastic; the mallet weighs about 20 g and a tap is a few newtons. No sharp edges,
  heat or liquid.

## Assets and licences

The game (tray, blocks, Phillip, mallet) is built from primitives and meshes in `task.py`: SimReady has no Don't Break
the Ice game. The three distractors are NVIDIA SimReady props, downloaded at first run from revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd` of `nvidia/simready-assets` on Hugging Face, © NVIDIA, CC-BY 4.0:

- [dishware_plastic_cup_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_plastic_cup_a01): mass set to 30 g.
- [dishware_plastic_smallPlate_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_plastic_smallPlate_a01): mass set to 50 g.
- [food_produce_apple_b01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/food_produce_apple_b01): mass set to 150 g.

`simready.py` is the SimReady loader from `tasks/cutters-off-pegboard/`, unchanged apart from its docstring. `mechkit.py`
is the mechanism kit from `tasks/heat-shrink-lug/` (another team's open-source kit), unchanged. `demokit.py` is the
closed-loop demo runner shared by several tasks in this repository (copied from `tasks/heat-shrink-lug/`), unchanged.

## Validation

All runs below are of this `task.py`, through `demokit.py`.

| Run | Where | Result |
|---|---|---|
| `full` (the demonstration), recorded | GPU | 1.0, success at step 510 (25.5 s); all six marked blocks slipped out alone (depth at release 4.9 to 12.4 mm), no other block moved out, Phillip stayed |
| `full`, second run, recorded | GPU | 1.0, success at step 510; the same six releases (4.9 to 12.4 mm) |
| `full` | CPU (`--fast`), twice | 1.0 both times, same six releases |
| `none` (do nothing) | CPU and GPU | 0.0 |
| `careless` (each marked block pressed down with the mallet at 1 cm/s) | CPU; GPU partly | 0.0: the first press already breaks the sheet around block (0,5) and the second brings Phillip down; on the GPU the same pattern (Phillip fell at the (3,1) press) before the run was stopped |
| `fingers` (each marked block jabbed with the closed fingertips, no mallet) | CPU | 0.0: the blocks pop out, but a finger touched the ice |
| `offcentre` (the same brisk taps aimed 18 mm off centre, on the line between two blocks) | CPU | 0.0: each blow pushes two blocks together, the sheet breaks and Phillip falls |
| `stroke0.6`, `stroke1.0`, `stroke2.0` (the demo with slower down strokes, averaging 0.07, 0.04 and 0.02 m/s) | CPU | 0.5 (five marked blocks out), 0.0 and 0.0 |

Model run: Dream-1, run once with the command above for 1,200 steps, picked the mallet up and held it by the tray's right side, but no marked block came out; it scored 0.0.

The task is one-armed (the other arm has no role; the tray stands taped down), so there is no separate one-arm red team.
