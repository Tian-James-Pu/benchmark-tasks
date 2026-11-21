# Level a dishwasher's upper rack left higher on one side with its height adjusters, then roll it in to dock

A dishwasher's loaded upper rack has been pulled out and left with its right side raised to the top notch and its left
side in the bottom one. Each side has a height adjuster: a grip on the side's top rail with a lever on its outer face.
To get the rack back in so the dishwasher can run, the robot has to notice the tilt and work an adjuster: press one
side's lever in and move that side down (or lift the other side up until it clicks into its notch) until both sides
match. Then it rolls the rack in until it docks, without spilling the glasses and mug. Pushing a side down without
pressing its lever does nothing. A rack pushed in uneven does not dock on the inlet: two notches apart scores 0, one
notch apart at most 0.25.

The instruction gives only the goal: "Put the upper rack back into the dishwasher so it is ready to run."

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: the scene, the adjusters' notches (an `on_step` hook), the scorer (`score`, `success`) and the scripted `demo()` |
| `mechkit.py` | The mechanism kit (articulations written as USD text, hooks, fingertip points), based on `tasks/heat-shrink-lug/mechkit.py` (another team's open-source kit) |
| `simready.py` | Loads SimReady props into a task file, based on `tasks/cutters-off-pegboard/simready.py` |
| `task.json` | The task's details for the board |

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena (`source <ARENA_DIR>/yam.env`).
- Network access the first time the scene is built: `simready.py` downloads the SimReady packages into `~/.cache/simready`.

## Run and score a trial

From the repository root:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/level-dishwasher-upper-rack/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Put the upper rack back into the dishwasher so it is ready to run." \
  --scorer task_score --max-steps 2400
```

The scripted demo (`demo()`, open loop, from the published starting layout):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/level-dishwasher-upper-rack/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=640 -E cam_height=360 \
  --instruction "Put the upper rack back into the dishwasher so it is ready to run." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/level-dishwasher-upper-rack
```

`--scorer task_score` reports 1.0 once full credit has been reached, and otherwise the score at the last step. Nothing is
randomised, so every seed gives the same scene (seed 0 used). Runtime: the demo reaches full credit at about 18.5 s of
simulated time (about 370 steps). A run takes about 1 to 3 minutes of wall time on an L40S-class GPU once shaders are
compiled. A full 2,400-step trial takes up to about 10 minutes.

## The mechanism

- **Runners**: the rack rolls along x, 30 cm from where it starts to its docked stop. It is docked when it is within
  5 mm of the stop.
- **Adjusters**: each side has three notches, 2.5 cm apart. A latched side is held at its notch by a stiff spring
  (4000 N/m at the side). Lifting a side more than 60% of the way to the next notch clicks it in there, as on the real
  racks, where you raise a side by lifting it. Pressing a lever in more than 4 mm (of its 8 mm travel) frees that side
  between the lowest and the highest notch. On letting go, the side latches in the notch at or below it (within 4 mm).
- **Tilt**: the rack is one rigid body on a hinge along x through the left adjuster line, carried by a vertical slider
  (the left adjuster). The right side's height follows from the tilt. While the left side moves, the right side keeps
  its own height.
- Simplified: the rack's weight is not carried by the adjusters, so a freed side stays where it is until a hand moves
  it. On a real rack the loaded side drops, and the manuals say to hold it. The inlet coupling is not modelled: docking
  is the runners' end stop, and the scorer checks that both sides are at the same height.

## Scoring

The scorer reads each side's height from its joints and the adjuster levers' travel. The fingertip points are those
from `mechkit.fingertips`: each finger's tip and a point 3 cm up it, at the finger's middle and both edges.

- **0 for the whole trial** if a tumbler or the mug ever leaves the rack: its origin more than 1 cm outside the rack's
  footprint, more than 3 cm below its floor, or more than 25 cm above it. This is recorded, and cleared only by the simulator's reset hook.
- **1.0**: both sides in the same notch (within 3 mm), with any of the three notches allowed. Both levers released (out
  by 2 mm or less). The rack within 5 mm of its docked stop. Every fingertip point at least 3 cm from the rack's box,
  grips and levers included. All of this held for 2 s, and 1.0 from then on.
- Otherwise, at the end: **0.5** for both sides in the same notch with both levers released but full credit not reached;
  **0.25** for the sides one notch apart with both levers released; **0** for anything else.

A real trial is scored with a tape measure and the eye:
- mark the three notch heights on each side of the tub's front flange;
- mark the docked position on a runner;
- check that nothing has fallen out;
- check the hands are off the rack.

## Demo and red-team results (scripted, GPU)

| Run | What it does | Score |
|---|---|---|
| demo (three runs) | Right arm pinches the right grip and lever together, pushes that side down two notches, lets go (it latches level with the left), then pushes the rack in by its front wall and moves away | 1.0, 1.0, 1.0 |
| none | No movement | 0.0 |
| careless | Pushes the rack straight in without levelling it | 0.0 |
| no lever | Presses the high side down on top of its grip with closed fingers (lever not pressed), then pushes the rack in | 0.0 |
| one notch | Lowers the high side only one notch, then pushes the rack in | 0.25 |

The demo uses one arm, and that is a valid route: on a real rack one hand per side is enough.

Dream-1 (one run of 2,400 steps) scored 0.25: it reached for both sides of the rack and left them one notch apart with both levers released, but never rolled the rack in.

## Real-world trials

- **Hardware**: any 24 in (60 cm) built-in dishwasher with a two-sided upper-rack height adjuster, for example a Bosch
  SHX-series with RackMatic, or a Samsung with adjuster handles. Use two glass tumblers and a ceramic mug.
- **Placement**: the simulation has only the dishwasher's upper section on the table. On a real built-in machine, mount
  the arms on a stand at the open door so the arm bases' plane is 1.5 cm below the rack floor's lowest setting (the
  table top in the simulation), with the tub's front edge 45 cm ahead of the bases and centred between the arms. The
  door, lower rack and counter stay as installed.
- **Setup**:
  - pull the rack out until its front wall is 30 cm short of docked;
  - set the left side in its bottom notch;
  - lift the right side into its top notch;
  - place the tumblers and mug upside down on their marks.
- **Measuring**: mark the notch heights on tape on the tub flange and the docked position on a runner. A phone video
  from the side shows each side's notch and whether the rack reached its mark (within 5 mm). A lever is released when
  it has sprung back out (within 2 mm of its rest position, checked by eye against the grip). The hands are clear when
  no fingertip is within 3 cm of the rack for 2 s; a glass or mug off the rack, or more than 25 cm above its floor,
  scores 0.
- **Safety**: the dishwasher is unpowered and its door is open. Hands stay clear of the arms.

## Importance and sources

Many households run a dishwasher every day, and upper racks with two-sided height adjusters are common on mid-range
models. The manuals warn about this exact fault:

- Bosch's SHX33 manual ([manualslib, page 19](https://www.manualslib.com/manual/19196/Bosch-Shx33ai.html?page=19))
  says to press the RackMatic handles inward to lower the rack, and that you do not need to press them to raise it.
  Before the rack goes back in, it says to "ensure that the rack height is the same on both sides. Otherwise, the
  dishwasher door will not close and the upper spray arm will not connect to the water circuit."
- A later Bosch manual (SHE53T52UC, [manualslib, page 17](https://Manualslib.com/manual/2396464/Bosch-She53t52uc.html?page=17))
  says the rack has 3 height steps: "To lower the rack, push the RackMatic lever inward. To raise the rack, hold the
  basket on the sides of the upper rack and lift it up."
- Samsung's support page ([How do I adjust the dishwasher rack height?](https://www.samsung.com/uk/support/home-appliances/how-do-i-adjust-the-dishwasher-rack-height/))
  says: "If you do not adjust with the left and right handle together simultaneously, the dishwasher door may not
  close properly."

The notch pitch (2.5 cm) is an estimate: these manuals give the dish sizes for each setting, not the travel.

## Assets

The tumblers, the mug in the rack and the distractors come from the Hugging Face dataset
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`dishware_glass_tumbler_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_glass_tumbler_a01)
- [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01)
- [`dishware_mug_b01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_b01) (distractor)
- [`cleaning_dishSoap_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/cleaning_dishSoap_a01) (distractor)
- [`cleaning_sponge_b01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/cleaning_sponge_b01) (distractor)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time:
- masses set: tumblers 250 g, rack mug 350 g, soap bottle 450 g, sponge 20 g, counter mug 330 g;
- the soap bottle's jointed parts are welded into one rigid body;
- the tumblers and the rack mug are turned upside down;
- the props collide as convex hulls.

SimReady has a dishwasher (`appliance_dishwasher_a01`), but its loader welds the racks into one body, so the tub, rack,
adjusters and counter here are built from primitives at real size in `task.py`. Every other mesh is generated by
`task.py` and `mechkit.py`.
