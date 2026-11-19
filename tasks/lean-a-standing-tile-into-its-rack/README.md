# Move a ceramic tile left standing on its edge into a lean-to rack

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [task.py](task.py).

A 30 x 15 cm glazed wall tile stands on its long edge in the open, balanced so that about 3 degrees of tilt
either way makes it fall flat. Behind it a lean-to rack is fixed to the table: two posts and a toe strip. The
robot has to carry the tile into the slot and leave it leaning back on both posts. The physical difficulty is
that the tile is only stable at the two ends of the job: standing untouched at the start and leaning at the end.
Lifted, carried and set down upright in the slot, it is as unstable as before, so it has to be controlled
(two hands on its top edge, a loosened grip that still cages it) until its weight is on the posts.

## Importance and sources

Thin flat things are commonly stored upright on edge and leaned against a support rather than stacked flat or
left balanced. For tiles specifically, the Tiles & Architectural Ceramics Society's collection-care guidance
says "Do not store tiles flat, one on top of the other" and "Do store tiles upright on edge in racks or boxes
appropriate to the size of the tile", and names moving, handling and poor storage as the greatest risks to
tiles ([TACS: Caring for the Tile Collection](https://tilesoc.org.uk/tacs/conservation/caring-tile-collection/)).
The same everyday action, leaning a tile, a framed picture, a cutting board or a tray back against something so
it cannot topple, is a firsthand household one; the category is claimed on that basis.

Limits: the cited source is conservation guidance for tile collections. It supports the storage practice (on
edge, in racks) and that handling is when tiles get damaged. It does not measure how often people do this, and
no statistic is claimed. The harm modelled is a fall that would crack the tile, not an injury to a person.

## Assets

The tile, posts and toe strip are box primitives. The three distractors are loaded by [simready.py](simready.py) from
the Hugging Face dataset [`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to
revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, and download at first run:

- [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01)
- [`tape_clear_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/tape_clear_a01)
- [`handTool_screwdriver_phillips_ph2_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_screwdriver_phillips_ph2_a01)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time:
masses are set (mug 300 g, tape 60 g, screwdriver 100 g) and the screwdriver uses a convex collision shape.

`mechkit.py`, in this folder, is an unchanged copy of the open-source mechanism kit used by `tasks/heat-shrink-lug/`.
This task uses only its step and reset hooks and its `fingertips()` (points on both grippers' fingers), for the
scorer's hands-clear check. Importing it hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without
modifying them.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/lean-a-standing-tile-into-its-rack/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the tile in the rack, leaning back on both posts." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/tile-rack-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 600`. The demo is a fixed,
open-loop plan from the published starting layout; it reads no simulator state.

The run prints `task_score` at the end. Scoring is computed by a tracker in `task.py` that runs after every
control step (`Trial.step`):

- a tilt past 45 degrees at any step is recorded as a fall and scores 0 for the rest of the trial; the record is
  cleared only by the scene reset hook (`Trial.reset`), never by the tile's position;
- the tile "leans" when it is at rest (under 1 cm/s and 0.5 rad/s), tilted 5 to 45 degrees toward the posts, its
  back face within 5 mm of the top front edge of both posts, and every point `fingertips()` reports on both
  grippers (tip to 3 cm up each finger) at least 3 cm outside the tile;
- 1.0 once it has leaned for 4 s (80 steps) without a break at no more than 25 degrees, with its bottom edge on the
  table (within 3 mm) between toe strip and posts and its centre within 3 cm of the midline; the trial then ends;
- 0.5 if at the end it leans but short of that; 0 otherwise, including an upright tile or one still held.

## Dream-1 run

Dream-1 scored 0 in one run of 2,400 steps: both arms moved about in front of the tile,
but never grasped it; the tile was still standing upright where it started when the step limit was reached.

## Runtime and seeds

- Demo: about 450 steps (22 s simulated), about 1 to 2 minutes on an L40S including start-up.
- Dream-1: up to 2,400 steps (2 minutes simulated), about 5 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

## Checks run on the scorer

The scripted demo scored 1.0 in two fresh runs (423 steps each; the trial ends once the 4 s hold completes).
Scripted variants of the demo, each run once with the current scorer:

- doing nothing: 0;
- the careless way, pushing the standing tile's top toward the rack with one closed gripper: 0 (it falls flat);
- the demo's motion with one arm alone, pinching 7 cm off-centre: 0.5 (it ends leaning on both posts with the
  gripper clear, short of the 1.0 conditions);
- the demo without the release, loosened grippers left around the leaning tile: 0 (fingertips within 3 cm).

Earlier checks of the same scene, before the scorer gained its hands-clear test: one arm caging the top edge
loosely and dragging the tile: 0 (it falls); carried into the slot and let go upright: 0; leaned on the posts
from in front of the toe strip: 0.5.
