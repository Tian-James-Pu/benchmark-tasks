# Straighten a stack whose middle carton has slid out

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [task.py](task.py).

Three cartons are stacked; the middle one has slid 5 cm out of line, so the top carton (glass vials) is only partly supported. The stack has to be straightened without tipping the top carton.

## Assets

The three cartons are box primitives. The three distractors are loaded by [simready.py](simready.py) from the Hugging
Face dataset [`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, and download at first run:

- [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01)
- [`tape_clear_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/tape_clear_a01)
- [`handTool_screwdriver_phillips_ph2_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_screwdriver_phillips_ph2_a01)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time:
masses are set (mug 300 g, tape 60 g, screwdriver 100 g) and the screwdriver uses a convex collision shape.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/restack-shifted-middle-carton/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The blue middle carton in this stack has slid out of line. Straighten the stack so all three cartons sit squarely on top of each other, without tipping the white top carton, which holds glass vials." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/restack-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 900`. In the demo the left arm lifts the top carton off and holds it, the right arm lifts the middle carton back in line, and the left arm sets the top carton back.

The run prints `task_score` at the end. `score()` in the task file returns 0 if the top carton ever tilts past 20 degrees; otherwise 1.0 for the three cartons stacked in order, level and at rest, each centred within 1.5 cm and turned within 10 degrees of the one below, with the bottom carton within 2 cm of its start; 0.5 when those limits are met only at 3 cm, 20 degrees and 5 cm; else 0. The scorer sees carton poses only, not the grippers, and ignores the distractors. `success()` is a score of 1.0 held for 3 seconds.

## Runtime and seeds

- Demo: 503 steps (25 s simulated), about 1.5 minutes on an L40S including start-up.
- Dream-1: 2,400 steps (2 minutes simulated), about 5 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.
