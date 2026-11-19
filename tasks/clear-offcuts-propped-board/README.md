# Clear scrap offcuts off a bench when one of them is propping up a varnished batten

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [task.py](task.py).

A varnished batten lies across the bench with one end on a pine bearer and the other on a plywood offcut that was used as a prop. The offcut is scrap and has to go in the bin with the other offcut, but taking it away like any other debris drops the batten onto the bench. The spare bearer lies beside the batten.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/clear-offcuts-propped-board/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the two dark plywood offcuts in the bin and leave the freshly varnished batten lying level on the two pine bearers." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/clear-offcuts-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 1200`.

The run prints `task_score` at the end. `score()` in the task file returns 0 if any part of the batten ever came within 1.5 cm of the bench (that record is cleared only when the scene is reset); otherwise 0.25 per dark offcut at rest in the bin, and 1.0 when both are in the bin and the batten is at rest, level within 3 degrees, on the two pine bearers, with no part of either gripper's fingers within 2 cm of it. `success()` is a score of 1.0 held for 3 s.

In the demo the left arm bins the small offcut, then grips the batten at its middle and raises it 2 cm; the right arm slides the prop out sideways by its far end, draws it forward and bins it, then slides the spare bearer in by its far end to where the prop stood; the left arm sets the batten down and lets go.

## Runtime and seeds

- Demo: 1,018 steps (51 s simulated), about 2 minutes on an L40S including start-up.
- Dream-1: 2,400 steps (2 minutes simulated), about 5 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

## Helper modules

- `mechkit.py` is copied from `tasks/tube-bender/` in this repository. Only its `fingertips`, `on_step` and `on_reset` are used: the scorer reads points on the gripper fingers to check that the batten has been let go of, and clears its record of a dropped batten when the scene is reset.
- `simready.py` is adapted from `tasks/cutters-off-pegboard/` in this repository; it loads the props below.

## Assets

The batten, bearers, offcuts and bin are primitives. The three distractors are props from NVIDIA's SimReady library, [`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd) at revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, © NVIDIA, CC-BY 4.0. They are downloaded into `~/.cache/simready` at first run; nothing of theirs is committed here.

| Object | Package |
| --- | --- |
| Mug | [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01) |
| Tape roll | [`tape_clear_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/tape_clear_a01) |
| Screwdriver | [`handTool_screwdriver_phillips_ph2_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_screwdriver_phillips_ph2_a01) |

Changes at load time: each prop's mass is set (mug 0.30 kg, tape 0.06 kg, screwdriver 0.10 kg) with its authored inertia scaled to match, its invisible grasp-guide curves are deactivated, and the screwdriver collides as convex hulls.
