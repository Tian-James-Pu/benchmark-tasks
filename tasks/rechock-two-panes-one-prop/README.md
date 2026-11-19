# Re-chock two panes standing in a rack when there is only one prop block to hold them

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [task.py](task.py).

Two panes stand on edge in a rack. A pane with nothing in front of it falls flat. A chock belongs between a pane and its short stop block, where there is room for one chock only. The left one has no chock and is held by the only prop block, standing beside the chock place; the right one is held by a split red chock that has to be replaced. The prop block can leave the left pane only after its new chock is in, and the red chock can come out only after the prop block is standing against the right pane.

## Assets

The rack, panes, prop block and chocks are box primitives. The three distractors are loaded by [simready.py](simready.py) from
the Hugging Face dataset [`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to
revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, and download at first run:

- [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01)
- [`tape_clear_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/tape_clear_a01)
- [`handTool_screwdriver_phillips_ph2_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_screwdriver_phillips_ph2_a01)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time:
masses are set (mug 300 g, tape 60 g, screwdriver 100 g) and the screwdriver uses a convex collision shape.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/rechock-two-panes-one-prop/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Both acrylic panes in the rack need a new beech chock between the pane and its stop block: fit one to the pane the prop block is holding, replace the split red chock on the other, and finish with the prop block and the red chock away from the rack. Do not lift the panes or let either one fall." \
  --scorer task_score --max-steps 3600 --store-frames --log-dir ~/yam-eval/rechock-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 1400`.

The run prints `task_score` at the end. `score()` in the task file returns 0 if either pane ever tilted more than 30 degrees or was raised more than 2 cm; otherwise 0.5 for each pane that ends at rest on the strip within 10 degrees of upright, with a new chock standing between it and its stop block (inside the stop block's 7 cm length) and with the prop block and the red chock at least 3 cm away from it. `success()` is a score of 1.0 with every loose task object at rest, held for 3 s.

## Runtime and seeds

- Demo: 1,217 steps (61 s simulated), about 2.5 minutes on an L40S including start-up.
- Dream-1: up to 3,600 steps (3 minutes simulated).
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

## Checks run on the scorer

Scripted variants of the demo, each run once:

- doing nothing: 0;
- taking the red chock out with nothing else at the right pane: 0 (the pane falls);
- taking the prop block from the left pane before its new chock is in, then doing the right side correctly: 0 (the
  left pane falls, and the good final state does not undo it);
- fitting the left chock and moving the prop block away, right side untouched: 0.5;
- fitting the left chock but leaving the prop block against the pane: 0;
- standing the new right chock against the pane beside the red chock (not in front of the stop block), then taking
  the red chock out, without using the prop block: 0.5 (the left pane only);
- sliding the red chock 9 cm along the right pane under load, out of the chock place, fitting the new chock and then
  taking the red chock away, without using the prop block: 1.0. The pane is held throughout, so this is scored as a
  full result; the prop block is the intended temporary support, not the only possible one.

The scorer reads object poses only and cannot tell a chock still held by a gripper from one that was released.
