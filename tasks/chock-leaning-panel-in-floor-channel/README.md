# Stand a fallen display panel back up in its floor channel and chock it before letting go

A task file for the `yam_arena` embodiment (yam-isaac-lab-arena, Isaac Lab-Arena 0.3.1) on the station scene: [task.py](task.py).

A plywood panel that belongs upright in a steel floor channel has lost its chock and fallen over onto the channel's right wall. Its bottom edge sits on a pivot rib, so it cannot stand by itself: one arm has to stand it up and keep hold of it while the other slides the chock into the channel beside it, and the first arm may only let go once the chock is in.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/chock-leaning-panel-in-floor-channel/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stand the leaning panel upright in its floor channel and secure it with the red chock so it stays up by itself." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/panel-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 900`, or `pipeline/run_trial.sh chock-leaning-panel-in-floor-channel demo` from the repository root.

The run prints `task_score` at the end. `score()` in the task file returns 0 if the panel ever fell after being stood up (past 20 degrees after having been within 6), ever leaned past 60 degrees, or ever had its bottom edge out of the channel. Otherwise, with the panel at rest within 6 degrees of vertical on its rib, the chock standing in the channel beside it, and both grippers' fingers more than 2 cm from the panel and the chock, it returns 1.0 if all of the chock is inside the open end and 0.5 if only 1 to 3.5 cm of it is; 0 for anything else. `success()` is a score of 1.0, and `task_score` gives 1.0 once that has held for 3 seconds.

The fall is remembered in module-level state for the whole trial. It is cleared only by a reset of the scene (`mechkit`'s `on_reset` hook), never by anything that happens during a trial.

## Files and assets

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: scene, scoring (`score`, `success`) and the scripted `demo()` |
| `mechkit.py` | Shared helper copied from `tasks/bookcase-shelf-pins/`; used here for the reset hook and for reading the grippers' finger positions |
| `simready.py` | Loads SimReady props into a task file, downloading them once from Hugging Face |

The channel, panel and chock are `yam_arena` boxes. The three distractors come from the Hugging Face dataset [`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01)
- [`tape_clear_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/tape_clear_a01)
- [`handTool_screwdriver_phillips_ph2_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_screwdriver_phillips_ph2_a01)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time: masses are set (0.30 kg, 0.06 kg, 0.10 kg) and the screwdriver collides as a convex hull. The files are not committed; `simready.py` downloads them into `~/.cache/simready` the first time the scene is built, which needs network access.

## Runtime and seeds

- Demo: 340 steps (17 s simulated), about 1.5 minutes on an L40S including start-up.
- Dream-1: 2,400 steps (2 minutes simulated), about 5 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic; its grasp targets are measured from the settled starting layout, so it does not tolerate jitter.
