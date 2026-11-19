# Take the temporary prop out from under a repaired shelf

A task file for the `yam_arena` embodiment (yam-isaac-lab-arena, Isaac Lab-Arena 0.3.1) on the station scene:
[task.py](task.py). [simready.py](simready.py) loads the NVIDIA SimReady props it uses.

A pine shelf lies across two fixed brackets with a full tin of soup on it. A red timber prop, 8 mm taller than the
brackets, was stood under the shelf while the right-hand bracket was replaced and is still there, so the shelf
rests on the prop and the left bracket, not on the new bracket. The prop has to come out and go on the blue mat
with the shelf left on both brackets and the tin standing.

## Run a trial

Dream-1:

```bash
source ~/IsaacLab-Arena/yam.env
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/remove-prop-from-under-shelf/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The shelf's right-hand bracket has been replaced, but the red prop that held the shelf up is still under it. Take the prop out and put it on the blue mat, leaving the shelf on both brackets with the tin of soup still standing on it." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/remove-prop-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 900`, or from the
repository root `pipeline/run_trial.sh remove-prop-from-under-shelf demo`.

The run prints `task_score` at the end. `score()` in the task file returns 0 if the tin ever tips past 30 degrees
or leaves the shelf, any part of the shelf ever drops more than 3 cm below the brackets' tops, or the prop ever
tips past 30 degrees while any part of it is under the shelf. Otherwise, with
the shelf lying on both brackets, the tin upright on it, everything at rest and no part of the prop under the
shelf, it returns 1.0 when the prop is on the mat and neither shelf end has moved more than 2 cm, else 0.5.
`success()` is a score of 1.0 held for 3 s.

In the demo the left arm grips the shelf across its width just left of centre and raises it, taking its weight
off the prop; the right arm draws the prop out by its front end; the left arm lowers the shelf onto both brackets
and lets go; the right arm carries the prop to the mat.

## Runtime and seeds

- Demo: 439 steps (22 s simulated), about 40 s on an L40S once shaders are compiled, a few minutes the first time.
- Dream-1: 2,400 steps (2 minutes simulated), about 5 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and
  deterministic; it scored 1.0 on repeated runs.

## Other strategies tried

Each is the task file with a different scripted `demo()`, run once.

| Variant | Score |
|---|---|
| Do nothing | 0 |
| Right arm alone pulls the prop out and puts it on the mat (the shelf's right end is dragged forward and falls to the table, and the tin falls off) | 0 |
| Right arm alone shoves the prop backward from the front (the shelf's right end is dragged back off its bracket) | 0 |
| Right arm alone knocks the prop over sideways so the shelf drops onto the new bracket, then tries to drag the fallen prop out | 0 |
| Both arms as in the demo, but the prop is left on the table in front of the shelf | 0.5 |

Lifting the whole shelf away with its tin, taking the prop out and putting the shelf back was not tried. It would
score 1.0 if the tin never tips and the shelf ends within 2 cm of where it started, and counts as a valid solution.

## Assets

The tin of soup and the three distractors come from the Hugging Face dataset
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`food_can_tomatoSoup_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/food_can_tomatoSoup_a01) (the tin)
- [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01)
- [`tape_clear_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/tape_clear_a01)
- [`handTool_screwdriver_phillips_ph2_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_screwdriver_phillips_ph2_a01)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time:
- masses are set: tin 350 g, mug 300 g, tape 60 g, screwdriver 100 g;
- the tin and the screwdriver collide as convex hulls instead of signed distance fields;
- the invisible grasp-annotation curves are deactivated.

The brackets, shelf, prop and mat are `yam_arena` boxes. The props' files are not committed: `simready.py`
downloads them into `~/.cache/simready` the first time the scene is built, which needs network access.
