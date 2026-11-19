# Clear a cue ball and its chalk-cube stop off a sloping return track

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [task.py](task.py).

A cue ball sits on a sloping two-rail return track whose lower end has no stop. The only thing holding it is a cube of cue chalk set on the rails below it. The ball belongs in its stand and the chalk on the coaster. Taking the chalk first lets the ball roll off the track, onto the table and into a mug of coffee. A roll of tape and a screwdriver beyond the track are distractors.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/cue-ball-held-by-chalk-on-return-track/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The cue ball on the sloping return track is held back only by the cube of chalk below it. Put the ball in its stand and the chalk on the coaster, without letting the ball roll off the open end of the track." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/cue-ball-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 900`. The left arm grips the ball and lifts it off the track; only then does the right arm take the chalk; the ball is seated in its stand and the chalk set on the coaster.

The run prints `task_score` at the end. `score()` in the task file returns 0 if the ball's underside ever came within 3 mm of the table outside the stand's opening or the mug ever tipped past 20 degrees; otherwise 1.0 with the ball at rest in its stand and the chalk at rest on the coaster, 0.5 with only the ball in its stand, and 0 in every other case. `success()` is a score of 1.0, held for 3 s.

## Assets

The mug, the tape roll and the screwdriver come from the Hugging Face dataset
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01)
- [`tape_clear_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/tape_clear_a01)
- [`handTool_screwdriver_phillips_ph2_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_screwdriver_phillips_ph2_a01)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time:
masses are set (mug 0.35 kg with its contents, tape 60 g, screwdriver 0.10 kg), the screwdriver collides as a convex
hull, and the invisible grasp-annotation curves are deactivated. The track, riser, ball, chalk, stand and coaster are
`yam_arena` primitives. The files are not committed: [simready.py](simready.py) downloads them into
`~/.cache/simready` the first time the scene is built, which needs network access.

## Runtime and seeds

- Demo: 408 steps (20 s simulated), about 1 minute on an L40S once shaders are compiled.
- Dream-1: 2,400 steps (2 minutes simulated), about 5 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.
- The rails are spawned level on the riser and drop into their sloped position, with the ball and the chalk on them, during the 0.5 s settle before the first frame; everything is at rest from the first step.
