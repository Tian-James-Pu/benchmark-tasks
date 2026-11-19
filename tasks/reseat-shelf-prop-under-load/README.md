# Put a knocked-out prop back under a loaded shelf

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [task.py](task.py).

A shelf plank with a paint tin on it lies across a fixed stand and a loose timber prop. The prop has been knocked almost out: 1.5 cm of it is still under the plank. Pushing the loaded prop drags the plank along with it, so one arm has to take the plank's weight while the other moves the prop.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/reseat-shelf-prop-under-load/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The prop under the right end of the shelf has been knocked almost out. Put it squarely back under the shelf without letting the shelf drop or the paint tin fall." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/reseat-shelf-prop-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 900`.

The run prints `task_score` at the end. `score()` in the task file returns 0 if the plank ever dropped 3 cm or the tin ever tipped past 30 degrees or left the plank; otherwise, with the plank level and at rest on the stand and the prop and the tin upright on it, 1.0 for at least 4.5 cm of the plank's width over the prop and the plank within 1 cm of its starting place, 0.5 for at least 3 cm and within 3 cm, else 0. `success()` is a score of 1.0, held for 3 seconds.

## Runtime and seeds

- Demo: 394 steps (20 s simulated), about 1.5 minutes on an L40S including start-up.
- Dream-1: 2,400 steps (2 minutes simulated), about 5 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.
