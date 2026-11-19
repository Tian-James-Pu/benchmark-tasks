# Free a mug whose handle is cocked on a loose retaining rod

A task file for the `yam_arena` embodiment (yam-isaac-lab-arena, Isaac Lab-Arena 0.3.1) on the station scene: [task.py](task.py).

A beech rod lies loose in two open brackets and runs through the handles of two mugs. The red mug, on the rod's overhanging open end, is twisted 26 degrees so its handle binds on the rod, and the rod has already been dragged to within 1 cm of dropping out of its far bracket. The robot has to get the red mug off the rod's open end and leave the rod pushed back through the far bracket with the blue mug still on it.

## Importance and sources

- **The physical problem is binding of a short sliding fit pulled off-axis.** A mug handle on a rod is a very short "bearing" (about 1.2 cm of handle against the rod) driven through a long moment arm (the mug body). Plain-bearing makers document a binding ratio (about 2:1, moment arm to bearing length, for common plain bearings) past which such a fit binds, and a larger drive force does not free it; the remedies are to shorten the moment arm, lengthen the bearing or reduce friction ([PBC Linear, "Demystifying the 2:1 Ratio"](https://pbclinear.com/blogs/blog/white-paper-demystifying-the-2-1-ratio); [igus, "The 2:1 rule" (PDF)](https://www.igus.com/contentData/wpck/pdf/US_en/DryLin%20Linear%20Bearings%202-1%20Rule.pdf)). In this task that means squaring the mug up and moving it from close to the rod, while the loose rod, which the binding handle would otherwise drag, is kept in its brackets.
- **Rings and handles catching on household bars are an everyday nuisance.** Consumer how-to pages describe curtain rings catching and sticking on poles ([eHow, "How to Keep Curtain Rings From Catching"](https://www.ehow.com/how_6150508_keep-curtain-rings-catching.html); [House Digest on rings sticking on wooden rods](https://housedigest.com/1772262/prevent-curtain-rod-from-sticking-to-wooden-curtain-rod-pledge)).
- **Retaining bars for mugs are a real fixture.** Boat galleys keep mugs in racks with a retaining bar so they "don't bang around" ([Practical Sailor, "Design For: A Mug Rack"](https://www.practical-sailor.com/belowdecks-amenities/galley/design-for-a-mug-rack/)).
- **Limits.** That galley rack's bar runs in front of the mugs, not through their handles; we did not find a manufacturer's product or a survey documenting the exact loose-rod-through-handles arrangement modelled here, nor any statistic on how often people meet a cocked mug handle. The importance claim rests on the everyday experience of freeing a ring or handle jammed askew on a loose bar, which the sources above show is common, not on this exact fixture.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/slide-mug-off-retaining-rod/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the red mug off the retaining rod and stand it on the table. Leave the rod seated through both brackets with the blue mug still on it." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/slide-mug-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 900`.

In its one 2,400-step run Dream-1 scored 0.0: it raised both grippers and brought the right one towards the red mug several times but never grasped it, so the red mug stayed cocked on the rod (the rod and blue mug were not disturbed).

The run prints `task_score` at the end. `score()` in the task file returns 0 if the rod ever sloped more than 5 degrees or left a bracket, the blue mug ever tilted more than 20 degrees or the red mug more than 45; otherwise 1.0 for the red mug upright on the table 2 cm clear of the rod, the rod level and resting on both slot floors (within 3 mm of its seated height) with its left end through the left bracket, the blue mug within 2 cm of its start, and no fingertip of either gripper within 3 cm of the red mug or the rod; 0.5 if all that holds except that the rod has not been pushed back through the left bracket or the blue mug moved 2 to 5 cm. `success()` is a score of 1.0 held for 3 s. The fingertip positions come from `mechkit.fingertips()` through the kit's step hook. A harm, once seen, lasts for the rest of the trial: it is cleared only in the kit's `on_reset` hook, after an environment reset, never because objects are back where they started.

## Runtime and seeds

- Demo: 458 steps (23 s simulated, including the 3 s hold), about 1 minute on an L40S once shaders are compiled.
- Dream-1: 2,400 steps (2 minutes simulated), about 5 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

## Files and assets

| File | What it is |
|---|---|
| `task.py` | Scene, `score()`, `success()` and the scripted `demo()` |
| `mechkit.py` | The mechanism kit, copied unchanged from `tasks/heat-shrink-lug/mechkit.py` in this repository (another team's open-source kit). This task uses only its step and reset hooks and `fingertips()`; the scene has no kit parts |
| `simready.py` | Loads SimReady props into a task file (`SimReady(...)`), downloading them once from Hugging Face. Adapted from `tasks/cutters-off-pegboard/simready.py` in this repository |

The two mugs are the YCB mug scan in the simulator's object library (`mug_ycb_robolab`), scaled to 86%, each given 150 g, the blue one recoloured. The rod and brackets are `yam_arena` boxes.

The two distractors are NVIDIA SimReady props from [`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`food_can_tunaFish_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/food_can_tunaFish_a01)
- [`cleaning_sponge_b01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/cleaning_sponge_b01)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time: masses are set (tuna tin 170 g, sponge 15 g) and the invisible grasp-annotation curves are deactivated. The files are not committed; `simready.py` downloads them into `~/.cache/simready` the first time the scene is built, which needs network access.

## Checks run with scripted variants of the demo

One run each; the scorer's result is shown. After the scorer gained its hands-off check and reset hook, the scripted demo was run twice (1.0 both times) and four rows were rerun with the same results: do nothing, the careless drag, one arm without pushing the rod back, and holding instead of letting go. The other rows are from the earlier scorer on the same scene.

| Variant | Score |
|---|---|
| Do nothing | 0 |
| Grip the twisted mug and drag it along the rod as it stands, rod not held | 0 (the handle takes the rod with it; the rod drops out of the left bracket) |
| Lift the mug straight up | 0 (the handle lifts the rod out of its brackets) |
| Pull the mug straight towards the robot | 0 (the mug stays on the rod) |
| Shove the mug towards the open end with a closed gripper | 0 (the rod is dragged out of the left bracket) |
| One arm only: square the mug up and slide it off, rod neither held nor pushed back | 0.5 |
| Both arms as in the demo, but the rod is not pushed back | 0.5 |
| The demo, but neither gripper lets go (red mug held on the table, rod held after pushing it back) | 0 (fingertips within 3 cm) |
| One arm only, carefully: square the mug up, slide it off, set it down, then push the rod back with the same arm | 1.0 |

The last row is intended: holding the rod with the second arm is the safer way, not the only way. What earns nothing is moving the mug while its handle is still cocked on the rod.
