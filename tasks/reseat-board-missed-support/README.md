# Reseat a board that missed one of its two supports

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena),
Isaac Lab-Arena 0.3.1) on the station scene: [task.py](task.py). [simready.py](simready.py) loads the NVIDIA SimReady
props.

## Run a trial

From the root of this repository, in the Isaac Lab-Arena environment (`source ~/IsaacLab-Arena/yam.env`).

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/reseat-board-missed-support/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Set the board across both wooden blocks without toppling a block, dropping the board or knocking the bottle over." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/reseat-board-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 600`. Both arms grip the board
across its width, raise it level and clear of the blocks, carry it 6 cm right, lower it onto both blocks, let go and
withdraw.

The run prints `task_score` at the end. `score()` in [task.py](task.py) implements the scoring in `task.json`; a full
score that holds for 3 s (`SUCCESS_HOLD_S`) ends the trial.

## Checks run

| Run | Score |
|---|---|
| Scripted demo | 1.0 |
| Doing nothing | 0 |
| Lifting only the low end level and pushing it 6 cm right, so the high end slides across its block | 1.0 (a valid, if less careful, recovery) |
| Gripping the low end and swinging it back and right along the table | 0 (the bottle is knocked over) |

## Runtime and seeds

- Demo: about 285 steps (14 s simulated), under a minute of wall clock on an RTX GPU once Isaac Sim has started (its
  first start on a machine takes several minutes while it compiles shaders).
- Dream-1: up to 2,400 steps (2 minutes simulated), about 5 minutes of wall clock.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

## Assets

The blocks and board are `yam_arena` boxes. The other props are from NVIDIA's SimReady library on Hugging Face
([`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets)), pinned to revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, and downloaded to `~/.cache/simready` on the first run (about 50 MB);
nothing large is committed:

- [`food_beverage_beerBottle_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/food_beverage_beerBottle_a01)
- [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01)
- [`tape_clear_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/tape_clear_a01)
- [`handTool_screwdriver_phillips_ph2_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_screwdriver_phillips_ph2_a01)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time: masses
are set to typical values (bottle 200 g, mug 300 g, tape 60 g, screwdriver 100 g), the screwdriver collides as convex
hulls, and the invisible grasp-annotation curves are deactivated.

`simready.py` is adapted from `tasks/cutters-off-pegboard/simready.py` in this repository.
