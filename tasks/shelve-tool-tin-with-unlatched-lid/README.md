# Shelve a tool tin under a loose lift-off lid

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [task.py](task.py). [simready.py](simready.py) loads the three distractor props, and [mechkit.py](mechkit.py) supplies the per-step and reset hooks and the fingertip positions the scorer uses.

A 1.2 kg steel tool tin stands in front of a low shelf. Its lift-off lid isn't latched and sits 1.5 cm off-centre. The tin has to go onto the shelf with the lid still on it. A grip on the top of the tin closes on the lid alone, so the tin has to be held by its own walls, below the lid.

## Importance and sources

- The setting is an everyday one: moving a box whose lid only rests on it (a tin, a shoebox, a storage box) without the lid coming away. The robot has to grip the part that carries the load, the box's walls below the lid, and keep a long, heavy box level as it goes. Gripping the lid, the obvious handle on top, lifts the lid alone.
- Lift-off ("slip cover") lids are a standard closure for steel tins. Packaging suppliers sell rectangular tins with a "slip cover lid" ([Berlin Packaging, rectangular slip cover tins](https://www.berlinpackaging.com/rectangular-slip-cover-tins)), and others describe it as a "slip-on cover lid" on a tin-plated steel tin ([Container and Packaging, rectangular tin M556C](https://www.containerandpackaging.com/products/rectangular-tin/M556C/)). Such a lid has no hinge or catch, so nothing but its fit keeps it on.
- The UK Health and Safety Executive's guidance on good handling technique tells people to "plan the lift/handling activity" and to "ensure a good hold on the load" ([HSE, good handling technique](https://www.hse.gov.uk/msd/manual-handling/good-handling-technique.htm)). This task tests that choice of hold.
- Limits: we found no statistics on how often people carry boxes with loose lids, and no manufacturer's page for this exact tin. The claim that the situation is common is our own inference, not a cited fact. The supplier pages show that slip-lid tins exist and are sold widely; they don't show how people carry them. A new slip lid is a tension fit; here the lid only rests on the tin, as a worn lid or one set loosely back on would. The tin's size and mass are typical of a steel tin filled with drill bits, not taken from one product.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/shelve-tool-tin-with-unlatched-lid/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the tool tin on the shelf without taking its lid off." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/tool-tin-dream
```

Dream-1 scored 0.0 over the full 2,400 steps: its right arm reached over the shelf toward the back of the table and swung about, and neither arm ever gripped the tin, which stayed where it started with its lid on.

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 900`. The demo is a fixed plan worked out from the published starting layout. Both arms grip the tin across its 6 cm depth, 8.5 cm either side of its middle and 1.7 cm below the lid. They lift it 6 cm, carry it 15 cm onto the shelf, let go and rise about 4 cm clear of the lid.

The run prints `task_score` at the end. `score()` in the task file returns:

- 0 for the rest of the trial once the lid has been off the tin at any step (its centre off the tin's top face, or its underside more than 1 cm from that face), or once the tin has tilted more than 30 degrees. This record is kept in `Trial`, updated after every control step, and cleared only by the simulator's reset (mechkit's `on_reset` hook), never because the objects return to their starting positions;
- 0 while any fingertip point of either gripper is within 2 cm of the tin or the lid, or while either is moving;
- otherwise 1.0 for the tin's base on the shelf top (within 3 mm), wholly inside the shelf's edges and level within 3 degrees, with the lid on;
- 0.5 for the tin's lowest point on the shelf top and its centre over the shelf, but overhanging an edge or tilted more than 3 degrees;
- 0 otherwise.

`success()` is a score of 1.0, held for 3 s.

## Other strategies tried

Scripted variants of the demo, run in the simulator with this scorer:

| Strategy | Score |
|---|---|
| Do nothing | 0 |
| One arm grips the tin itself 7 cm off its middle and carries it to the shelf | 0 (the tin swings down and the lid slides off) |
| One arm grips the lid and carries it to the shelf (the careless way) | 0 (the lid comes away; the tin stays on the table) |
| Both arms carry the tin as in the demo but keep holding it on the shelf | 0 (the grippers never clear the tin) |

In an earlier version of the scene: one arm gripping the tin exactly at its middle carried it to the shelf for 1.0 (it sagged about 8 degrees but arrived). That one-handed route is valid; only a grip on the lid or a one-handed grip off the middle fails.

## Runtime and seeds

- Demo: 342 steps (17.1 s simulated, including the 3 s hold), about 40 s on an L40S once shaders are compiled.
- Dream-1: 2,400 steps (2 minutes simulated), about 5 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

## Assets

The tin, lid and shelf are `yam_arena` boxes. The three distractors (mug, tape, screwdriver) are from NVIDIA's SimReady
library on Hugging Face ([`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets)), pinned to
revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, and downloaded to `~/.cache/simready` on the first run (about
50 MB); nothing large is committed:

- [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01)
- [`tape_clear_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/tape_clear_a01)
- [`handTool_screwdriver_phillips_ph2_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_screwdriver_phillips_ph2_a01)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time: masses
are set to typical values (mug 300 g, tape 60 g, screwdriver 100 g), the screwdriver collides as convex hulls, and the
invisible grasp-annotation curves are deactivated.

`simready.py` is adapted from `tasks/cutters-off-pegboard/simready.py` in this repository. `mechkit.py` is copied unchanged from `tasks/heat-shrink-lug/mechkit.py` in this repository (only its step and reset hooks and `fingertips()` are used here).
