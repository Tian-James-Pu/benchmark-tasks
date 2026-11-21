# Free a riser that is sitting on a pencil, without spilling the open jar on it

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [task.py](task.py). [simready.py](simready.py) loads the SimReady props.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/free-riser-foot-from-pencil-under-open-jar/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The riser under the open jar of varnish is sitting on a pencil. Get it sitting flat on the table where it is, with the jar still on it." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/riser-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 900`. The left arm grips the
riser's near end across its width (3 mm left of the midline, on the leaning riser's centreline) and raises it 2.5 cm, pivoting it on its far end; the right arm takes the freed
pencil 7 cm to the right; the left arm lowers the riser flat and lets go.

The run prints `task_score` at the end. `score()` in the task file returns 0 if the jar ever tilts past 15 degrees
(varnish over the rim). Otherwise, with the riser, jar and pencil at rest and no gripper finger within 2 cm of any of
them, it returns 1.0 for the riser flat (all four bottom corners within 2 mm of the table), within 1 cm of where it
started and 5 degrees of straight ahead, the jar standing upright on it (within 3 degrees; its base centre inside
the riser's top and its lowest point within 4 mm of it) and the pencil at least 2 cm clear; 0.5 if the riser is flat
with the jar upright on it and the pencil not overlapping it seen from above, but the riser moved or turned further
than that or the pencil lies within 2 cm; 0 otherwise. `success()` is a score of 1.0, and must hold for 3 s.
Lifting the jar off and putting it back upright is a valid route and is not penalised.

The record of a spill is module-level state kept for the whole trial. It is cleared only in `mechkit`'s `on_reset`
hook, which runs after every reset of the scene, never because the objects return to where they started. The
hands-off check uses `mechkit`'s `fingertips()`, read after every control step. Run one trial per process to be safe.

## Real-world trial

Use a planed oak block 26 x 7 x 4 cm, an ordinary hexagonal pencil, and an 8 oz straight-sided glass with water up to
0.30 kg in total. Lay the pencil, set the block down with its right edge 1.5 cm over the pencil tip, and stand the
glass on it. Measure with a ruler: each corner of the block against the table (a 2 mm feeler or a stack of two coins
must not slide under any corner), the block's centre against tape marks at its start, the pencil's gap to the block.
A spill is any water over the rim; wipe it up before the next trial.

## Importance and sources

A wooden block or riser on a bench that has been set down on a stray pencil rocks, and what stands on it leans. Here
the thing on it is an open jar of oil-based varnish, a common finishing product. Its manufacturer classifies it as a
flammable liquid and vapour, and its spill instructions are to shut off ignition sources and absorb the spill with
inert material ([MINWAX Fast-Drying Polyurethane Clear Satin, Safety Data Sheet, revised 16 July 2024](https://www.paintsupply.com/wp-content/uploads/2024/05/SDS-Minwax-027426710283-101484-60.pdf)).
The physical difficulty is that the pencil is pinned by the load: pulling it drags the riser and the jar along, and
tipping the riser up carelessly tips the jar. One hand takes the weight off that side while the other clears the
pencil, and the riser is checked for sitting flat before work goes on.

Limits: the source shows the hazard of a varnish spill, not how often things get set down on a pencil; we found no
statistic for that, and the claim that it is an everyday occurrence is our own judgement. The riser is a plain block
standing in for any small stand or bench block.

## Assets

The jar and the three distractors come from the Hugging Face dataset
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`dishware_glass_8oz_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_glass_8oz_a01)
- [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01)
- [`tape_clear_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/tape_clear_a01)
- [`handTool_screwdriver_phillips_ph2_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_screwdriver_phillips_ph2_a01)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time:
masses are set (glass 0.30 kg, standing for a jar with varnish in it; mug 0.30 kg; tape 0.06 kg; screwdriver
0.10 kg), the glass and the screwdriver collide as convex hulls, and the invisible grasp-annotation curves are
deactivated. No joints are involved. The riser and the pencil are `yam_arena` boxes. `mechkit.py` is copied unchanged from
`tasks/heat-shrink-lug/mechkit.py` in this repository (another team's open-source mechanism kit); this task uses
only its `fingertips`, `on_step` and `on_reset` hooks. The files are not committed:
`simready.py` downloads them into `~/.cache/simready` the first time the scene is built, which needs network access.

## Runtime and seeds

- Demo: 384 steps (19 s simulated), about 1 minute on an L40S including start-up.
- Dream-1: 2,400 steps (2 minutes simulated), about 5 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.
- The riser is spawned level on the pencil tip and settles onto its left edge in the 0.5 s before the first frame;
  it is at rest, leaning 7 degrees, from the first step.
