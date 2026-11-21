# Recentre a floor pad on its pad driver and click the centre lock home

A task file for the `yam_arena` embodiment (Isaac Lab-Arena, station scene): [task.py](task.py).

A new 17-inch red floor pad has landed 5 cm off-centre on a rotary floor machine's pad driver, which lies face up.
The centre lock sits raised in the driver's hub. Its stem fits the pad's 9 cm hole only when the pad is within 1 cm of
concentric, so pressing the lock on the off-centre pad does nothing: the stem lands on the pad. The robot has to slide
the pad concentric, then press the lock 41 mm down until its clip catches, then take its hands away.
The scripted demo uses both arms. The left arm slides the pad in from its rim and holds it, and the right arm presses
the lock. One arm can also do it (slide the pad, let go, then press the lock): the pad stays where it was slid.

| File | What it is |
|---|---|
| `task.py` | The task file: scene, scoring (`score`, `success`, kept by a `Trial` tracker) and the scripted `demo()` |
| `mechkit.py` | Mechanism kit (articulated parts from plain-text USD, `fingertips`, step and reset hooks), copied unchanged from another team's open-source task `tasks/heat-shrink-lug/` in this repository |
| `simready.py` | Loads NVIDIA SimReady props, downloading them once from Hugging Face |
| `task.json` | The task's details for the board |

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/reseat-floor-machine-pad/task.py -E scene=station -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Centre the new pad on the pad driver and lock it on with the centre lock." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/reseat-floor-machine-pad
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 420`.

The run prints `task_score` at the end.

In its one 2,400-step run Dream-1 scored 0.0: it brought both grippers down near the pad's near rim and hovered there, but never slid the pad towards centre or pressed the centre lock.

## Runtime and seeds

- Demo: 420 steps (21 s simulated), about 1.5 minutes on an L40S including start-up once shaders are compiled.
- Dream-1: 2,400 steps (2 minutes simulated), about 5 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The demo is open loop, planned from the published
  starting layout. It reads no simulator state.

## Scoring

The code is `Trial.step` in `task.py`, run after every control step; `score()` and `success()` return what it kept.

- **Centred:** the pad's centre within 1.0 cm of the driver's axis. **Flat:** pad tilt at most 3 degrees, its
  underside within 3 mm of the driver's face.
- **Clicked:** the centre lock pushed to within 2.5 mm of fully home (41 mm of travel), where the clip catches and holds
  it. That is possible only through a pad within about 1 cm of centre (stem 7.0 cm, hole 9.0 cm).
- **Hands clear:** every fingertip point of both grippers at least 3 cm from the pad and the centre lock.
- **1.0:** clicked, centred, flat and hands clear, held together for 2 s (40 steps). After that the score stays 1.0.
- **0.5:** at the end, clicked, centred and flat, but full credit was not reached (for example a gripper still on it).
- **0.25:** at the end, centred and flat, not clicked.
- **0:** anything else. Also 0 for the rest of the trial once the pad comes off the driver (underside more than 1 cm
  below the driver's face, or tilted more than 15 degrees). That record is cleared only by the reset hook.

On a real rig all of this is read with a ruler (pad rim to driver rim, all round), a level, and the audible click
plus the lock's flange sitting on the pad.

## Importance and sources

- Janitors and cleaners: 2,172,500 employed in the US, whose duties include "heavy cleaning duties, such as cleaning
  floors" (BLS OEWS, May 2023, SOC 37-2011): https://www.bls.gov/oes/2023/may/oes372011.htm
- The manufacturer's pad installation procedure for NSS burnishers with a centre lock: "Remove the center lock by
  squeezing the wire spring clip. Center the pad on the driver. Position the center lock in the center of the pad then
  push on the center lock until you hear the wire spring click into place." It adds that pads "must be installed
  concentric (centered) with the outside edge of the pad driver", or the result is "excessive machine vibration,
  reduced pad life, and unsatisfactory burnishing results". NSS Charger 1500 / Mustang 1500 / Charger 2500 operator
  manual: https://d2z4qs2e3spnc1.cloudfront.net/product_file/file/1610/nss-charger-1500-high-speed-floor-burnisher-operator-manual.pdf

## Fidelity and simplifications

- The pad driver, pad and centre lock are custom meshes at real size. SimReady has no floor-machine parts. The pad is
  one rigid body, not a compressible nonwoven, and the driver's grip face is plain friction, not hooks or bristles.
- The driver lies level and fixed, as if taken off the machine. On a real machine it is usually changed with the
  machine tipped back, the driver facing up at an angle.
- The real centre lock is removable. Here it is captive on a sprung slider (150 N/m), raised at the start. Its spring
  clip is code: when the lock gets within 2.5 mm of fully home it is latched there (`mechkit.lock`). Releasing the clip
  is not modelled.
- Uniform rubber-like friction. No force sensing.

## Assets

The distractors come from the Hugging Face dataset
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`cleaning_windowSpray_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/cleaning_windowSpray_a01) (spray bottle)
- [`box_glove_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/box_glove_a01) (box of disposable gloves)
- [`cleaning_brush_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/cleaning_brush_a01) (scrub brush)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time:
masses are set (spray bottle 600 g, glove box 250 g, brush 200 g), and the spray bottle's jointed trigger and cap are
welded into one rigid body. Nothing large is committed: the files download at first run.
