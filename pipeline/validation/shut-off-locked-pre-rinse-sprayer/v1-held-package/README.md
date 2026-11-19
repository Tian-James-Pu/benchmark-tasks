# Shut off a pre-rinse spray valve left locked on

**Prototype held for hardware redesign.** The cited manufacturer drawing does not match this prototype's lever orientation or added thumb tab. The successful simulation does not establish fidelity to a commercial valve; this folder is not ready for submission.

The hose-suspended valve is already running when the episode starts: its steel hold-down ring retains the spring-loaded squeeze handle. The goal is to shut it off and leave it hanging over the sink. The demonstration steadies and squeezes the valve with one gripper, slides the unloaded ring with the other, then releases both.

Food preparation workers clean equipment, utensils and dishes as part of their regular work; the [BLS occupational profile](https://www.bls.gov/ooh/food-preparation-and-serving/food-preparation-workers.htm) reports 908,500 jobs in 2025. Commercial [T&S spray valves](https://www.tsbrass.com/index.php/products/parts-accessories/spray-valves) use a spring-loaded handle and hold-down ring. This is a representative mechanism, not a dimensionally exact model of a named product.

## Run and score

Requires the `yam_arena` embodiment from [yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1, and Inspect Robots. From the repository root:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/shut-off-locked-pre-rinse-sprayer/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "The pre-rinse sprayer has been left on. Turn it off and leave it hanging over the sink." \
  --scorer task_score --max-steps 800 --store-frames --log-dir /tmp/pre-rinse-demo
```

For Dream-1 use `--policy dream --max-steps 2400`, camera width 320 and height 176, and a distinct log directory. `inspect-robots video <trial.json>` renders recorded frames. The trial prints `task_score`. The layout is fixed (`object_jitter=0`), with no random seed or procedural variation.

The demo has a 31-second waypoint plan (620 control steps); trials may stop earlier after full success. The model receives 2,400 steps, or 120 simulated seconds. The two fresh demos each finished in 448 control steps (22.4 simulated seconds); the second took 299 seconds wall time under shared GPU load, including startup. Wall time depends on GPU load and shader compilation.

## Mechanism and scoring

A 16 cm valve hangs from a 50 cm hose approximated by two damped articulated links. The handle spring is 2.16 N m/rad with a 5-degree preload. The 15 g sliding ring has 1 N friction. These are disclosed engineering approximations. Ring/handle engagement uses position-dependent joint limits: squeezing unloads the ring; once it slides below the handle tip the spring can return the handle to off. Robot contacts move the handle and ring; the demonstration does not set their positions.

Scoring is read-only. One callback per control step updates the exposure latch and consecutive hold counters; repeated score/success calls do not advance time. Full score requires the shut valve and displaced ring, grippers at least 3 cm clear, nozzle speed below 3 cm/s, and nozzle within 8 cm of its initial rest position, all continuously for 3 seconds. Partial credit is 0.4 for a shut and released valve with speed below 3 cm/s for 2 seconds but without the complete final-position/hold condition. All other outcomes score 0. Any earlier spray exposure or spray directed out of the basin permanently forces 0 for that episode. `task.json` specifies the geometric tests and thresholds.

The translucent stream is a geometric proxy for hot rinse water. It has no fluid forces, heat transfer or damage model. A physical reproduction uses cold water and the same no-contact flow criterion. No hot-water exposure is necessary. The fixed faucet is outside the work envelope; the manipulated control is the spray head's own handle and ring. Reset restores the joints, lock, props, initial swing and all history.

## Assets and credits

The sink, faucet, hose and working valve are custom structural/mechanism meshes and colliders. No suitable articulated pre-rinse assembly is supplied in the prop collection. The sponge, plate and mug use NVIDIA SimReady meshes with their measured proportions and simple collision geometry. `simready.py` downloads the named assets from `nvidia/simready-assets` at revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`; these assets are copyright NVIDIA and licensed CC BY 4.0. Internet/cache access is required on first use.

`mechkit.py` derives from the mechanism helper distributed with `heat-shrink-lug`; `simready.py` derives from the loader distributed with `cutters-off-pegboard`. Both helpers are included so the folder runs independently.

## Validation

Two fresh demo trials scored **1.0**. Idle, ring-only, squeeze-only, incomplete ring travel, keeping the handle squeezed, shutting off but leaving a hand within 3 cm, and crossing the spray before an otherwise successful shutdown all scored **0.0**. Every contact sheet was visually reviewed. The near-hand variant reached the off stop but correctly failed release; the exposure variant reached the correct final state but retained its earlier failure. CPU checks verify pure scoring, the 59/60-step hold boundary, exposure persistence, and reset state. The Dream-1 score remains pending until the model trial finishes.
