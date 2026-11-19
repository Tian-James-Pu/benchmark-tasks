# Shut off a pre-rinse spray valve left locked on

The assembly is full-size and uses a separate free ring retained by ordinary physical contacts.

The hose-suspended valve is already running when the episode starts. Its plain brass hold-down ring retains the spring-return trigger. The goal is to shut it off and leave it hanging over the sink; any successful arm strategy is valid.

Food preparation workers regularly clean equipment, utensils and dishes; the [BLS occupational profile](https://www.bls.gov/ooh/food-preparation-and-serving/food-preparation-workers.htm) reports 908,500 US jobs in 2025. Commercial [T&S pre-rinse valves](https://www.tsbrass.com/products/parts-accessories/spray-valves/B-0107-J-SWV) use this trigger/ring mechanism.

Hardware is reconstructed from the manufacturer’s [B-0107-J drawing](https://www.cooneybrothers.com/userfiles/ad/documents/T_S_B-0107-J_Line_Drawing.pdf), [complete grip/swivel drawing](https://www.webstaurantstore.com/documents/specsheets/510b0107jswv_specsheet.pdf), and [000907-45 ring drawing](https://www.webstaurantstore.com/documents/specsheets/t_s_000907-45_spec_sheet.pdf). Explicit dimensions are 274 mm assembly length, 38 mm nozzle, 231 mm bare-valve trigger reach, and a 51 mm OD / 38 mm ID / 6 mm thick ring. The curved trigger pivots near the nozzle and extends toward the hose; no gripping tab is added. Undimensioned contours and offsets are estimates, not manufacturer CAD. The conservative 1.18 kg assembly mass follows published product weight, whose packaging contribution is unspecified. Individual masses are approximations; the 12 g ring follows its brass volume.

## Run and score

Requires the `yam_arena` embodiment from [yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1, and Inspect Robots. From the repository root:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/shut-off-locked-pre-rinse-sprayer/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "The pre-rinse sprayer has been left on. Turn it off and leave it hanging over the sink." \
  --scorer task_score --max-steps 1150 --store-frames --log-dir /tmp/pre-rinse-demo
```

For Dream-1 use `--policy dream --max-steps 2400`, camera width 320 and height 176, and a distinct log directory. `inspect-robots video <trial.json>` renders recorded frames. The trial prints `task_score`. The layout is fixed (`object_jitter=0`), with no random seed or procedural variation.

The model receives 2,400 control steps, or 120 simulated seconds. The contact-based demonstration has reached full score in 849 steps (42.45 simulated seconds). It includes waiting for the freely suspended head to settle. Two fresh runs reproduced this result. Dream-1 reached score 0.0 over its full 2,400-step budget. Wall runtime is approximately 4–6 minutes on an available GPU; competing GPU jobs can increase it substantially.

## Mechanism and scoring

The spring trigger has 3 Nm/rad stiffness and 3 degrees preload. These are engineering approximations. The ring is a free rigid body with a segmented annular collider; ordinary collisions with grip and trigger retain it. There is no scripted lock, attached gripper, axial ring guide or programmed ring motion. A 500 mm hose is approximated by two damped links. The ring can move or tilt in any physically available direction.

Scoring is read-only. A once-per-control-step callback updates permanent spray-exposure history and consecutive hold counters. Full score requires the trigger within 1 degree of off, all sampled gripper points 3 cm clear of valve/lower hose/ring, nozzle speed below 3 cm/s, and nozzle within 8 cm of its nominal hanging position, continuously for 3 seconds. Partial 0.4 requires shut/released and 2 seconds of low nozzle speed. Other outcomes score 0. Any spray violation permanently forces 0 until an actual episode reset; restoring object poses does not clear it. The scoring text specifies the geometric spray tests.

The translucent stream represents rinse water; no fluid forces or heat transfer are simulated. Use cold water for a physical reproduction. Faucet controls are outside arm reach. Reset restores free-body and joint poses and clears history only through episode reset.

## Assets and credits

The sink, faucet, hose and working valve are custom structural/mechanism meshes and colliders. No suitable articulated pre-rinse assembly is supplied in the prop collection. The sponge, plate and mug use NVIDIA SimReady meshes with their measured proportions and simple collision geometry. `simready.py` downloads the named assets from `nvidia/simready-assets` at revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`; these assets are copyright NVIDIA and licensed CC BY 4.0. Internet/cache access is required on first use.

`mechkit.py` derives from the mechanism helper distributed with `heat-shrink-lug`; `simready.py` derives from the loader distributed with `cutters-off-pegboard`. Both helpers are included so the folder runs independently.

## Validation

Two fresh contact-based demonstrations each reached score 1.0 in 849 steps. It pinches the ordinary ring before unloading the trigger, raises the ring past the trigger tip, releases both hands and waits for the head to settle. The scorer requires the actual released state throughout its three-second full-score dwell. Physical adversarial trials scored 0 for doing nothing (400 steps), squeezing only (900 steps), holding the ring after shutting the trigger (1,000 steps), and manipulating only the ring (1,150 steps). Videos were visually checked. Dream-1 scored 0.0 in a completed 2,400-step evaluation of this geometry. The model run took approximately 15 minutes of wall time, including API latency. The API retried expired requests during this run; the reported score and model video describe that recorded run.

Synthetic observer replays pass the dwell boundary, repeated read-only score queries, held-state rejection, history persistence after restoring initial poses, actual episode reset, partial-only off-position outcome and flowing-state rejection. These are scorer checks, not substitutes for physical trials.

The recognisable distractors come from these pinned packages (© NVIDIA, CC-BY 4.0):

- [cleaning_sponge_c01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/cleaning_sponge_c01)
- [dishware_plastic_bigPlate_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_plastic_bigPlate_a01)
- [dishware_mug_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01)

The loader applies the stated poses and masses, welds asset joints if present, and supplies simple collision geometry. Asset meshes are downloaded at runtime, not redistributed in this folder.
