# Prepare an infant car seat left on its base latched on one side only

An empty infant carrier sits on its car-seat base. It looks seated, but only the left end of its locking bar is
locked: the right end rests on top of the right lock's pawl, about 1.5 cm high, and the carrier is rolled 2.7
degrees. The instruction is only: **Prepare the infant car seat for use.** The robot has to notice that one side
is not locked and push that corner down until the pawl clicks over the bar, then let go and leave the carrier
locked on both sides.

## Run and score

Use the Python environment that has `yam_arena` and `inspect-robots`. From the repository root:

```bash
# scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/prepare-infant-carrier-on-base/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "Prepare the infant car seat for use." \
  --scorer task_score --max-steps 400 --store-frames \
  --log-dir ~/yam-eval/prepare-infant-carrier-on-base/demo

# model trial
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/prepare-infant-carrier-on-base/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Prepare the infant car seat for use." \
  --scorer task_score --max-steps 2400 --store-frames \
  --log-dir ~/yam-eval/prepare-infant-carrier-on-base/dream
```

The trial prints `task_score`. The layout is fixed (`object_jitter=0`) and nothing depends on the seed. The demo
is a fixed open-loop plan from the published starting layout (it reads no simulator or scorer state). It ends with
the full score after about 11 simulated seconds (about 230 control steps); 400 steps is 20 simulated seconds. The
model limit is 2,400 steps (2 minutes at 20 Hz). A trial takes about 1 to 2 minutes of wall time once the GPU's
shaders are compiled.

## The mechanism

The base has a lock at each end of the carrier's locking bar, 31 cm apart. Each lock is a pocket (floor, a guide
wall on the arm side, a rear block) with a sprung pawl above it on a slider joint. The pawl's face toward the bar is
a 60-degree ramp; its underside is flat, 2 mm above the seated bar. Pushing the bar down the ramp cams the pawl
back (16 mm travel); once the bar is past, the spring snaps the pawl forward over the bar. Lifting the bar then
presses on the flat underside, which the slider's spring does not oppose, so a locked side holds. Each pawl spring
is 800 N/m with 8 N of preload against its stop, so the bar goes in with roughly 20 N pushed down at the corner,
and the 3.45 kg carrier's own weight does not push it in. These forces are estimates, not measurements of a product.

During the 0.5 s settle before the first frame, the pawl springs are held stiff so the carrier comes to rest on
the right ramp; at reset (mechkit's `on_reset` hook) they are switched to the 800 N/m, 8 N preload spring used for
the whole trial. The first frame shows the carrier at rest with the right pawl pushed back 4.8 mm by the bar.

The carrier is rebuilt from primitives to the size and weight of a 7.6 lb (3.45 kg) infant carrier: a 70 cm long plastic
shell with walls, a reclined fabric seat pad, a harness and buckle, and an upright carry handle. Two runners under
its head end rest on pads on the base. The base is fixed to the table, which stands in for the vehicle seat.

## Scoring

`Trial.step()` runs once per control step (mechkit `on_step`); `score()` and `success()` only read it.

- **1.0:** both sides latched (each bar end within 3 mm of seated height and within 8 mm of its lock centre front to back
  horizontally, and its pawl within 1.5 mm of fully forward), all four rear runner points within 3 mm of their
  pads, carrier tilt at most 1 degree, carrier speed under 5 mm/s and spin under 0.05 rad/s, and every sampled
  fingertip point (mechkit `fingertips`) more than 3 cm outside the carrier's bounding box, all continuously for
  3 seconds.
- **0.5:** both sides latched at the end, but the full condition not met (for example, still pressing on it).
- **0:** anything else, including the starting state.
- **Harm:** a carrier that ever tilts more than 25 degrees, or either bar end drops more than 5 cm below its
  seated height (off the base), scores 0 for the rest of the trial. This record is cleared only at reset
  (mechkit `on_reset`), never because the carrier looks right again.

Lifting the carrier off and setting it back down locked is allowed; the scorer only asks for the final state.
## Real-world trial

- **Preparation:** with the base installed (or held flat on a table), set the empty carrier down with the left end
  of its bar clicked into the left lock and the right end resting on the right lock without clicking. Check it: the
  right front corner sits about 1.5 cm higher than the left (a ruler from the table top to the corner lip), and
  pulling it up lifts it.
- **Measurement after the trial:** a side counts as latched when its front corner is within 3 mm of the other
  corner's height (ruler) and does not rise when pulled straight up by hand with about 50 N (a luggage scale), as the
  manual's own check says. The carrier is flat when a phone inclinometer on the shell's rim reads within 1 degree of
  its reading on a correctly latched carrier and a 3 mm feeler gauge does not fit between either rear runner and its
  pad. Hands-off and stillness for 3 seconds are read from the video. The pawl position used in simulation is hidden
  inside a real base; the pull test stands in for it.
- **Reset:** squeeze the release handle, lift the carrier off and set it back as in Preparation. Nothing is consumed.

## Validation

| Run | File | Score |
|---|---|---:|
| demo 1 | `task.py` | 1.0 (226 steps) |
| demo 2 | `task.py` | 1.0 (226 steps) |
| do nothing | red-team variant | 0.0 |
| press and never let go | red-team variant | 0.5 |
| press the corner that is already locked | red-team variant | 0.0 |

The red-team variants were copies of `task.py` with a different `demo()`, deleted after the runs. In its one 2,400-step run, Dream-1 scored 0.0: it pulled both arms back in front
of the carrier and held them there without touching it, so the right side stayed unlatched.

## Importance and sources

- The [Graco SnugRide 35 Lite LX owner's manual](https://www.gracobaby.ca/on/demandware.static/-/Sites-gracoca-master/en_CA/dw5f1dde52/ProductManualPdf/NWL0001004345B%20SnugRide35%20Lite%20LX%208BS04%20ENG.pdf)
  (section 2-C, "Attaching Car Seat to Base", p. 18) describes this mechanism: align the locking bar at the front
  of the car seat with the locks in the base, push down on the front of the car seat until you hear it click, then
  pull up on the front corners to make sure it is locked. Its checklist ("Did You Do Everything Correctly?") lists
  "Car Seat Is Properly Attached To Base".
- The carrier's weight is the carrier-only weight measured in [BabyGearLab's review of the SnugRide Lite LX](https://www.babygearlab.com/reviews/vehicle-safety/infant-car-seat/graco-snugride-lite-lx),
  7.6 lb. Its 70 x 38 x 47 cm envelope follows the overall dimensions listed for the product,
  27.4 x 15.5 x 18.07 in, on its [Target listing](https://www.target.com/p/-/A-79267447) (a Graco answer on the
  same page gives the width as 14.41 in).
- NHTSA's [National Child Restraint Use Special Study](https://crashstats.nhtsa.dot.gov/Api/Public/ViewPublication/812157)
  (DOT HS 812 157, 2015) estimated that 46 percent of car seats and boosters were misused, 49 percent of rear-facing
  infant seats. That figure covers all kinds of misuse, not carriers left half-latched in particular.

The simulated carrier and base are a reconstruction of this kind of carrier and base, not a replica of the Graco
product, and no product force was measured.

## Assets and helpers

The carrier and base are authored structural and mechanical parts (`mechkit.py`). The two distractors are from
the Hugging Face dataset [`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets),
pinned to revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, and are downloaded at first run:

- [`food_beverage_bottledWater_b01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/food_beverage_bottledWater_b01): mass set to 0.52 kg.
- [`food_beverage_coffeeCup_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/food_beverage_coffeeCup_a01): mass set to 0.25 kg.

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes at load time: masses
set as above, joints (if any) welded into one rigid body. Nothing from the dataset is committed here.

`simready.py` is the open-source loader from `tasks/cutters-off-pegboard`. `mechkit.py` is the shared mechanism kit
from `tasks/heat-shrink-lug`, unchanged.
