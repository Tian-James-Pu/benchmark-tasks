# Open a 15 oz can on a lever electric can opener

**Instruction:** *Open the can with the electric can opener and set the open can down on the counter, leaving the lid on the magnet.*

A countertop lever-type electric can opener stands on the kitchen counter facing the arms, its piercing lever raised. A sealed
15 oz (No. 300) steel food can stands on the counter in front of it. The robot has to pick the can up, raise it until the bead of its
top seam sits on the opener's geared wheel under the cutting wheel, and hold it there while the other hand presses the lever down until
the cutter pierces the lid and the lever latches. The motor then turns the can until the lid is cut free and stops. The robot takes the
can again, lifts the lever (the lid comes up on the magnet), draws the can off the wheel and stands it, open, on the counter.

## Importance and sources

Opening a can on an electric opener is an ordinary kitchen job that most adults have done, and it is recognisable in a 10-second
clip. The task follows the maker's own instructions for a current, purchasable opener:

- **The product.** Hamilton Beach 76702 *Extra-Tall Can Opener with Removable Cutting Lever*: 9 in tall, 4.6 in wide, 5.7 in deep;
  "Automatic shutoff to free your hands"; extra-tall design for small to large cans. About $25 to $42 at retail (the maker lists a
  76702G variant at $24.99; Walmart lists the 76702 at about $42).
  https://hamiltonbeach.com/extra-tall-can-opener-with-removable-cutting-lever-black-76702
- **The maker's Use & Care guide** for this opener (model 76700 family, the guide linked from the 76702 page),
  https://useandcares.hamiltonbeach.com/files/840200101.pdf . Its parts drawing names the *Removable Piercing Lever*, *Magnetic Lid
  Holder*, *Can Guide Pin* and *Geared Wheel*. Its "How to Use" pictures show: lift the piercing lever (2); raise the can so its rim
  sits on the geared wheel (3); press the lever down (4); and (5) one hand holding the opened can while the other lifts the lever, the lid
  held on the magnet. Its notes: "The motor will automatically stop when can is open." and "It is not necessary to hold lever down or
  hold can while can is being opened."
- **The can.** The No. 300 can is 3 in by 4 7/16 in (15.22 US fl oz; cranberry sauce, pork and beans), in the standard can-size table
  that uses the Can Manufacturers Institute's three-digit sizes: https://en.wikipedia.org/wiki/Tin_can . Three-piece food cans have a
  double-seam rim at each end, slightly wider than the body.

**The knack.** The can's top seam stands only 1.9 mm proud of its wall. It has to be raised into a gap about 1 cm high between the
geared wheel (below) and the cutting wheel (above), pushed back until its wall meets the wheel, and set down so the bead rests on the
wheel, and held there while the other hand presses the lever about 25 degrees with roughly 12 to 14 N. Held 3 mm too high, the cutter
lands on the bead and the lever stops short; held in front of the wheel, the cutter meets nothing. Once pierced, the hand on the can has
to let go: the motor turns the can, and a can still gripped stalls it. When the motor stops, the can hangs only from the cutter's
clamp, so it has to be held again before the lever is lifted, or it slides off the wheel and falls (the guide's picture 5 shows exactly
this two-handed release). One hand cannot seat the can and press the lever at once.

The numbers we could not source (the lever's force, the clamp's detent, the motor's speed and torque) are stated as estimates in
*How the simulator models it* and in `fidelity`.

**Distinct from** the nearest tasks we know of:

| Nearest | What it tests | Difference here |
|---|---|---|
| Open a can with a hand-held rotary opener (PR #1851, accepted) | one hand keeps the opener's handles squeezed on the rim while the other turns its key; the can stands on the table | a powered countertop opener: the can is lifted 4 cm off the counter and its bead seated on a fixed geared wheel under the cutter, the lever is pressed by the other hand, the hand on the can must let go while the motor turns it, and must catch it again before the lever is lifted |
| Bench can opener (PR #1732, closed) | a lever held down while the other hand cranks | no cranking here; the work is seating the rim, the hand-off to the motor, and the two-handed release |
| Open a crown-capped bottle on a mounted bottle opener (PR #1146, accepted) | levering a cap off on a fixed opener | a can hung by its rim in a powered opener; nothing is levered |

No title on the board mentions an electric can opener (checked against the board list of about 3,500 PRs on 21 November 2026, searching
"opener", "can", "lid" and "magnet"). LIBERO, RoboCasa, BEHAVIOR and SimplerEnv have no powered can opener task that we know of.

## Run and score

From the repository root, in the Python environment that has `yam_arena` and `inspect-robots`:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/electric-can-opener-rim-seat/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the can with the electric can opener and set the open can down on the counter, leaving the lid on the magnet." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/electric-can-opener-rim-seat
```

The scripted demonstration is closed loop (it reads the can's pose, the lever's angle and the can's turn, all visible, never the
scorer), so it runs through `demokit.py` from the task folder:

```bash
cd tasks/electric-can-opener-rim-seat
python demokit.py task.py ~/yam-eval/electric-can-opener-rim-seat/demo --mode full          # writes full/demo.mp4, setup.png, result.json
python demokit.py task.py ~/yam-eval/electric-can-opener-rim-seat/redteam --no-record --mode none,onearm,careless,clamp,high
```

`result.json` holds `task_score`. The layout is fixed (`object_jitter=0`); there are no seeds or random choices.
The scripted demo takes about 900 control steps (45 simulated seconds); the run limit is 2,400 steps. A CPU `--fast` run (no cameras) takes about 1.5 minutes of wall time.

## How the simulator models it

Everything is a PhysX rigid body; every state change follows from contact and joint angles, never from a gripper being near
something.

- **Opener body** (`housing`, fixed): 14.5 x 11.7 x 22.9 cm. Below the geared wheel its face stands 5.8 mm forward, so the bottom seam
  of a can hanging in the jaws leans on it, as a can leans on a real opener's body; the chrome band that carries the wheel is set
  back, and the upper front is set back so the lever folds onto it. Glossy plastic: friction 0.05 (an estimate).
- **Piercing lever** (`opener`, a fixed articulation: two cheeks carrying the pivot 16.6 cm above the counter, and one link): a hinge
  about the opener's width; 0 is raised, 25 degrees is pressed home on a seated can's lid, 31 degrees its stop with no can. It carries
  the **cutting wheel** (an 18 mm steel disc, a real collider), the magnet and, hidden until the cut closes, the lid it will hold. A
  return spring holds it raised: about 4 N at the grip raised and 10 N pressed home (an estimate; the maker publishes no force).
- **Geared wheel** (`gear`, fixed): 20 mm across, standing 8 mm out of the band, its top 150 mm above the counter. Its teeth are not
  modelled: it is smooth and slippery (friction 0.1), so a can rests on it only while the cutter clamps the rim, as in the guide's
  picture 5, where the can is held while the lever is lifted.
- **Can** (`can`, a free articulation, 0.47 kg): 76.2 mm over the seams, 72.4 mm body, 112.7 mm tall, lid 3.2 mm down in its
  countersink. Its top (the seam bead and the lid, 40 g) is the articulation's frame; its body (wall, bottom seam, label; 0.43 kg) turns
  on a joint about the can's axis, locked until the lid is pierced.
- **Pierced**: the lever within 3 degrees of home while the cutter's edge is on the lid inside the seam (within 1.5 mm of the lid's
  surface) and the bead's underside is on the geared wheel (from 2 mm below to 1 mm above the wheel's top, the wheel's front edge at
  least 0.3 mm under the bead), the can within 8 degrees of upright. The latch then presses the cutter onto the lid with up to 18 N
  (0.5 N m on the lever), so the can hangs in the jaws.
- **Cutting**: a speed-controlled motor turns the body at 45 degrees a second (one turn in 8 s) with at most 0.3 N m (both
  estimates). On the real opener the whole can turns past a fixed cutter; here the top ring stays clamped and the body turns under it,
  which is the same for an axisymmetric seam. The cut is the body's turn **as seen from the opener** since piercing, so a hand that
  grips the can stops the cut (the motor stalls), and while the rim is out of the jaws the motor stops. The cut line is drawn on the lid
  as it is made.
- **Cut through** (360 degrees): the lid is shown on the magnet and the can open, the motor stops, and the latch keeps pressing until
  the lever is lifted 6 degrees; then the spring raises it. Lifted with nothing holding the can, the can slides off the slippery wheel and
  falls. Metal cutting itself is not simulated.

## Scoring

The tracker reads the state every control step; `score()` returns its result. `task.json` `scoring` is this text:

- **0.0** if, at any time after the lid was pierced, the can tilts more than 20 degrees from upright, or falls (its base comes down
  3 cm within 0.1 s: falling free out of the opener it covers its 4 cm in 0.09 s, while a hand lowers it far more slowly). A pierced or
  opened can spills: a 15 oz can is filled to about 1 cm below the lid, and its contents reach the rim at about 16 degrees. Cleared only
  at a reset.
- **1.0**: lid cut through (360 degrees) and held on the magnet with the lever raised (within 10 degrees of its stop), the can upright
  (within 10 degrees) with its base on the counter (within 3 mm), nothing moving, every fingertip at least 5 cm from the can, all held
  for 1 s.
- **0.75**: lid cut through and on the magnet, without the rest of 1.0 (for example the can still hanging in the opener, or held).
- **0.5**: the cut at least 270 degrees round the lid, not through.
- **0.25**: the lid pierced (lever pressed home with the bead on the geared wheel under the cutter), cut under 270 degrees (for
  example the motor stalled by a gripping hand).
- **0.0** if the lid was never pierced.

Possible scores: 0, 0.25, 0.5, 0.75, 1.0. Every condition is read at the end state or from the video: the cut's extent with a
protractor ring on the lid, tilt and base height with a square and a ruler, the lever's angle against its stop, the hands' clearance and
any fall in the video frames.

## Real trial: preparation, reset and measurement

- **Kit:** a Hamilton Beach 76702 (or the same lever design) on the counter, its front face on a tape mark facing the arms; a case of
  identical 15 oz (No. 300) cans of one product (beans or tomatoes, about $1 each), one per trial.
- **Preparation:** stand a fresh can on its tape mark (27 cm forward of the arm bases, 13 cm right of the centre line). Draw a
  protractor ring (10-degree ticks) on a paper disc stuck on the lid, so the cut's extent can be read from a photo if the trial stops
  part-way. Raise the lever.
- **Rig protection:** the cut lid's edge and the can's cut rim are sharp. The lid stays on the magnet; the robot never touches the cut
  edge in the demo route. Fit cut-resistant sleeves over the finger pads if trials are run with a model. A tray under the opener catches
  juice if a can is dropped.
- **Measurement at the end:** a top photo (the lid on the magnet, the lever raised); tilt with a square against the can; base height
  with a ruler (or a feeler under the base); the hands' clearance from the last video frames. Tilt past 20 degrees after piercing is
  read from the video (the contents spill).
- **Reset (about 2 minutes):** take the lid off the magnet and discard it, empty the opened can, wipe the cutter and wheel, raise the
  lever, and stand a fresh can on the mark.

## Assets and licences

The opener and the can are authored meshes: SimReady has no electric can opener, and its food cans are single solid bodies, without
the separate seam, lid and turning body that this mechanism needs. The three distractors are NVIDIA SimReady props, downloaded at first
run from the pinned revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd` of `nvidia/simready-assets` on Hugging Face, © NVIDIA,
CC-BY 4.0:

- [dishware_bowl_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_bowl_a01): mass set to 0.35 kg.
- [dishware_mug_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01): mass set to 0.30 kg.
- [cleaning_cloth_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/cleaning_cloth_a01): mass set to 0.05 kg.

Nothing large is committed. `simready.py` is the loader from `tasks/cutters-off-pegboard/`, `mechkit.py` the mechanism kit from
`tasks/heat-shrink-lug/` and `demokit.py` the closed-loop demo runner from `tasks/heat-shrink-lug/`: other teams' open-source files in
this repository, used unchanged.

## Validation

**Not yet validated: work in progress.** All runs so far are CPU-physics `--fast` runs (no cameras); no GPU-rendered demo,
screenshot or video exists yet.

| Run (CPU, `--fast`) | Score | What happened |
|---|---:|---|
| demo `full`, three fresh sessions on the previous revision (wall servoed 0.8 mm short of the wheel) | 1.0 | seated, pierced at 25 degrees, cut 360 degrees in about 8 s hanging hands-free, re-gripped, lever lifted, can stood on the counter |
| demo `full`, current file, first mode of a session | 1.0 | the same |
| demo `full`, current file, fifth mode of the same session | 0.0 | the re-gripped can slid 2.4 cm in the hand and leaned 15 degrees when the lever was lifted, and fell over when set down |
| `none` | 0.0 | nothing done |
| `onearm` | 0.0 | the can, let go on the wheel, slid off the slippery wheel before the lever could be pressed; never pierced |
| `careless` | 0.0 | cut through, then the lever was lifted with nothing holding the can: it fell out of the opener (fall rule) |
| `clamp` | 0.0 | the hand kept hold of the can: the motor stalled (cut 2 to 5 degrees in 20 s, 0.25 while stalled); the script then carried the pierced can on and tipped it past 20 degrees |
| `high` | 0.0 | the bead held 5 mm high: the cutter pushed the can down onto the wheel and pierced anyway; the can later fell when the lever was lifted |

Open issues: the can hangs about 11 degrees off upright while it turns (friction where its bottom seam leans on the body), which makes
the re-grip and the hand-off at the lever unreliable; the `high` red team does not test what it was meant to.
