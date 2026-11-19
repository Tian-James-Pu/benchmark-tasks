# Press home a kettle lid left resting on its catch

A full 1.7 L cordless kettle stands on its power base. Its hinged lid was dropped shut but not pressed home: it
rests on its spring catch about 5 degrees open, its front edge about 1 cm proud of the rim. The instruction is
only: **Prepare the kettle for use.** The robot has to notice that the lid is not latched and press it down until
it clicks under its catch, without tipping the full kettle or knocking it off its base, then let go.

## Run and score

Use the Python environment containing `yam_arena` and `inspect-robots` (`source ~/IsaacLab-Arena/yam.env`), from
the repository root.

Scripted demonstration (one arm presses the lid's front down until it clicks, lifts off and leaves):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/prepare-kettle-lid-latched/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "Prepare the kettle for use." \
  --scorer task_score --max-steps 400 --store-frames \
  --log-dir ~/yam-eval/prepare-kettle-lid-latched/demo
```

Model trial (Dream-1):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/prepare-kettle-lid-latched/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Prepare the kettle for use." \
  --scorer task_score --max-steps 2400 --store-frames \
  --log-dir ~/yam-eval/prepare-kettle-lid-latched/dream
```

The run prints `task_score`. Render a video with `inspect-robots video <log>.json`. The layout is fixed
(`object_jitter=0`); there is no seed-dependent randomisation. The demo ends after about 200 control steps
(10 simulated seconds), when the 3-second success hold completes; each demo trial took about 3 to 5 minutes of
wall time on a shared L40S-class GPU, including scene load and shader compilation. The model limit is 2,400 steps
(2 minutes at 20 Hz).

## The mechanism

- **Kettle:** a free rigid body of 2.0 kg (about 1.0 kg empty kettle plus 1.0 L of water carried as rigid
  mass, centre of mass low in the body), standing loose on a fixed power base 17.6 cm across and 1.6 cm tall.
  Body, lid, lid-release button and switch are the meshes of the SimReady kettle (25 cm tall, 15.7 cm wide,
  22 cm spout to handle); collisions use convex hulls of the jug, the handle, the lid and the button.
- **Lid:** a gravity-loaded hinge at the handle side (taken from the SimReady file's lid joint). Lowered onto
  the catch it rests on the catch's spring (15 N m/rad, so about 1.2 N m, roughly 11-12 N at the lid's front, is
  needed to push it past the catch). Once it is pressed to within 0.4 degrees of shut, it clicks: a stiff joint
  drive then holds it shut against anything a gripper does, until the lid-release button on top of the handle is
  pressed 4 of its 6 degrees, which lets it spring back up onto the catch. Lifted more than 1 degree clear of the
  catch, the lid swings freely on its hinge; lowered again it rests on the catch.
- **Not modelled:** water sloshing, heating, steam, the auto shut-off itself and the electrical contacts. The
  catch force is an engineering estimate, not a measured value for any one product. Uniform high friction, no
  force sensing.

## Scoring

The scorer runs once per control step (`Trial.update`) and reads the kettle's pose, velocities, lid and button
joints, and the grippers' fingertip points (24 sampled points, both arms).

- **1.0:** the lid is latched (clicked under its catch and within 0.4 degrees of shut); the kettle is centred
  on its base within 1 cm, its bottom within 2 mm of the base's top and upright within 2 degrees; its speed is
  below 5 mm/s and 0.05 rad/s; every fingertip point is more than 3 cm from the kettle (body cylinder of radius
  8 cm from the base's bottom to 26 cm up, plus the handle's box) and so from the base; all continuously for
  3 seconds.
- **0.5:** the lid is latched and the kettle has never tilted over 15 degrees, but the full conditions are not
  met (for example a hand still on the lid, or the kettle off its base).
- **0:** the lid is not latched, including a lid latched and then released again with its button; or the kettle
  tilted more than 15 degrees at any time in the trial (a full kettle starting to pour or fall).

The tilt record is kept from step to step and cleared only by a reset (mechkit's `on_reset` hook), never by a
good final state. Lifting the kettle off its base, opening the lid fully and closing it again are allowed, as
long as it ends latched and seated. A person can check the result by eye and with a ruler: the lid flush and
locked (it does not lift when its edge is pulled gently), the kettle seated on its base and nothing touching it.

**Real trial.** Any 1.7 L cordless kettle with a hinged, click-to-latch lid and a release button works. Tape the
power base to the counter, fill the kettle with 1.0 L of cold water (unplugged; no heating is needed), press the
release button and lower the lid gently onto its catch so it rests about 1 cm proud at the front without clicking.
Measure the kettle's centring on its base with a ruler, check the lid by pulling its front edge up gently, and
judge tilt over 15 degrees from the video.

## Validation

| Trial | What it does | Score |
|---|---|---:|
| demo 1 | press the lid's front until it clicks, lift off, leave | 1.0 |
| demo 2 | same, fresh run | 1.0 |
| nothing | the arm waits | 0.0 |
| hold | clicks the lid shut and keeps pressing on it | 0.5 |
| shove | careless: knocks the kettle body sideways with the hand, then presses the lid | 0.0 |

Both demos ran on the final source and each ended at control step 201 (the lid clicked at about step 100, the
hand was clear by step 140 and the 3-second hold then completed). The kettle tilted at most 1.9 degrees on its
base during the press. Doing nothing leaves the lid on its catch (0). Clicking the lid shut and keeping the
hand on it earns the partial credit (0.5) and never the full score. Knocking the kettle tipped it over (85
degrees), recorded as harm, and the lid was never latched (0). The red-team files were copies of `task.py` with
a different `demo()` and are not part of the submission. Contact sheets of every run were inspected.

Dream-1 scored 0.0 in its one 2,400-step run: both arms moved forward and hovered beside the kettle
but never pressed the lid, so it stayed resting on its catch.

## Importance and sources

Kettle manuals tie the lid's latch to both hazards. The
[Hamilton Beach 1.7 L kettle use and care guide](https://useandcares.hamiltonbeach.com/files/840265900.pdf) shows
the lid and its lid-opening button and warns that scalding may occur if the lid is lifted during the heating
cycle, and that the kettle switches itself off when the water has boiled. The
[Gastroback 42442 kettle manual](https://www.manualslib.de/manual/664040/Gastroback-42442.html?page=35) says to
press the lid onto the kettle until it engages with a click, and lists a lid that is not closed correctly as a
cause of the kettle not switching off. Consumer NZ's
[survey of faulty kettles](https://www.consumer.org.nz/articles/faulty-kettles-what-can-you-do) found that in 24%
of faulty kettles the lid would not stay closed or popped open during boiling, and that lid problems led to
kettles that would not shut off (15%). The recovery is a hazard nobody names: the robot must see that the lid
sits proud and fix it with a firm, controlled press, without tipping a 2 kg kettle full of water.

## Assets and helpers

The kettle and the distractors are NVIDIA SimReady props, downloaded at first run from
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`. © NVIDIA, CC-BY 4.0. Nothing large is committed.

- [`kitchenware_kettle_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/kitchenware_kettle_a01):
  its meshes are read from the USD at first run (in a separate process, cached as an `.npz`) and rebuilt as a
  mechanism: body and switch welded into one free 2.0 kg body, lid and release button on hinges at the file's
  own joint positions, convex-hull colliders.
- [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01) (distractor): mass set to 0.35 kg.
- [`food_box_sugar_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/food_box_sugar_a01) (distractor): mass set to 0.50 kg.
- [`cleaning_sponge_b01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/cleaning_sponge_b01) (distractor): mass set to 0.03 kg.

The power base (plate, contact disc and cord) is a primitive structural part. The distractors are scored by
nothing. `simready.py` is the open-source loader from `tasks/cutters-off-pegboard`; `mechkit.py` derives from the
shared mechanism kit of `tasks/heat-shrink-lug`, with joint-drive helpers (`hold_at`, `slacken`) and a frame
centre of mass added.
