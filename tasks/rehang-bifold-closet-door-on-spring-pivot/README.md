# Rehang a bifold closet door that popped off its floor bracket

A full-size two-panel bifold closet door hangs from its spring-loaded top pivot and its guide in the track, but its bottom
pivot has popped out of the jamb bracket on the floor. The door stands folded open in front of the left jamb, its bottom pin
resting on the floor 5.5 cm toward the room and 1.3 cm along the opening from the bracket's centre hole. The instruction
is only: **The bifold closet door has popped off its floor bracket. Put it back so it hangs plumb.**

The knack: the pin cannot be slid back, because the bracket's toothed rails stand 5 mm proud of the floor. The whole door
(two 79 in panels) has to be lifted 5 to 10 mm, compressing its top pivot spring until the door's top nearly meets the
track, carried back about 5.5 cm with its bottom pin 2 m below the hands that lift it, set over a 10 mm slot under the
door where neither the camera nor the hands can see it well, and lowered so the pin drops in at the spot directly under the
top pivot. Dropped straight back, the pin lands 13 mm off that spot and the door hangs out of plumb. Once the wheel is down,
its teeth lock it in place along the slot, so a misplaced pin can only be corrected by lifting the door again.

## Files

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: the closet, door, track and bracket, the pivot spring and the bracket's teeth, the scorer (`score`, `success`) and the closed-loop `demo_script` |
| `task.json` | The task's details |
| `demokit.py` | The closed-loop demo runner, unchanged |
| `mechkit.py` | The mechanism kit (articulations from plain-text USD), unchanged |
| `simready.py` | Loads NVIDIA SimReady props, downloading them once from Hugging Face |

## Run and score a trial

From this folder, in the environment that has `yam_arena` and `inspect-robots`:

```bash
# a model trial (Dream-1 or another policy), 1,200 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The bifold closet door has popped off its floor bracket. Put it back so it hangs plumb." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/bifold-model
```

`task_score` is 1.0 when `success()` comes true (the tracker counts its own 2 s hold, `SUCCESS_HOLD_S = 0`), and otherwise
the last step's `score()`. Nothing is randomised: every seed gives the same scene (`object_jitter=0`).

## The demo

```bash
python demokit.py task.py OUT                                                                # the full-score run, with video (GPU)
python demokit.py task.py OUT_checks --no-record --mode none,drag,straight_back,hold,drop_early,onearm   # the checks
python demokit.py task.py OUT_fast --fast --mode full                                         # physics on the CPU, no cameras
```

`OUT/full/result.json` holds the harness's `task_score`; `OUT/full/demo.mp4` and `setup.png` are the video and screenshot.
The demo is a scripted reference, not a policy. It reads only what a camera-based policy could estimate (the door's pose and
its visible joints) and its own arms' joints, never the scorer's state. Both hands, level, pinch the two folded panels
together at their hinged edge, the left 14 cm and the right 34 cm above the door's bottom, as a person takes a door at its
edge with both hands. Steering by the bottom pin's position, both hands lift until the pin is 8.5 mm up (the door's top is
then close under the track), carry it back over the slot at the mark and lower it in; they then press down a few
millimetres so that the bracket rather than the arms carries the door, let go, and back away along the panels. On the GPU
a run reaches full credit (success, after the 2 s hold) at 21.0 simulated seconds (420 steps); `run_limit` is 1,200 steps
(60 s), about 2.9 times that. On a shared GPU a trial ran at 1 to 2 steps a second with cameras (about 4 to 7 minutes of
wall time for the demo, up to about 20 minutes for a 1,200-step model trial).

## Importance and sources

Category: **Societal relatability.** Bifold doors are the stock closet door in North American homes; every major interior
door maker sells them as 24 to 36 in sets with their hardware, and the makers' own instructions describe this exact
operation:

- JELD-WEN, *Installation Instructions for Bifold Doors (JII103)*
  ([Lowe's copy](https://pdf.lowes.com/productdocuments/5f7d7313-4b2d-4b77-aee4-2799f863d26e/61295829.pdf),
  [Home Depot copy](https://images.thdstatic.com/catalog/pdfImages/c6/c683cfbc-5706-4de0-8a4f-21ce3778dab0.pdf)): "Place
  the upper pivot pin in the hole in the bracket and the guide in the track ... Lift the door assembly slightly and drop
  the lower pin into the lower bracket slot. Verify there is a 1/4" clearance between the pivot door and the jamb." For
  adjustment: "Lift assembly slightly until the adjustment wheel clears the slot in the lower bracket ... Be sure the teeth
  lock firmly into bracket when finished." It also gives the pivot holes "1 1/4" from edges" and 1-3/8 in thick doors.
- A bifold hardware set's instructions
  ([Home Depot PDF](https://images.homedepot-static.com/catalog/pdfImages/cb/cb077847-5307-4e99-a07d-5b7ef14344fc.pdf)):
  "place top pivot pin (C) in hole, lift doors (compressing pin spring) and seat bottom pivot into jamb bracket slot";
  "Place jamb bracket tight to floor so that center hole of bracket is plumb with hole in the top pivot bracket (in
  track)"; "Lift door and slide bottom pivot bolt into desired position"; the knob goes "approximately 36-40" from floor".
- Masonite's bifold instructions
  ([Home Depot PDF](https://images.thdstatic.com/catalog/pdfImages/4a/4a074e91-61cf-431b-a99e-99266d852a5e.pdf)): the
  standard finished opening for a 79 in panel is "24" x 80-3/4"", which "allows for a 1/8" (3.2 mm) gap at the top of the
  bifold and a 1/4" (6.4 mm) space on the sides"; "Masonite bifolds are hinged with 1" top track"; "Install the door by
  inserting the top pivot into the pivot bracket in track. Lift the door and seat bottom pivot into jamb bracket."
- Another maker's sheet ([Lowe's PDF](https://pdf.lowes.com/productdocuments/c6606070-9f85-4724-b3fa-9aa0625787c7/12059312.pdf)):
  "With the top pivots inserted, lift upward on the panels and position the bottom pivot into the jamb bracket on the
  floor. Let the bottom pivot sit onto the jamb bracket firmly with the knurled teeth engaged."
- Family Handyman, [How to Fix Bifold Closet Doors](https://www.familyhandyman.com/project/how-to-fix-stubborn-bifold-closet-doors/):
  "After a few years of use, bifold doors often start to sag, scrape against the floor or pop out of their brackets";
  "The bottom pivot often loosens and slips"; "you can either raise the door slightly to shift the pivot in or out".

Limits of this evidence: the makers' instructions establish the operation and the hardware; Family Handyman establishes
that the fault is common. No source gives a count of households with bifold doors; the claim that most adults have met a
bifold door off its track is ours.

Why it is a robot problem: the hands work 14 to 34 cm above the floor on a 2 m door that pivots about a point at its
top, so moving them moves the bottom pin by about the same amount, but the pin is out of sight under the panels; the
lift has a window of about 5.5 mm (the rails need 5 mm of lift, the track stops the door at 10.5 mm), and the pin must go into a
10 mm slot within 4 mm of one spot along it.

### Distinct from

| Task | What it tests | Difference |
|---|---|---|
| TK-LVMJWW (#3188, pending): re-hang a sliding cabinet door, top up into its channel then foot onto the rail | A 29 cm cabinet panel lifted into a deep top groove, then its grooved foot dropped onto a floor rail | A sliding panel with no pivot or spring; here a 2 m hinged pair hangs on a spring-loaded pivot, and the task is a hidden pin into a slot at one spot (plumb), with teeth that lock it |
| TK-ZHRV7N (#1410, rejected): hang a bypass closet door on its track | Hanging a 14 cm model bypass door on rollers | Rejected as a miniature; this door is full size (79 in), and a bifold on pivots, not a bypass door on rollers |
| TK-V76PZX (#3313, rejected), TK-NVTZYW (#3094, pending), TK-JG9NPW (#166, rejected) | Hanging or lifting doors on removable-pin or lift-off hinges | Butt-hinge pin alignment; no spring pivot, floor bracket or plumb set |

External sets: LIBERO, RoboCasa, BEHAVIOR and SimplerEnv open and close cabinet and room doors; none rehangs a door on its
pivots or seats a pivot pin in a floor bracket.

## Scale and mass

**Scale: 100%.** Nothing is rescaled: the robot works at the door's bottom, where the arms reach, so the door, opening, track
and bracket are full size.

| Part | Real (source) | Modelled |
|---|---|---|
| Panel | 79 in tall (Masonite chart), 1-3/8 in thick (JELD-WEN JII103), 11-3/4 in wide each for a 24 in opening with 1/4 in at the jamb (JELD-WEN: set 1/2 in narrower than the opening) | 200.7 x 29.8 x 3.5 cm |
| Opening | 24 x 80-3/4 in for 79 in panels (Masonite) | 61.0 x 205.1 cm |
| Track | 1 in (Masonite) | 2.54 cm channel |
| Pivot and guide holes | 1-1/4 in from the panel edges (JELD-WEN) | 3.18 cm |
| Jamb gap | 1/4 in (JELD-WEN, Masonite) | 6.35 mm |
| Top gap | 1/8 in after adjustment (Masonite); the hardware adjusts | 3/8 in (9.5 mm), adjusted up so the door can be lifted to clear the rails |
| Knob | 36-40 in from the floor (hardware sheet) | 36 in |
| Door mass | about 9-10 kg for a 24 x 80 in hollow-core molded pair (our estimate from its construction: two 1/8 in hardboard skins of about 0.6 x 2.0 m, pine stiles and rails, paper honeycomb; retailers list 20-27 lb, which we could not open to verify) | 2.4 kg (1.2 kg per panel) |

**Mass:** reduced to a quarter of the real door so that the two YAM arms can lift it and hold it steady (2.4 kg plus a 5
to 8 N spring is about 29 N, shared by the two grippers' friction). The real door weighs 9-10 kg, which a person lifts
with both hands.

**Not modelled from a source:** the top pivot spring (300 N/m, 5 N preload when seated: an estimate), the pin's tilt play
at the top (joint limits 5 degrees either side of the starting tilt), the jamb bracket's exact plate and rail dimensions, the wheel (22 mm) and
pin (8 mm, protruding 4 mm), and the closet's depth behind the wall (26 cm instead of about 61 cm, because the bench ends
there; nothing in the task happens inside the closet).

The table is the floor: the closet's floor is the table top (z 0), and the whole door, wall and track stand on it at
their real heights. Nothing is lowered.

## The mechanism, and how the simulator judges each rule

The door is one articulation (`mechkit.mechanism`) fixed at the top pivot:

- `tilt_x`, `tilt_y`: free hinges about two horizontal axes through the top pivot, limits 5 degrees either side of the
  starting tilt (which is 1.6 degrees from plumb): the play of the pin in the top bracket and the track that lets the
  bottom swing out. No spring; gravity and the floor hold
  the door where it is.
- `lift`: a slider along the pivot axis with a 300 N/m spring that pushes the door down off the top pivot pin (the spring
  pushes the pin up into the bracket), 5 N when seated. Its upper limit is where the door's top meets the track (9.5 mm
  above seated).
- `pivot_panel`: the door's swing about the pivot axis, from closed to 95 degrees open. The panel carries the bottom pivot
  bolt (wheel and pin, with a rounded-off tip) as colliders that rest on the floor or on the bracket.
- `lead_panel`: the fold hinge on the panels' closet faces, from flat to folded. Its guide sits in the track between the
  channel's walls as a collider, so the pair folds and slides as a real bifold does.

The bracket is a fixed part: two plate halves either side of the slot and two rails with rounded inner edges, as convex
colliders. **The teeth** are modelled as a lock: once the pin is inside the slot and its tip is within 0.8 mm of the seated
height, the `tilt_x` joint (which moves the pin along the slot) is held where it is (`mechkit.lock`), as meshed teeth hold
the wheel; it is let go once the tip is lifted 1.5 mm above seated. The lock never acts across the slot, where the slot's
walls hold the pin, and never moves the pin.

`Trial.step` runs once per control step. It computes the pin's tip from the door's pose and joints; the guide's position
from the lead panel's pose; the panels' speeds; and the smallest distance from mechkit's 24 fingertip points to either panel
(each taken as its full slab). `score()` and `success()` only read the result. The record that the guide left the track is
cleared only in mechkit's `on_reset` hook.

## Scoring

- **Seated:** the pin's centre within 1.5 mm of the slot's centre line across the slot, inside the slot's length, and its
  tip at most 2.5 mm above the floor (seated it reads 1 mm; standing on the rails it reads 5 mm).
- **Plumb:** the pin's centre within 4 mm, along the slot, of the painted mark under the top pivot (the door then leans
  less than 0.12 degrees).
- **Over the slot:** the pin's centre within 6 mm of the slot's centre line and inside its length.
- **1.0:** seated and plumb; both panels still (under 5 mm/s at the top pivot, under 0.01 rad/s); every sampled finger
  point more than 3 cm from both panels; all continuously for 2 s.
- **0.5:** seated, but not 1.0: off the mark by more than 4 mm, or still held, or moving.
- **0.25:** over the slot but not seated: resting on the rails' inner edges, or held above the slot.
- **0:** anything else, such as the pin on the floor beside the bracket, or on the plate or a rail more than 6 mm off the
  slot's centre line. And 0, whatever the end state, if the lead panel's guide ever left the track (more than 14 mm off
  its centre line).

Possible scores: 0, 0.25, 0.5, 1.0. Lifting, tilting and regrasping the door are not penalised; folding or swinging it is
allowed; the door can end at any swing angle.

## Real trial

**Build.** Use a real 24 x 80 in two-panel hollow-core bifold with its maker's hardware (any of the sources above; for
example a JELD-WEN or Masonite 24 in molded bifold, which come with track, top pivot bracket, spring top pivot, bottom
pivot and jamb bracket). Frame a test opening 24 x 80-3/4 in in a freestanding stud wall (2x4 studs, 1/2 in drywall or
plywood skin) screwed to a plywood floor platform, with the robot's bench beside it so that the floor platform is level
with the bench top and the arms stand where the table edge is here. Hang the door to the maker's instructions, set the
bottom pivot bolt so the top gap is 3/8 in, and paint a mark on the bracket plate beside the slot directly under the top
pivot (use a plumb bob from the top pivot bracket's hole). Distractors: a folded hand towel, a small cardboard box inside
the closet and a small waste bin, as in `setup`.

**Prepare each trial.** Lift the door out of the bracket and set the pin's tip on the floor on a tape mark 5.5 cm toward the
room and 1.3 cm away from the jamb from the painted mark, the door folded open about 85 degrees with its guide in the track.
Check the pin is on the mark and the top pivot and guide are in the track. This takes about a minute; nothing is consumed.

**Measure,** before anyone touches the door: seated if the wheel stands on the rails with its teeth engaged and the pin is
in the slot (check that a 2 mm feeler gauge does not pass under the wheel); plumb if a ruler shows the pin's centre within
4 mm of the painted mark; still and hands clear from the video's last 2 s; guide in the track by eye. A level held against
the pivot panel's edge after the reading is a cross-check.

**Safety.** The real trial uses the real door at its real mass (9-10 kg), which is heavier than the simulated 2.4 kg, so
a real robot must lift about four times the load; an arm that cannot should fail the lift, not force it. The door stays
captive at its top pivot and guide whatever happens at the bottom, so it cannot fall: check that both are in the track
before every trial. Keep people out of the door's swing, and stop the trial if the guide comes out of the track.

## Validation

All runs below are demokit runs on the GPU (PhysX on the GPU, as a model trial runs), with no seed-dependent randomisation.
The two full demos were recorded with cameras at 640 x 360; the checks ran with `--no-record`.

| Run (`--mode`) | What it does | `task_score` | Steps |
|---|---|---:|---:|
| full (1) | the two-handed demo described above | **1.0** (success) | 420 |
| full (2) | the same, a fresh run | **1.0** (success) | 420 |
| none | does nothing | 0.0 | 440 |
| drag | careless: pinches the panels and pushes the bottom back toward the mark without lifting; the pin stops against the bracket | 0.0 | 435 |
| straight_back | lifts and seats the pin straight back, 13 mm along the slot from the mark; the teeth hold it there | 0.5 | 495 |
| hold | seats the pin at the mark but keeps both hands on the door | 0.5 | 552 |
| drop_early | cheapest shortcut: lifts, carries the pin back to 6 mm short of the slot's centre line and lets go before it is over the slot; it does not drop in | 0.0 | 393 |
| onearm | the full demo with the left hand alone | 0.0 | 687 |

In the one-hand run the single pinch slipped while carrying the door and the pin ended 37 mm from the mark. The same
script reached 1.0 with one hand on the CPU physics, so one hand is not ruled out; the scorer does not require two. A first
check session crashed in the renderer (a segmentation fault in the GPU foundation plugin, not a physics blow-up) after
three modes; the remaining three were rerun in a new session. No run printed a joint blow-up. The recorded demo's top and
wrist camera frames were inspected: the door stays on its top pivot and guide throughout, the hands leave the panels at
the end, and no arm or door touches a distractor.

One Dream-1 run (1,200 steps, 320 x 176 cameras) scored 0.0: its arms swept the floor near the waste bin and the towel, tipping the bin over, and never touched the door, whose pin stayed on the floor beside the bracket.

## Assets and licences

The closet, door, track, bracket and hardware are authored structural and mechanical parts (plain meshes and convex
colliders) built in `task.py`. The distractors are NVIDIA SimReady props, downloaded at first run from
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets) pinned to revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, © NVIDIA, CC-BY 4.0:

- [`consumable_microfiber_towel_folded_small_b01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/consumable_microfiber_towel_folded_small_b01): mass set to 0.12 kg.
- [`box_cardboard_d01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/box_cardboard_d01): mass set to 1.2 kg.
- [`bin_waste_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/bin_waste_a01): mass set to 1.3 kg; its lid joints welded into one rigid body by the loader.

Nothing large is committed: files download at first run. `simready.py` is adapted from the open-source
`cutters-off-pegboard` loader in this repository. `mechkit.py` and `demokit.py` are taken unchanged from another team's
open-source task in this repository, `tasks/heat-shrink-lug/`.
