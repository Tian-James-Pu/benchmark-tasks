# Clamp an infusion pump to its IV pole at the tape: hold it up level while the other hand screws the pole clamp tight

**Instruction:** *Mount the pump on the pole at the tape.*

A BD Alaris GP volumetric infusion pump stands on the table in front of an IV pole mast, its face towards the arms and its back
towards the pole. Its pole clamp is folded out and unscrewed, as the maker's directions say to leave it before mounting: the throat
is open 3 mm wider than the pole on each side. A band of yellow tape on the pole marks where the ward wants the pump. The robot has to
lift the pump by its carry handle, bring it back onto the pole so that the pole enters the open throat, and hold it there, level and
at the tape, while its other hand screws the clamp's knob in until the moving jaw bites (about four turns, in regrips of half a turn).
Then both hands let go and the pump must hang on its own clamp, at the tape, upright, without sliding.

## Importance and sources

Hanging a pump on an IV pole is the first physical step of every infusion run on a pole-mounted pump, and it comes again whenever the
pump has to move to another pole with its patient (our inference; no source counts how often). It is done by the nurses who run the
infusion.

- U.S. FDA, *Infusion Pumps*: "Infusion pumps are in widespread use in clinical settings such as hospitals, nursing homes, and in the
  home." https://www.fda.gov/medical-devices/general-hospital-devices-and-supplies/infusion-pumps
- O*NET 29-1141.00 Registered Nurses, core task: "Administer medications to patients and monitor patients for reactions or side
  effects." https://www.onetonline.org/link/details/29-1141.00 . BLS: registered nurses "held about 3.5 million jobs in 2025"
  (3,465,400). https://www.bls.gov/ooh/healthcare/registered-nurses.htm
- O*NET 29-2061.00 Licensed Practical and Licensed Vocational Nurses, core task: "Administer prescribed medications or start
  intravenous fluids, noting times and amounts on patients' charts." https://www.onetonline.org/link/details/29-2061.00 . BLS: about
  666,900 jobs in 2025. https://www.bls.gov/ooh/healthcare/licensed-practical-and-licensed-vocational-nurses.htm
- The product and the operation: BD (Cardinal Health) *Alaris GP Volumetric Pump, Directions For Use*, 1000DF00152 Issue 1.
  Pole Clamp Installation: "A pole clamp is fitted to the rear of the pump and will provide secure fixing to vertical I.V. poles of a
  diameter between 15 and 40 mm." "1. Pull the folded pole clamp towards you and unscrew the clamp to leave enough room for the size
  of the pole. 2. Place pump around pole and tighten screw until the clamp is secured to the pole." Warning beside it: "Never mount
  the pump such that the infusion stand becomes top heavy or unstable." Specifications: "148mm (w) x 225mm (h) x 148mm (d). Weight:
  approx. 2.5kg". Operating precautions: "If this pump is dropped, ... remove it from service for inspection". A copy of the DFU:
  https://cdn.shopify.com/s/files/1/0579/5374/7139/files/Alaris_GP_-_User_manual.pdf

Evidence limits: the O*NET statements name giving medications and starting IV fluids, not mounting the pump itself; mounting is the
physical first step of every infusion run on a pump, but no source gives how many times a day a nurse does it. The tape mark stands
for the height the ward sets (the DFU's "top heavy or unstable" warning is why a height is set at all); the exact height is a local
choice, not a published figure.

**The knack.** The pump hangs from its clamp 9 cm off the pole's axis and its clamp closes by a lead screw (1.75 mm a turn), so
nothing holds it until the jaw has travelled about 6 mm onto the pole: about 3.4 turns of the knob, in strokes of half a turn with
a regrip between each, about 35 s in the demo. All that time one hand has to carry the whole pump by its handle, with the pole inside the 31 mm
throat, level, and at the tape, against its weight and against the knob being twisted; the other hand turns the knob. Let go before the
jaw bites and the pump drops down the pole (a dropped pump must be taken out of service). Tighten it while it stands on the table and
it is clamped at the wrong height, 5 cm low; hold it while the jaw bites but tilted, and it is clamped cocked.

**Distinct from** the nearest tasks we know of:

| Nearest | What it tests | Difference here |
|---|---|---|
| Load an IV set into an Alaris pump module (PR #1966, accepted) | soft tubing into a pump's channel and safety-clamp recess | the pump itself is mounted: carried by its handle onto a pole and clamped there; no tubing |
| Fit a lever set to a bored door: hold the outside lever level while both screws are driven (PR #2018, accepted) | one hand holds a light part level, the other drives screws | the held part is the whole device (2.5 kg real, 1.0 kg simulated), carried in the air at a marked height off the pole's axis, and the fastener is the device's own clamp screw, turned by regrip strokes for about four turns |
| Clamp a small part before drilling it (PR #776, accepted); trigger bar clamp on a pine joint (PR #3409, pending) | closing a clamp on parts that rest on a bench | here nothing rests: the clamp is what will carry the load once tight, and until then a hand does |

We know of no task in LIBERO, RoboCasa, SimplerEnv, BEHAVIOR, DataCenterBench or RoboDojo that hangs a device on a pole by its own
clamp; this rests on their task lists as we know them, not on a fresh audit of each.

## Scale and mass

| | Real (source) | Simulated |
|---|---|---|
| Pump case | 148 mm (w) x 225 mm (h) x 148 mm (d) (Alaris GP DFU, Specifications) | the same, scale 1.0 |
| Pump mass | approx. 2.5 kg (same) | **1.0 kg** |
| Pole | 15 to 40 mm, any vertical I.V. pole (DFU, Pole Clamp Installation); 25 mm here | 25 mm |
| Pole stand | a free-standing IV stand, mast about 1.5 to 2 m up from a castor base | a 55 cm mast in a flange screwed to the table (see below) |
| Tape mark | wherever the ward sets it | a 50 mm band centred 20 cm above the table |

- **No rescaling.** The pump is at its full published size. Before building, the arms' reach was checked offline: the handle's bar on
  the hanging pump (32 cm above the table) is out of reach for a straight-down grip but in reach for a grip tilted 50 degrees from
  vertical, and the knob is in reach for a level grip rolled 85 degrees either way. So nothing was shrunk.
- **Mass cut from about 2.5 kg to 1.0 kg.** In the simulator the YAM's wrist joints (DM4310 motors) are limited to 7 N m and are soft
  (10 N m per radian). In an earlier version of this scene at 2.5 kg, a level grip on the handle could not lift the pump off the table:
  the hand tilted about 17 degrees under the load and the pump stayed down. At 1.0 kg the arm carries it, and a closed-loop correction
  (see The demo) holds it level. Every dimension, the clamp and the hold the task asks for are unchanged; only the load is lighter. The
  real trial uses the real 2.5 kg pump, so the holding hand's job there is harder than in the simulation.
- **The pole's stand is a mounting stand-in.** A real IV stand rises from a castor base on the floor, and a pump is mounted on it about
  a metre up, out of the arms' reach from a table. Here the table top stands in for the stand's lower part: the mast rises from a flange
  on the table, and the tape band is set 20 cm up it, where the arms reach. What the task tests, the pump's clamp round the pole, its
  9 cm offset, the pole's 25 mm diameter and the vertical mast, is kept at real size and orientation.

## Run and score

From the repository root, in the Python environment that has `yam_arena` and `inspect-robots`:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/clamp-infusion-pump-on-iv-pole-at-mark/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Mount the pump on the pole at the tape." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/clamp-infusion-pump-on-iv-pole-at-mark
```

The scripted demonstration is closed loop (it reads the pump's pose and the knob's angle, never the scorer), so it runs through
`demokit.py` from the task folder:

```bash
cd tasks/clamp-infusion-pump-on-iv-pole-at-mark
python demokit.py task.py ~/yam-eval/clamp-infusion-pump-on-iv-pole-at-mark/demo                  # writes full/demo.mp4, setup.png, result.json
python demokit.py task.py ~/yam-eval/clamp-infusion-pump-on-iv-pole-at-mark/checks --fast --mode none,onearm,careless,low,held
```

`result.json` holds `task_score`. The layout is fixed (`object_jitter=0`); there are no seeds or random choices. The demo reaches full credit at step 1,233 on the GPU (62 simulated seconds, about 4 turns of the knob in 10 strokes; 1,143 steps in a CPU `--fast` run, which needs 9 strokes); the run limit is 2,400 steps, about twice that. A GPU-rendered run took about 20 minutes of wall time on a heavily shared GPU (1 step a second); a CPU `--fast` run (no cameras) takes about 1 minute.

## How the simulator models it

All in `task.py`. Nothing is triggered by proximity: the pump hangs, or not, on contact and friction at its jaws.

- **Pump** (free articulation, 1.0 kg): a rigid case 148 x 148 x 225 mm with a screen, keypad, door and rubber feet as surface
  detail; a carry handle on top, a 20 x 16 mm bar 110 mm long running fore and aft on two posts, 37 mm above the case. On its back the
  folded-out pole clamp: a back plate 6 mm proud of the case, a fixed jaw on the pump's left and the screw's housing arm on its right,
  each 40 mm tall, centre line 15 cm up. The moving jaw (a slider, 50 g) is inside the housing; open, it leaves a 31 mm throat. The
  knob (a hinge on the screw's axis, 30 g, 35 mm across, 25 mm long, eight ribs and a white index mark) is on the pump's right side.
  Contact friction 1.0 static, 0.9 dynamic.
- **Lead screw** (`Screw` in `task.py`): PhysX reports the knob's angle wrapped, so its turns are counted step by step. The moving
  jaw's drive (200 kN/m, at most 200 N) is set to 1.75 mm of travel per clockwise turn, but never more than 1 mm past where the jaw
  actually is: once the pole stops the jaw, it presses the pole with 200 N. At that moment the knob meets a stop, a 2 N m/rad spring
  holding it where it is: the gripper's pads slip on it at about 1 N m, as a hand-tight knob does. Turning it back 3 degrees frees it.
  The pitch is our estimate (an M12 coarse screw); the DFU does not give it.
- **Pole** (fixed): a 25 mm stainless tube, 55 cm tall, in a 60 x 8 mm flange; the tape band is a 50 mm sleeve on it.
- **Distractors**: three SimReady props, loose, out of the arms' paths (see Assets).
- **Not modelled:** the pump's internals and its fold-away clamp hinge (the clamp starts folded out, as the DFU's step 1 leaves it),
  forces and torques (nothing is scored on them), the castor stand.

## Scoring

Everything is read on the pump and the pole; nothing on forces.

- **Harm (0, for the rest of the run, whatever happens after):** at any moment the pump moving down faster than 0.3 m/s (dropped, or
  sliding down the pole: a free fall of 4.6 mm reaches it), tipped more than 45 degrees from upright, or with its lowest corner more than
  3 cm below the table top (off the table). The record is cleared only by a reset (`on_reset`).
- **Hanging:** the pole inside the clamp's throat with both jaws within 1 mm of it (the screw is done up on the pole), the pump's
  lowest corner at least 1 cm above the table, no loose object (a distractor) standing under the pump's outline, and every fingertip point of both
  grippers at least 3 cm from the pump's outline (case, handle, clamp and knob).
- The score is read once the pump has been hanging for 3 s without a break (`window`), and only while it still is:
  - **1.0**: the clamp's centre line on the tape band (17.5 to 22.5 cm above the table), the pump within 3 degrees of upright, and
    sunk no more than 5 mm since it began hanging. The trial ends there.
  - **0.75**: on the band and upright, but sunk more than 5 mm.
  - **0.5**: on the band, but more than 3 degrees from upright.
  - **0.25**: hanging, but the clamp's centre line off the band.
  - **0**: otherwise: harm, not hanging (standing on the table, held, propped, loose on the pole), or hanging for less than 3 s.

`task_score` is 1.0 when full credit is reached, otherwise the last step's score.

## Real trial: setup, reset and measurement

The real trial uses the real, full-size product at its real mass.

**Parts.** A BD (formerly Cardinal Health / CareFusion) Alaris GP volumetric pump, 2.5 kg, with its pole clamp; a retired or training unit is fine, it need not
pump, and it should not be returned to clinical use after trials (it may be dropped). A 25 mm (1 in) stainless steel tube 60 cm long,
for example a replacement IV pole mast, in a 1 in floor flange screwed to the bench; the tube must stand vertical within 1 degree (check
with the angle gauge). Yellow 50 mm (2 in) tape. Props: a box of examination gloves, a 10 cm plastic cup, a small box of bandages.

**Setup** (from tape marks on the bench, in cm from the midpoint between the arm bases, x forward, y left): the pole's axis at (40, 0);
the tape wrapped round it from 17.5 to 22.5 cm above the bench. The pump stands on its feet with its underside's centre at (24.75, 0),
face towards the arms; mark its two front feet on the bench. Its clamp folded out, and the knob turned back until a 31 mm gauge (a
31 mm drill blank or a spacer block) slides into the throat with the moving jaw just touching it. Props at (60, 30) turned 17 degrees,
(60, -30) and (14, 27).

**Reset** (about 2 minutes): if the pump is on the pole, hold it, turn the knob back about four turns until the 31 mm gauge slides in,
lift it off and stand it on its foot marks. Check the throat with the gauge each time: the starting opening is the fault-free start the
DFU describes ("unscrew the clamp to leave enough room") and must be the same every trial.

**Measurement** (filmed from the side at 60 fps, with a ruler taped beside the pole, plus a top view):
- Dropped: the pump's bottom edge moves down 5 mm or more between two frames (0.3 m/s). Tipped or off the table: on video.
- Jaws on the pole: a 1 mm feeler gauge does not enter between either jaw and the pole.
- Hanging: a 10 mm gauge block slides under the pump's lowest corner; nothing stands under the pump; on the video, every fingertip
  at least 3 cm from the pump for the last 3 s.
- On the band: a line scribed round the clamp at its centre line (15 cm up the pump's back) lies between the tape's edges.
- Upright: a digital angle gauge on the pump's top, front to back and side to side, both within 3 degrees.
- Sunk: a pencil mark on the pole at the clamp's top edge when the last hand leaves; the gap to the clamp's top edge 3 s later is at
  most 5 mm.

**Rig safety.** No blades, heat or liquids. The pump is unplugged. A drop is at most 5 cm onto the bench; keep feet and the arms'
cables out from under the pole.

## Assets and licences

The pump, its clamp and the pole are authored primitives in `task.py` (SimReady has no volumetric pump of this kind with a working
pole clamp; its one infusion pump, `pump_infusion_a01`, is a syringe-driver style device with a different, fixed clamp). The three
distractors are NVIDIA SimReady props, downloaded at first run from the pinned revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd` of
`nvidia/simready-assets` on Hugging Face, © NVIDIA, CC-BY 4.0:

- [box_glove_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/box_glove_a01): mass set to 0.30 kg.
- [dishware_plastic_cup_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_plastic_cup_a01): mass set to 0.02 kg.
- [bandage_medical_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/bandage_medical_a01): mass set to 0.02 kg.

Each is one rigid body as loaded (any joints welded). Nothing large is committed. `simready.py` is the loader from
`tasks/cutters-off-pegboard/`, `mechkit.py` the mechanism kit from `tasks/heat-shrink-lug/` and `demokit.py` the closed-loop demo
runner from `tasks/heat-shrink-lug/`, all unchanged: other teams' open-source files in this repository.

## The demo

`demo_script` in `task.py`:

1. The left hand comes down on the carry handle from above and behind, fingers tilted 50 degrees from vertical and closing across
   the bar, and grips it.
2. It lifts the pump 5 cm and carries it back 6 cm onto the pole, so the pole passes into the open throat and sits 2 mm short of the
   clamp's back plate. All the while it reads the pump's pose and, each step, shifts its command by a quarter of the clamp's position
   error and turns its grip by 30% of the pump's tilt from upright: that takes out the wrist's droop under the load.
3. The right hand comes in beside the knob from the right, fingers level and rolled 85 degrees, and closes on it. It turns it 170
   degrees clockwise (as seen facing the knob), opens, turns back, closes and turns again; the left hand keeps holding the pump level at
   the tape throughout. When a stroke turns the knob less than 90 degrees, the jaw has bitten and the knob has met its stop.
4. The right hand backs off and rises clear, then the left lets go of the handle, backs off and moves away. The pump hangs on its clamp.

## Validation

All runs on the final `task.py`, with `demokit.py` from the task folder.

| Run | What it does | `task_score` | Steps |
|---|---|---|---|
| `full`, GPU, rendered (twice) | the demo above | 1.0, 1.0 | 1,233, 1,233 |
| `full`, CPU `--fast` | the same, no cameras | 1.0 | 1,143 |
| `none` | nothing | 0.0 | 163 |
| `onearm` | the left hand alone: carried onto the pole at the tape and let go, knob never turned; the pump drops down the pole (harm) | 0.0 | 370 |
| `careless` | the left hand lets go after the first half turn, before the jaw bites; the pump slides down the pole (harm) | 0.0 | 659 |
| `low` | the clamp screwed tight with the pump standing on the table (clamp 5 cm under the band, not hanging) | 0.0 | 1,354 |
| `held` | clamped tight at the tape, but the left hand keeps hold of the handle | 0.0 | 1,298 |
| off-band (a variant file with the lift cut to 1.5 cm, since deleted) | clamped tight and released, hanging, its clamp 6 mm under the band's lower edge | 0.25 | 1,321 |

The scorer was also checked offline on constructed poses: hanging at the band and hands clear for 3 s gives 1.0; for 2.9 s, 0;
with a fingertip on the pump, 0; with a box under it, 0; 4 cm high or 3 cm low, 0.25; tilted 4 degrees either way, 0.5; sunk 8 mm
during the hold, 0.75; jaws 3 mm open, 0; an earlier drop (0.8 m/s) then hanging, 0.

The demo's highest downward speed of the pump is 0.02 m/s, far under the 0.3 m/s drop rule; the `careless` slide reached it.

**Model run.** Dream-1 (`--policy dream`, the command above, 2,400 steps) scored 0.0: its grippers came towards the pump but never lifted it, and the pump stayed standing on the table for the whole run.

