# Focus a diode laser before cutting robot parts

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [task.py](task.py).

An open-frame 8 W diode laser engraver of the AlgoLaser DIY KIT MK3 type stands unplugged on the table with its honeycomb platform
and a 3 mm basswood plywood sheet. Its laser module hangs on a vertical slide on the gantry's carriage, clamped by a knurled
thumbscrew beside it, 12 mm higher than focus for this sheet. A focus stick pivots on the module's front face. To focus, the robot
flips the stick down, takes the module's weight, loosens the thumbscrew, lowers the module until the hanging stick touches the sheet,
tightens the thumbscrew, stows the stick and lets go. It must not let the loosened module drop, and must not move the sheet.

Instruction (goal only): "Focus the laser module on the plywood sheet with its focus stick, and leave the engraver ready to cut."

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/focus-diode-laser-module/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Focus the laser module on the plywood sheet with its focus stick, and leave the engraver ready to cut." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/focus-laser-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 900 -E cam_width=640 -E cam_height=360`.
The run prints `task_score` at the end.

## Runtime and seeds

- Demo: 770 control steps (38.5 s simulated, including the 2 s success hold; the episode ends when it completes); about
  1.5 minutes per trial on one RTX-class GPU once shaders are compiled.
- Dream-1: 2,400 steps (2 minutes simulated).
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

## The scene

| Object | Model | Real counterpart |
|---|---|---|
| Engraver frame, gantry beam | fixed boxes (2 cm aluminium profiles, 4 x 4 cm beam) | AlgoLaser DIY KIT MK3 frame (400 x 400 mm work area) |
| Honeycomb platform | fixed box, 45 x 45 x 2.3 cm | AlgoLaser Honeycomb Platform (Honeycomb Plate), 400 x 400 mm working area |
| Plywood sheet | free box, 20 x 30 x 0.3 cm, 90 g, friction 0.5/0.4 | 3 mm basswood plywood |
| Laser module | prismatic link, 0.6 kg, 5 x 5 x 9 cm body over a 2.5 cm orange shield | the MK3's 8 W module |
| Thumbscrew | hinge link, knurled head 12 mm x 20 mm, seated at 0 deg | the carriage's knurled clamp screw |
| Focus stick | hinge link on the module, steel strip 3 x 6 mm, rounded end, 40 mm pivot to tip, pivot knob 12 mm | the module's built-in focus stick |
| Caliper, mug (distractors) | SimReady props, loose | a digital caliper and a mug on the honeycomb, ignored by the scorer |

All positions, sizes and masses are in `task.json` ("setup") and at the top of `task.py`.

## How `score()` reaches the score

Everything below runs in `task.py` from mechkit's step hook, every control step (20 Hz), and the record is cleared only in its reset
hook (`Trial.reset`), never because the objects look right again.

- **The clamp** (`Clamp`): the slide's drive is authored stiff (200 kN/m, 2 kN), so the module stays clamped through the reset's
  settle. While the thumbscrew is loosened less than 45 deg from its seat the drive holds the slide where it is; loosened further the
  drive's stiffness is zero and the module is held only by its 8 N s/m damping and whatever holds it. A real thumbscrew clamps by
  friction over a range of turn; the 45 deg threshold is an estimate.
- **Focus error** `e`: the hanging stick's tip height above the sheet's top face, from the slide's position and the sheet's pose. The
  stick hanging touches the sheet at `e = 0`.
- **Latched violations** (score 0 for the rest of the trial): the slide comes down more than 1.5 mm in one 50 ms step (over 3 cm/s:
  a free-falling module passes this within a few millimetres; a held lowering is ten times slower), or any corner of the sheet moves
  more than 3 mm from where it lay after the settle.
- **Released and still**: all 24 fingertip points (mechkit `fingertips`) 3 cm or more from the module's box (body, shield, stick
  and knob) and from the thumbscrew's head; the slide moved under 0.5 mm over the last step; the sheet under 1 cm/s.
- **Score**: 1.0 while `|e| <= 0.5 mm`, clamped, stick within 15 deg of stowed, released and still have held for the last 40 steps
  (2 s); losing any of them restarts the count, so full credit does not stick. 0.75 in focus, clamped and released with the stick
  not stowed (or before the 2 s); 0.5 for `0.5 < |e| <= 2.0 mm`, clamped and released; 0.25 in focus, released, not clamped (the
  module resting on its stick); 0 otherwise (including anything still held or moving). `success()` is the 2 s hold.

Possible scores: 0, 0.25, 0.5, 0.75, 1.0. Doing nothing scores 0 (`e` stays 12 mm).

## Real-world trial

Kit (about US$345 in all before tax and shipping, prices read on 20 November 2026; see the sources below):

- AlgoLaser DIY KIT MK3, 8 W, "Machine" variant: US$249, in stock at the maker's store. It is a complete machine: frame, gantry,
  carriage with thumbscrew, 8 W module with its built-in focus stick. It stays unplugged; the laser never fires.
- AlgoLaser Honeycomb Plate, 400 x 400 mm working area: US$55.99, in stock at a US retailer (the maker's own store listed its
  400 x 400 mm variant at US$69 but sold out on that date). The platform only raises the work; any flat, rigid 20-25 mm support
  under the sheet gives the same task, since focus is measured from the sheet.
- AlgoLaser 3 mm basswood plywood, 12 x 12 in sheets, pack of 10: US$39; one sheet cut to 200 x 300 mm.
- Measurement: a 150 mm digital caliper, a luggage scale and a cord loop, masking tape, a paint pen.

Preparation and reset: mark the thumbscrew's head and the carriage with the paint pen with the screw seated (finger tight). Set the
module 12 mm above focus: hold it, loosen the screw, set the hanging stick on a 12 mm drill shank lying on the sheet, tighten, take
the shank away and stow the stick. Tape the sheet's corners and the gantry's position on the frame.

Measuring the outcome: `e` with the caliper's depth rod from the shield's lower edge to the sheet, minus the stick's reach below the
shield (measured once with the caliper); clamped: the paint marks within 45 deg, and a 10 N downward pull on the module (luggage
scale) moves it less than 0.2 mm (caliper before and after); stowed: the stick within 15 deg of upright; drop: an overhead or side
video at 240 fps, the shield's edge against a steel rule taped to the carriage, frame by frame; sheet: its corners against the tape.

## Checks run

`DEMO_MODE` at the top of `demo()` selects a red-team variant; each was run as a copy of `task.py` with only that line changed
(the copies were deleted after the runs; to repeat one, copy `task.py` and change that line). Scores from the yam_scripted policy, 900-step limit:

| Variant | What it does | Expected | Scored |
|---|---|---|---|
| full (twice) | the demo | 1.0 | 1.0, 1.0 |
| nothing | no motion | 0 | 0 |
| onearm | the right hand alone: nothing holds the loosened module | 0 (drop) | 0 |
| careless | grips the module but loosens without taking its weight | 0 (drop) | 0 |
| hold | never lets go of the module | 0 (not released) | 0 |
| nostow | leaves the stick hanging | 0.75 | 0.75 |
| unlocked | never tightens the thumbscrew again | 0.25 | 0.25 |

Other routes, reasoned from the code rather than run: pushing the clamped module down does nothing (the clamp's drive is far stiffer than a gripper);
lowering it to the right height without flipping the stick down earns the same credit, which is the same end state; pressing the
module onto the sheet with the stick hanging moves the sheet (latched 0) or leaves `e` negative beyond 0.5 mm.

## Fidelity and estimates

Rigid bodies throughout. The frame, gantry, carriage and honeycomb are fixed (a real gantry rolls; the module's horizontal position
does not change the focus). The thumbscrew has no thread: the clamp rule above stands in for its friction. The stick and thumbscrew
hinges have estimated friction (0.03 and 0.05 N m), the slide estimated damping (8 N s/m). The module's mass (0.6 kg) and the
module, stick, knob and screw dimensions are estimates scaled from the maker's product photos; the stick's reach below the shield
(10 mm) is an estimate (the real value is measured once with the caliper, above). The machine is built from primitives: SimReady has
no laser engraver.

The demo is open loop. It uses forward-pointing grips (3 deg below horizontal) to pinch the two knobs and roll about their axes:
`task.py` lets a `Waypoint`'s `yaw` also be a 3 x 3 rotation matrix (a number keeps yam_arena's behaviour). The left arm's 9 mm
give under the module's 6 N was measured in the simulator and is built into the demo (it pushes up 9 mm against the clamp before
the screw is loosened, as a person takes the weight first). The demo reads no simulator state.

## Importance and sources

Category: Impact on AI timelines. Desktop laser cutters make the plate parts of small robots, and an open-frame diode laser has no
autofocus: before each sheet a person sets the module's height by hand. A robot that can do this can run the machine that cuts
parts for robots. No occupation-size claim is made: the category is Impact on AI timelines (building robots with their real
tools), not Impact on jobs, and we found no occupation count for diode-laser operators to cite.

- Robot parts cut from 3 mm sheet: the MeArm robot arm's repository ships "Laser cut (3mm) dxf files"
  ([MeArm on GitHub](https://github.com/MeArm/MeArm)).
- The product, its focus stick and price: AlgoLaser's DIY KIT MK3 page lists the 8 W machine at US$249 (store data read on
  20 November 2026) and shows the module with its stick: "With the built-in focus stick, just flip it down, touch your material,
  and lock it in." ([AlgoLaser DIY KIT MK3](https://algolaser.com/products/algolaser-diy-kit-mk3-diode-laser-engraver)). The same page's
  photo shows the knurled thumbscrew on the carriage beside the module; its sizes here are scaled from that photo (an estimate).
- The honeycomb platform: the maker's page gives "The size of working area is 400*400mm", "Product size: 450mm*450mm*25mm" and
  "Product thickness: 2.3cm" ([AlgoLaser Honeycomb Platform](https://algolaser.com/products/algolaser-400mm-400mm-honeycomb-platform-engraving-machine-accessories));
  its dimensioned drawing shows 450 mm outside, 400 mm working area, 22.5 mm frame height and 1.9 kg
  ([drawing](https://algolaser.com/cdn/shop/files/AlgoLaser_Honeycomb_Platform_Thickness_22.5mm_600x600.jpg?v=1735190955)). The model
  uses 23 mm; the difference does not enter the score. In stock at US$55.99, "Dimensions: 400 mm x 400 mm x 22 mm"
  ([Wellbots: Algolaser Honeycomb Plate](https://www.wellbots.com/products/algolaser-honeycomb-plate)).
- The plywood: 3 mm basswood, 12 x 12 in sheets, 10 for US$39
  ([AlgoLaser basswood plywood](https://algolaser.com/products/basswood-plywood-sheets-10pcs)).
- The operation and the drop hazard, on a comparable open-frame machine (xTool D1 Pro): "Put the focal length setting bar down",
  "Loosen the thumb screw on the other side and slide the laser module upward or downward", "After determining the position, tighten
  the thumb screw and put the focal length setting bar back", and "Prevent the middle plate and laser module from falling down
  themselves. Otherwise, the limit pieces may be damaged." ([xTool D1 Pro Quick Start Guide](https://support.xtool.com/article/208)).
  That the same care applies to the MK3's module is an inference from the shared design (a free slide clamped by a thumbscrew).

## Distinct from

| Nearest task | Its skill | Why this one differs |
|---|---|---|
| TK-WX26G2, angle grinder guard (revising) | loosen a band clamp, rotate the guard, re-clamp | here the loosened part is a 0.6 kg load on a free vertical slide: one hand must carry it (it drops otherwise) while the other works the screw, and the stop is set by touching a hanging gauge to the work |
| TK-UZH2XJ, microscope focus (pending) | turn a focus knob until the image is sharp | no knob drive or image judgement here: a direct, gauge-referenced height set and a friction clamp |
| TK-A8X2KJ, flex a 3D printer's build plate (accepted) | flex a plate to free a part | the same robot-building workshop, a different machine and skill |

The held-control family (hold one sprung control while working another) is different: nothing here springs back; the held object
falls if let go, and its final position is chosen by the gauge, not by a stop.

## Earlier submission of this idea

This is an independent implementation of an idea first submitted by another team in PR #2885, whose hold lapsed without a
revision. No code was reused; the xTool guide quoted below is a public source both use. Its review raised these points, each addressed here:

- "The only fully specified real-trial machine costs about $1,300": this task's machine is the AlgoLaser DIY KIT MK3 8 W, a
  complete engraver (frame, gantry, carriage, module), US$249 and in stock on 20 November 2026.
- "the separately listed module has no current price or complete mounting setup": no separate module is used; the module, its
  carriage and its thumbscrew come mounted on that machine. The whole kit, with the honeycomb (US$55.99) and the plywood
  (US$39), is about US$345 (see "Real-world trial").
- "while preserving the actual focus bar, sliding carriage and screw-clamping action": the MK3's module has a built-in focus stick
  that flips down to touch the work, a vertical slide on its carriage and a thumbscrew clamp, and the scene models all three as
  working joints (the stick hinge, the prismatic slide under gravity, the screw hinge driving the clamp). The laser stays
  unplugged and the instruction does not mention the beam.

## Assets and credits

The distractors come from the Hugging Face dataset [`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets),
pinned to revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`instrument_caliper_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/instrument_caliper_a01)
- [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time: masses set
(caliper 120 g, mug 300 g); the caliper's sliding jaw is welded to its beam (one rigid body). Files download at first run; nothing
large is committed.

- `simready.py` is the loader from `tasks/cutters-off-pegboard/` in this repository (another team's open-source task), unchanged
  apart from its docstring.
- `mechkit.py` is the mechanism kit from `tasks/heat-shrink-lug/` in this repository (another team's open-source kit), unchanged apart from a credit line in its docstring.
- Every other mesh is generated by `task.py`; there are no textures and no other third-party assets.
