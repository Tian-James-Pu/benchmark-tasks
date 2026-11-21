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
  -E task=tasks/focus-diode-laser-module-320mm/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Focus the laser module on the plywood sheet with its focus stick, and leave the engraver ready to cut." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/focus-diode-laser-module-320mm-dream
```

Scripted demo: the same command with `--policy yam_scripted --max-steps 1100 -E cam_width=640 -E cam_height=360`.
The run prints `task_score` at the end.

## Runtime and seeds

- Demo: 913 controls (45.65 seconds simulated), including the 2 second success hold. The demo video contains 914 synchronized frames with the top camera on the left and the right wrist camera on the right.
- Dream-1: 2,400 steps (2 minutes simulated). The recorded simulator runs took 3.1 minutes for the scripted demo and 5.5 minutes for Dream-1 on shared RTX GPUs.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

## The scene

| Object | Model | Real counterpart |
|---|---|---|
| Engraver frame, gantry beam | fixed boxes (2 cm aluminium profiles, 4 x 4 cm beam) | AlgoLaser DIY KIT MK3 frame (400 x 400 mm work area) |
| Honeycomb platform | fixed 37 x 37 x 2.4 cm body; 32 x 32 cm honeycomb work grid | AlgoLaser 320 mm honeycomb-platform variant |
| Plywood sheet | free box, 20 x 30 x 0.3 cm, 90 g, friction 0.5/0.4 | 3 mm basswood plywood |
| Laser module | prismatic link, 0.6 kg, 5 x 5 x 9 cm body over a 2.5 cm orange shield | the MK3's 8 W module |
| Thumbscrew | hinge link, knurled head 12 mm x 20 mm, seated at 0 deg | the carriage's knurled clamp screw |
| Focus stick | hinge link on the module, steel strip 3 x 6 mm, rounded end, 40 mm pivot to tip, pivot knob 12 mm | the module's built-in focus stick |
| Caliper, mug (distractors) | SimReady props, loose | a digital caliper and a mug on the bench behind the gantry, ignored by the scorer |

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
- **Score**: 1.0 while `|e| <= 0.5 mm`, clamped, stick within 15 deg of stowed, released and still have held for 40 steps (2 s);
  0.75 in focus, clamped and released with the stick not stowed (or before the 2 s); 0.5 for `0.5 < |e| <= 2.0 mm`, clamped and
  released; 0.25 in focus, released, not clamped (the module resting on its stick); 0 otherwise. Losing any full-credit condition resets the 2 s hold.

## Real-world trial

On 20 November 2026 the listed 8 W DIY KIT MK3 machine was US$219 and the 320 x 320 mm honeycomb-platform variant was US$59: US$278 before plywood, tax and shipping. The platform product page showed stock for the product; verify that the 320 mm size remains selectable when ordering. A ten-sheet 3 mm basswood pack was listed at US$39. Measurement tools are separate.

- AlgoLaser DIY KIT MK3, 8 W, machine only: frame, gantry, carriage with thumbscrew, module and built-in focus stick. It stays unplugged; the laser never fires. [Manufacturer listing](https://algolaser.com/products/algolaser-diy-kit-mk3-diode-laser-engraver).
- AlgoLaser honeycomb platform, 320 x 320 mm working area. The manufacturer lists 320 x 320 and 400 x 400 mm variants; a current 320 mm listing identifies the assembled body as 370 x 370 x 24 mm. Place it directly on the bench as in `task.json`, with the plywood flat on the grid. [Manufacturer platform listing](https://algolaser.com/products/algolaser-400mm-400mm-honeycomb-platform-engraving-machine-accessories); [320 mm product dimensions](https://www.geekbuying.com/item/AlgoLaser-ALHP2-0-Honeycomb-Laser-Bed-320-320mm-12-6-12-6-inch-526932.html).
- 3 mm basswood plywood, cut to 200 x 300 mm. The listed ten-sheet pack is US$39.
- Measurement: a 150 mm digital caliper, a luggage scale and a cord loop, masking tape, a paint pen.

Preparation and reset: mark the thumbscrew's head and the carriage with the paint pen with the screw seated (finger tight). Set the
module 12 mm above focus: hold it, loosen the screw, set the hanging stick on a 12 mm drill shank lying on the sheet, tighten, take
the shank away and stow the stick. Tape the sheet's corners and the gantry's position on the frame.

Measuring the outcome: `e` with the caliper's depth rod from the shield's lower edge to the sheet, minus the stick's reach below the
shield (measured once with the caliper); clamped: the paint marks within 45 deg, and a 10 N downward pull on the module (luggage
scale) moves it less than 0.2 mm (caliper before and after); stowed: the stick within 15 deg of upright; drop: an overhead or side
video at 240 fps, the shield's edge against a steel rule taped to the carriage, frame by frame; sheet: its corners against the tape.

## Checks run

The instrumented scripted trial scored 1.0 at 913 controls. The first stow sweep stopped at about 54 degrees from upright. The scripted plan opens the jaws, resets wrist roll in free space, then regrips the same knob to finish stowing. The stick reaches its upright stop and the scorer verifies release for two seconds.

| Control | Outcome | Score |
|---|---|---|
| nothing | No motion | 0 |
| hold | Keeps holding the module | 0 |
| unlocked | Leaves the clamp loose | 0.25 |
| nostow | Leaves the stick down | 0.75 |
| onearm | Turns the screw without supporting the module | 0 |
| careless | Unlocks before taking the module weight | 0 |

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
parts for robots. No occupation-size claim is made for this category.

- Robot parts cut from 3 mm sheet: the MeArm robot arm's repository ships "Laser cut (3mm) dxf files"
  ([MeArm on GitHub](https://github.com/MeArm/MeArm)).
- The product, its focus stick and price: AlgoLaser's DIY KIT MK3 page lists the 8 W machine at US$219 (store data read on
  20 November 2026) and shows the module with its stick: "With the built-in focus stick, just flip it down, touch your material,
  and lock it in." ([AlgoLaser DIY KIT MK3](https://algolaser.com/products/algolaser-diy-kit-mk3-diode-laser-engraver)). The same page's
  photo shows the knurled thumbscrew on the carriage beside the module; its sizes here are scaled from that photo (an estimate).
- The platform listing offers a 320 mm working-area variant for this layout; its 320 mm product dimensions are 370 x 370 x 24 mm. The MK3
  guide places flat 3 mm plywood in the work area and uses the included focus gauge.
  [AlgoLaser MK3 setup](https://algolaser.com/products/laser-engraved-fridge-magnets).
- The maker lists the 3 mm basswood ten-sheet pack at US$39. [AlgoLaser basswood sheets](https://algolaser.com/products/basswood-plywood-sheets-10pcs)
- The operation and the drop hazard, on a comparable open-frame machine (xTool D1 Pro): "Put the focal length setting bar down",
  "Loosen the thumb screw on the other side and slide the laser module upward or downward", "After determining the position, tighten
  the thumb screw and put the focal length setting bar back", and "Prevent the middle plate and laser module from falling down
  themselves. Otherwise, the limit pieces may be damaged." ([xTool D1 Pro Quick Start Guide](https://support.xtool.com/article/208)).
  That the same care applies to the MK3's module is an inference from the shared design (a free slide clamped by a thumbscrew).

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
- `mechkit.py` is the mechanism kit from `tasks/heat-shrink-lug/` in this repository (another team's open-source kit), unchanged.
- Every other mesh is generated by `task.py`; there are no textures and no other third-party assets.
