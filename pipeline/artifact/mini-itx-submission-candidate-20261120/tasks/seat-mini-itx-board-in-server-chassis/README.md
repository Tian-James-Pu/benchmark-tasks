# Seat a Mini-ITX server board in a 1U chassis: on all four standoffs, its D-sub ports through the I/O shield cut-outs

**Instruction:** "Install the motherboard in the server chassis, seated on its standoffs ready for its screws."

**Category:** Impact on AI timelines (building and servicing data-centre and edge servers with the real parts).

A full-size Mini-ITX server board (modelled on Supermicro's X11SCL-iF) lies on anti-static foam in front of an open
1U Mini-ITX server chassis (modelled on Supermicro's SC505-203B). The chassis has its four board standoffs on the
Mini-ITX hole pattern, its power supply beside the board bay, and a full-height rear panel with the board's I/O
shield. The robot lays the board in the chassis so that all four mounting holes sit on their standoffs, ready for
the screws. The screws themselves are outside the trial.

## Why it is a manipulation problem

The board's VGA and COM connectors are D-sub shells that stand 6 mm proud of their flanges and pass through
cut-outs in the I/O shield that are only 1.0 mm larger all round. The rear panel is full height (43 mm), so the board
cannot simply be lowered onto its place. Lowered straight down, the shells land on the shield's lower lip and the
board stays propped on it at about 11 degrees, with the rear holes 25 to 29 mm above their standoffs (red team
"careless", below). The board has to come in a little forward of its place, or tilted with its I/O edge first. It
then goes rearward, level and at standoff height, until the shells enter their cut-outs and the connector faces meet
the shield. Only then do the four 3.96 mm holes lie over the four standoffs. The board is 170 x 170 mm and 0.5 kg,
and its only graspable features are the PCIe slot, the memory modules and the board edges. Two hands, one on each
side of the centre of mass, keep it level while it is carried over the chassis wall and moved those last few
millimetres.

## Importance and sources

- **The real operation.** Supermicro's SC505 chassis manual, section 5-5 "Installing the Motherboard"
  ([SC505.pdf, page 5-6](https://www.supermicro.com/manuals/chassis/1U/SC505.pdf)): "3. Compare the mounting holes
  in the motherboard to those in the chassis then add or remove standoffs as needed. 4. Lay the motherboard in the
  chassis, aligning it with the standoffs. 5. Secure the motherboard to the chassis using the rounded, Phillips head
  screws." The same page says the SC505 "includes permanent and removable standoffs in locations used by Mini-ITX
  6.75" x 6.75 size motherboards". This task is step 4.
- **The chassis.** [Supermicro SC505-203B product page](https://www.supermicro.com/en/products/chassis/1u/505/sc505-203b):
  1U, 437 x 43 x 249 mm (W x H x D), "Optimized for Mini-ITX (6.75" x 6.75") Motherboard", with a 200 W PWS-203-1H
  power supply of 192 x 76 x 40.3 mm. Its rear I/O lists VGA, IPMI, LAN, COM and USB. The manual's Figure 5-9
  ([page 5-10](https://www.supermicro.com/manuals/chassis/1U/SC505.pdf)) shows the power supply at the rear, beside
  the board, with its long side along the chassis depth. *Inference:* the 34 mm gap between the board bay and the power
  supply, and the bay's distance from the left wall, are estimates from that drawing, not measured.
- **The board.** [Supermicro X11SCL-iF product page](https://www.supermicro.com/en/products/motherboard/X11SCL-iF):
  Mini-ITX, 6.7" x 6.7" (17.02 x 17.02 cm), Xeon E-2100/E-2200, two DDR4 UDIMM slots, one PCIe 3.0 x16 slot, VGA and
  COM ports, dual 1 GbE plus a dedicated IPMI LAN. The [X11SCL-iF quick reference guide (QRG-2088)](https://www.supermicro.com/QuickRefs/motherboard/X11/QRG-2088.pdf)
  shows the layout (VGA and COM1 on the rear edge, DIMMA1/DIMMB1, the 24-pin JPW1). *Inference:* component positions
  in `task.py` are read off that layout drawing and are approximate to a few millimetres.
- **Hole pattern, standoff height, connector line.** ATX Specification 2.2 ([copy of the formfactors.org PDF](https://xdevs.com/doc/_PC_HW/Form_factors/atx2_2.pdf)):
  section 3.2 and Figure 3 give the mounting holes, which Mini-ITX boards share (holes C, F, H, J). Section 3.3.5 says
  "the face of all I/O connectors should be placed 0.445" (11.30mm) from the reference datum", and that a typical board
  is 0.062" (1.57 mm) thick. The keep-out notes require "≥ 0.250" clearance to the bottom planar surface of the
  motherboard PCB". Intel's [Thin Mini-ITX system design guide, rev 1.2, section 6.1](https://www.intel.com/content/dam/www/public/us/en/documents/product-specifications/thin-mini-itx-based-pc-system-design-guide-rev-1-2.pdf)
  repeats the "0.25 inch high motherboard backside standoff height as defined in the ATX Specification Version 2.2".
  The standoffs are therefore 6.35 mm tall. *Estimates, not from a source:* the 3.96 mm (0.156 in) hole diameter,
  the D-sub shell size and 6 mm protrusion, and the 1.0 mm cut-out clearance.
- **Who does it.** O*NET 49-2011.00, Computer, Automated Teller, and Office Machine Repairers
  ([summary](https://www.onetonline.org/link/summary/49-2011.00)), lists the task statements "Update existing
  equipment, performing tasks such as installing updated circuit boards or additional memory" and "Reassemble machines
  after making repairs or replacing parts", with 72,100 employed (2025). The BLS
  [OEWS May 2023 profile](https://www.bls.gov/oes/2023/May/oes492011.htm) gives 77,580. These counts are well below the
  Impact-on-jobs bar, and that category is not claimed. The task is filed under Impact on AI timelines because
  assembling and repairing servers is part of building and servicing AI compute. *Inference:* boards are also laid
  into chassis by server integrators and data-centre technicians, who have no single SOC code. No number is claimed
  for them.

## Independent implementation of a lapsed idea

This is an independent implementation of an idea first submitted by another team in PR #2747 ("Seat a Mini-ITX
server board on four locating standoffs"). That PR was reviewed "Not accepted" on 2026-11-19 at 02:00 UTC and no
revision followed within the 48-hour hold. Nothing from it is reused: the scene, mechanism, scorer, demo and text
here are our own. Each point of that review is addressed as follows:

| Review point | How this task meets it |
|---|---|
| "Four projecting locating pins and an open plate replace the named server chassis and its mounting arrangement" | The scene is a 1U Mini-ITX server chassis at the SC505-203B's size: floor tray, side and front walls, a full-height rear panel with the I/O shield, the power supply block beside the bay, and four plain-topped 6-32 hex standoffs 6.35 mm tall on the Mini-ITX C/F/H/J holes. There are no pins, matching the real chassis, whose standoffs take the screws. |
| "Use a representative server chassis, its actual standoff arrangement and rear-I/O opening, so the robot performs the installation in its real approach constraints" | The full-height rear panel and the shield's D-sub cut-outs are what constrain the approach: the board cannot be lowered straight down (red team "careless" scores 0). It has to come in short of its place, or tilted, and go rearward until the shells pass the shield. The chassis wall, 43 mm tall, also has to be cleared. |
| "Make the 0.5 score require the stated four-pin engagement, including vertical engagement" | 0.5 needs all four holes engaged. Each must be concentric with its standoff within 0.6 mm, so the standoff's tapped bore shows whole through the 3.96 mm hole, and must rest on it: the gap is at most 0.3 mm, so a 0.3 mm feeler gauge does not enter. The earlier 2 mm horizontal allowance is gone. A board held, or propped, even 0.6 mm above the standoffs (a shim's height) has no engaged hole. |
| "Add tests for misaligned and unengaged cases" | `test_scoring.py` runs offline and checks eleven cases: seated; 0.5 mm off; 0.8 mm and 2 mm misaligned; set down 7 mm forward on the standoff tops; 0.25, 0.6 and 5 mm above; rotated 0.5 degrees; tilted 0.3 degrees; and the start pose. Red-team trials in the simulator cover the rest (below). |
| "List the reused scene and its license in the README" | No scene is reused. The chassis, board and standoffs are our own primitives in `task.py`; `mechkit.py` and `simready.py` are shared helper modules (credited below); the distractors are SimReady props, listed with their licence below. |
| "Rerun Dream-1 and provide a full-score demo for the revised scene within the action limit" | The scripted demo scores 1.0 at about step 400 of 2,400, and it did so in two consecutive runs. The Dream-1 run is made with the submission and its score is reported in `task.json`. |

## Distinct from

| Nearest task | What it tests | Why this differs |
|---|---|---|
| Fit the air shroud in a rack server (#2055, revising) | tuck a shroud's tabs under a fan cage, then rotate it down to seat | here a 0.5 kg board is carried level over a wall, set down, and translated at standoff height so connector shells thread shield cut-outs; the end state is four holes concentric on standoffs |
| Refit a 2U server's fan assembly on hidden guide pins (#2836, pending) | lower a module onto guide pins, then cam levers | no pins or levers here; the locating feature is the I/O shield, reached by a horizontal move after set-down |
| CPU cooler installation (#2393, accepted) | place a cooler on a socket with a thermal-paste rule | different part, a top-down placement with no horizontal approach constraint |

The earlier Mini-ITX test-bench submissions (#1349, #2029) had no chassis or rear panel.

## Scene (all full size)

Coordinates are in cm from the midpoint between the arm bases (x forward, y left), and heights are above the table
top (0.7475 m).

- **Chassis** (fixed, steel, 1 mm sheet). Tray x 23.2 to 48.1, y -30.6 to 13.1; walls 4.3 tall. The rear panel's
  inner face is at x 48.0. The I/O shield spans y -8.45 to 7.0, with two cut-outs for the D-sub shells: COM at y -7.34
  to -5.43 and VGA at y 3.63 to 5.51, both 1.00 to 2.04 high. It also has openings, 0.5 mm clear all round, for the IPMI LAN (y -4.81 to -2.75), USB
  (-2.70 to -1.16) and dual-LAN (-0.99 to 0.82) stacks, from 1.00 high to 3.54, 2.54 and 3.54; those stacks' faces sit
  flush with the shield's inner face, so only the D-sub shells pass through it.
- **Standoffs** (fixed, brass, 6-32 hex 4.8 mm across, 6.35 mm tall, tops 0.74 high). C (46.87, 7.87), F (44.58,
  -7.88), H (31.38, 7.87), J (31.38, -7.88).
- **Power supply** (fixed, steel block as the PWS-203-1H, 19.2 x 7.6 x 4.03). x 28.8 to 48.0, y -19.5 to -11.9.
- **Board** (loose, 0.50 kg, estimated, with two DDR4 UDIMMs fitted). 170 x 170 x 1.57 mm FR-4 with: a VGA and a COM
  D-sub (bodies 12 mm deep, 12.5 mm tall; shells 16.9 x 8.4 mm standing 6 mm proud of the connector faces); USB, dual
  LAN and IPMI LAN stacks (16 to 26 mm tall); a PCIe x16 slot (89 x 7.2 x 11.2 mm); the two memory modules (32 mm
  tall); the 24-pin power connector; the CPU socket and PCH. It has four 3.96 mm mounting holes. It starts flat,
  I/O edge toward the chassis (+x), centred at (14.2, 0), on a pink anti-static foam sheet 18 x 20 x 0.6 (fixed). When
  seated its centre is at (39.39, 0) and the connector faces touch the shield's inner face.
- **Distractors** (loose, SimReady, ignored by scoring). PH2 Phillips screwdriver (18.8 cm, 100 g) at (12, -20); a
  roll of clear tape (10 cm across, 60 g) at (13, 21); a ceramic mug (300 g) at (42, 23).

## Scoring

Everything is read from the board's pose and velocity, and from points on both grippers, at each control step. The
score reported is the last step's, and a run ends early once 1.0 has been reached.

- **Engaged hole** (each of C, F, H, J): the hole's centre is within 0.6 mm horizontally of its standoff's axis, and
  the board's underside at that hole is between 0.5 mm into and 0.3 mm above the standoff top. The 0.5 mm is the
  solver's contact allowance; a board resting on a standoff reads 0.0 mm in the simulator.
- **Dropped**: the board's downward speed ever exceeds 0.5 m/s (about 13 mm of free fall). This is recorded across the
  whole trial and cleared only by a scene reset, in the `on_reset` hook. It is never cleared by object positions.
- **1.0**: all four holes engaged, both grippers at least 3 cm from the board's outline, and the board still (under
  5 mm/s and 0.05 rad/s) for 20 consecutive steps (1 s), with no drop.
- **0.5**: all four holes engaged, with no drop. The board may still be held.
- **0.25**: not all four engaged, but the board in the bay: centre within 20 mm of its seated place, tilt at most 5
  degrees, every hole within 3 mm of its standoff top's height, and no drop. This covers a board set down on the
  standoffs short of home, or one slightly misaligned or rotated.
- **0**: anything else. That includes any dropped board, a board propped on the shield lip, a board still on the foam
  or in the air, and a board resting anywhere else.

Possible scores: 0, 0.25, 0.5, 1.0.

**On a real chassis:** use a photo straight down each mounting hole, in which the standoff's tapped bore shows whole
inside the hole for engagement; a 0.3 mm feeler gauge at each standoff; a ruler for the 20 mm bay test; a phone
inclinometer for tilt; and the video for the drop and the 1 s hands-off.

## Scripted demo

`demo()` is a fixed plan from the published starting layout. It reads no simulator state while running. The left
hand pinches the PCIe slot and the right hand the two memory modules, both across the board's centre-of-mass line.
They lift the board level, carry it over the chassis wall, lower it into the bay 7 mm forward of its place, and set
it flat on the standoff tops. They then slide it rearward at standoff height until the D-sub shells are through the
cut-outs and the connector faces meet the shield, let go, and move away. The plan includes small offsets measured
once in the simulator for the arms' lag under the board's weight, which are declared in `task.py`. With the board
released, all four holes measured 0.02 mm from their standoff axes and 0.0 mm gap.

## Red team (scripted variants of the demo, run in the simulator)

| Variant | What it does | Score |
|---|---|---|
| none | arms stay up | 0 |
| one arm | the demo's plan with the right hand alone, on the memory modules: the board pitched 4 to 5 degrees as it was carried and came to rest 14 to 16 mm short of its place, tilted 3.2 degrees, with one corner down off its standoff | 0 |
| careless | straight down onto the place, as if there were no rear panel: the shells land on the shield's lip and the board stays propped at about 11 degrees | 0 |
| forward | set down flat on the standoff tops 7 mm short and let go, never slid home: in the bay, no hole engaged | 0.25 |
| hold | the demo, but the hands keep holding the seated board: all four engaged, grippers not clear | 0.5 |
| drop | let go over the bay at carrying height (board underside 63 mm above the table): drop recorded | 0 |

We also tried a "slide it in held high" plan, aiming 6 mm above the standoffs. The arms could not hold the board
that high: its rear edge settled onto the rear standoffs and the plan became the demo's route, reaching 1.0 with the
holes 0.2 to 0.42 mm off-axis. Going in tilted or high, then seating, is a valid route and is not penalised.

## Run a trial

Requires the YAM Isaac Lab Arena simulator (`inspect-robots`). From the repository root:

    # scripted demo (full score)
    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=tasks/seat-mini-itx-board-in-server-chassis/task.py -E object_jitter=0 -E control_hz=20 \
      -E cam_width=640 -E cam_height=360 \
      --instruction "Install the motherboard in the server chassis, seated on its standoffs ready for its screws." \
      --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/seat-mini-itx-board/demo

    # Dream-1 (the evaluated model)
    inspect-robots run --embodiment yam_arena --policy dream \
      -E task=tasks/seat-mini-itx-board-in-server-chassis/task.py -E object_jitter=0 -E control_hz=20 \
      -E cam_width=320 -E cam_height=176 \
      --instruction "Install the motherboard in the server chassis, seated on its standoffs ready for its screws." \
      --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/seat-mini-itx-board/dream

    # offline scorer checks (no simulator)
    python tasks/seat-mini-itx-board-in-server-chassis/test_scoring.py

The demo reaches 1.0 at about step 400 (20 s of simulated time). On one L40S-class GPU a demo trial takes 1 to 4
minutes of wall time once shaders are compiled. The layout is fixed (`object_jitter=0`), so there is no seed; the
demo is deterministic up to solver noise.

## Real-world setup, reset and measurement

- **Hardware.** A Supermicro CSE-505-203B chassis with its lid off, and an X11SCL-iF board (an unpowered, retired
  unit) with two UDIMMs fitted. Any 1U Mini-ITX chassis with a full-height rear panel and the board's own I/O shield
  reproduces the constraint. The chassis is clamped to the table at the stated position and the foam is taped down.
- **Protection.** Unpowered hardware only. Pad the grippers with ESD-safe foam, and ground the chassis and the foam
  through a wrist-strap lead. In the demo the hands keep their grip while the board slides the last 7 mm on the
  standoff tops, so the standoffs bear only part of its 0.5 kg; on real hardware, check the solder side for scratches
  after each trial.
- **Reset** (under 2 minutes). Lift the board out by the PCIe slot and the memory modules, and lay it flat on the foam
  at its tape marks, I/O edge toward the chassis. Check that the shield's cut-outs are not bent. Nothing is consumed.
- **Measurement.** Before touching anything, photograph straight down each hole, slide a 0.3 mm feeler gauge at each
  standoff, and measure the board's position from the chassis edge with a ruler.

## Fidelity

The parts are at real size, with the real hole pattern, standoff height and connector line. The chassis, standoffs,
power supply and board are custom rigid primitives, because SimReady has no server parts. The board is one rigid body
with an estimated mass and centre of mass. Its D-sub shells, port stacks, slots and modules are boxes, and small
headers and capacitors are not modelled. The USB and LAN stacks are drawn flush with the shield, so only the D-sub
shells pass through it. The standoff threads, the screws, the shield's grounding fingers (which in a
real chassis press lightly on the port shells) and electrostatic effects are not simulated. Friction is 0.6 static and
0.5 dynamic on the steel and the board, and 1.0/0.9 on the foam. The optional chassis fans and the drive bracket are
not fitted.

## Assets and licences

- **Distractors**, from the Hugging Face dataset [`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets),
  pinned to revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, © NVIDIA, CC-BY 4.0:
  - [`handTool_screwdriver_phillips_ph2_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_screwdriver_phillips_ph2_a01) (mass set to 100 g, convex collider);
  - [`tape_clear_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/tape_clear_a01) (mass set to 60 g);
  - [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01) (mass set to 300 g).

  The files are downloaded at first run into `~/.cache/simready`; none are committed.
- **Everything else** (chassis, standoffs, power supply, I/O shield, board, foam) is built from primitives in
  `task.py`. No third-party scene or CAD is used.
- `mechkit.py` (mechanism, state and fingertip helpers) and `simready.py` (the SimReady loader) are shared helper
  modules used unchanged by many tasks in this repository.

## Files

- `task.py`: the scene, the scorer (`Trial`, `board_check`) and the scripted demo.
- `task.json`: the submission fields.
- `test_scoring.py`: offline checks of the engagement and bay tests.
- `mechkit.py`, `simready.py`: helper modules (see above).
