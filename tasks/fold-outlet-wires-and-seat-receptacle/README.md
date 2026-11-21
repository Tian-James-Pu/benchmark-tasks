# Fold a receptacle's stiff copper conductors into its wall box, seat it and drive both mounting screws

A short section of drywall on a 2x4 stud stands on the bench in front of the two arms. Set into it, flush with its
face, is an empty single-gang PVC wall box. A 15 A duplex receptacle hangs 12 cm out of the box on three solid
14 AWG copper conductors (black to its brass terminal, white to its silver terminal, bare to its green ground
screw) that come out of a cable entering the back of the box. A push-to-start cordless screwdriver lies on the bench
to the right; two pairs of pliers lie on the left.

The robot has to end with the receptacle flush on the wall, both of its mounting screws driven home into the box,
every conductor inside the box, and both hands off it.

The conductors are solid copper: they keep whatever bend they are given and push back. Pushed in by its face, the
receptacle stops short of the wall on a bundle of wire, its captive screws do not reach their holes, and it springs
back out when let go. One hand has to hold it pressed in, square and on the line of the box's screw holes, while the
other brings the driver's bit onto a screw head and pushes until the screw catches and draws the strap down. The
second screw goes in the same way once the first holds.

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: the scene, the scorer (`score`, `success`) and the scripted `demo_script` |
| `conductors.py` | The three conductors: chains of rigid links on two-axis bending joints with copper's elastic-plastic law |
| `fastening.py` | The push-to-start screwdriver's motor switch and the two mounting screws' threads |
| `closedloop.py` | Plays `demo_script` under the harness's own `yam_scripted` policy |
| `mechkit.py`, `demokit.py` | Mechanism and closed-loop-demo kits, unchanged (see Credits) |
| `simready.py` | Loads the SimReady props (see Assets) |
| `wire_start_points.csv` | Every conductor's starting path, joint by joint, in millimetres from the box opening's centre |
| `task.json` | The task's details for the board |

## Run a trial

Model run (an LLM through Inspect Robots, or Dream-1):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/fold-outlet-wires-and-seat-receptacle/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "INSTRUCTION" \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/outlet-dream
```

Scripted demo: the same command with `--policy yam_scripted -E cam_width=640 -E cam_height=360`. `closedloop.py`
makes `yam_scripted` play the task file's closed-loop `demo_script` (demokit's API) one step at a time.

The run prints `task_score` at the end. `OUTLET_LOG=<file>` writes, every 5 steps, the receptacle's pose, each screw's
strap gap and thread state, the driver's pose and spindle travel, and every conductor's points.

## Importance and sources

- **Occupation and size.** Electricians held about 821,000 jobs in the US in 2025
  ([BLS Occupational Outlook Handbook, Electricians](https://www.bls.gov/ooh/construction-and-extraction/electricians.htm),
  "Number of Jobs, 2025"; [O*NET 47-2111.00](https://www.onetonline.org/link/summary/47-2111.00) gives the same
  821,000). Their listed duties include "Install and maintain wiring, control, and lighting systems" and "Repair or
  replace wiring, equipment, or fixtures using handtools and power tools" (BLS, What Electricians Do).
- **This exact operation is core work.** O*NET's task statements for electricians include "Assemble, install, test,
  or maintain electrical or electronic wiring, equipment, appliances, apparatus, or fixtures" and "Fasten small metal
  or plastic boxes to walls to house electrical switches or outlets"
  ([O*NET 47-2111.00](https://www.onetonline.org/link/summary/47-2111.00)). Every receptacle installed or replaced
  ends with the step this task tests: the maker's instructions say "Fold the wires into the box, keeping the
  grounding wire away from the WHITE and HOT terminals. Screw the receptacle to the box and attach the faceplate"
  ([Leviton receptacle installation sheet PK-A3410](https://leviton.com/content/dam/leviton/digital-marketing/gfci-support-images/PK-A3410-10-00-2B-M-X1.pdf),
  "Complete the installation"). *Inference, not sourced:* a residential trim-out puts this step in front of an
  electrician dozens of times per house.
- **Why the wire fights back.** The National Electrical Code requires at least 6 in. of free conductor from where it
  enters the box, at least 3 in. of it outside the opening (NEC 300.14, as summarised by
  [EC&M Code Q&A](https://ecmweb.com/national-electrical-code/qa/article/21255990/code-qa-free-conductor-length-requirements)),
  so every device comes with 6 to 8 in. of solid copper to fold into a box 3-1/4 in. deep. Annealed copper yields
  at about 69 MPa (10 ksi) and has a modulus of about 117 GPa
  ([Aurubis C11000 data sheet](https://aurubis.com/dam/jcr:ec116d17-1541-419a-a897-ad31b4fa6c93/c11000-cu-etp-us.pdf)):
  it bends plastically and keeps its bend, so the bundle stands in the box like a spring.
- **The products.** Receptacle: a 15 A, 125 V duplex receptacle of the Leviton T5320-W class, 1.31 x 0.98 x 4.20 in.
  ([distributor listing](https://www.stoneway.com/LEVITON-WIRING-DEVICES-T5320-W-Leviton-T5320-W-Tamper-Resistant-Double-Pole-Straight-Blade-Duplex-Receptacle-Screw-Mount-125-Volt-15-Amp-White/p2297521)).
  Box: Carlon B120A one-gang nail-on PVC box, 20 cu in., 3-7/8 x 2-1/4 x 3-1/4 in.
  ([ABB catalogue](https://empower.abb.com/ecatalog/ec/EN_NA/p/B120A-UPC);
  [distributor listing with dimensions](https://www.CESCO.com/electrical-boxes/outlet-boxes/one-gang-nail-on-new-work-outlet-box-volume-20-cubic-inches-length-3-78-inches-width-2-14-inches-depth-3-14-inches-color-blue-material-pvc-mounting-means-captive-nails)).
  Driver: a Bosch GO class 3.6 V screwdriver, which starts when it is pushed against the screw ("simply push the
  screwdriver forward or press the power button to get started",
  [3DJake product page](https://www.3djake.com/bosch/go-cordless-screwdriver)); 360 rpm no-load, 5 N m maximum
  torque, 0.32 kg ([Bosch technical data](https://www.bosch-pt.co.in/manuals/professional/in/en/online-manual/200430718/en-GB/12502367499.html)).
  The mounting screws are the receptacle's own captive 6-32 screws (32 threads per inch, 0.79 mm per turn), on
  centres 3-9/32 in. (83.3 mm) apart as for every single-gang device.
- *Inference, not sourced:* the 14 AWG conductor is 1.63 mm across (the standard AWG size); its insulation is
  modelled 2.8 mm across.

## Distinct from

| Nearest task | What it tests | Why this one differs |
|---|---|---|
| TK-23JFGK, land a ferruled wire in a push-in terminal (accepted) | Pushing one ferrule straight into a terminal entry | No terminal is made here: the conductors are already landed. The skill is folding three stiff, plastically bending conductors into a box and fastening the device against their spring-back |
| PR 2596, strip a wire and push it into a receptacle (pending) | Wire stripping and back-wire termination | Termination is the step before this one; this task starts where it ends |
| PR 2838, cap a bare splice with a wire nut before closing a junction box (pending) | Twisting a wire nut on, then fitting a flat cover | No device, no folding, no screws driven into the box |
| Accepted driver tasks (for example the drawer knob, TK-VJ53VK) | Driving a screw with a powered bit | Here the screw can only catch once the receptacle is held flush against the wires, so one hand holds and aligns while the other drives |

LIBERO, RoboCasa, BEHAVIOR-1K, RoboDojo and RoboARC have no electrical-device installation task (checked against
their task lists; the RoboARC index covers knots, threading and interlocked parts, not stiff-wire folding).

## An independent implementation of a lapsed idea

The idea of folding a receptacle's stiff conductors into its box and seating it was first submitted by another team,
in PR #2405. Their hold on it lapsed without a revision after its last review. This folder is an independent implementation: its
scene, conductor model, screwdriver, scorer, demo and text were written from scratch. The last review of the original
asked for one thing, and today's review bar asks for more. How each point is met here:

| Point | How it is met |
|---|---|
| The last review: "Include the two mounting screws" | Both of the receptacle's captive 6-32 mounting screws are driven into the box's screw bosses with a push-to-start cordless screwdriver. Full credit needs both screws driven home and the strap flush (Scoring). |
| The earlier review: the 2 mm wire rule could not be checked behind the strap | The wire rule is now judged at the wall's face only, which a straightedge and a side photo check, and the strap must be flush within 1 mm at both screws, so no conductor can be under it unseen (a 14 AWG conductor is 2.8 mm thick with insulation). |
| The earlier review: starting folds and work-hardening were not reproducible | There are no pre-bent folds. Each trial starts with fresh, straight-cut conductor, shaped by hand to the starting arc from `wire_start_points.csv` (Real trial). |
| Stiff copper modelled faithfully | Chains of 12 mm links on two-axis joints with elastic stiffness from E = 117 GPa, yield from 69 MPa, linear hardening and a cap at 200 MPa (Fidelity); a bend stays bent. |
| Real product at real scale | Leviton T5320-W class receptacle, Carlon B120A box, 1/2 in. drywall on a 2x4 stud, 14 AWG conductors 194 mm long, a Bosch GO class driver (Importance and sources). |
| Goal-only instruction | The instruction names the end state, not the fold, the order or the holding. |
| Numeric scoring, partial credit, end state | Five outcomes with millimetre thresholds (Scoring). |
| Violation history only in `on_reset` | The only state carried across steps (the screws' threads, the copper's set, the success hold) is cleared in `Trial.reset`, registered with `mechkit.on_reset`; nothing is cleared because things look right again. |
| Released, checked with fingertips | Every `mechkit.fingertips` point at least 3 cm from the receptacle and from every conductor, and the driver's bit at least 2 cm from the receptacle, for the whole 1 s hold. |
| SimReady props, 2-4 distractors | The two pliers are SimReady props and are ignored by the scorer (Assets). |

## Scoring

Judged on the end state, at every control step, from the receptacle's pose, 49 points along each conductor (three per
link), the two screws' thread state, the driver's bit and the grippers' fingertip points (`mechkit.fingertips`).

A step is **in place** when all of these hold:

- **every conductor inside the box:** no point of any conductor's centreline more than 1.0 mm in front of the wall's
  face (a conductor is 2.8 mm thick, so one lying on the wall, on the box's rim or under the strap fails);
- **let go:** every fingertip point of both grippers at least 3 cm from the receptacle (strap, face and body, sampled)
  and from every conductor point, and the driver's bit at least 2 cm from the receptacle.

Then:

| Outcome | Score |
|---|---|
| Not in place (a conductor showing, pinched or lying on the wall; or a hand or the driver still on the receptacle), or no screw driven home | 0 |
| In place, one mounting screw driven home | 0.25 |
| In place, both screws driven home, but the strap more than 1.0 mm off the wall at either screw, or the face more than 3 degrees off square | 0.5 |
| In place, both screws home, the strap within 1.0 mm of the wall at both screws and square within 3 degrees, the receptacle moving under 5 mm/s, all held for 1 s | 1.0 |

A screw is **driven home** once its thread has drawn the strap down to the wall at that screw (Fidelity). Screws
cannot be backed out, so "home" stays true; every other condition is read afresh at each step. The trial ends with
success once the 1.0 row has held for 1 s; otherwise the score is the last step's row. Doing nothing scores 0 (no
screw is home).

### Checking a trial on a real rig

Read before anyone touches the receptacle.

- **Flush:** a 1.0 mm feeler gauge (or a 1 mm shim) must not slide under the strap's ear at either screw.
- **Square:** a small square against the wall and the face; 3 degrees is 3.5 mm over the face's 67 mm.
- **Screws home:** each screw's head is down on the strap and the strap is flush beside it; then, after the photos, a
  #2 screwdriver turned with two fingers does not advance it.
- **Conductors inside:** a straightedge laid on the wall across the opening on each side of the receptacle touches no
  conductor, and a photo along the wall's face from each side shows no conductor in front of the wall. With the strap
  flush, nothing can hide under it.
- **Let go:** a frame of the video from the last second shows both grippers at least 3 cm from the receptacle and the
  bit at least 2 cm from it.

## Real trial: setup, preparation and reset

All dimensions are from the box opening's centre on the wall's face unless stated. The arms' frame: x away from the
robot, y to its left, z up; the table top is z 0.

- **Wall:** a piece of 1/2 in. (12.7 mm) gypsum board 26 cm wide and 20 cm tall, standing on the table, its face
  32 cm in front of the arm bases, from 17 cm right of the bases' midline to 9 cm left of it; screwed to a 2x4 stud
  (38 x 89 mm) on its back, right of the box, on a 2x4 plate lying on the table. Cost: offcuts.
- **Box:** Carlon B120A (20 cu in., blue PVC, 57 x 98 x 83 mm outside, 3 mm walls), nailed to the stud with its rim
  flush with the wall's face, opening centred 4 cm right of the midline and 11 cm above the table. Its two screw
  bosses are on the opening's vertical centre line, 83.3 mm apart. About $1.
- **Cable and conductors:** a stub of 14/2 NM-B cable (yellow sheath) entering the bottom back of the box, its sheath
  ending 16 mm in front of the box's back wall and 9.5 mm above its floor. Free conductors: black and white
  (THHN-insulated, 2.8 mm across) 194 mm from the sheath to their terminals, bare 195 mm. Fresh conductor for every
  trial: cut the cable stub new (about $0.50), strip 16 mm, and land the black on the upper brass screw, the white on
  the upper silver screw and the bare on the green screw, each clockwise under its screw.
- **Receptacle:** a Leviton T5320-W class 15 A duplex receptacle (white, 33 x 25 x 107 mm, 45 g), its two captive
  6-32 x 1 in. mounting screws in its strap (standing 17 mm out behind it). It is never connected to power.
- **Starting shape:** with the conductors landed, hold the receptacle upright, face to the robot, its strap 8.0 cm in
  front of the wall and its centre 5 mm above the opening's centre, and shape the black and white conductors by hand
  into the arc of `wire_start_points.csv` (rising from the clamp to about 3.7 cm above the opening's centre and
  down to the terminals); the bare follows below them. Let go: the receptacle hangs on its conductors. Check with a
  ruler: strap 8.0 +- 0.5 cm out, centre within 5 mm of the drawing. In the simulator the starting pose is the one the
  copper settles to before the first step (the receptacle sinks a few millimetres as the copper takes a set).
- **Driver:** a Bosch GO class push-to-start screwdriver (190 mm, 0.31 kg) with a #2 Phillips insert bit, clutch at
  its middle setting, lying on its side on the table pointing at the wall, centred 16.5 cm in front of the bases and
  12.5 cm left of the midline.
- **Distractors:** a lineman's pliers (19 cm, 0.32 kg) centred 10 cm out and 21.5 cm left of the midline, and a
  long-nose pliers (23.5 cm, 0.18 kg) centred 12 cm out and 30 cm left, both lying flat, handles toward the robot.
- **Reset (about 5 minutes):** back both screws out, pull the receptacle, cut the conductors at the sheath, strip
  the cable back and land fresh conductor as above. Used conductor is never reused, so the copper starts annealed
  (no work hardening) every trial.
- **Rig safety:** the cable stub is not connected to anything; the strap's edges are deburred and the gripper pads
  touch only the receptacle's plastic sides, the driver's housing and the conductors' insulation.

## Fidelity

- **Conductors** (`conductors.py`): each a chain of 16 rigid links about 12 mm long on two-axis bending joints (no
  twist, no stretch). Each joint has an elastic stiffness E I / L from E = 117 GPa for the 1.63 mm copper, a yield
  moment sigma_y d^3 / 6 from 69 MPa, and, once it yields, a permanent set (its rest angle moves to where it was bent,
  less the elastic springback), with linear hardening of 0.03 N m per radian of plastic rotation up to the moment at
  200 MPa. The hardening rate and cap are estimates from annealed copper's stress-strain range, not measurements. The
  insulation adds no stiffness. The conductors collide with the box, the receptacle and each other.
- **Screws** (`fastening.py`): the driver's spindle is a sprung slider (2,000 N/m, 4 mm travel); pushed in past
  1.5 mm (about 3 N) its motor runs, at 5 turns a second. A running bit within 1.5 mm of a screw head's axis, against
  the head and within 12 degrees of in line turns that screw. A turning screw whose pointed tip is inside its boss's
  hole catches; from then on its thread allows the strap no further from the wall than where it caught less 0.79 mm
  per turn, pulling the strap in (a stiff spring capped at 60 N); at zero the screw is home and holds the strap with
  15 N. Thread engagement is computed, not contact-resolved; a screw cannot be backed out.
- **Receptacle, box, wall:** true size, rigid, primitive meshes; the screw bosses are blocks with a square hole
  5.2 mm wide for the 3.5 mm shank. Uniform friction 0.6 (box, wall), 1.0 (driver).
- **Grip:** the pads hold everything by friction; nothing is attached by rule.

## Assets

The two pliers are SimReady props from the Hugging Face dataset
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`handTool_pliers_combination_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_pliers_combination_a01)
- [`handTool_pliers_long_nose_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_pliers_long_nose_a01)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time: the
pliers' hinge is removed and the handles become one rigid body; masses are set (lineman's 0.32 kg, long-nose
0.18 kg). The files are not committed: they download on first use.

The receptacle, box, wall, conductors and driver are built from primitive meshes in the task file, because they need
working joints (the conductors' bends, the driver's spindle) or exact screw geometry that no SimReady prop has. The
SimReady library has no receptacle, single-gang box or inline cordless screwdriver.

## Credits

`mechkit.py` and `demokit.py` are taken unchanged from another team's open-source task in this repository,
`tasks/heat-shrink-lug/`, published under the MIT licence (`mechkit.py` carries one added credit line).
`simready.py` is adapted from `tasks/cutters-off-pegboard/` (loader unchanged). `conductors.py`, `fastening.py` and
`closedloop.py` were written for this task. Leviton, Carlon and Bosch are trademarks of their owners, named only to
identify the products the scene is sized from.

PLACEHOLDER_REST
