# Bend a 45 degree offset in 1/4 in soft copper tube on a RIDGID 604 lever bender

An offset steps a tube line sideways round an obstacle and carries it on parallel: two opposite 45 degree bends a measured
distance apart. On a lever bender the tube is laid in the form's groove under the tube latch, slid until its first mark sits on
the carriage's 45 degree line, and bent by swinging the carriage handle round the form. For the second bend the carriage is swung
back off the tube, the tube is slid along the groove to its second mark and turned half a turn about its own axis, so that the
second bend goes the other way, and bent again. Copper cannot be unbent: a bend in the wrong place, past its angle, or the
same way as the first (a U instead of an offset) spoils the tube.

Instruction: "Bend a 45 degree offset in the copper tube, with its two bends centred on the black marks."

**The knack, with numbers.** Each bend must be centred within 5 mm of its mark, so the tube has to be slid 25 mm and then
70.7 mm along the groove to within a few millimetres, and the second bend needs the tube turned 180 degrees (within 15) about its
axis between the bends. Each pull meets the tube's plastic bending moment, about 2.5 N m, 8.5 N at the cushion grip, and the
tube springs back 1 degree when the lever is let go, so the lever has to be taken about 1 degree past 45. The roles are
different for each arm: the right hand slides the tube through the latch, the left pulls the carriage handle and turns the tube
by its bent leg. Skipping the turn makes a U (score 0); skipping the measuring puts a well-shaped offset in the wrong place
(score 0.5).

## Files

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: the bender and tube, the bending model, the scorer (`score`, `success`) and the scripted `demo()` |
| `check_scorer.py` | Checks every score line on hand-built tube shapes, with no simulator: `python check_scorer.py` |
| `mechkit.py` | Mechanisms written as plain USD (hinges, sliders, effort-capped drives, fingertip points), unchanged from another team's open-source task in this repository (`tasks/heat-shrink-lug/`) |
| `simready.py` | Loads the NVIDIA SimReady props, unchanged from another team's open-source task in this repository (`tasks/cutters-off-pegboard/`) |
| `task.json` | The task's details for the board |

## Run a trial

From the repository root, with `yam-isaac-lab-arena` installed (`source ~/IsaacLab-Arena/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream -E task=tasks/bend-45-offset-on-hand-tube-bender/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Bend a 45 degree offset in the copper tube, with its two bends centred on the black marks." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/offset
```

The score is printed as `task_score`. The scripted demo is the same command with `--policy yam_scripted -E cam_width=640
-E cam_height=360 --max-steps 1240`; it scores 1.0 (two runs, the same result: bend B -45.7 degrees centred 1.7 mm from mark B,
bend A 45.3 degrees centred 0.1 mm from mark A, ends parallel within 0.4 degrees).

- **Runtime per trial:** the demo ends at about 57 s of simulated time (1,145 steps). Each run took 6 to 10 minutes of wall time
  on a shared RTX GPU; the articulation has 43 links.
- **Seeds:** none needed. The start is fixed and `object_jitter=0`, so runs are deterministic. That says nothing about robustness.
- **The scripted demo** is open loop, a fixed plan from the published starting layout. It reads no simulator state. Four
  constants were measured in earlier runs of the same start and are marked in `task.py`: how far the arm lags the lever under
  load (4 degrees), how far the tube lags the hand in each slide (1.0 and 3.9 mm), and how far the turned tube overran (7.7
  degrees).
- `python check_scorer.py` checks every score line on hand-built tube shapes (no simulator).

## Scoring

Everything is read on the tube where it lies in the bender, once the tube and lever have been still for 1 s with both grippers
3 cm or more from the tube and from both handles. Until then the score is 0. "Still" means no hinge, slide, turn or lever moving
more than 0.3 degrees (0.3 mm) over 0.3 s. A bend is a run of the tube's hinges carrying more than 0.3 degrees each, the same way;
its angle is their sum and its middle is their angle-weighted centre along the tube, which on the real tube is halfway along the
curved part.

| Score | Condition |
|---|---|
| 1.0 | Exactly two bends, opposite ways, each 42 to 48 degrees; the straight ends parallel within 4 degrees; each bend's middle within 5 mm of its mark (the bend nearer E2 to mark B); the offset between the straight ends' centre lines 50.5 +/- 4 mm |
| 0.75 | Two opposite bends, each 38 to 52 degrees, ends parallel within 8 degrees, each middle within 12 mm of its mark |
| 0.5 | Two opposite bends, each 38 to 52 degrees, ends parallel within 8 degrees (anywhere) |
| 0.25 | Exactly one bend, 42 to 48 degrees, its middle within 5 mm of a mark |
| 0 | Anything else: no bend, two bends the same way (a U), three or more bends; and for the rest of the trial, once it has happened: a bend set past 55 degrees, a bend made with the tube turned more than 15 degrees out of its marked plane, or a bend reaching more than 4 cm beyond the marks |

Possible scores: 0, 0.25, 0.5, 0.75, 1.0. The three "ever" outcomes are kept in a record that only a reset clears (mechkit's
`on_reset` hook); nothing else in the scene clears it. Harmless handling (lifting the tube's free end, sliding it back and forth,
turning it either way, swinging the lever back) costs nothing. The offset 50.5 mm is what 70.7 mm between bend middles gives
with two 45 degree bends of this radius; the tolerances are this task's own, not a maker's.

**Measuring a real trial:** a protractor between the straight lengths for each angle; a ruler along the tube for each bend's
curve (mark both tangent points with a square, then the mark should lie within 5 mm of halfway between them); a straightedge
along each end and a tape between them for the offset and parallelism. Read with the tube still in the bender, then lift the
latch and take it out.

## Red team (run on the simulator, scripted the same way)

| Variant | What it does | Score |
|---|---|---|
| none | Nothing | 0 |
| no_roll | Slides to mark B without turning the tube: both bends the same way, a U (bends +45.3 and +45.7 at their marks) | 0 |
| off_marks | Skips the first measuring slide: two well-shaped opposite bends (-45.7 and +45.2 degrees, ends parallel within 0.5) whose middles are 43 mm and 25 mm from their marks (the last review's case: a good offset in the wrong place) | 0.5 |
| hold | The full demo (a correct offset at both marks), but the left hand stays on the carriage handle at the end, 1.6 cm from it | 0 |
| one_arm | The left arm alone: bends where the tube lies (never slid to its mark), turns the tube by its bent leg, pulls again. The turned tube rests 20 degrees out of its plane, so the second pull is an out-of-plane bend; the tube is spoiled | 0 |

These are the shortcuts the reviews named or the task invites: skipping the turn (a U), skipping the measuring (a good offset in
the wrong place, the last review's case), and leaving a hand on the tool. Each variant is a file in this folder
(`redteam_*.py`, the demo with one change), run with `TASK_FILE=redteam_<name>.py` in place of `task.py`.

## The real setup and reset

- **Bender:** a RIDGID 604 lever tube bender (catalogue no. 38033; 1/4 in OD tube, 5/8 in bend radius, 0.76 kg), its vise clamp
  block held in a bench vise, the bend plane level and the carriage handle on the side away from the vise's screw. The
  simulation puts the form's axis at (31, 0) cm (x forward from the arm bases, y left) and the tube's centre line 13.4 cm above
  the bench.
- **Tube:** 1/4 in OD x 0.030 in wall soft (annealed) copper refrigeration tube to ASTM B280, cut 36 cm from a coil, straightened
  along a straightedge, and marked all round with a fine black marker 15.0 cm (mark B) and 22.07 cm (mark A) from one end (E2).
- **Start:** carriage handle swung 15 degrees back from the form's 0 line (rollers off the tube); tube in the groove on the form's
  far side under the shut latch, E2 on the form handle's side, mark A 2.5 cm short of the carriage's 45 degree line.
- **Distractors** (ignored by scoring): a steel speed square at (62, -2), combination pliers at (56, 30) and a slotted
  screwdriver at (58, -28), all on the bench.
- **Reset:** a bent tube is consumed, so each trial takes a new marked length; swing the carriage back, lay the tube as above,
  and check the start against the screenshot. Under 2 minutes. Nothing in the trial is sharp, hot or powered; the pads touch
  only the tube and the cushion grips.

## Importance and sources

- **Occupations.** Bending tube is a listed task of heating, air-conditioning and refrigeration mechanics: "Measure, cut,
  thread, or bend pipe or tubing, using pipe fitter's tools" ([O*NET 49-9021.01](https://www.onetonline.org/link/summary/49-9021.01)),
  and of plumbers, pipefitters and steamfitters: "Cut, thread, or hammer pipes to specifications, using tools such as saws,
  cutting torches, pipe threaders, or pipe benders" ([O*NET 47-2152.00](https://www.onetonline.org/link/summary/47-2152.00)).
  The BLS Occupational Outlook Handbook gives 425,200 jobs in 2024 for the first
  ([BLS OOH](https://www.bls.gov/ooh/installation-maintenance-and-repair/heating-air-conditioning-and-refrigeration-mechanics-and-installers.htm))
  and 504,500 in 2024 for the second ([BLS OOH](https://www.bls.gov/ooh/construction-and-extraction/plumbers-pipefitters-and-steamfitters.htm)).
  *Inference, not from those pages:* that offsets in small copper refrigeration and gauge lines are a routine part of that
  bending; O*NET lists bending, not offsets specifically.
- **The real operation.** The RIDGID 600 series instruction sheet
  ([PDF](https://res.cloudinary.com/dahlzxcv/image/upload/v1749721403/akeneo-assets/600-Sarja_KAYTTOOHJE.pdf)) names the parts
  modelled here (form, form handle, carriage, carriage handle, tube latch), gives the steps (move the carriage handle and tube
  latch away, position the tube in the form groove and secure it with the latch, lower the carriage handle to the form's 0, rotate
  it to the desired degree), and for 45 degree bends says to "align the mark on the tube with the 45° line on the Carriage", "the
  center of the arc segment will be this distance from the feature": so each mark here is the middle of its bend. It says to
  "expect to overbend approximately 1° to 3° to compensate for spring back", that copper springs back less than harder tubing,
  and that the bender can be used hand held or in a vise.
- **The product.** The same sheet gives the 604: 1/4 in tube, 5/8 in bend radius, 0.76 kg, and a 45 degree bend adjustment of
  0.03 in. The RIDGID catalogue sheet ([PDF](https://www.itm.com/pdfs/cache/www.itm.com/38058/datasheet/38058-datasheet.pdf))
  describes roll dies in the bending carriage, extra-long handles, a vise clamp block and cushioned grips, and gives the 604's
  carton as 15.5 x 3.25 x 2.25 in.
- **Layout (geometry, not a source):** two 45 degree bends with their middles 70.7 mm apart along the tube step it 50.5 mm
  (offset = spacing x sin 45 degrees, corrected for the 5/8 in radius); 70.7 mm is the usual 1.414 x offset for a 50 mm offset.
- **Our estimates, not measured:** handle length (36 cm from the form's axis, grips centred 29.5 cm out, chosen to fit the
  carton), the carriage's shape and roller size, and the tube's plastic moment: Z_p = (D^3 - d^3)/6 = 23.9 mm^3 for 1/4 x 0.030
  in tube, times about 105 MPa, the flow stress of annealed copper at the bend's mean strain from textbook strain-hardening
  constants, gives about 2.5 N m, 8.5 N at the grip. Measure it before trials: a spring balance on the carriage handle's grip,
  pulled square to the handle in the middle of a bend.

**Distinct from** the accepted tasks nearest it: the lever-bender 90 degree bend with springback (TK-QF4WMY: one bend, no
location, the difficulty is reading springback); squeezing locking pliers on a tube (TK-EMWW6H); and the pending screw-feed tube
cutter (TK-WLAWYA). Here the difficulty is placing two bends on marks and reversing the second, with the tube slid and turned in
the tool between them.

## Earlier idea, and how its reviews are met

This is an independent implementation of an idea first submitted by another team in PR #2123. Nothing of that submission's code or text is used. Each point of its three reviews, and how this version meets it:

| Review point | Here |
|---|---|
| Don't zero the score for harmless handling (a tube crossing a bed boundary) | No handling penalty at all. The only lasting zeros are spoiled tubes: a bend past 55 degrees, a bend made out of the marked plane, a bend beyond the marks |
| Specify the tube's material, wall and temper | 1/4 in OD x 0.030 in wall soft (annealed) copper to ASTM B280, 36 cm |
| Use a representative hand tube bender with its two handles, roll dies and latch; no added grip block | A named product, RIDGID 604: form handle, carriage handle carrying two rollers, sprung tube latch, degree lines on the form, 0 and 45 lines on the carriage. Nothing is added to the tube or handles |
| Model enough resistance to need the real bending action at roughly real force | The carriage handle meets the tube's plastic moment (2.5 N m, about 8.5 N at the grip) through an effort-capped stop that follows the tube's set angle, so the tube bends only while the handle is pushed past its set and springs back 1 degree. The lever has no resistance going back, as the real carriage lifts off |
| Specify the real bender and tube for repeatable trials; reset | "The real setup and reset" above |
| Score each bend's position against its mark with a measurable tolerance, in the real trial and the simulation, and show the demo meets it | Each bend's middle within 5 mm of its mark for 1.0 and within 12 mm for 0.75 (a ruler on the real tube); the demo's bends are 1.7 and 0.1 mm from their marks |

## What is simulated, and what is simplified

- **Bending is a model, not a material.** The tube is a chain of 4 mm links on hinges in the plane of its marks, from 4 cm
  before mark B to 4 cm past mark A (39 hinges), plus a slide along the groove and a turn about its own axis. While the carriage
  is down on the tube (lever at the form's 0 or past it) the slide and turn are locked; the tube's point at the form's 0 line
  takes a bend that wraps onto the form at 5/8 in radius as the lever turns, and keeps it. The carriage handle is held back by a
  drive whose torque is capped at the plastic moment and whose rest follows the set angle less 1 degree of springback. Swung
  back past the set, it is free. A bend made with the tube turned near 180 degrees goes the other way in the tube's own frame.
  Turned more than 15 degrees from flat, a bend is out of plane and the tube is scored 0 (it can only bend in its marks' plane
  here). No wall thinning, ovality or twist.
- **Masses:** the tube's links are 3 g each (20 g for the long end links), heavier than real copper (about 0.12 g per 4 mm), for
  the solver; they have no gravity. The bender is fixed where its clamp block is held.
- **The vise** is NVIDIA's SimReady 5.5 in bench vise drawn at 0.55 scale, to stand for a 3 in bench vise, so that the tube sits
  at a height both arms reach with a top-down grip. It only holds the clamp block.
- **Everything else** of the bender (form, handles, carriage, rollers, latch, lines) is built from boxes and cylinders in
  `task.py`, at the published radius and estimated sizes.

## Assets

NVIDIA SimReady props from the Hugging Face dataset [`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets),
pinned to revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, © NVIDIA, CC-BY 4.0. Files download at first run; nothing large is
committed.

- [`equipment_bench_vise_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/equipment_bench_vise_a01): fixed in place, scaled 0.55, joints welded
- [`measurement_ruler_speedsquare_a02`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/measurement_ruler_speedsquare_a02): distractor, mass set to 0.15 kg
- [`handTool_pliers_combination_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_pliers_combination_a01): distractor, mass 0.32 kg, joints welded, convex colliders
- [`handTool_screwdriver_slotted_65mm_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_screwdriver_slotted_65mm_a01): distractor, mass 0.12 kg, convex colliders

The bender and tube are generated by `task.py` with `mechkit.py`; no third-party meshes.

## Licence

MIT, apart from the SimReady props (CC-BY 4.0, above).
