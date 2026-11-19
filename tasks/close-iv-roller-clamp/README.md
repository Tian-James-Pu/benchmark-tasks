# Close the roller clamp on an IV line left running free

A primed gravity IV set runs from a 1 L saline bag on a pole, across an overbed table, to a cannula taped on a
training forearm. Its roller clamp lies on the table wide open, so the line runs free. The robot has to roll the
clamp's 12 mm wheel 22 mm along its track to the narrow stop. The 6 g clamp body is loose on soft tubing. If the
wheel is pushed without the body being held still, the clamp slides along the tubing, or drags the line across the
table, and the wheel stays open. Pulling the line toward straight (a tug on the cannula) voids the trial.

A task file for the `yam_arena` embodiment (Isaac Lab-Arena, station scene): [task.py](task.py).

| File | What it is |
|---|---|
| `task.py` | Scene, scoring (`score`, `success`) and the scripted `demo()` |
| `mechkit.py` | Mechanism kit: the clamp is a mechkit mechanism (body, wheel slider, tubing slider) |
| `cordkit.py` | Capsule-chain tubing: the two sections of the line |
| `simready.py` | Loads the SimReady props, downloading them once from Hugging Face |
| `task.json` | The task's details for the board |

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/close-iv-roller-clamp/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Close the roller clamp on this IV line." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/close-iv-roller-clamp-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 460 -E cam_width=640 -E cam_height=360`.

The run prints `task_score` at the end. `score()` returns the scorer's running result, and `success()` is true once
the closed state has held for 3 s (`SUCCESS_HOLD_S = 0`: the task's own tracker counts the hold).

## Runtime and seeds

- Demo: the plan is 460 steps (23 s); the run ends at step 261, when the 3 s closed hold completes (the wheel reaches
  its stop at about step 160). About 2 minutes on an L40S including start-up (several more the first time a GPU
  compiles its shaders).
- Dream-1: 2,400 steps (2 minutes simulated), about 8 minutes.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The demo is open loop and deterministic.

## The scripted demo

One hand, the way a thumb does it. The left gripper's closed fingertips come down on the top of the wheel, press it
(and through it the clamp) down onto the table, and roll it toward the patient end. The press is what keeps the body
from sliding: through the wheel's axle it loads the clamp onto the table. The hand then lifts straight off and goes
away; the right arm stays at rest. The demo follows a fixed plan from the published starting layout (the clamp's
settled position and its -8 degree yaw were measured once and written into `demo()`); it reads no simulator state.

A two-handed version (one gripper pinching the clamp's walls at its ridged tail while the other pushes the wheel)
was tried and is not physically possible with these grippers: on a 54 mm clamp the two wrists collide about 16 cm
apart. A real nurse often holds the body in one hand and rolls the wheel with the other thumb, but the task does not
need two hands and the scoring does not ask for them.

## Scoring

All of it is in `Trial.step` in `task.py`, run once per control step:

- **Whole-run voids (latched, cleared only in mechkit's `on_reset` hook):** either tubing section pulled to within
  1.5 cm of straight (end-to-end distance at least its length minus 1.5 cm), or the clamp slid 4 cm or more along its
  tubing (the `carrier` joint, see Fidelity).
- **End conditions:** every gripper fingertip point (`mechkit.fingertips`) at least 2 cm from the clamp's box and
  from every tubing link, and everything at rest for 1 s (clamp under 1 cm/s, every tubing link under 2 cm/s, wheel
  and slide joints changing under 0.2 mm per step). Otherwise the score is 0.
- **Score:** 1.0 once the wheel is within 0.5 mm of its narrow stop and that, the hands clear and the rest have held
  together for 3 s (60 steps); 0.4 with the wheel 60 percent (13.2 mm) or more of the way; 0 otherwise.

In a real trial: a photo of the wheel against its moulded stop with a ruler in view, a pen mark on the tubing at the
clamp's tail to read any slide along the line, and video of the hands and line. Closing is confirmed physically by no
drops forming in the drip chamber for 30 s.

## Real-world trial

Low cost: a real primed gravity set (for example the BD set below, 264 cm, standard bore, 20 drops/mL) from a 1 L bag
of saline or water on a pole, draining through a 20 G cannula taped into a foam or training arm and on into a waste
container under the table. Lay the set and clamp out as in `setup`. Nothing sharp is handled and nothing is
pressurised; the robot only touches the clamp and the tubing on the table. Full credit: the wheel at its moulded stop
in the photo, the hands clear, and no drops in the drip chamber for 30 s.

## Validation

Two demo runs on the final files, each 1.0 (contact sheets checked: the fingertips ride on the wheel, the clamp stays
on the table, the hand lifts clear). Red-team variants, each a copy of `task.py` with a different `demo()`, run
through the same scorer and then deleted:

| Variant | What it does | Score |
|---|---|---|
| Nothing | both arms stay at rest | 0 |
| Pinch the line, push the wheel | one hand pinches the tubing 21 cm upstream, the other pushes the wheel from behind | 0 (the clamp slides along its line; the wheel stays open) |
| Careless push | one hand pushes the wheel from behind at axle height, without pressing or holding the clamp | 0 (the clamp and line move; the wheel stays open) |
| Part way | the demo's press-and-roll stopped 10 mm short | 0.4 |
| Hold instead of release | the demo, with the hand left pressed on the closed wheel | 0 (hands not clear) |
| Tug the line | the demo, then the hand presses the clamp's tail and drags it 16 cm away from the forearm | 0 (patient line pulled straight, latched) |
| Lift and carry | the clamp pinched across its walls, lifted 12 cm, carried 9 cm and put back | 0 |
| Drag while pressed | the demo, then the hand keeps pressing the wheel and drags the clamp 12 cm up the table | 0 |
| Light touch | the demo's roll with the fingertips only 3 mm into the wheel instead of 16 mm (no real press) | 0 (the wheel stays short of 60 percent) |

Dream-1 run (2,400 steps): 0. It brought the left gripper down beside the clamp and hovered there without pressing or rolling the wheel, while the right gripper kept reaching toward the training forearm's hand near the cannula; the wheel never reached its stop and the run ended at the step limit.

## Fidelity

- **The clamp's grip on its tubing.** In a real open roller clamp the tubing lies in the channel and the clamp slides
  along it under a light pull. Here the tubing inside the clamp is a hidden link (`carrier`) on a prismatic joint
  along the clamp, held by 1.5 N of joint friction, an estimate set below the wheel's 3 N so that an open clamp
  slides on its line before its wheel moves, as a real one does. Both tubing sections are fixed to that link, not to
  the clamp body, so holding the tubing gives the body no support: it slides, with friction, past a pinched line.
  The tubing's two end links do not collide with the clamp body's own colliders (they start inside its channel).
  Measured in the simulator (a ramped push on two copies of the clamp fixed to the table): the wheel starts to creep
  under about 0.5 N and breaks away at about 3.3 N; the tubing slider breaks away at about 1.4 N. A 10 N downward load
  did not change the wheel's breakaway.
- **The wheel** is a ribbed 12 mm disc on a prismatic joint along its 22 mm track: it slides rather than rolls, and
  the tubing is not squeezed; closed is read from the wheel's position (the 4.1 mm tubing would be pinched to about
  1.1 mm, its two walls, at the narrow stop).
- **The tubing** is two capsule chains (25 mm links) with bend springs; it does not stretch, and no fluid flows.
- The forearm, cannula, dressing, bag, spike and drip chamber are fixed custom meshes; the IV pole and the three
  distractors are SimReady props.
- Clamp dimensions are modelled values for a standard-bore roller clamp (54 x 18 x 10 mm body, 12 mm wheel, 22 mm
  travel), not taken from a manufacturer drawing (none is public); the tubing size (3.0 x 4.1 mm) is the
  standard-bore fit listed by clamp suppliers (source below). The wheel's 3 N and the slide's 1.5 N are estimates,
  to be measured with a spring scale on the real clamp used for the trial.

## Importance and sources

Who does it:

- Registered nurses (O*NET 29-1141.00, 3,465,400 employed in 2025): "Administer medications to patients and monitor
  patients for reactions or side effects." https://www.onetonline.org/link/summary/29-1141.00
- Licensed practical and licensed vocational nurses (O*NET 29-2061.00, 666,900 employed in 2025): "Administer
  prescribed medications or start intravenous fluids, noting times and amounts on patients' charts."
  https://www.onetonline.org/link/summary/29-2061.00
- These figures are whole occupations; not every nurse runs gravity lines every shift. O*NET's nursing assistant
  profile (31-1131.00) lists no IV task, and we do not claim that closing a clamp is an aide's job. Aides do handle
  lines while moving patients, though: ECRI's free-flow case below began with a nurse's aide.

The operation:

- BD, which makes the modelled set, makes closing the roller clamp the first and last step of priming its Alaris
  pump-module sets: "1. Close the roller clamp. ... 6. Close the roller clamp when priming is complete." (Recommended priming procedure for the
  BD Alaris Pump Module administration set)
  https://www.bd.com/content/dam/bd-assets/na/medication-management-solutions/documents/brochure/Alaris-Recommended-Priming-Tip%20Sheet-FINAL_TS_EN.pdf
- BD's pump-module FAQ: "To prevent free-flow, close set roller clamp when the safety clamp fitment is open. The
  roller clamp is the primary safety mechanism to regulate the infusion rate and to prevent or stop flow to the
  patient."
  https://www.bd.com/content/dam/bd-assets/na/medication-management-solutions/documents/brochure/IF_Alaris-Pump-Module_FQ_EN.pdf
- A nursing skills text repeats the step at priming ("Close roller clamp") and at every bag change ("Pause the EID or
  close the roller clamp on a gravity infusion set"): *Clinical Procedures for Safer Patient Care*, chapter 8.4,
  https://opentextbc.ca/clinicalskills/chapter/8-4-iv-assessment-maintenance-troubleshooting-and-discontinuation/

Why it matters:

- ECRI describes a 1991 case in which a nurse's aide took an infusion set out of a pump "without first closing the
  manual clamp on the line". The free flow gave the mother nearly 35 times the prescribed dose of Pitocin, and the
  infant died. ECRI: "Free-flow, or unrestricted gravity flow, refers to the uncontrolled delivery of an infusion to a
  patient when a controlled or metered delivery was intended."
  https://home.ecri.org/blogs/ecri-blog/advancing-patient-safety-through-better-designs-or-processes-top-10-health-tech-hazards-part-2

The modelled hardware (fidelity, not importance):

- BD gravity administration set 42493E: roller clamp, 264 cm, standard bore, 20 drops/mL,
  https://www.bd.com/en-us/products-and-solutions/products/product-page.42493e
- A roller clamp's standard-bore tubing fit, 0.118 x 0.161 in (3.0 x 4.1 mm): Qosina roller clamp 14082,
  https://datasheets.globalspec.com/ds/qosina/14082/fc135069-20bb-4c54-aece-2e9a4f596240

Nearest tasks: loading an IV set into a pump module (its safety clamp is pressed into a recess; no roller wheel);
scrubbing an IV connector's hub; spiking a bag and priming a chamber (their roller clamp is not operated); resting
a cannulated forearm on a wedge without dragging its line (the same hazard to the cannula, but the arm is moved and
no clamp is touched). None operates a roller clamp's wheel.

## Assets

The IV pole and the distractors come from the Hugging Face dataset
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`IV_pole_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/IV_pole_a01) (fixed)
- [`dishware_plastic_cup_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_plastic_cup_a01) (distractor, 20 g)
- [`bandage_medical_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/bandage_medical_a01) (distractor, 16 g)
- [`box_glove_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/box_glove_a01) (distractor, 450 g)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time:
masses set to the values above, the cup and bandage box given convex colliders, any joints welded into one rigid
body. The files are not committed: `simready.py` downloads them into `~/.cache/simready` the first time the scene is
built.

`mechkit.py` is the mechanism kit copied unchanged from `tasks/heat-shrink-lug/mechkit.py` in this repository.
`cordkit.py` is the capsule-chain rope of `tasks/tie-overhand-knot` (MIT licence), as adapted in
`tasks/alaris-set-load`; changed here: an `anchor_filter` option lets a link fixed to an anchor body ignore that
body's other colliders.
