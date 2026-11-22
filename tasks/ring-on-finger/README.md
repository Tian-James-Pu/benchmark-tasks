# Put a steel ring on a hand form's ring finger, past the knuckle to the base

Put the ring on the hand form's ring finger, all the way down to the base of the finger, and leave it there.

Most people have put a ring on a finger. A ring sized to pass the knuckle is loose at the base and tightest at the
middle (PIP) knuckle, so the band has to be pushed square over a soft bulge. Here a US 7.5 steel band (17.75 mm bore)
must pass an 18.0 mm silicone knuckle and reach the base line of the finger.

## Setup and reset

See `task.json` (setup) for every object, its size, material and start pose, and `fabrication/` for the hand form,
plate, reset template and push-test clamp drawings. Two arms at their ready pose, grippers open. A life-size left
hand form (printed PLA) stands on a 150 x 150 x 10 mm steel plate with its ring finger vertical at (300, 0) mm; that
finger has a cast Dragon Skin 10 Medium skin over a 9.6 mm bone. The ring lies flat on the table at (200, 160) mm.

Randomisation: one draw per trial, plate offset within +/-15 mm in x and y, ring offset within +/-20 mm in x and y.
Seed k (1-100) uses row k of `draws.csv`, the same printed table used to reset the real rig; seed 0 is the nominal
layout. The setup image shows seed 1 (plate +15, +1 mm; ring -9, -3 mm).

Reset: slide the ring off by hand, set the plate on its printed template at the trial's draw and check the tape marks,
lay the ring on its template circle at the draw, arms to ready, grippers open. Every 10 trials caliper the PIP crest
(at least 17.95 mm, otherwise fit a spare cast skin). About two minutes. Nothing is consumed.

## Run

Python 3.12, Isaac Sim 6.1, Isaac Lab 17.0.2, IsaacLab-Arena 0.3.0, yam-arena 0.1.0 and inspect-robots 1.0.0 /
inspect-robots-yam 0.38.0, pinned in `ENVIRONMENT.json` (the same stack as the accepted peg-out-a-tshirt task). An NVIDIA
GPU is required (GPU FEM and camera rendering). From the repository root:

```sh
# unit tests (scorer rows and geometry run on CPU with NumPy; the policy tests need yam_arena)
(cd tasks/ring-on-finger && python -m unittest test_scoring test_geometry test_policy -v)

# scripted demo, seed 1 (the uploaded demo video)
OMNI_KIT_ACCEPT_EULA=YES python tasks/ring-on-finger/run.py --policy demo --seed 1 --out /tmp/ring-demo-s1

# Dream-1 (dreamlabs/dream-1-beta), seed 2 (the uploaded model-run video)
OMNI_KIT_ACCEPT_EULA=YES python tasks/ring-on-finger/run.py --policy dream --seed 2 --out /tmp/ring-dream-s2

# baselines
OMNI_KIT_ACCEPT_EULA=YES python tasks/ring-on-finger/run.py --policy nothing --seed 1 --out /tmp/ring-nothing-s1
OMNI_KIT_ACCEPT_EULA=YES python tasks/ring-on-finger/run.py --policy hover --seed 1 --out /tmp/ring-hover-s1
```

`run.py` builds the seeded scene, runs one trial of at most 2,400 control steps (2 minutes at 20 Hz; physics 240 Hz),
scores it and writes `score.json` (score, conditions, readings), `run.json` (versions, source hashes, stage audit,
settle and diagnostics), `trace.jsonl.gz` (every command and measured state), `poses.npz` (ring, hand and gripper
poses per step), the three camera videos and start/final frames. `python run.py --check-only` compiles and hashes the
sources without starting the simulator.

- Policies: `demo` (scripted: left wrist camera RGB, the published rig calibration and joint readings only), `dream`
  (Dream-1 through inspect-robots-dream: the standard camera images, joint readings and the instruction only; the
  title is not sent), `nothing` (holds the ready pose) and `hover` (holds the ring over the fingertip, never releases).
- Dream credentials are resolved by the official dreamlabs-client (its environment variable or credential file);
  `dream_policy.py` never reads, copies or logs them. Run one Dream trial per API key at a time.
- `--wall-limit` (default 2,150 s) stops stepping early on a slow machine; the trial is still scored.

### Runtime per trial (measured)

Measured on one NVIDIA L40 (48 GB) with the three trials below running at the same time on the same GPU; wall
time includes the 21-29 s scene build.

| Trial | Control steps | Simulated time | Wall time |
|---|---|---|---|
| demo, seed 1 | 1,181 | 59.1 s | 742 s |
| demo, seed 2 | 1,030 | 51.5 s | 691 s |
| Dream-1, seed 2 | 2,400 | 120.0 s | 1,255 s (1,184 model requests, 2 action rows each, one server session, no expiries) |

## Judging

Read once at the end, from body poses in simulation and from two photos, a 300 mm rule and the run video on the real
rig. Released (every robot part at least 30 mm from the ring and the hand form), Still (ring moves under 2 mm and tilts
under 2 deg over the final 1 s) and Around (ring centre within 5 mm of the finger axis) gate every tier. Then 1.0 if
the ring's top face is below the PIP line (26 mm from the web) and its lower face reaches the base line's top edge
(8 mm); 0.5 if only past the PIP line; 0.25 if its top face is past the DIP line (50 mm); otherwise 0. Moving the
plate more than 5 mm or dropping the ring off the table latches 0. A held ring, a ring on the fingertip or on another
finger scores 0. Full rule: `task.json` (scoring) and `scoring.py`.

## Recorded trials

All on the files in this folder: the Python sources, `params.json` and `draws.csv` are byte-identical to the hashes
recorded in each trial's `run.json` (SHA-256 below). `run.py` reads only the instruction from `task.json`, which is
unchanged.

- Demo, seed 1: **1.0** ("seated past the PIP"): ring 0.20 mm off the finger axis, lower face at the palm top, robot
  126 mm away, ring still. This is the uploaded demo video (left wrist camera and scene camera side by side).
- Demo, seed 2: **0.5** ("past the PIP, not seated"): the ring stopped with its lower face 16.0 mm up the finger.
  Not uploaded; reported so the demo's reliability is not overstated.
- Dream-1, seed 2: **0.0** ("not around"): the ring never left its start position (156 mm from the finger axis).
  This is the uploaded model-run video (left wrist, scene and right wrist cameras side by side).

The demo reads the ring and finger positions from the left wrist camera. It applies a fixed calibration offset to
those fits (finger 0.2 / 1.0 mm, ring 0.3 mm), measured once against simulator poses in an earlier development run and
then held constant, like a camera calibration. It never writes poses, attaches the ring or reads poses during a trial.
The pick retries up to six times when the gripper reading shows an empty pinch.

| File | SHA-256 |
|---|---|
| arena_ring.py | d5cc73f2488eae30fe83e6606071104480800e9704428d25ccc091c296d2def0 |
| demo_policy.py | 73f95facfa65164c9e5c5a4b9e212eb83e3f27e9803697d02b969599b493984b |
| dream_policy.py | 7467b24bbe4bafcd0a3cd9f4a445b88ae7155ff03078268fa0727571b0cbde6f |
| geometry.py | 159baaedc7d9d19b0cc2573a596cdb22cf253f937d5d5b6a13b5ffa8bdbb5d17 |
| native.py | 1ca611b536a4903c2972df593e9f76298b1524efebd25e850c0a6a32f1635c73 |
| perception.py | e12a6449511a9bd4ebd3fab5e0960093b5cf40d2d6bba98e1088898913dc3320 |
| run.py | ab430f732549cde7c45d9c53f0fcb098e9ff3f1ace231e43507c8a5bd1ba6451 |
| scene.py | e016921b850b971327c9f9a9044ebd39d621d4bf2bc0cbf188f21ec6f6b52527 |
| scoring.py | e6b644d953f52457114bc6f16c74fd35b98086dc45918e9293be7dcaee405d25 |
| task.py | 714cb074663d19c3b03383454f9cc3c62758681e6087316c7e8329d21c2ee4e4 |
| test_geometry.py | 0a3d8f7eac0f6fdfbd449228b6b999757c7499404a8d251cb7a6a7590f7476c9 |
| test_policy.py | 25175119efebf0fcea6f435bf358ab27f8d0c89f20ffd34fec1a6881f2c0db50 |
| test_scoring.py | cbf55508a78618ac8433f843e9eeb997c5352ed9dc35bc02325488ac1e6ed67a |
| params.json | ab10a8c03d22fba22871dd9eee8e916925e24ddfadd481887226c283972a440d |
| draws.csv | 627213cebdf411fe149d6aee3bb68bcb5523920796b56d1bacf791b093903b9a |

## Simulation

- Ring: one rigid body of 48 convex pieces (bore 17.75, outside 21.55, width 4.0 mm, 0.5 mm edge radii), 3.74 g.
  The bore is flat (as turned); the "comfort-fit" wording in `params.json`'s ring description is an older label and
  is not modelled (`params.json` is kept byte-identical to the recorded trials).
- Hand form: one rigid body (plate and PLA form by per-collider density) resting free on the table.
- Ring-finger skin: native PhysX volume FEM, 1,080 nodes and 4,032 tetrahedra (about 2 x 1.5-2 x 4 mm cells; PhysX
  rejects tetrahedra under about 1 mm3), a sleeve on the rigid 9.6 mm bone. Young's modulus 260 kPa (neo-Hookean fit
  of the datasheet's 100% modulus), Poisson 0.45, 1,070 kg/m3. The 7.5 mm tip dome is part of the rigid form.
- Contact: rest offset 0 on ring, skin and hand; contact offsets 0.5 mm (ring, skin) and 1 mm (hand). Radial
  interference at the PIP crest is 0.125 mm at the skin polygon's flats and 0.20 mm at its nodes. Friction ring-skin
  0.8, ring-pad and ring-table 0.5, PLA 0.4 (estimates).
- The knuckle resistance comes only from contact and friction. No attachments, fixed joints outside the arms,
  kinematic bodies or pose writes; `run.py` audits the stage and fails closed.
- Simplified: the painted lines are not rendered (the scorer reads poses); no silicone tack, wear or tearing.

## Physical values

Datasheet-sourced (Smooth-On, Dragon Skin 10 Medium technical data): Shore hardness 10A; 100% modulus 22 psi
(151.7 kPa; used as E = 260 kPa through a neo-Hookean fit); specific gravity 1.07 (1,070 kg/m3).

Design values (set by the drawings in `fabrication/`): every dimension of the hand form, skin mould, plate, template
and ring; the ring size (US 7.5, bore 17.75 mm).

Estimates (not measured): ring-skin friction 0.8, pad-steel friction 0.5, PLA friction 0.4; skin Poisson ratio 0.45;
hand-form mass 260 g; plate mass 1,770 g; ring mass 3.74 g from a 316L density of 7.98 g/cm3; the push over the PIP
(about 2.0 N square, 6.9 N tipped 12 deg) and the 3.4x tilt ratio quoted in the context.

## Files

| File | What |
|---|---|
| `task.json` | the task's details |
| `task.py` | yam_arena task file: objects, instruction, per-step scorer hook |
| `scene.py` | layout, per-trial draws, USD authoring of the hand form and ring |
| `native.py` | the ring finger's skin as native PhysX volume FEM |
| `geometry.py` | every dimension (from `params.json`), the skin mesh, ring pieces, scorer geometry |
| `arena_ring.py` | yam_arena / Arena hooks (custom bodies, state readers) |
| `scoring.py` | the scoring rule, read from rigid poses and the robot's collision hulls |
| `perception.py`, `demo_policy.py` | scripted demo (left wrist camera, rig calibration, joint readings) |
| `dream_policy.py` | Dream-1 adapter (session-recovery audit; credentials left to the SDK) |
| `run.py` | one trial, scored, with all evidence files |
| `params.json` | all task dimensions and settings |
| `draws.csv` | the 100 printed reset draws (seed k = row k) |
| `test_scoring.py`, `test_geometry.py`, `test_policy.py` | unit tests (scorer rows for every tier and baseline, geometry, perception and a demo dry run) |
| `ENVIRONMENT.json` | pinned runtime |
| `fabrication/` | hand form, skin mould, plate, reset template and push-test clamp (OpenSCAD), build notes |

## Assets and licences

- Task code, parameters and fabrication drawings: own work, written for this task. No third-party hand or ring
  mesh is used; the hand form, ring and skin are generated from `params.json`.
- YAM arm model: loaded from yam-arena / i2rt (MIT License, I2RT Robotics); not redistributed here.
- Isaac Sim, Isaac Lab, IsaacLab-Arena, inspect-robots and dreamlabs-client: their own licences, installed separately.
- Dragon Skin 10 Medium and Silc Pig (Smooth-On) are purchased materials; datasheet values are cited only. No product
  images, text or models are redistributed.
