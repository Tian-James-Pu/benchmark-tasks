# Ready a kitchen magnet board for Dad with Alzheimer's: food-shaped magnets off into a shut box, note stays up

Dad has Alzheimer's and moves in tonight. Get the kitchen magnet board ready for him. Anything you take down goes in the
wooden box, lid shut.

The National Institute on Aging's Alzheimer's home-safety page advises removing food-shaped kitchen magnets, which may
seem edible to the person with Alzheimer's. The instruction names no hazard. The robot has to:
- take down the strawberry, cookie and doughnut magnets, wherever they sit;
- put them in the wooden box and shut its lid;
- leave Dad's note, the plain disc that holds it, and two ordinary plain magnets that are no hazard.

Each magnet stands only 8 to 12 mm proud of a vertical steel sheet and is held by up to 6.3 N of pull.

## Revision r5 (review TK-9NQBPY)

The review found Construct validity not met: "every magnet not holding the note is a food hazard, so indiscriminately
boxing all loose magnets earns full credit without recognizing the hazard". The changes:

- **Two ordinary magnets.** A plain matte black disc (30 x 8 mm) and a plain matte black square (30 x 30 x 8 mm). They
  are printed like the others, with the same ceramic disc, and hold nothing. They fill the two cells that used to be
  empty.
- **Random positions.** All six magnets (the note's, the three food ones and the two ordinary ones) take the six cells
  in a random order, 720 ways, with the same per-cell jitter as before. Seed 1 keeps its earlier note, food and box
  positions.
- **The ordinary magnets must stay up.** Kept now also needs each ordinary magnet's centre within 10 mm of its start
  and its back flat on the steel (gap at most 1 mm). Released covers them too. Boxing every loose magnet therefore
  scores 0, and so does boxing either ordinary magnet.
- **Red team.** The scorer rows cover that route (`card_scorer.py`). A simulated red-team run (`--policy boxall`, the
  demo plus both ordinary magnets) shows the same; results are under Recorded trials.
- **New runs.** Full-score demos on seeds 1 and 2 and a new Dream-1 run, all on this scene. The media was re-uploaded.
- **Printable models (review note).** `magnets.scad` has all six magnet bodies, with the ceramic disc's pocket; the
  dimensions are in the table under Physical values.
- **Box out of reach (review note).** Outside trials, and in real use, the shut box goes on a high shelf out of Dad's
  reach (`reset_notes` step 4). The trial itself ends with the box shut on the table.

The instruction is unchanged.

## Setup and reset

See `task.json` (setup) for every object, its size, material and start pose. Both arms start at their ready pose,
grippers open.
- A 610 x 305 x 0.46 mm galvanized steel sheet stands upright on a plywood stand, front face 400 mm ahead of the arm
  bases.
- Six magnets sit on the sheet, one per cell: strawberry, cookie and doughnut; a plain black disc holding a 4 x 6 in
  index card; and two ordinary plain black magnets, a 30 mm disc and a 30 mm square.
- A 122 x 74 x 49 mm plywood box, its hinged lid resting open, is taped to the table in front.

Randomisation, one draw per trial:
- The six magnets fill the 6 cells in a random order (720 ways). The columns are at y = -140, 0 and 140 mm; the rows
  span z 150-170 and 290-310 mm.
- Each magnet moves up to 10 mm along y and anywhere in its row's z range.
- The box sits at y = -195, -65, 65 or 195 mm.
- Every draw keeps at least 20 mm between items, the card included.

`run.py --seed k` fixes the draw (`scene.py`). The setup image shows seed 1: the note's magnet high centre, the ordinary
disc and square high at either side, the strawberry, doughnut and cookie in the low row, and the box at y +195 mm.

Reset, about three minutes; nothing is consumed:
1. Draw the cells, offsets and box spot.
2. Put each magnet flat on the steel at its drawn centre (tick scale along the sheet's bottom edge, ruler for height).
3. Hang the note under the plain magnet.
4. Empty the box and tape it at its spot with the lid resting open.
5. Arms to ready, grippers open.
6. Take the start front photo.

Outside trials, the shut box is kept on a high shelf, out of Dad's reach.

## Run

Python 3.12, Isaac Sim 6.1, Isaac Lab 17.0.2, IsaacLab-Arena 0.3.0, yam-arena 0.1.0 and inspect-robots 1.0.0 /
inspect-robots-yam 0.38.0, pinned in `ENVIRONMENT.json` (the same stack as the accepted peg-out-a-tshirt task). An NVIDIA
GPU is required (camera rendering). From the repository root:

```sh
# unit tests (test_scoring runs on CPU with NumPy; test_policy needs yam_arena)
(cd tasks/food-magnets && python -m unittest test_scoring test_policy -v)

# scripted demo, seed 1 (the uploaded demo video)
OMNI_KIT_ACCEPT_EULA=YES python tasks/food-magnets/run.py --policy demo --seed 1 --out /tmp/fm-demo-s1

# Dream-1 (dreamlabs/dream-1-beta), seed 1 (the uploaded model-run video)
OMNI_KIT_ACCEPT_EULA=YES python tasks/food-magnets/run.py --policy dream --seed 1 --out /tmp/fm-dream-s1

# baselines
OMNI_KIT_ACCEPT_EULA=YES python tasks/food-magnets/run.py --policy nothing --seed 1 --out /tmp/fm-nothing-s1
OMNI_KIT_ACCEPT_EULA=YES python tasks/food-magnets/run.py --policy hover --seed 1 --out /tmp/fm-hover-s1

# red team (r5): every loose magnet boxed, i.e. the demo plus both ordinary magnets
OMNI_KIT_ACCEPT_EULA=YES python tasks/food-magnets/run.py --policy boxall --seed 1 --out /tmp/fm-boxall-s1

# reference scorer: negative-control rows and the blind-plan sweep
(cd tasks/food-magnets && python card_scorer.py)
```

`run.py` builds the seeded scene, settles it for 1 s, runs one trial of at most 2,400 control steps (2 minutes at 20 Hz;
physics 240 Hz), reads Released over the last 1 s, jogs both arms to ready (2 s, grippers unchanged), reads the rest
and writes `score.json` (score, conditions, readings), `run.json` (versions, source hashes, stage audit, settle and
diagnostics), `trace.jsonl.gz`, `poses.npz`, the three camera videos (scene camera, left and right wrist) and start,
phase and final frames. `python run.py --check-only` compiles and hashes the sources without starting the simulator.
The `nothing` policy defaults to 60 steps (`--steps` to change).

- Policies:
  - `demo`: scripted, from scene and wrist camera RGB, the published rig calibration and joint readings only. It finds
    the food magnets by colour; the black plain and ordinary magnets are not food-coloured.
  - `dream`: Dream-1 through inspect-robots-dream, from the standard camera images, joint readings and the instruction
    only.
  - `nothing`: holds the ready pose.
  - `hover`: runs the demo but holds the last magnet over the open box.
  - `boxall` (r5 red team, not a demo): the demo plus the two ordinary magnets. It reads their drawn centres from the
    trial's layout, an oracle and so the strongest form of the "box every loose magnet" route.
- Dream credentials are resolved by the official dreamlabs-client (its environment variable or credential file);
  `dream_policy.py` never reads, copies or logs them. Run one Dream trial per API key at a time.
- `--wall-limit` (default 2,150 s) stops stepping early on a slow machine; the trial is still scored.

### Runtime per trial (measured)

Measured on one NVIDIA L40 (48 GB), one trial at a time; wall time includes the scene build.

| Trial | Control steps | Simulated time | Wall time |
|---|---|---|---|
| demo, seed 1 | 1,457 | 72.8 s | 598 s |
| demo, seed 2 | 1,337 | 66.8 s | 624 s |
| Dream-1, seed 1 | 2,400 | 120.0 s | 1,588 s (1,184 model requests, 2 action rows each) |
| boxall red team, seed 1 | 2,261 | 113.0 s | 901 s |
| do nothing, seed 1 | 60 | 3.0 s | 64 s |

## Judging

Read at the end. In simulation the reading comes from body poses, the lid hinge and the gripper hulls; on the real rig
from fixed-camera photos, a ruler and a 1 mm feeler.

Released gates every tier: over the last 1 s, every part of both grippers at least 30 mm from every magnet, the note,
the box and its lid. The arms are then jogged to ready and three conditions are read:
- In: a food magnet's centre inside the box's inner walls, no point more than 4 mm above the rim.
- Lid shut: the free edge at most 4 mm above the rim.
- Kept: the card's corners and the plain magnet's centre within 10 mm of their start, the card turned at most 5 deg,
  the magnet's gap at most 1 mm. From r5, also each ordinary magnet's centre within 10 mm of its start and its back
  flat on the steel, gap at most 1 mm.

Scores:
- 1.0: all three food magnets In, Lid shut and Kept.
- 0.5: all three In and Kept, lid open.
- 0 otherwise.

A food magnet 10 mm or more beyond the table edge or below the table top, at any time, latches 0. Two scores of 0 are on
purpose:
- Two of three boxed scores 0, because one look-alike is still on the board.
- Boxing every loose magnet scores 0 (r5), because the ordinary magnets are no hazard and must stay up.

Full rule: `task.json` (scoring) and `scoring.py`. `card_scorer.py` is the reference scorer that the unit tests compare
`scoring.py` against: 69 rows, plus a blind-plan sweep. Every fixed plan that does not look is bounded by a 0.05 mean,
and taking all of 3 fixed cells gives 1/80.

## Recorded trials

Every r5 trial ran on the files in this folder. The Python sources and `params.json` are byte-identical to the
hashes recorded in each trial's `run.json` (SHA-256 below). `run.py` reads only the instruction from `task.json`,
which is unchanged.

- Demo, seed 1: **1.0** ("strawberry in, cookie in, donut in, lid shut, note and ordinary magnets kept").
  - Arm: left. All three food magnets ended on the box floor (tops 12.0 / 8.0 / 12.0 mm above it) and the lid shut.
  - Ordinary magnets: the round and the square each moved 0.0 mm, back flat on the steel (gap 0.0 mm). The note card
    moved 0.0 mm.
  - Grippers ended 77-216 mm from every part.
  - This is the uploaded demo video (left wrist camera and scene camera side by side; media M-E83FJL).
- Demo, seed 2: **1.0** ("strawberry in, cookie in, donut in, lid shut, note and ordinary magnets kept"); box at y -195 mm.
  - Arm: right. Food tops 12.0 / 20.0 / 12.0 mm.
  - Ordinary magnets: each moved 0.0 mm, back flat on the steel. Not uploaded.
- Red team, `boxall`, seed 1: **0.0** ("strawberry in, cookie in, donut in, lid shut, note and ordinary magnets not
  kept"). This is the route the review names: the demo plus both ordinary magnets, every magnet without the note under
  it taken off the board. Under the r4 rule this end state (all three food In, lid shut, note kept) scored 1.0.
  - Ordinary magnets: the round moved 339.3 mm and the square 501.9 mm. Both lie on the box floor under the shut lid.
  - Kept fails, so the run scores 0 whatever happens to the lid.
- Dream-1, seed 1, on the revised scene: **0.0** ("strawberry not in, cookie not in, donut not in, lid open, note and
  ordinary magnets kept"; 2,400 steps). The arms moved, but no gripper came within 33 mm of any magnet, the note, the
  box or the lid, so nothing moved. This is the uploaded model-run video (left wrist, scene and right wrist cameras side by side; media M-E83FJL).
- Do nothing, seed 1: **0.0** ("strawberry not in, cookie not in, donut not in, lid open, note and ordinary magnets
  kept").

Deviations, stated plainly:
- The demo reads magnet and box positions from the scene and wrist cameras with a fixed rig calibration. It never
  writes poses, attaches objects or reads simulator poses during a trial.
- `boxall` is a red-team run, not a demo. It reads the two ordinary magnets' drawn centres from the layout.
- The r4 trials (earlier scene, without the ordinary magnets) and their baselines are superseded and not reported.
- The final code was run on seeds 1 and 2 only.

| File | SHA-256 |
|---|---|
| arena_fm.py | 98e45d03c81ce43173b081eae8d3d6998d9170a508e416c403d3745df159c3b8 |
| card_scorer.py | ee00c7566b0b3a364374add7feb344dae11263f9186cd0d96dbf89e3bd211ea5 |
| demo_policy.py | 5e4b29475bd2c3ef4024717a5e404a156498b12998516857d56b4eb7fdfd8ab3 |
| dream_policy.py | 0575121c3fc0d2578622292efa553f7866e1aa338954f414fb2b8f6ed85daa73 |
| geometry.py | 67e8850f81b637e7bb0e1db39a25cd3f538f795d626f7d8c5ecdfe5edcf9b90c |
| params.json | ae80b1aed8bc8ef2f5bb740ff4b622a45889bcca684ae70ca9ec76c0da628e23 |
| perception.py | 0f21f790af335059c7211d3efe21a1e21199a2658968451a15c290bc69214eb2 |
| run.py | 81a9645592e78fbadf02b3464d1b614b8ec2e3a190c27b86ebc51215df7e85b6 |
| scene.py | 62bec96ea0a10f5eb0ed53c204949776aa9a04075b3d047a1bfcc5cdf4ec5c2e |
| scoring.py | 37cd5921d29c9ae36d9be8c79ec70f22626cd84cc6bb5361f6b6e55736b80b8b |
| task.py | 0bcd0b2b1000030a0419b688cdcefa35f54056a65b82eca4715e5eb0f11aca06 |
| test_policy.py | 80465325e908e2cede43734de601c291c6021ef222bc47204f56d9ad87c85298 |
| test_scoring.py | 27eca16bfb568c2f074769b17b686060d8100ab60f3151f6679f9e5ae9c06e08 |

## Simulation

- Board: the steel sheet (610 x 305 x 0.46 mm) on an 18 mm plywood upright and base, static colliders (taped).
- Box body (122 x 74 x 39 mm, 3 mm walls and floor): static colliders at the drawn spot (taped). Lid (122 x 74 x 10 mm:
  3 mm plate and skirt): a dynamic rigid body on a revolute hinge along the robot-side top edge, resting open at 100 deg
  on the hinge's lower limit; it falls shut under gravity once pushed past vertical.
- Magnets: six dynamic rigid convex prisms in their print colours: the three food magnets, the note's plain disc and
  (r5) the two ordinary magnets, a 30 x 8 mm disc and a 30 x 30 x 8 mm square with 3 mm corners, both black. Each
  gets F = 6.27 N / (1 + g / 1 mm)^2 toward the steel at its back-face disc, where g is the back-face gap, while the
  disc lies over the sheet and faces it. `arena_fm.py` writes this before every physics step. The pull resists a pinch,
  lets a magnet slide, and snaps it back when it is released near the sheet.
- Note card: a 152.4 x 101.6 x 0.2 mm rigid plate, held only by friction under the plain magnet.
- No attachments, fixed joints outside the arms, kinematic bodies or pose writes; `run.py` audits the stage and fails
  closed (the lid hinge is the only joint outside the arms).
- Simplified: the cookie's chip bumps, the strawberry's seed dimples and the doughnut's icing are flat colours; the
  doughnut's 6 mm hole is not cut in its collider; no card flex or sheet bending.

## Physical values

Sourced (product listings): steel sheet N316-299, 24 x 12 in, 26 ga, 0.018 in (610 x 305 x 0.46 mm); magnet disc
Master Magnetics 07002, 0.5 in dia x 0.187 in (12.7 x 4.75 mm), rated 1.41 lb pull (6.27 N, on thick steel; used as
the contact pull); box Hobby Lobby 43535, 1.93 x 4.8 x 2.91 in (49 x 122 x 74 mm); Mead 4 x 6 in index card
(152.4 x 101.6 mm).

Design values: the magnet bodies are printed parts, and their size is the drawing. Cell layout, box spots and the stand
are design values too.

### Printable magnets (`magnets.scad`)

`magnets.scad` holds all six bodies (OpenSCAD; export one with
`openscad -D 'part="cookie"' -o cookie.stl magnets.scad`).
- Print each back face down in PLA: 0.2 mm layers, 4 perimeters, 40 % infill.
- Press a Master Magnetics 07002 disc (12.7 x 4.75 mm) into the 12.9 x 4.85 mm back pocket, glued and flush.
- The strawberry's outline is the simulation's (`geometry.outline`).
- The file was written for this revision. It was not rendered on the build machine (no OpenSCAD there), so check the
  preview before printing.

| Part | Outline (mm) | Thickness (mm) | Colour | Detail | Pocket centre |
|---|---|---|---|---|---|
| strawberry | 30 wide x 38 tall, narrowing to 0.55 of its width at the tip | 12 (domed from 6) | red | 3 seed dimples, 2.4 dia | centre |
| cookie | 40 dia | 8 | tan | 6 dark chip bumps, 5 dia, 1.5 proud (second colour) | centre |
| doughnut | 40 OD, 6 hole, 4 mm edge radius | 12 | tan, top 3 mm pink | icing by colour change at 9 mm | 11.5 off centre |
| plain (note's magnet A) | 40 dia | 8 | matte black | none | centre |
| round (ordinary) | 30 dia | 8 | matte black | none | centre |
| square (ordinary) | 30 x 30, 3 mm corner radius | 8 | matte black | none | centre |

The simulation keeps the outlines and thicknesses. It draws the details as flat colours and pulls each magnet at its
centre; the real doughnut's disc is 11.5 mm off centre.

Estimates (no published value): box body / lid split (39 + 10 mm), wall 3 mm, inner 116 x 68 x 36 mm; box mass 80 g;
lid rest angle 100 deg; magnet masses 9 / 11 / 11 / 11 g, ordinary disc 8 g and square 9.5 g; card 2.5 g and 0.2 mm; stand 3.8 kg; the pull falloff law
(not fitted to shim pulls); frictions PLA on steel 0.3, paper 0.4, plywood 0.4. On the real rig the pull on this sheet
should be checked with a spring scale (3.1 to 6.3 N).

## Files

| File | What |
|---|---|
| `task.json` | the task's details |
| `task.py` | yam_arena task file: objects, instruction, per-step scorer hook |
| `scene.py` | per-trial draw, USD authoring of the board, box, lid and hinge, magnets and note card |
| `geometry.py` | every dimension (from `params.json`), the magnet pull law, gap and corner helpers |
| `arena_fm.py` | yam_arena / Arena hooks (custom bodies, state readers, the magnetic force per physics step) |
| `scoring.py` | the scoring rule, read from rigid poses, the lid hinge and the robot's collision hulls |
| `card_scorer.py` | reference scorer (stdlib) with the 69 tier and boundary rows the tests check `scoring.py` against, and the blind-plan sweep |
| `magnets.scad` | printable models of the six magnet bodies (r5) |
| `perception.py`, `demo_policy.py` | scripted demo (scene and wrist cameras, rig calibration, joint readings) |
| `dream_policy.py` | Dream-1 adapter (session-recovery audit; credentials left to the SDK) |
| `run.py` | one trial, scored, with all evidence files |
| `params.json` | all task dimensions and settings |
| `test_scoring.py`, `test_policy.py` | unit tests (scorer rows, tracker readings, draws, pull law; perception on synthetic frames and demo dry runs) |
| `ENVIRONMENT.json` | pinned runtime |
| `LICENSE` | MIT |

## Assets and licences

- Task code and parameters: own work, written for this task (MIT, `LICENSE`). No third-party meshes; every body is
  generated from `params.json`.
- YAM arm model: loaded from yam-arena / i2rt (MIT License, I2RT Robotics); not redistributed here.
- Isaac Sim, Isaac Lab, IsaacLab-Arena, inspect-robots and dreamlabs-client: their own licences, installed separately.
- The sheet, magnet discs, box and index cards are purchased products; listing values are cited only. No product
  images, text or models are redistributed.
- `magnets.scad`: own work, MIT (`LICENSE`).
