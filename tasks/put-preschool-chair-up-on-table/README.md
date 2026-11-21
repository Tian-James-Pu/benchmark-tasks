# Chairs up for the night: lift a preschool chair, turn it over, set it seat-down on its table and push it fully on

A 12 in classroom stack chair stands on the floor in front of its 20 in activity table, facing the arms, its back to
the table. The robot has to put it "up" for the night the way classrooms do before the floor is mopped: lift it, turn it
over, set it down upside down with its seat flat on the tabletop and its back hanging over the edge, and leave it fully
on the top, without dropping it and without its legs touching the table.

**Instruction:** "End of the day: put this chair up on the table so the floor can be mopped, upside down with its seat
flat on the tabletop and pushed fully onto it. Set it down; don't drop it or knock its legs on the table."

**Why it is hard.** The chair weighs 2.72 kg and is 56 cm tall, and its weight sits behind and below the seat (the back
and the steel legs), so it swings as it is turned. Its seat can only lie flat on the top with the back hanging over the
edge. A gripper holding the seat's side edges has one finger under the seat, so the seat cannot be lowered flat onto the
top with that finger under it. The demonstrated way (one of several) is to set the chair down with the hands just outside
the near edge, the seat partly over the edge, then let go and push the chair fully on by its hanging back. Done
carelessly, the chair is let go above the top (a drop), or turned past level so a leg strikes the top.

**Arms.** Both. Each hand holds one side edge of the seat. One hand alone cannot turn the chair: in our one-arm test the
chair swings out of the grip and falls (score 0).

## Independent implementation of a lapsed idea

The chairs-up idea was first submitted by another team in PR #2610 ("End-of-day chairs up"). It was reviewed "Not
accepted" (revisable), no revision followed, and its 48-hour hold lapsed. This folder is our own implementation: a
different chair (one with a published mass), a table at the height its maker pairs with that chair, our own scene,
scorer and demo. We read only that PR's task text and review, not its code. Its review asked for three changes, and this
version answers each:

| Review point | How this version answers it |
|---|---|
| "The demo releases the chair about 4 cm above the table, while the instruction requires setting it down without dropping it. Define and score a controlled placement." | Scored as **dropped (0)**: the chair is let go (no fingertip within 12 mm of it) while every part of it is more than 1.5 cm above what is under it. Checked every step and kept for the rest of the trial; only a reset clears it. The text in `task.json` says the same. |
| "Provide a full-score demo that achieves it within the run limit." | In the demo the seat is lowered until it rests on the top (seat corners within 5 mm of the top) before the fingers open. Full score is reached at about step 950 of 2,400, twice in a row. Our red-team run that lets go about 3.4 cm above the top scores 0. |
| "Check contact by the whole leg, not just its foot, so a leg shaft cannot touch the tabletop during the carry without being detected." | `LEG_PTS` samples each leg about every 8 mm from where it meets the frame down to the glide's underside, 41 points per leg. Any point whose tube surface comes within 3 mm of the tabletop slab (its top face or its edges), at any time, scores 0. Our red-team run that turns the chair past level near the table, so a front leg's shaft strikes the top, scores 0. |
| Flag: "Confirm the real chair's mass and grip geometry." | The chair is the Regency Andy 12 in stack chair, model 4500, listed at 6 lb (2.72 kg) with a 12 x 12 in seat and 22 x 14 x 16 in overall (sources below). The hands hold the seat's side edges, 3 cm in, as a person does. |

## Importance and sources

- **The operation is an everyday routine.** At the end of the school day, chairs go up on the desks and tables so the
  floors can be cleaned. A school-cleaning article states the rule as "Stack all student chairs or place on top of
  desks." ([Spaces4Learning, "What Should You Expect When Your Room Is Cleaned?", 1 Nov 2010](https://spaces4learning.com/Articles/2010/11/01/What-Should-You-Expect-When-Your-Room-Is-Cleaned.aspx?p=1)).
  *Inference:* most adults did this themselves as pupils, which is why we claim Societal relatability, not Impact on jobs.
- **The people who do it daily.** Janitors and building cleaners held 2,432,600 jobs in 2025, and their duties include
  "Sweep, mop, or vacuum building floors" ([BLS Occupational Outlook Handbook](https://www.bls.gov/ooh/building-and-grounds-cleaning/janitors-and-building-cleaners.htm)).
  Childcare workers held about 1,010,300 jobs in 2025 ([BLS OOH](https://www.bls.gov/ooh/personal-care-and-service/childcare-workers.htm)).
  *Inference, not stated by BLS:* staff in preschools and schools put chairs up as part of this floor care. We don't
  claim it as a core duty of either occupation.
- **The chair.** Regency Andy 12 in polypropylene stack chair, model 4500 (4500BK in black). The listing gives 22 in H x
  14 in W x 16 in D, a 12 in seat width, depth and height, a weight of 6 lbs, and a chrome tubular steel frame
  ([Staples Advantage listing, 4500BK](https://od.staplesadvantage.com/regency-seating-andy-12-polypropylene-stack-chair-black-4500bk/product_24287951)).
  *Our estimates (not published):* the shell thickness (6 mm), the rolled rim under the seat's edges (28 mm deep), the
  leg tube (19 mm), the back's rake (10 deg) and the split of the mass between shell and frame (1.38 kg / 1.34 kg).
- **The table and its height.** Correll 24 x 48 in activity table (AWB2448DE-REC-80-14-39): 24 x 48 in top, 1 1/4 in
  thick, high-pressure laminate over a particle-board core, legs adjustable from 19 to 29 in in 1 in steps, $170.99
  ([WebstaurantStore listing](https://www.webstaurantstore.com/correll-24-x-48-rectangular-white-19-29-adjustable-height-high-pressure-dry-erase-board-top-activity-table-with-silver-legs-and-green-t-mold/3842448DE526.html)).
  Set at 20 in because Community Playthings' height guide pairs a 12 in chair with a 20 in table top, for ages of about
  3.5 to 6 ([table and chair height guide, PDF](https://www.communityplaythings.com/-/media/files/cpus/product-information/tablechairheightguides-2026.ashx)).
  Lakeshore Learning gives 19 to 23 in tables for 12 to 13 1/2 in chairs in kindergarten and grade 1 ([size chart](https://www.lakeshorelearning.com/assets/html/modals/pdp/modal-nonflex-sizechart.html)).
  *Our estimate:* the table's leg section (32 mm square).
- **Cost of a real trial:** one chair (a few tens of dollars; sold in packs of 4) and one table ($171).

### Distinct from

| Nearest task | What it tests | Here instead |
|---|---|---|
| Hang washed wine glasses upside down in an overhead bar rack (accepted) | Inverting a light glass into a rail | A 2.7 kg, 56 cm object turned over between two hands, lowered onto an edge and pushed home |
| Retain a housing in a rollover fixture, turn it over and lock it (accepted) | A fixture turns the part | Nothing holds the chair but the two hands; its weight swings off the grip line |
| Open a folding table's leg and slide its brace ring over the knee before the table is turned over (accepted) | Locking a leg brace | Turning over and placing a whole chair, then pushing it fully on |

We found no chairs-up task among RoboARC's 106 standard tasks. We could not check BEHAVIOR-1K's activity list for one,
and make no claim about it.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source ~/IsaacLab-Arena/yam.env`). `mechkit.py` and `simready.py`
are in this folder. Importing them hooks `yam_arena` at load time without changing it on disk. The SimReady props
download at first run.

## Run a trial

From the repository root:

```bash
inspect-robots run --embodiment yam_arena --policy dream -E task=tasks/put-preschool-chair-up-on-table/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "End of the day: put this chair up on the table so the floor can be mopped, upside down with its seat flat on the tabletop and pushed fully onto it. Set it down; don't drop it or knock its legs on the table." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/chair-up
```

The scripted demo is the same command with `--policy yam_scripted` (we record it at 640 x 360). It plays `demo()` in
`task.py`. `python check_score.py`, run in this folder, feeds the real `Trial` hand-built states for each outcome
below, with no simulator.

**Runtime and seeds.** The demo reaches full score at about step 950 (47.5 s simulated). A trial takes about 3 to 4
minutes of wall time on one RTX-class GPU, start-up included. Nothing is random (`object_jitter=0`). The top camera is
moved up and back (`CAMERA_POS`, `CAMERA_TARGET` in `task.py`) so that the raised table stays in view. The change is the
same for every policy.

## The scene

The station's bench top is the classroom floor. Positions are (x forward from the arm bases, y left) in cm, heights
above the floor.

- **Table:** the Correll top is a fixed 61 x 122 x 3.2 cm slab, its top face 50.8 cm up. Its long side runs across the
  arms, its near edge at x = 40, centred on y = 0. Four 3.2 cm square legs stand 6 cm in from its ends, and a green
  T-mold band runs round its edge.
- **Chair:** one rigid body of 2.72 kg, its centre of mass taken from its parts: 3.9 cm behind the seat's middle and
  3.5 cm below its top face. It stands upright with its seat centre at (7.7, 0), facing the arms, the back's top 12 cm
  from the table's edge.
  - Seat: 30.5 x 30.5 cm, 0.6 cm thick, top face 30.5 cm up, with a 0.5 cm wall rolled under its front and sides to
    2.8 cm below the top face.
  - Back: 29 x 25.8 cm, raked back 10 deg, rising from the seat's rear edge, with a carry slot.
  - Frame: two 1.9 cm steel rails under the seat, and four splayed 1.9 cm steel legs on nylon glides, 33.4 cm apart
    side to side and 36.7 cm front to back at the glides.
- **Distractors** (NVIDIA SimReady props on the tabletop; scoring ignores them): a spiral notebook at (84, 36), a paper
  cup at (88, 10) and a plastic crayon basket at (86, -30), all beyond where the chair ends up.

## Scoring

Scoring is read every control step by `Trial` in `task.py` (registered with mechkit's `on_step`). It uses the chair's
pose and velocity and the 24 fingertip points that mechkit's `fingertips()` reports. A drop or a leg strike is recorded
when it happens and cleared only in `on_reset`. Nothing is cleared because the chair later looks right.

| What happened | Score |
|---|---|
| Lifted while held (a fingertip within 12 mm of the chair), with every part of it 5 cm or more above what is under it | 0.25 |
| Also turned over while held off the floor: the chair's up axis within 30 deg of straight down | 0.5 |
| Also at rest upside down (within 60 deg) on the tabletop, touching nothing else: a part within 5 mm of the top, none within 2 cm of the floor. Every fingertip 3 cm or more from the chair, and still (under 1 cm/s and 0.05 rad/s) for 1 s | 0.75 |
| Also seat flat (within 5 deg), all four seat corners within 5 mm of the top's height and no more than 4 cm out past its edge, held 3 s (the tracker's 2 s, then `SUCCESS_HOLD_S` = 1 s) | 1.0 |
| **Dropped:** no fingertip within 12 mm while every part of the chair is more than 1.5 cm above what is under it. Or **a leg or glide within 3 mm of the tabletop** at any time | 0 |

"What is under it" is the tabletop for a point above the top's outline, and the floor otherwise. The levels are cumulative:
0.75 also needs the chair to have been turned in the hands, and 1.0 needs everything in 0.75.

**Why 4 cm.** Set down with its back hanging over the edge, the chair is stable only while its centre of mass is over the
top. Our demo first sets it down 11 cm over the edge, with its centre of mass about 1 cm inside it. A bump would tip it
off, onto the floor or a child, so that placement earns 0.75. Pushed home (2 cm over in the demo), its centre of mass is about
9 cm inside the edge.

**A real trial.** Mark the chair's glides and the table's legs on the floor with tape (reset in under a minute). Score
from the video and a ruler:

- a drop: the chair leaves the hands more than 1.5 cm, about a finger's width, above the top;
- a leg strike: any leg or glide touches the top, visible and audible;
- the end state: a level on the upturned seat (within 5 deg), and a ruler from the table's edge to the seat's rear edge
  (4 cm at most);
- hands clear: 3 cm, against a mark.

A YAM gripper's pads are rubber and touch only the plastic seat, so the rig needs no protection.

## Results (all through the station's scripted policy, `run_trial`)

| Run | What it does | Score |
|---|---|---|
| demo (twice) | Grips the seat's side edges, lifts and turns the chair, sets it down 11 cm over the edge with the seat resting on the top, lets go, slides the fingers out, goes round the back and pushes the chair home (2.2 cm over), moves away | 1.0, 1.0 |
| none | The arms wait | 0 |
| one arm | Only the left hand does the demo's motions | 0: the chair swings out of the grip and falls (dropped) |
| careless drop | The demo, but the chair is let go about 3.4 cm above the top | 0: dropped (it then tumbles off) |
| hold | The demo up to the set-down, then never lets go | 0.5 |
| no push | Set down 11 cm over the edge, let go, hands away | 0.75 |
| legs | Turned past level close to the table, so a front leg's shaft strikes the top | 0: leg strike |

`check_score.py` also runs, on hand-built states: left standing (0); lifted and set back (0.25); turned and put upside
down on the floor (0.5); set down 11 cm over (0.75); pushed home 2 cm over (1.0); 3.9 cm over (1.0); 4.5 cm over (0.75);
a hand left 2 cm off the seat (0.5); still held (0.5); still for only 1.5 s (0.75); tilted 8 deg on the top (0.75);
still sliding at 2 cm/s (0.5); let go 4 cm above the top and landing home (0); a leg's shaft 2 mm from the top while
held, then home (0).

## What is simplified (fidelity)

- The chair is one rigid body made of plates and tubes at the maker's overall sizes and listed weight. It has no moulded
  curves. Its rim depth, tube size, back rake and mass split are our estimates.
- Friction is uniform: 0.9 static, 0.8 dynamic. No force is read.
- "Held", "let go" and "hands clear" are read from fingertip points, not from contact forces.
- The table is fixed, and the bench top stands for the floor.
- The arms' simulated wrists are compliant. Under the chair's weight they sag several centimetres and over-turn by up to
  about 20 deg, so the scripted demo's waypoints are pre-compensated with offsets we measured in earlier runs (it reads
  no simulator state). The demo is open loop.

## Assets and licences

All chair and table geometry is generated in `task.py` with `mechkit.py`'s meshes. SimReady has no classroom stack
chair or activity table, so both are built from primitives at the products' published sizes. The three distractors are
NVIDIA SimReady props, downloaded at first run from the pinned revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`,
© NVIDIA, CC-BY 4.0. No meshes or textures are bundled.

- [misc_notebook_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/misc_notebook_a01): mass set to 0.12 kg.
- [dishware_cup_paper_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_cup_paper_a01): mass set to 0.01 kg.
- [misc_basket_plastic_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/misc_basket_plastic_a01): mass set to 0.15 kg.

`simready.py` is adapted from the open-source loader in `tasks/cutters-off-pegboard`, and `mechkit.py` is copied
unchanged from `tasks/heat-shrink-lug`, both in this repository. Product names identify the real items modelled and are
trademarks of their owners.

## Licence

MIT
