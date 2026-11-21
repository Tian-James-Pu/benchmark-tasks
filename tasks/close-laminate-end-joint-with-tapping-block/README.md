# Close a laminate plank's open end joint with a tapping block and a mallet

**Instruction:** *Close the open end joint of the laminate plank that was just laid.*

A floating laminate floor is being laid. The plank just put down in row 2 (a Pergo Pro plank, 1200 x 190 x 10 mm, Uniclic
joints) is locked flat along its long edge into row 1, but its far end stands 5 mm short of the plank before it in the row: the
end joint is open. Its near end, the end groove with its protruding lower lip, faces the arms. A Roberts 10-49 tapping block lies
on the subfloor in front of it and a rubber mallet lies on the laid floor to the right.

The end only closes by tapping. A plank locked along its long edge will not slide under a steady push, and the mallet must not
touch the plank itself: struck on its own end, the groove's lower lip is crushed and the next plank will not lock. So one hand takes
the block by its D-handle and sets it flat and square against the plank's end, with the block's relief over the lip. The other hand
holds the mallet choked up near the head and taps the block's back in line with the plank's centre. Each tap moves the plank a
fraction of a millimetre to about 2 mm. The robot taps until the plank stops moving, then puts the mallet down and lets go.

## Importance and sources

- **Who does it.** Installing floors is core work for carpenters: O*NET 47-2031.00 lists the task "Install structures or
  fixtures, such as windows, frames, floorings, trim, or hardware, using carpenters' hand or power tools." It reports 889,700
  carpenters employed in 2025. Floor layers (O*NET 47-2042.00, "Floor Layers, Except Carpet, Wood, and Hard Tiles", 29,800
  employed) lay floor coverings as their whole job. Their task list includes "Trim excess covering materials, tack edges, and join
  sections of covering material to form tight joint."
  General maintenance and repair workers (O*NET 49-9071.00, 1,621,800 employed) have floors in their core task list too: "Paint
  or repair roofs, windows, doors, floors, woodwork, plaster, drywall, or other parts of building structures."
  - https://www.onetonline.org/link/summary/47-2031.00
  - https://www.onetonline.org/link/summary/47-2042.00
  - https://www.onetonline.org/link/summary/49-9071.00

  Limit: these are occupation totals. Not every carpenter or maintenance worker lays click laminate every week, and no source
  counts how many do. A review of another team's click-lock vinyl tile task (#1920, below) did not accept the carpenter count for
  click-lock flooring.
- **The operation, from the maker.** Pergo's installation instructions for Pergo Pro + WetProtect (Lowe's, January 2026)
  give two ways to join planks. Steps 8-9 angle each plank in: short end first, then the long side. Step 10, "Alternate Tap
  Method", is the start state of this task, and reads: "Install the long side first by placing a tapping block no closer than 8
  inches from either end and tap lightly along the long side until the joint is closed tightly. Then tap the end into a locked
  position using the tapping block. Note: Uneven tapping or use of excessive force may damage the joint." For a plank slid under
  a door casing (steps 15-16), where it cannot be angled in, the order is "Tap the long-side joint closed first. Then tap the end
  joint closed." The tool list includes a hammer and a tapping block. The Pergo Outlast+ Uniclic instructions (2016) carry the
  same steps.
  - https://pdf.lowes.com/productdocuments/40986e4d-2722-4099-a100-1044be725634/80667380.pdf
  - https://images.homedepot-static.com/catalog/pdfImages/06/0647a886-6aef-4b58-b46a-a7984c9eb457.pdf
- **The plank.** Pergo Pro technical data sheet TDS_0053 (August 15, 2023). It gives 190.0 x 1,200.0 mm, 10 mm total (8 mm HDF
  plus a 2 mm attached pad), Uniclic joint, and 11 planks and 17.6 kg per carton, so 1.6 kg a plank. ISO 24334 joint strength is
  over 4.5 kN/m on the short side.
  - https://pdf.lowes.com/productdocuments/93715f80-6cae-4c63-9788-7cee8abcac28/64792536.pdf
- **The tool.** Roberts' product sheet for the 10-49 Double Tap reversible tapping block gives a 10 in x 3-1/2 in x 3/4 in body
  in reinforced nylon. It is "designed to push planks together without damaging the flooring or your fingers", with a click-lock
  side and an "offset handle [that] keeps fingers and knuckles away from the strike zone". The sheet adds that "the offset handle
  allows you to secure the flooring in place using a soft-head mallet to tap and connect boards." It sells for about $11 to $21.
  - https://robertsconsolidated.com/wp-content/uploads/2025/04/10-49_Reversible_Tapping_Block_PDS_H1725-12562.pdf

**Why it is a robot problem.** It is tool-on-tool work with two hands. One hand keeps a 254 mm block flat on the subfloor and
square to the plank's end, against the block's own rebound. The other swings a mallet low over the floor, fast enough to break the
joint's static friction. Each tap must land on the block's back, not on the plank, and the robot watches the plank's end to decide
when to stop. In simulation a block set on the end and let go, then tapped one-handed, also closes the joint. That route is
allowed (red team `onearm`), as it is for a person, though installers hold the block. A steady shove, however hard, does nothing (red team `push`). The obvious shortcuts close the joint and score 0: the mallet
straight on the plank's end, which closes it in one blow (red team `careless`), and ramming the end with a closed gripper (red
team `punch`).

## Distinct from

| Nearest | What it tests | Why this is different |
|---|---|---|
| #897, TK-BBQ5RL, "Lay three floorboards tight together, leaving a 10-20 mm expansion gap at each wall" (not accepted: final) | Placing short, plain-edged boards side by side in a miniature bay | Rejected for one-fifth-length boards and for having "no tongue-and-groove or click-joint engagement". Here a full-size, real-product plank is already in its long-edge lock, and the task is the engagement itself: closing the click end joint by tapping through a block. |
| #2206, TK-W437WY, "Start a nail and drive it home" (accepted) | Hold a nail upright while the other hand taps it in with a claw hammer | Vertical blows into wood that the nail penetrates. Here the blows are horizontal, low along the floor, through an intermediate block that must stay flat and square. The struck part is a 1.2 m plank that slides in a lock and must not itself be touched. |
| #2612, TK-4KAT4L / TK-D9RUH2, "Set a drop-in expansion anchor in concrete with its setting tool and a hammer" (pending) | Steady a setting punch in an anchor and strike it down until its shoulder seats | A vertical punch held in a hole that guides it. Here a loose block must be placed and squared on the plank's end, with the relief over the lip, and kept there while the plank moves away under the taps. Success is a closed joint, read with feeler gauges. |
| #2916, TK-UAEDED / TK-YS8M33, "Revise paint-can lid closure with measured mallet strikes" (pending) | Mallet blows straight on a lid rim | There the mallet strikes the work directly. Here striking the work is exactly what is forbidden; the force goes through a tool held in the other hand. |
| #1920, TK-6TJTBW / TK-YZPPJL, "Lay a click-lock vinyl floor tile: angle its tongue into the laid row's groove, then lower it flat to lock" (not accepted: final, on importance) | Angle a tile's long-side tongue into the laid row's groove and lower it flat to lock | The other joining method. There the long side is made by angling in, with no tool. Here the long side is already locked and the end joint can only be closed by tapping through a block (Pergo's tap method, the only one under a door casing). The robot must place a tool, strike a tool with a tool, and never touch the work. |
| BEHAVIOR-1K `laying_wood_floors` | Carry plywood sheets into a room and put them `ontop` the floor and `nextto` each other | No joint, no engagement, no tool: a placement goal (https://behavior.stanford.edu/knowledgebase/tasks/laying_wood_floors-0.html). |

LIBERO, RoboCasa, SimplerEnv and DataCenterBench have no flooring or tapping-block tasks. A search of every task pull request's
title in this repository (21 November 2026) for "laminate", "floor", "mallet", "hammer", "tapping" and "plank" found only the
rows above as near neighbours.

## Run and score

From the repository root, in the Python environment that has `yam_arena` and `inspect-robots`. A model run:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/close-laminate-end-joint-with-tapping-block/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Close the open end joint of the laminate plank that was just laid." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/close-laminate-end-joint-with-tapping-block
```

The scripted demonstration is closed loop, so it runs through `demokit.py` from the task folder. It reads the block's and
mallet's poses and the plank's position (its near end is in view), never the scorer.

```bash
cd tasks/close-laminate-end-joint-with-tapping-block
python demokit.py task.py ~/yam-eval/close-laminate-end-joint-with-tapping-block/demo --mode full        # writes full/demo.mp4, setup.png, result.json
python demokit.py task.py ~/yam-eval/close-laminate-end-joint-with-tapping-block/redteam --no-record --mode none,push,onearm,careless,punch
python demokit.py task.py ~/yam-eval/close-laminate-end-joint-with-tapping-block/cal --fast --mode calibrate   # the joint's friction, by steady pushes (CPU)
```

`result.json` holds `task_score`. The layout is fixed (`object_jitter=0`); there are no seeds or random choices. The demo ends at
about 550 to 600 control steps (27 to 30 simulated seconds); the run limit is 2,400 steps. A recorded GPU run took about 10 minutes
of wall time on a heavily shared GPU; a CPU `--fast` run (no cameras) takes about a minute.

## How the simulator models it

- **The floor.** One fixed mechanism, mounted on the table: the plywood subfloor, rows 0 and 1, and the previous plank of row 2,
  all welded to the world. The plank just laid is its one moving link, a slider along its own length (joint 0 = 5.0 mm open,
  +5.0 mm = closed, with a stop there, where the far end meets the previous plank).
- **The lock's resistance.** The slide has joint friction: 70 N static, 10 N sliding, plus 20 N s/m viscous, set at every reset.
  This stands in for the long-edge lock's grip and the end lock's click. The numbers are not measured; they were chosen against
  the robot. Pushed alone (no arms) on the CPU solver, the plank creeps 0.08 mm under 60 N held 1 s and slides under 70 N
  (`calibrate` mode). The arm's hardest steady push through the block, its command up to 12 cm past the end, moved it 0.2 mm
  (`push` mode). Then the arm's grip on the D-handle slips and the block tips, as a real block does under a hard shove. A tap
  delivers a short impulse that breaks the static friction, and the plank slides until the sliding friction stops it.
- **The block.** A free rigid body built from boxes at the Roberts 10-49's size: body, relief, and D-handle with posts and bar.
  The relief clears the plank's groove lip, so the block bears on the face above it, as the click-lock side of the real block does.
- **The mallet.** The SimReady rubber mallet at its own size. Its single SDF mesh collider jittered on the floor and gripped
  poorly, so at spawn it is switched off and replaced by two cylinders matching the head and the handle. The authored mass (0.35
  kg, centre of mass on the handle) is replaced by 0.45 kg: a 0.33 kg head on a 0.12 kg handle.
- **Direct blows.** At every control step the scorer samples 90 points on the mallet head's surface (both faces and the barrel)
  and mechkit's 24 fingertip points. It measures their distance to the plank's end zone: the end face, the protruding lower lip,
  and the top within 25 mm of the end. A head point within 2 mm, or a fingertip point within 7 mm, is recorded as a direct blow.
  The record is cleared only by mechkit's `on_reset` hook.
- **Hands clear.** mechkit's `fingertips()` gives 24 points on the two grippers. Each must be at least 30 mm from the plank's box
  and lip and from the tapping block's box (handle included).

## Scale and mass

Nothing is scaled. The plank is the one large object, and only its near end is worked, so it is modelled at its full 1,200 mm.
The floor section is cropped to 1.64 x 0.82 m and stands on the table as the room's floor. It overhangs the table's far edge onto a
trestle, and the plank's far end and its joint sit 1.6 m out, beyond the top camera's view. The plank's near end shows the same
5 mm of travel, and the robot stops tapping when the end stops moving.

| Object | Real (source) | Modelled |
|---|---|---|
| Pergo Pro plank | 1,200 x 190 x 10 mm, 1.6 kg (17.6 kg per 11-plank carton; TDS_0053) | same size and mass |
| Floor (subfloor, rows 0 and 1, previous plank) | a room's floor | a 1.64 x 0.82 m section at table height, fixed; same plank size |
| Roberts 10-49 tapping block | 254 x 89 x 19 mm body (Roberts product sheet); about 91 mm with handle (retailer listings); mass not published | same size; handle bar and posts estimated from photos; 0.30 kg estimated |
| Rubber mallet (SimReady `handTool_hammer_rubberMallet_b01`) | 299 mm long, head 67 mm dia x 114 mm (the SimReady model's own size) | scale 1.0; mass 0.45 kg, head-heavy (authored 0.35 kg, handle-heavy) |

## Scoring

The gap is the far end joint's opening: 5.0 mm at the start, and 0 when the plank has slid to its stop.

- **0 for the whole trial** if the mallet's head or a gripper ever touches the plank's end: its end face, the groove's
  protruding lower lip, or its top within 25 mm of the end. In simulation that is the head within 2 mm, or any of mechkit's
  fingertip points within 7 mm. The points lie mid-finger, 4.5 mm inside the finger's face. This is a direct blow, which crushes
  the lip, and it is cleared only at a reset. Ramming the end with the closed gripper closes the joint too (red team `punch`),
  and it is the same damage.
- **1.0:** gap 0.5 mm or less, plank still (sliding slower than 2 mm/s), and every fingertip at least 3 cm from the plank and from
  the tapping block. All of this must hold continuously for 2 s.
- **0.75:** gap 0.5 mm or less without the rest: a hand still on the block, say.
- **0.5:** gap 1.5 mm or less.
- **0.25:** gap 3.0 mm or less.
- **0** otherwise. Doing nothing leaves 5.0 mm.

Holding the block is not required: a block set on the end and tapped one-handed counts if the joint closes. The mallet may be left
anywhere off the plank's end. These are `level()` and `Trial.step()` in `task.py`.

## Real trial: preparation, reset and measurement

**Parts (about $150):**
- One carton of Pergo Pro laminate (11 planks).
- A 4 x 8 ft sheet of 3/4 in plywood, cut to 1.64 x 0.82 m, for the subfloor.
- A Roberts 10-49 Double Tap tapping block.
- A 16 oz rubber mallet (about 300 mm long, 65 mm head).
- Feeler gauges (0.05 to 1.0 mm).
- A 5.0 mm spacer (a stack of feeler leaves, or a 5 mm flooring spacer).
- Masking tape and a marker.
- The distractors: a retractable utility knife with its blade retracted and taped, cloth work gloves, and a roll of tape.

**Preparation.**
1. Clamp the plywood on the robot's table with its near edge 10 cm in front of the arm bases' line. Its far end goes on a sawhorse
   or a second table at the same height, so the panel is flat and level.
2. Lay rows 0 and 1 from cut planks, with their end joints at x = 55 cm and 76 cm (45 and 66 cm from the panel's near edge),
   all clicked together as Pergo describes.
   Hold them with double-sided carpet tape along their undersides: a floating floor in a room is held by its own weight and walls,
   and the tape stands in for those.
3. Cut a 13.5 cm piece from the groove end of a plank: this is the previous plank of row 2. Click it to row 1 at the far end and
   tape it down.
4. Angle a full plank into row 1's long-side groove with its end tongue towards the previous plank, and slide it until its tongue
   is in that plank's groove. Put the 5.0 mm spacer in the end joint and fold the plank down flat, so its long edge locks. Pull
   the spacer out.
5. Check with feeler gauges at both edges of the end joint: 5.0 mm, within 0.2 mm.
6. Set the block and mallet on their tape marks (the block's bearing edge's middle at (25.5, 12) cm, yaw 12 degrees; the mallet's
   head centre at (21.5, -15) cm, handle pointing right).
7. Set the distractors on theirs.

**Measurement**, before anyone touches the floor:
- The gap at the far end joint, with feeler gauges at both long edges. The reading is the larger of the two.
- "Still" and "hands clear" are read from the last 2 s of the top-camera video.
- A direct blow is read from the video (any frame with the mallet's head on the plank's end) and confirmed on the plank: a dented
  or crushed lip on its near-end groove.

**Reset**, about 2 minutes:
1. Lift the plank's free long edge about 2.5 cm to unlock it.
2. Slide it back against the 5.0 mm spacer and fold it down.
3. Re-check the gap.
4. Return the block and the mallet to their marks.

A plank whose near-end lip is dented is replaced (the carton holds spares). The setup lasts many trials: the Uniclic joint is
designed to be unlocked and relaid.

**Rig protection.** The block is nylon and the mallet rubber; there are no blades or edges in the work area. The knife's blade
is retracted and taped. The plank's top is a wear layer, and the grippers do not touch it in the demonstration.

## Assets and licences

The floor, the planks and the tapping block are authored primitives: SimReady has no laminate flooring or tapping block. The mallet
and the three distractors are NVIDIA SimReady props. They download at first run from the pinned revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd` of `nvidia/simready-assets` on Hugging Face, © NVIDIA, CC-BY 4.0:

- [handTool_hammer_rubberMallet_b01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_hammer_rubberMallet_b01):
  its mesh collider is switched off and two cylinders are added. Mass 0.45 kg, centre of mass and inertia set for a 0.33 kg head
  on a 0.12 kg handle (`_mallet_colliders()` in `task.py`).
- [handTool_knife_retractableBlade_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_knife_retractableBlade_a01):
  mass 0.12 kg, convex-hull collision.
- [ppe_gloves_cloth_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/ppe_gloves_cloth_a01):
  mass 0.12 kg.
- [tape_clear_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/tape_clear_a01):
  mass 0.06 kg.

Nothing large is committed. Three files are other teams' open-source files in this repository:
- `simready.py`, the loader from `tasks/cutters-off-pegboard/`, unchanged apart from its docstring. `task.py` wraps its
  `_prepare` for the mallet only.
- `mechkit.py`, the mechanism kit from `tasks/heat-shrink-lug/`, unchanged.
- `demokit.py`, the closed-loop demo runner from `tasks/heat-shrink-lug/`, unchanged.

## Validation

All on the GPU solver unless marked CPU. A `full` run is played fresh in its own simulator session. The layout has no randomness,
and three fresh GPU runs gave the same tap sequence: two recorded, one without video. The earlier recorded run was made before the
debug output was removed from `task.py`.

| Run | Score | What happened |
|---|---|---|
| `full`, recorded (the submitted video) | **1.0** | Block squared on the end, then 5 taps. Gap 5.03, 4.29, 1.95, 1.44, 0.93, 0.00 mm. Mallet laid on row 1, both hands clear, success held 2 s at step 546 (27.3 s). |
| `full`, second fresh run (no video) | **1.0** | The same: 5 taps, success at step 546. |
| `none` | 0.0 | Gap stays 5.0 mm. |
| `push`: block held on the end and pushed, command 4, then 8, then 12 cm past the end, 3 s each, no mallet | 0.0 | Gap 5.0 to 4.92 mm. At 8 cm the grip slides on the handle and the block tips. |
| `onearm`: block set on the end and let go, then tapped one-handed | 1.0 | 5 taps close it. Allowed: the score is the end state. |
| `careless`: block set aside, mallet straight on the plank's end | 0.0 | One blow closes the joint, and the direct blow zeroes it. |
| `punch`: no tools, the closed right gripper rams the plank's end 16 times | 0.0 | Gap 5.0 to 2.3 mm (0.25 on the gap alone); gripper contact with the end zeroes it. |
| `calibrate` (CPU): steady pushes on the plank, no arms, 1 s each | n/a | 10 to 60 N: 0.01 to 0.08 mm of creep. 70 N: slides to closed. |

IK worst error in the demo: 0.1 mm, 0.0 degrees for each arm. The closest the mallet's head came to the plank's end zone was 84 mm,
and the closest fingertip 63 mm (CPU run with the same script).

**Model run.** Dream-1, with the instruction above, ran the full 2,400 steps on the GPU solver and scored 0.0: its right gripper
went to the mallet's handle and stayed there without lifting the mallet, and neither hand touched the tapping block, so the joint was
never tapped.
