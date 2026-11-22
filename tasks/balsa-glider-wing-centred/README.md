# Fit a balsa glider's wing through the slot in its body, centred and square

Instruction: *"Fit the wing through the glider's body, centred and square, and leave the glider on the table."*

A Guillow's Jetfire (No. 30) balsa glider, the 12-inch dime-store toy, at full size. Its tailplane and plastic nose clip are already
on, and the glider stands on the table on its tailplane and the lower edge of its body. Its one-piece balsa wing lies flat on the
table beside it. The robot has to slide the 2 mm wing through the slot in the 3.2 mm balsa body, which is only 0.8 mm taller than the
wing is thick, push it on until it stands out equally on both sides and square to the body, and leave the glider standing on the
table with both hands away. A wing levered or wrenched in the slot pries it open and splits the thin strip of balsa under it.

| File | What it is |
|---|---|
| `task.py` | Scene, the slot's give and crack, the scorer and the closed-loop demonstration (`demo_script`) |
| `task.json` | The task's details |
| `mechkit.py` | The mechanism kit, unchanged apart from a credit line |
| `demokit.py` | The closed-loop demo runner, unchanged |
| `simready.py` | The SimReady prop loader, unchanged |

## Importance and sources

A cheap balsa glider, bought at a toy shop, a hardware store or a museum gift shop and put together at the kitchen table, is a
common childhood job: three or four die-cut balsa parts, no glue, and the one fiddly step is getting the wing through the body. The
sources below establish the product, its construction and its reach; how many adults have done it is not counted anywhere we
could find, so that part of the claim is ours.

- The product: Guillow's Jetfire, No. 30, "A high performance flyer with the style of a jet fighter. Large 12″ wing span with a built in
  dihedral", $4.99 from the maker (the current 100th-anniversary print is No. 30-100, also $4.99).
  https://www.guillow.com/?p=5825 and https://www.guillow.com/products/jetfire
- Construction: "No glue required, slot and tab construction", pre-cut balsa parts and a plastic nose piece, ages 6 and up; also sold as a
  pack of 48 (HobbyLinc). https://www.hobbylinc.com/htm/gui/gui30.htm
- "The Jetfire glider is a fairly basic design, consisting of a body, nose-clip, wings, and a tail", whose "pieces fit together with no
  glue or additional fasteners needed" (Rainbow Resource). https://rainbowresource.com/018653.html
- How the wing goes on, in the words of Guillow's designer Mark Tennant: "The wing slot in the fuselage is longer than the wing, so you
  can slide that wing forward and backwards in the plane", and where it sits decides whether the glider flies straight or does stunts
  (WBUR, 29 May 2026, on the company's 100 years in Wakefield, Massachusetts; the photo caption calls the Jetfire "iconic").
  https://www.wbur.org/news/2026/05/29/balsa-wood-airplanes-guillows-100-years-wakefield
- Reach: Guillow's has mass-produced "simple, inexpensive balsa gliders" since 1953, with sales estimated at US$5 million to $6 million a
  year in 2011 (Wikipedia, citing the company's history). At a few dollars a glider that is on the order of a million gliders a year;
  that figure is our inference, not a published count. https://en.wikipedia.org/wiki/Paul_K._Guillow,_Inc.

**The knack.** The wing is a 2 mm sheet of balsa weighing about 4 g, and the slot it goes through is 0.8 mm taller than the sheet,
3.2 mm deep and 7 mm above the table. The wing's tip (45 mm chord) goes in only when it meets the slot within about 0.4 mm of its
middle height and within about one degree of its plane (0.8 mm over 45 mm); otherwise it butts against the side of the body. Once in,
the wing has to be slid about 150 mm on along its own plane while the hand holding it, up to 18 cm from the slot, keeps it level with
the slot: raising or lowering that hand levers the wing about the slot, and past about 14 degrees (0.8 mm over the 3.2 mm depth) it
pries the slot's top and bottom apart, which splits the strip of balsa under it. Then the wing has to be stopped with equal overhang
either side (within 3 mm of centre) and square to the body (within 3 degrees), which nothing in the slot enforces: the slot is longer
than the wing's chord and only 3.2 mm deep. Done carelessly, the wing ends up crooked or off to one side, and the glider banks and
turns away in flight.

Holding the glider is not required by the scoring. The demonstration steadies the body with one hand while the other feeds the wing,
as people do, but in the simulation a wing fed level slides through the slot of a free-standing glider without moving it, and the
one-arm check reached full credit (see Validation). The real slot is a light friction fit, which is why people hold the body; the
simulated slot has no friction preload (see "How the simulator models it").

**Distinct from** the nearest tasks we know of:

| Nearest | What it tests | Difference here |
|---|---|---|
| Fit a U-joint cross through its yoke and slide it back to centre (TK-QJ8724, accepted) | rigid steel journals through steel ears, a swing to level, then centring by the ears | a 305 mm, 4 g sheet through a slot only 0.8 mm taller than it, where levering it in the slot splits the part; square and centre are judged by eye, nothing stops the wing at centre |
| Push a straw through the X-slot in a cold cup's lid (TK-KAXT6V, accepted) | forcing a straw through flaps into a standing cup | a flat sheet that must stay in its plane, then be centred and squared |
| Assemble a slotted partition (TK-AXP9BD, accepted) | dropping notched boards onto each other from above | a slide-through fit along the sheet's own plane, with a part that breaks if forced |
| Table fan reassembly (PR #2189, being revised) | blade on a pin, a spinner screwed on, a grille clipped | a toy's no-glue slot fit; nothing to screw or clip |

No board title mentions a glider, balsa or a model aircraft. Of the external sets, LIBERO, RoboCasa, SimplerEnv, BEHAVIOR and RoboDojo
have no glider or slot-through-sheet assembly that we know of.

## Run and score

From the repository root, with the simulator's environment loaded (`source ~/IsaacLab-Arena/yam.env`):

```bash
# a model (or any policy) on the task, scored by the task's own scorer
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/balsa-glider-wing-centred/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the wing through the glider's body, centred and square, and leave the glider on the table." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/balsa-glider-wing-centred
```

The scripted demonstration is closed-loop (it reads the glider's and the wing's poses as a camera would, never the scorer), so it
runs through `demokit.py` from the task's folder:

```bash
cd tasks/balsa-glider-wing-centred
python demokit.py task.py OUT                       # the full-score run, with video: OUT/full/demo.mp4, OUT/full/result.json
python demokit.py task.py OUT_checks --no-record --mode none,onearm,careless,held,offcentre   # the checks in Validation
python demokit.py task.py OUT_fast --fast --mode full   # physics on the CPU, no cameras: for debugging only
```

`result.json` holds `task_score`, the harness's own score for the trial. There are no seeds to set: the layout is fixed
(`object_jitter=0`). A trial is at most 2,400 control steps (2 minutes at 20 Hz); the demonstration takes about 1,100 to 1,200 steps (55 to 60 s of simulated time; it retries finding the slot when the first sweeps miss). On a GPU shared with other jobs one trial took 5 to 15 minutes of wall time.

## How the simulator models it

Everything the robot touches is real contact geometry; nothing is triggered by proximity or by a gripper's opening.

- **The glider** (`glider`, a free articulation of 5 g in all): the body is a flat balsa profile 280 mm long and 3.2 mm (1/8 in) thick,
  built from convex pieces round the wing slot (nose, the part above the slot, the tail end and the fin), with the tailplane
  (120 x 40 x 1.6 mm) through its tail slot and the plastic nose clip. The tailplane is a press fit and is part of the body. The glider
  stands on the table on the lower edge of its body and its tailplane, as a real one does.
- **The wing slot** is 95 x 2.8 mm, its bottom 7 mm above the body's lower edge. It is longer than the wing's 75 mm root chord, as the
  maker describes, so the slot fixes neither the wing's fore-and-aft position nor its squareness: only the wing's height and tilt.
- **The slot's give and the crack.** The 7 mm strip of balsa under the slot (`lip`) is a separate link that slides down against an
  8 N/mm spring: the balsa's give, lumped into one place. A wing pushed through flat never loads it (0.8 mm of clearance). A wing pushed
  through tilted bears on the top of the slot at one face and its bottom at the other and wedges it open; past 0.5 mm of give (about 4 N
  of wedging) the strip has split from the slot's ends, as balsa splits along its grain. That is recorded as a crack and kept until the
  scene is reset. The 4 N and 8 N/mm are estimates for 1/8 in balsa, not measurements.
- **The wing** (`wing`, a free body of 4 g): a flat balsa plate 2.0 mm thick, 305 mm span, root chord 75 mm tapering to 45 mm at the
  tips, with a straight trailing edge and a swept leading edge. Its printed stars and bands are visual only.
- **Not modelled:** the wing's built-in dihedral (the plate is flat) and its flex; force on the wing itself (only the slot can crack);
  crushing of balsa under the fingers (the grips used are 6 N on the wing's edges and 25 N on the body's faces, well under what crushes
  medium balsa across the grain over a pad).
- **The fit is a sliding fit with no preload:** a wing fed level touches neither face of the slot, so it slides in without pushing the
  glider, and a free-standing glider can take it (the one-arm check). On a real glider the die-cut slot grips the wing lightly.
- Uniform friction: 0.6 static, 0.5 dynamic on balsa; the arms' pads as in the simulator.

## Scoring

Read from the end state by a tracker that runs every control step (`Trial` in `task.py`); `score()` returns what it holds.

| Score | Condition |
|---|---|
| 0 | The slot has ever cracked (the strip under it forced more than 0.5 mm open). Recorded the moment it happens and cleared only when the scene is reset, so a good final state does not erase it |
| 1.0 | Through; offset at most 3 mm; out of square at most 3 degrees; the glider standing on the table (lower edge within 3 mm of it, tilted at most 10 degrees); glider and wing still; every fingertip point at least 3 cm from the glider and the wing; all held for 1 s |
| 0.5 | Through; offset at most 10 mm and out of square at most 8 degrees; without the rest of 1.0 (for example the glider still held, or 3 to 10 mm off-centre) |
| 0.25 | Through, but further off-centre or out of square than that |
| 0 | Otherwise: the wing not through the slot |

Definitions, as `measure()` computes them in the glider's frame:

- **Through:** the wing's centre line (through the middle of its root chord, along the span) crosses the body's mid-plane inside the
  wing slot (within 3 mm of its length and 2 mm of its height), at least 5 mm in from the wing's tip. A rigid wing cannot cross the body
  anywhere else, so this is the wing passing through the slot. In a real trial: seen in the photo.
- **Offset:** how far the wing's centre lies from the body's mid-plane, along the span: half the difference between the two overhangs,
  each measured from the side of the body to the wing's tip.
- **Out of square:** the angle, seen from above, between the wing's span and the perpendicular to the body.
- **Standing:** the body's lower edge within 3 mm of the table, the body tilted at most 10 degrees.
- **Still:** neither origin moved 0.5 mm, nor the wing turned 0.5 degree, in one control step.
- **Hands clear:** every one of the 24 fingertip points that `mechkit.fingertips` reports at least 3 cm from boxes round the body, the
  tailplane and the wing.

Full credit lapses if the slot cracks or the wing or glider later leaves the 1.0 tolerances (the tracker re-checks every step). The partial levels need no hands clear:
they are progress. Possible scores: 0, 0.25, 0.5, 1.0. No force is scored: the crack is a visible split in a real trial.

## Real trial: preparation, reset and measurement

- **Parts (about $5):** one Guillow's Jetfire No. 30, its tailplane pushed fully into its tail slot and its nose clip on; the wing
  out. A table, painter's tape, a 30 cm steel rule, a small set square, a phone camera on a stand straight above. Nothing sharp, hot or
  wet: safe for the arms' pads. Balsa is soft, so use the same glider for every trial until its slot splits.
- **Preparation:** tape an L at the body's lower edge (tail end and nose) and at one tailplane tip, and stand the glider in it, as in
  the screenshot (body along the table's depth, nose away from the arms, its wing slot 30 cm in front of the arms' bases and 10 cm left of
  their midline). Lay the wing flat, printed side up, its trailing edge towards the arms, its centre 13.5 cm right of the midline, turned
  6 degrees, inside a taped outline.
- **Measurement, before anyone touches the glider:**
  1. *Crack:* look at the strip of balsa under the wing slot, from both sides (photo). Any split running from the slot's ends is a
     crack: score 0.
  2. *Through:* the wing passes through the slot (photo from above).
  3. *Offset:* from the top photo, or with the rule laid on the table under each tip, the distance from each side of the body to the
     tip of the wing along its trailing edge. Half the difference of the two is the offset (3 mm for 1.0, 10 mm for 0.5).
  4. *Square:* the set square against the side of the body, on the table; the gap between its edge and the wing's trailing edge, 100 mm
     out from the body, divided by 100 is the angle's tangent (5.2 mm is 3 degrees, 14 mm is 8 degrees). The same from the photo.
  5. *Standing:* the body's lower edge on the table (photo from the side).
  6. *Video:* both hands at least 3 cm away and nothing moving for the last second.
- **Reset (about 2 minutes):** slide the wing out of the slot, lay it in its outline, stand the glider in its tape marks. Replace the
  glider if its slot has split.

## Assets and licences

The glider and its wing are authored meshes in `task.py` (SimReady has no balsa glider; the slot fit is the mechanism under test). The
three distractors are NVIDIA SimReady props, downloaded at first run from the pinned revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd` of
`nvidia/simready-assets` on Hugging Face, © NVIDIA, CC-BY 4.0:

- [dishware_mug_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01): mass set to 0.30 kg.
- [misc_notebook_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/misc_notebook_a01): mass set to 0.12 kg.
- [stationery_marker_thin_blue_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/stationery_marker_thin_blue_a01): mass set to 0.01 kg.

Nothing large is committed. `simready.py` is the loader adapted from `tasks/cutters-off-pegboard/`, `mechkit.py` the mechanism kit from
`tasks/heat-shrink-lug/` (unchanged apart from a credit line) and `demokit.py` the closed-loop demo runner from `tasks/heat-shrink-lug/`
(unchanged): other teams' open-source files in this repository.

## Validation

All runs closed-loop through `demokit.py`, object_jitter 0, on the GPU with cameras unless marked CPU.

| Run | Score | What happened |
|---|---|---|
| Full demonstration, run 1 | 1.0 | Found the slot on the third approach, pushed through, centred (offset 0.0 mm, square 0.0 degrees), both hands clear; success at step 1,140. Slot give at most 0.02 mm |
| Full demonstration, run 2 (the submitted video) | 1.0 | Same, success at step 1,140 |
| Do nothing | 0.0 | Wing on the table, not through |
| One arm (glider not held) | 1.0 | The level-fed wing slid through the free-standing glider's slot without moving it. Holding the body is not needed in the simulation (see "The knack") |
| Careless push (hand let rise 0.6 mm per 3 mm pushed) | 0.25 | The wing was levered in the slot, jammed 68 mm off centre and slipped out of the 6 N pinch; the slot gave at most 0.03 mm, no crack. With a 20 N squeeze (CPU check) the same kind of levering forced the slot 1.4 mm open: cracked, 0 |
| Glider still held at the end | 0.5 | Centred and square, but a hand on the body: no full credit |
| Stopped 15 mm short of centre | 0.0 (GPU), 0.25 (CPU) | On the GPU the tip never found the slot in six approaches, so the wing was not through: 0. On the CPU it went in and stopped 13 to 17 mm off centre: 0.25 |

Model run: Dream-1, given the instruction for the full 2,400 steps, moved both arms near the glider and the wing but never picked the wing up or fed it into the slot; it scored 0.0.

Possible scores: 0, 0.25, 0.5, 1.0. The slot search is the fragile step: on the GPU the full demonstration needed three approaches before
the tip dropped in, and one scripted run (off-centre) never found it in six; CPU runs needed between one and five.
