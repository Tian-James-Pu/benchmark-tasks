# Close the lever-lock ring on a filled 5-gallon steel pail

**Instruction:** *Close the lever-lock ring to lock the lid on the pail.*

A filled 5-gallon open-head steel pail stands on a packing bench. Its lever-lock lid has been started the way the maker's
instructions begin: the cover is set on the pail, and the lever-lock ring, opened wide, has been slipped over the cover with its
lever standing out at the front right, ready to close clockwise. The robot has to finish the closure. The cover rides 6.5 mm high
on its uncompressed gasket, and the ring's lower lip passes under the pail's rolled bead only while the cover is held down. So one
hand presses the cover onto its gasket, and the other pushes the lever about 120 degrees clockwise round its pivot, against the
band's pull, until it goes over centre and lies on the band. Then both let go. Closed without holding the cover down, the band draws
tight on top of the bead, not under it: the ring stands about 5 mm proud and the lid is not locked.

## Importance and sources

Filling and closing pails is regular work in packing rooms that ship paint, coatings, adhesives, lubricants, inks and chemicals in
5-gallon open-head pails. A lever-lock ring is one of the usual closures for steel ones (Uline sells its pails with a lug lid or a
lever-lock lid).

- **Occupation.** O*NET 53-7064.00, *Packers and Packagers, Hand*: 555,500 employed (O*NET's 2025 figure). Two of its twelve
  task statements cover this job: "Place or pour products or materials into containers, using hand tools and equipment, or fill
  containers from spouts" (the filling) and "Seal containers or materials, using glues, fasteners, nails, and hand tools" (the
  closing, this task).
  https://www.onetonline.org/link/summary/53-7064.00
  *Evidence limit:* O*NET doesn't count how many hand packers close lever-lock pails in particular. Many pack cartons instead. The
  claim is that closing filled pails is core, repeated work (every filled pail is closed once) for the packers in plants that ship
  in pails, not that every hand packer does it.
- **The product and its closing procedure.** The pail is the Uline S-20030 (5 gallon, open top, unlined, 28-gauge steel, $13 to $15
  each), and the lid the Uline S-21135 lever-lock lid ($11), both listed on Uline's steel pails page:
  https://www.uline.com/BL_507/Steel-Pails-and-Lids . The pail's page gives its size: 13.4 in tall without the lid, 13 3/4 in with
  it; 11 3/4 in outside diameter at the top, 12 in with the lid; 10 3/4 in at the bottom; 2.41 lb:
  https://uline.com/Product/Detail/S-20030GR/Pails/Steel-Pail-5%20Gallon-Open-Top-Unlined-Gray .
  Uline's closing instructions for the lid (IS-21135, https://img.uline.com/is/content/uline/IS-21135pdf) give the sequence this task
  ends with:
  - step 1, "Place the cover on the pail. Press down along the edges and in the center of the cover to ensure that it is seated properly";
  - step 3, "Open the lever lock ring as wide as possible, then slip it over the pail ... in a manner that allows it to be closed by
    moving the lever clockwise onto the ring";
  - step 5, "Apply downward pressure to the cover and release the lever, allowing the ring to slide onto the cover/curl edge";
  - step 6, "Close the ring clockwise by applying pressure to the lever until it collapses onto the ring";
  - step 7, "If the ring is locked properly, it cannot be rotated or moved."
- **Why the closure matters.** For UN-rated packagings, US hazmat rules make following the maker's closure instructions part of the
  shipper's duty: 49 CFR 173.22(a)(4)(i), "applying closures consistent with the manufacturer's closure instructions"
  (https://www.law.cornell.edu/cfr/text/49/173.22). This particular Uline pail is sold as unrated ("Not leakproof or airtight with
  lid", on its page), so the regulation is cited for the general practice, not as a requirement on this product.

**The knack.** The cover sits on its gasket 6.5 mm above where it seats, and the ring's channel rides on the cover's curl, so the
ring's lower lip starts level with the top of the pail's 6 mm rolled bead. Only with the cover held down (about 3 mm or more, some
9 to 17 N on this gasket) does the lip get below the bead's widest point, and the band's contraction then wedges it under. One hand
has to keep the cover down while the other swings the lever through 120 degrees round its pivot on the band, pushing against the
band's pull (0.7 N m just before it goes over centre: about 9 N at the lever's end). Closed with the cover loose, or with the cover pressed
and let go first, the band contracts onto the top of the bead and locks there, the ring standing about 5 mm proud.

**Distinct from** the nearest board tasks we found (searched every PR title for pail, drum, bucket, lid, ring, gasket, lever lock,
toggle and over-centre):

| Nearest | What it tests | Difference here |
|---|---|---|
| #2038 (not accepted): close a swing-top bottle, wire bail over centre | one small over-centre bail on a bottle | a 31 cm band round a 20 kg pail; the toggle only seals if a second hand holds the cover down on its gasket at the same time |
| #2762 (not accepted): secure a moving enclosure lid with its over-center latch | a single latch catch | no catch: a band contracting round the whole rim, whose lip must pass under a bead |
| #922 (accepted): lever a paint tin's lid off with a screwdriver | opening a press-fit lid with a tool | closing an open-head pail with its own ring, two hands in different roles |
| #2916 (pending): close a paint can with rubber-mallet strikes | seating a press-fit plug lid by impact, one hand steadying the can | no tool or strikes: the pail's own ring is drawn round the rim by its lever, with the cover held down on its gasket |
| #2659 (not accepted): clamp a gusset with toggle clamps | flipping fixture clamps | a free ring on a free pail, with the cover's gasket under it |

LIBERO, RoboCasa, BEHAVIOR and SimplerEnv have lids put on pots and jars and containers closed; none has a band clamp closed over a
gasketed cover, which needs one hand pressing while the other drives a toggle.

## Scale and mass

Nothing is rescaled. The pail, cover and ring are modelled at Uline's published full size (sources above): pail 34.0 cm (13.4 in)
tall, 29.8 cm (11 3/4 in) across the top bead and 27.3 cm (10 3/4 in) across the base; cover 30.5 cm (12 in) across its curl; the
ring's band 30.9 cm across outside. An offline reach check showed both arms reach the cover's centre and the lever's whole path at
this size with tilted grips, so no scale change was needed. The bench is the station's table at its own height (74.75 cm).

| Item | Real | Modelled |
|---|---|---|
| Pail, filled | 1.09 kg empty (2.41 lb, Uline) plus 18 to 19 L of product | 20.0 kg in all (the arms never lift it) |
| Cover | steel, 12 in across (Uline) | 0.65 kg (an estimate) |
| Ring | steel band with lever | 0.31 kg (an estimate) |
| Cover's gasket | not published | 3 N/mm, 6.5 mm free travel above seated (estimates) |
| Lever's resistance | not published | 0.4 N m/rad back towards open: 0.7 N m, about 9 N at its end, just before centre (an estimate; a real lever is probably firmer) |

No mass was reduced: nothing is carried. The forces are disclosed estimates, not measurements.

## Run and score

From the repository root, in the Python environment that has `yam_arena` and `inspect-robots`, a model run:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/seal-steel-pail-with-lever-lock-ring/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Close the lever-lock ring to lock the lid on the pail." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/seal-steel-pail-with-lever-lock-ring
```

The scripted demonstration is closed loop (it reads the cover's height, the lever's angle and the ring's pose, never the scorer), so
it runs through `demokit.py` from the task folder:

```bash
cd tasks/seal-steel-pail-with-lever-lock-ring
python demokit.py task.py ~/yam-eval/seal-steel-pail-with-lever-lock-ring/demo --mode full        # writes full/demo.mp4, setup.png, result.json
python demokit.py task.py ~/yam-eval/seal-steel-pail-with-lever-lock-ring/redteam --no-record --mode none,onearm,late,sequential,hold,pressdown
python demokit.py task.py ~/yam-eval/seal-steel-pail-with-lever-lock-ring/fast --fast --mode full    # CPU physics, no cameras
```

`result.json` holds `task_score`. The layout is fixed (`object_jitter=0`); there are no seeds or random choices. The demo reaches
full score and the trial ends at control step 411 (20.6 simulated seconds); the run limit is 1,200 steps (60 s), about three times
that. A CPU `--fast` run takes under a minute of wall time; a GPU-rendered run took 8 to 11 minutes on a heavily shared GPU.

## How the simulator models it

- **Pail and cover** (`PAIL`, one articulation, free, 20 kg): the body and its rolled bead are lathe meshes, the bead 6 mm tall
  with its widest point (29.84 cm across) 3 mm below the top. The cover is a link on a vertical slide: its joint zero is the loose
  cover resting on the uncompressed gasket 6.5 mm above seated, and its spring (3,000 N/m) is the gasket. It can't tilt or slide
  sideways, a simplification.
- **Ring** (`RING`, its own free articulation, 0.31 kg): the band's top web and outer wall are the rigid frame, sitting on the
  cover's curl. Its lower lip is four segments, one per quadrant, each a link that slides inward (up to 8 mm) as the band contracts.
  The lever is a hinged strap on the band, authored open 120 degrees, with a spring (0.4 N m/rad) pulling it back open: the band's
  pull.
- **The toggle** (`Band.step`, run once per control step from the joints' own positions): while the lever is short of centre (100
  degrees of closing), the lips are driven inward in proportion to how far it has closed, so contact decides where they stop. Past
  100 degrees the lever snaps shut against its stop (a 3 N m/rad drive) and the lips are pushed fully in with a 4.5 N preload; six
  control steps later, with the lever within 10 degrees of shut, the lips are locked where contact let them go. Pulled back under
  100 degrees, everything is released again. Nothing is triggered by where a gripper is.
- What decides the outcome is plain contact: with the cover held down at least about 3 mm, the lips' lower edges are below the
  bead's widest point and slide under it. With the cover loose they meet the bead's upper slope and lock on top of it. After the
  hands let go, the gasket pushes the cover and ring up until the lips bear on the underside of the bead: the ring's top then rests
  0 to 0.3 mm from its reference height in the runs below. A ring locked on top of the bead rests about 5 mm higher.

## Scoring

`score()` reads `Trial`, which is updated once per control step (`Trial.step`). The text in `task.json` `scoring` is this:

- **Ring height** is the height of the ring's top above the pail's base at each tape mark (far, left, near, right), read on the
  middle of the band's top web, 15.0 cm from the pail's axis. The reference is its height on this pail sealed correctly: 345.9 mm in
  the simulator (`SEATED_RING_TOP`).
- **Harm (latched, cleared only by a reset):** the pail tilts more than 10 degrees at any time: 0 from then on.
- **Base conditions, at the end:** pail within 5 degrees of upright; ring on the rim (its centre within 8 mm of the pail's axis and
  its underside 0 to 15 mm above the bead's top); lever within 10 degrees of the band (its tip within about 13 mm of it); every
  fingertip point at least 3 cm from the pail's top (the zone within 17 cm of its axis, from 2 cm below the bead's top to 3.5 cm
  above it). Any of them missing: 0.
- **Possible scores:**
  - 0: the harm, a base condition missing, or nothing done (the lever is still open).
  - 0.25: the ring stands more than 2 mm above its reference at 2 or more marks: closed on top of the bead.
  - 0.5: at exactly 1 mark: a cocked ring.
  - 0.75: at no mark, but ring and pail not yet still for 1 s.
  - 1.0: at no mark, with the ring still (under 1 cm/s and 0.1 rad/s) and the pail still (under 1 cm/s) for 1 s without a break.
    Credit doesn't stick: if a hand comes back or the ring is knocked, the score drops with it.

No force is read. Every condition is a length or angle on a visible surface, measured once the hands are clear.

## Real trial: preparation, reset and measurement

- **Parts (about $80, Uline's minimum order of three of each):** a Uline S-20030 pail and an S-21135 lever-lock lid (cover with
  gasket, and ring); two 10 kg bags of play sand inside the pail as its contents (dry, so a tipped pail spills nothing, and nothing
  harms the pads); blue masking tape; a steel rule or a caliper with a depth rod; a printed 120-degree angle template; a phone on a
  stand for a top video. The ring's and lever's edges are rolled: no sharp edges for the pads.
- **Layout:** the pail's base on a 27.3 cm circle taped on the bench, its axis 36 cm in front of the arms' base line, midway between
  them. Tape strips just under the bead at the far, left, near and right sides. A tape mark on the bench at the front right shows
  where the lever's pivot goes (azimuth 252 degrees, 15.7 cm from the axis).
- **Reference (once per cover):** close the lid by hand per Uline's steps 5 to 6 (press the cover down, push the lever closed
  clockwise until it collapses onto the ring). Measure the height of the ring's top above the bench at each tape mark: the reference
  heights. Open it again.
- **Start of each trial:** cover set on the pail, centred, not pressed (it rests on its gasket); ring opened as wide as it goes and
  slipped over the cover with its pivot at the bench mark; lever opened to 120 degrees from the band with the template.
- **Measurement, before anyone touches the pail:**
  1. the video: whether the pail ever tilted over 10 degrees; whether the hands were clear (3 cm from the pail's top) and nothing
     moved for the last second;
  2. the lever: the gap between the lever's tip and the band, with the rule: 13 mm or less is within 10 degrees;
  3. the ring's top above the bench at each tape mark, with the rule or the caliper's depth rod: more than 2 mm above that mark's
     reference counts as standing proud;
  4. the ring's centring: the gap between the band and the pail's wall just below it at the four marks; half the difference of
     opposite gaps is the offset (8 mm allowed).
- **Reset (about 2 minutes):** lift the lever open, take the ring off, lift the cover and set it back loose and centred, slip the
  ring back on and open the lever to the template. If a cover's reference heights drift by more than 0.5 mm over trials (its gasket
  taking a set), replace the cover and measure new references.

## Assets and licences

The pail, cover and ring are authored meshes (SimReady has no steel pail or lever-lock ring). The four distractors are NVIDIA
SimReady props, downloaded at first run from the pinned revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd` of
`nvidia/simready-assets` on Hugging Face, © NVIDIA, CC-BY 4.0:

- [tape_clear_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/tape_clear_a01): mass set to 0.06 kg.
- [handTool_knife_retractableBlade_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_knife_retractableBlade_a01): mass set to 0.12 kg; its joints welded into one body.
- [stationery_marker_thin_blue_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/stationery_marker_thin_blue_a01): mass set to 0.01 kg.
- [ppe_gloves_lightDuty_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/ppe_gloves_lightDuty_a01): mass set to 0.06 kg.

Nothing large is committed. `simready.py` is the loader from `tasks/cutters-off-pegboard/` and `mechkit.py` the mechanism kit from
`tasks/heat-shrink-lug/`, both unchanged apart from a credit line in their docstrings; `demokit.py` is the closed-loop demo runner from `tasks/heat-shrink-lug/`, unchanged. All
three are other teams' open-source files in this repository.

## Validation

All runs below use `demokit.py` from this folder. GPU runs A and B were made before the last edits to `task.py`, which changed
only comments and docstrings (no number, geometry or logic); run C and the CPU runs were made on the submitted file.

**Demonstration, GPU, rendered at 640 x 360 (the final mode, whose `demo.mp4` and `setup.png` are the submitted media):**

| Run | `task_score` | Ended | Notes |
|---|---|---|---|
| A | 1.0 | success at step 411 (20.6 s) | 0 blank frames |
| B | 1.0 | success at step 411 (20.6 s) | 2 blank frames from another job on the shared GPU; the score is unaffected |
| C | 1.0 | success at step 411 (20.6 s) | on the submitted `task.py`; 0 blank frames; its `demo.mp4` and `setup.png` are the submitted media (byte-identical to run A's) |

In both, the left hand held the cover 5.5 mm down while the lever went over centre. After both hands had left, the ring's top
stood 0.08 to 0.09 mm above its reference at all four marks, the cover 5.4 mm below its loose height, the lever shut (120 degrees),
the pail upright and unmoved. On the submitted file, the same script on the CPU (`--fast`) scored 1.0 twice (414 and 398 steps).

**Red team** (CPU `--fast` on the submitted file, the same physics without cameras; modes of `demo_script`):

| Mode | What it does | Score | Why |
|---|---|---|---|
| `none` | nothing | 0.0 | the lever is still open |
| `onearm` | the right hand closes the lever; the cover is never pressed | 0.25 | the band locked on top of the bead: ring 5.2 mm above its reference at all four marks |
| `late` | the lever is closed first, then the left hand presses the cover and lets go | 0.25 | the lips were already locked on the bead; ring 5.2 mm proud |
| `sequential` | the cover is pressed and let go, then the lever is closed | 0.25 | the gasket lifted the cover back before the band closed; ring 5.2 mm proud |
| `hold` | the full demo, but the left hand stays on the cover | 0.0 | the ring is seated (0.3 mm below its reference) but a hand is on the pail |
| `pressdown` | one arm only: the right bears down on the lever's top edge (commanded 4 cm low) as it pushes it round | 0.25 | pushing down on a lever hinged on a free ring doesn't hold the cover down; ring 5.2 mm proud |

The cheapest routes (one arm, or pressing and closing one after the other) earn only the 0.25 for a ring closed on top of the
bead, and doing nothing earns 0. A demo that seals the pail but leaves a hand on it earns 0 until the hand is gone.

**Model run:** Dream-1, run once on the submitted file with the command above, scored 0.0: in its 1,200 steps both arms
drew back towards their bases and never touched the pail, the cover or the lever.

Not checked: other layouts (the layout is fixed).

## Files

| File | What it is |
|---|---|
| `task.py` | The scene, the ring's toggle (`Band`), the scorer (`Trial`, `score`, `success`) and the closed-loop demo (`demo_script`) |
| `task.json` | The task's details |
| `mechkit.py` | The mechanism kit, unchanged |
| `simready.py` | The SimReady loader, unchanged |
| `demokit.py` | The closed-loop demo runner, unchanged |
