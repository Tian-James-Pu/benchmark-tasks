# Open a new glue tube: unscrew the cap, turn it over, push its point through the nozzle's seal and screw it back on

Tubes of all-purpose glue, ointment, oil paint and similar products are often sold with the nozzle closed by a thin
aluminium membrane, and the cap carries a point on its top for opening it. Opening one takes five things in a row:
unscrew the cap one and a half turns, turn a 15 mm cap end over, push its point squarely into a 4.5 mm opening until
the membrane gives (about 10 N), turn the cap back the right way up and screw it down again, all without squeezing the
soft tube. Here the tube stands upright in a holder, its cap screwed down, and the robot is asked only to open it and
put the cap back on. Which end of the cap does the opening, and that it has to be turned over twice, is left to it.

**This is an independent implementation of an idea first submitted by another team in PR #2964 ("Open a new
superglue tube"), whose 48-hour hold lapsed with no revision.** The scene, mechanism, scorer, demo and text here are
our own; nothing was copied. How each point of that PR's review is met is listed under
[Review points of the earlier submission](#review-points-of-the-earlier-submission).

## The scene (full size)

All positions are object centres on the table: x forward from the arms' bases, y to the left, metres. The table top
is the station's (0.7475 m).

| Object | What it is | Size, material, mass | Where |
|---|---|---|---|
| Holder (fixed) | A pine block with a 3 mm saw kerf along y, 20 mm deep, which grips the tube's folded end. It stands in for the hand that holds the tube | 50 x 90 x 30 mm, pine | centre (0.30, -0.12) |
| Tube (fixed in the holder) | New 30 ml aluminium tube, membrane-sealed nozzle | 25 mm across, 100 mm long (80 mm of round and flattening body above the block, 20 mm of folded end in the kerf); shoulder 4 mm; M11 neck 9 mm tall; 4.5 mm orifice; membrane 0.5 mm below the rim; 40 g filled | axis at (0.30, -0.12), upright |
| Cap (loose) | White ribbed polypropylene screw cap with a piercing point moulded on its top | 15 mm across, 11.5 mm tall; point 3.5 mm at its base, 5 mm tall; 2 g | screwed down on the neck |
| Mug (distractor) | SimReady `dishware_mug_a01` | 114 x 90 x 80 mm, ceramic, 300 g | (0.20, 0.10) |
| Tape roll (distractor) | SimReady `tape_clear_a01` | 100 mm across, 50 mm tall, 60 g | (0.42, 0.06) |
| Notebook (distractor) | SimReady `misc_notebook_a01` | 229 x 170 x 10 mm, paper, 120 g | (0.16, 0.27) |

The distractors are what lies on a table where something is being glued. They are well outside the arm's work area,
none of them can help open the tube, and the scorer ignores them.

## How the simulator works it

- **Thread.** The cap is a free rigid body. While it is on the thread, a step hook holds it on the neck's axis,
  upright, at the height its turn sets: 1.5 mm per turn, from fully down (turn 0) to 1.5 turns out. It cannot be
  pulled or tipped off, and it does not turn clockwise past fully down. Turned out past 1.5 turns it is off the
  thread and rests on the thread's start (2.25 mm above fully down) until lifted. A cap set back on the neck open end
  down, within 2.5 mm of the neck's axis and tilted less than 12 degrees, rests on the thread's start; turned 20
  degrees clockwise there it takes up the thread, and from then on it screws down as it is turned. This is the
  thread's lead-in, which centres a cap that is roughly placed.
- **Membrane.** A 4 mm aluminium disc on a sprung slider (5,000 N/m) in the neck's bore, 0.5 mm below the rim. Pushed
  2 mm in (10 N), it tears: it drops out of the way and a torn membrane is shown. It cannot be restored. Only something
  narrower than the 4.5 mm bore reaches it, which on this table is the cap's point; the cap's flat inside and a closed
  gripper's fingertips are both too wide.
- **Squeezing.** The tube body is rigid in the simulator. A squeeze is counted when the fingertip points of one gripper
  are on opposite sides of the tube's body, each within 4 mm of its surface, for 5 control steps running (0.25 s).
  That is a grip on the body; touching or brushing it is not a squeeze. A squeeze is recorded for the rest of the trial
  and is cleared only when the simulator resets the scene.

## Scoring

Evaluated every control step (20 Hz). Possible scores: 0, 0.25, 0.5, 0.75, 1.0.

| Score | Condition |
|---|---|
| 0 | A gripper squeezed the tube's body at any time in the trial (whatever happens afterwards) |
| 1.0 | The membrane is pierced, the cap is on the thread within a quarter turn (90 degrees, 0.375 mm) of fully down, and every fingertip point is at least 3 cm from the cap's middle, all held for 1 s; `task_score` then needs that to last a further 1 s (`SUCCESS_HOLD_S`), so 2 s in all |
| 0.75 | The membrane is pierced and the cap is back on the thread, but not screwed down to within a quarter turn, or a gripper is still at it |
| 0.5 | The membrane is pierced and the cap is not on the thread (left on the table, still held, dropped) |
| 0.25 | The membrane is intact, and the cap has come off the thread at some point |
| 0 | Anything else, including doing nothing |

The 1.0 conditions are checked at the end state, and stop holding as soon as a gripper comes back to the cap, so
credit does not stick.

**In a real trial** (tools: a phone camera, feeler gauges, a ruler):
1. Before the trial, with the cap screwed fully down by hand, measure the gap between the cap's skirt and the tube's
   shoulder with feeler gauges (G0, typically 0.2 to 0.5 mm) and mark a pencil line across cap and shoulder.
2. Squeeze: a visible dent in the tube's body, or gel outside the nozzle or on the pads, after the trial is 0.
3. At the end, before touching anything: the cap on the neck or not (photo); the skirt-to-shoulder gap with feeler
   gauges. A gap within 0.4 mm of G0 is "within a quarter turn of fully down" (0.375 mm at 1.5 mm per turn).
4. Then unscrew the cap and photograph the nozzle from above against a lit background: pierced means daylight through
   the membrane, or a bead of gel when the tube is pressed gently at the shoulder. An unpierced membrane passes no gel.

## Real-trial setup, reset and rig protection

- **Tube.** Blue Sky Solutions *25 mm x 100 mm Plain White Aluminium Tube (30 ml), sealed nozzle (to be pierced
  with cap), white ribbed cap to suit*, product code ALTW25100WRC-PP (case of 383; samples on request; quoted
  price). Each tube's membrane is the manufacturer's own, so every trial starts with an identical, factory-fresh seal.
  Any equivalent aluminium tube of the same size with a membrane ("blind") nozzle and a piercing cap works; membrane
  tubes and piercing caps are standard catalogue items of tube makers (see the sources).
- **Contents: an inert stand-in, not glue.** Fill each tube through its open end with 25 ml of clear aloe vera gel or
  a water-based ultrasound gel (non-toxic, washes off with water), flatten the end, fold it twice 5 mm deep and press
  the fold with pliers. Filling takes about a minute a tube; a batch is made before a session. No adhesive is used, so
  a squeeze cannot glue anything to the grippers; any gel on the pads is wiped off with a damp cloth.
- **Holder.** A pine offcut, 50 x 90 x 30 mm, with one 3 mm handsaw kerf 20 mm deep along its length, clamped or taped
  to the table at the marked position. The tube's folded end goes 20 mm into the kerf.
- **Reset (about 1 minute).** Put a new filled tube in the kerf, check that its cap is screwed fully down by hand
  (factory state), measure G0 and mark the line. The used tube is set aside (its membrane cannot be restored; it can
  be emptied and used for other tests).
- **Pierce force.** The 10 N tear force in the simulator is an estimate (no published figure was found). Before a
  series of trials, press the cap's point into one tube's nozzle with the tube standing on a kitchen scale and note
  the reading at the moment the membrane gives; if it differs much from 10 N, say so with the results.

## Runs

From this folder, on an instance set up by the simulation walkthrough:

```bash
INSTR="Open this new tube of glue so it is ready to use, then put its cap back on."

# Dream-1 (or any policy)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/glue-tube-dream

# The scripted demonstration (scores 1.0)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$INSTR" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/glue-tube-demo
```

- **Seed and layout:** `-E object_jitter=0`. The layout is fixed and nothing in the scene is random, so no seed is
  needed.
- **Runtime per trial:** DEMO_RUNTIME
- **Score:** `task_score` is 1.0 when `success()` has held for 1 s, otherwise the last step's `score()`.

## The scripted demonstration

A fixed plan from the published starting layout, played open loop by the `yam_scripted` policy (it reads no
simulator state while it runs). The right arm does the work; the left arm stays at its ready pose.

1. Unscrew: three anticlockwise strokes of 220 degrees with a straight-down grip, letting go and turning back between
   them (1.5 turns free the cap; the rest turns it on the thread's start).
2. Grip the loose cap with the fingers' lower edge at it, lift it, tip the grip level (pointing 60 degrees from +x)
   and lay the cap on its side on the table, point towards the robot.
3. Grip the lying cap from above with its point at the fingers' edge and tip the grip level again: the cap is now
   point down, below the fingers. Lower it over the nozzle, push 4.5 mm in (the membrane gives at 2 mm) and withdraw.
4. Tip back and lay the cap on its side again; grip it from the side and tip back to straight down: it is now open
   end down.
5. Set it on the neck and screw it down with clockwise strokes; let go and move the arm away.

The cap is only 15 mm across and the fingers are 36 mm wide, so it is turned over in two regrasps through a lying
position; a single grasp cannot turn it end over end with the point free below the fingers. Each tip is a 90 degree
turn of the wrist about the fingers' closing direction.

## Files

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: the scene, the thread and membrane hooks, the scorer (`score`, `success`) and the scripted `demo()` |
| `task.json` | The task's details |
| `mechkit.py` | Props, mechanisms and hooks for a `yam_arena` task file, taken unchanged from another team's open-source task in this repository (`tasks/heat-shrink-lug/`) |
| `simready.py` | Loads NVIDIA SimReady props, taken unchanged from the same shared copy used by other tasks in this repository |

## Fidelity and what is simplified

- Tube, cap and holder are custom meshes at the product's size. The tube is rigid and fixed in its holder; its
  softness is represented only by the squeeze rule, which is a fingertip-position proxy, not a force.
- The thread is kinematic (a step hook keeps the cap on the axis at the right height while it is on the thread); the
  turn needed to take it off and put it back on, and its lead per turn, are those of the tube; there is no thread
  friction or torque. The lead (1.5 mm per turn) and the 1.5 turns to free the cap are typical of an M11 tube neck and
  should be checked on the tube bought (count the turns from fully down to free).
- The membrane is a sprung disc that tears at a set travel; its 10 N is an estimate. The gel is not simulated.
- Friction is uniform and rubber-like on every contact. No force sensing is used anywhere.

## Importance and sources

- **The operation is real and common.** A cap whose top carries the piercing point, which is "removed from the
  container, inverted 180 degrees and then ... pressed onto the seal to effect rupture", is described as an existing
  closure type in the background of US patent 5,758,788, *Piercing container cap* (Merck, 1998), which names
  "sealed containers for medicine, glue and the like":
  [patents.google.com/patent/US5758788A](https://patents.google.com/patent/US5758788A/en).
- **Membrane tubes and piercing caps are standard products.** A European aluminium tube maker lists "Type OP: with
  membrane" in diameters from 11 to 50 mm, with M7 to M15 neck threads, including 25 mm tubes holding 27 to 60 ml:
  [Alltub, aluminium collapsible tubes data sheet](https://www.alltub.com/wp-content/uploads/2024/04/ft-tubes-alu-en.pdf);
  the same maker's RoyAl closure has an "M11 thread with integrated membrane" and an "Integrated piercing spike in
  cap": [alltub.com/products/innovation](https://www.alltub.com/products/innovation).
- **The product modelled.** Blue Sky Solutions, *25mm x 100mm Plain White Aluminium Tube (30ml), Sealed nozzle (to be
  pierced with cap), White Ribbed cap to suit*, code ALTW25100WRC-PP, "designed to pierce the sealed nozzle on first
  use":
  [blueskysolutionsuk.com](https://blueskysolutionsuk.com/product/25mm-x-100mm-plain-white-aluminium-tube-30ml-sealed-nozzle-to-be-pierced-with-cap-white-ribbed-cap-to-suit/).
  The listing gives 25 x 100 mm and 30 ml. The M11 neck is one of the standard membrane necks in the tube maker's data
  above (the listing does not give the thread); the cap's size, the 4.5 mm orifice, the 1.5 mm lead and the point's size are our estimates and are to be measured on
  the tubes bought (inference).
- **Factory-sealed glue tubes that the user opens are a mass-market consumer format.** For example, Loctite's Super
  Glue Gel tube is opened by screwing its cap "down all the way to the tube shoulder, puncturing the tube":
  [loctiteproducts.com](https://www.loctiteproducts.com/en.product/SAP_0201DGL029Z0.html). That product pierces by
  screwing rather than by a point on the cap; it is cited only for the format (inference: that most adults have opened
  a sealed tube of glue, ointment or paint this way is our judgement, not a measured figure).
- **Category.** Societal relatability: an everyday job people do with their own hands. No occupation is claimed, so no
  workforce figure is given.

**Why it is a robot problem.** The cap is 15 mm across, smaller than the gripper's 36 mm fingers, and must be turned
end over end twice: once to bring the point down and once to bring the open end back down. The point must then land
within about 2 mm of the nozzle's axis and be pushed against the membrane's resistance, and the cap must be screwed
back on, all with a parallel gripper and without closing on the tube. Unscrewing alone earns 0.25.

### Distinct from

| Nearest task | What it does | Why this is different |
|---|---|---|
| TK-Z97WAY, take the cap off a child-resistant medicine bottle | Push down and turn a cap off | Ends where this task starts; no reorientation, piercing or recap |
| TK-J9DAXP / TK-4TJ2UQ, unscrew or screw on a flashlight's tail cap | Turn a large cap off or on | No turning a small part over in the hand, no aimed push through a seal |
| TK-6XCNQM, put a pea-sized dab of toothpaste on a child's toothbrush | Squeeze a measured amount from an open tube | The tube here is sealed and must be opened with its own cap; squeezing is what must not happen |

Unscrewing caps appears in other task sets (opening bottles and jars); turning a cap over and piercing a membrane with
it does not appear in LIBERO, RoboCasa, SimplerEnv, BEHAVIOR or RoboDojo as far as we could find.

## Review points of the earlier submission

The earlier submission (PR #2964) was not accepted on scoring, clarity and reproducibility, and feasibility. This
implementation answers each point:

| Review point | How it is met here |
|---|---|
| Scoring: the 2 mm penetration and 1.5 mm pin-to-axis checks were hidden inside the nozzle, with no real-trial measurement | No hidden coordinates are scored. Piercing is an outcome (the membrane torn), checked in a real trial by daylight through the nozzle or gel at a gentle press; the recap is the skirt-to-shoulder gap on feeler gauges; the squeeze is a visible dent or gel outside the nozzle. See "In a real trial" |
| Clarity: an unspecified glue-style tube and replacement foil, with no repeatable source or preparation for the cap, seal and piercing force | A named, purchasable tube with a factory membrane nozzle and a piercing cap (Blue Sky Solutions ALTW25100WRC-PP); every trial uses a new tube, so the seal is the manufacturer's and identical each time; a filling recipe; a kitchen-scale check of the pierce force |
| Feasibility: glue can contaminate the grippers; no inert substitute or pad protection | The tubes are filled with an inert, water-washable gel (aloe vera or ultrasound gel), never glue; a squeeze cannot bond anything, and pads are wiped with a damp cloth |
| Add observable real-trial checks for pin alignment and seal penetration, such as reference marks, camera views or an outcome-based puncture check | Outcome-based: the puncture is checked by light and gel, not by tip position; alignment is not scored at all, since an off-axis point simply fails to pierce |

## Assets and licences

- `mechkit.py`: from `tasks/heat-shrink-lug/` in this repository (another team's open-source task), unchanged.
- `simready.py`: the shared SimReady loader used by other tasks in this repository, unchanged.
- NVIDIA SimReady props from [`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets),
  pinned to revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, © NVIDIA, CC-BY 4.0. Downloaded at first run into
  `~/.cache/simready`; nothing is committed. Changes at load time: masses set as listed above; any joints welded into
  one rigid body.
  - [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01)
  - [`tape_clear_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/tape_clear_a01)
  - [`misc_notebook_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/misc_notebook_a01)
- The tube, cap and holder are built procedurally in `task.py` (our own code). No third-party meshes or textures.
