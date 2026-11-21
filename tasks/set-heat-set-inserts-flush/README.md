# Set four heat-set inserts in a printed plate with a soldering iron, finishing each flush with a steel block while hot

A printed robot part gets its threads: four brass M3 heat-set inserts stand loose in the holes of an orange PETG sensor plate, and
a hot soldering iron with an insert tip lies in its holder. The robot sets every insert flush with the plate's face, then puts the
iron back in its holder.

**Instruction:** `Set the four threaded inserts flush into the plate with the soldering iron, then put the iron back in its holder.`

The instruction gives the goal only. It does not say how to finish an insert, that the iron's shoulder marks the plastic, or that the
steel block is there for the last step.

This is an independent implementation of an idea first submitted by another team in PR #2509. None of its code or text is used. "Changes from the earlier submission's review" below lists how each point of
that review is met.

## The skill

The inserts' maker describes the method ([CNC Kitchen, "Tips & Tricks for Heat-Set Inserts"][cnck]):

- "Set your soldering iron 10 to 20 °C above your printing temperature" (245 °C here, for PETG).
- "always melt the inserts only to about 90% of the depth with the soldering iron tip and do the last, short way with a tool."
- "Hold them in the final position for a few seconds until the plastic has cooled back down and solidified."
- "the inserts have a tendency to move slightly out of the component immediately after melting."
- The insert tip's "flat shoulder behind it pushes on the rim of the insert rather than on the plastic around it".
- The maker's finishing tool has "a large, flat face that presses the insert perpendicular and flush in one move".

The earlier submission's reviewer put the last step this way: a flat piece of metal is laid on top to seat the last little bit while
it is still hot. So each insert takes two tools in turn, against the clock:

1. **Melt it in with the iron, and stop short.** The tip goes onto the insert, its pin in the insert's bore. The iron is held there
   while the brass heats the plastic (about 1 s here before anything moves), then pushed gently down as the insert sinks. It must
   stop about 1 mm proud: taken to flush, the hot shoulder presses a ring into the plate's face (a melt mark).
2. **Finish it flush with flat steel while it is still soft.** The iron comes off, and a steel 1-2-3 block's flat face goes down on
   the insert at once. The block's own weight (0.48 kg) presses the insert flush and holds it while the plastic sets; a hand may also
   press it. If the block arrives late, the plastic has set and the insert stays proud. If it is lifted too soon, the insert creeps
   back up.

The natural division of work is one hand for the iron and one for the block: the iron must be clear before the block comes down, and
the block must arrive within about 4 s. The task also needs care with a hot tool. Only the iron's grip may be touched, never the
hot brass, and the iron ends back in its holder, not lying on the bench.

## Importance and sources

Category: **Impact on AI timelines** (building robots with their real parts).

- **Printed robot parts take their threads this way.** The Berkeley Humanoid Lite build guide's tool list says: "when installing
  heat inserts into your 3D-printed parts, a heat insert stand ... is highly recommended to ensure a precise, vertical insertion"
  ([Berkeley Humanoid Lite docs, "Preparing the tools"][bhl]). The operation is part of assembling an open-source humanoid.
  *Inference:* other printed robot arms, grippers and sensor mounts are threaded the same way. We do not cite a count.
- **The real operation, step by step:** the maker's guide ([cnck]), quoted above: the temperature, the 90 % rule, the finishing
  tool, the creep, and the shoulder that must bear on the insert's rim and not on the plastic. The guide also gives the hole depth:
  "A blind hole should be slightly deeper than the length of the insert (approx. 1 mm)". It is 7.0 mm here.
- **The insert:** CNC Kitchen M3 standard: M3, 5.7 mm long, 4.6 mm across, hole 4.0 mm, minimum wall 1.6 mm. A pack of 100 costs
  $8.78 ([3DJake product page][insert]).
- **The iron:** Hakko FX-8801, the handpiece of the FX-888D station: 217 mm long and 46 g with a T18-B tip, "measured without cord"
  ([Hakko FX-8801 specifications][hakko8801]). The FX-888D set's packing list includes the "Handpiece (FX-8801)" and the "Iron holder
  (FH-800 ...)" ([Hakko FX-888D set][hakko888]).
- **The insert tip:** Virtjoule "Heat-Set Insert For Soldering Irons - #4-40 / M3 Inserts" (Adafruit 4239, $9.95; the page lists
  2.1 mm, and says any iron "that use[s] Hakko tips will do the job"; [Adafruit][tip]). Adafruit no longer stocks it. Any M3 insert
  tip for Hakko T18 handpieces does the same job.
- **The block:** a hardened steel 1-2-3 block (1 x 2 x 3 in, 25.4 x 50.8 x 76.2 mm), for example All Industrial 55510: a pair,
  "precision-grade tool steel hardened and ground on all 6 sides", 23 holes, "Weight: 2.10 LBS" ([All Industrial][block]). That is
  0.48 kg for each block, which is the mass used here.
- **Occupation size (context only):** US employment of "Electrical, Electronic, and Electromechanical Assemblers, Except Coil Winders,
  Tapers, and Finishers" was 267,440 in May 2023 ([BLS OEWS 51-2028][bls]). That is under the jobs category's bar, and we have no
  source that setting inserts is their core work. So no Impact on jobs claim is made: the case is robot building, as above.

## Distinct from

| Nearest | What it tests | Why this is different |
|---|---|---|
| TK-X5ZRTM (accepted): heat-shrink a sleeve over a cable lug with a heat gun | an even heat dose from a tool that must not touch | here the hot tool must touch and push, stop at a depth, and a second tool must finish the job inside a cooling window |
| TK-R4X34T (accepted): put a circuit board down on a mat where a hot iron lies | noticing and avoiding a hot iron | here the hot iron is used and put away; the hazard is handled while working |
| TK-RP86Y7 (closed): drive a bushing to its depth with a stepped driver | a press fit to a depth with a cold rigid tool | nothing here moves until the brass has softened the plastic; the iron alone cannot finish without marking the part; the flat finish must come within seconds, and be held |

External sets: no LIBERO, RoboCasa, BEHAVIOR-1K, SimplerEnv, DataCenterBench or RoboDojo task uses a soldering iron or heat-set
inserts. The nearest are peg-in-hole insertions (Factory, IndustReal), where a peg slides home once aligned.

## Files

| File | What |
|---|---|
| `task.py` | The task: scene, the plastic model (heat, flow, creep, melt marks), the scorer, and the scripted demo `demo()` |
| `task.json` | Submission details |
| `plate.scad` | The plate to print (OpenSCAD; `openscad -o plate.stl plate.scad`) |
| `mechkit.py` | Mechanism kit (plain-text USD props, joints, hooks), copied unchanged from `tasks/heat-shrink-lug/` in this repository, another team's open-source task |
| `simready.py` | SimReady prop loader, copied from `tasks/cutters-off-pegboard/` in this repository (another team's open-source task) |

## Run a trial

In the Isaac Lab-Arena environment of the simulation walkthrough (`source ~/IsaacLab-Arena/yam.env`), from the repository root:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/set-heat-set-inserts-flush/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Set the four threaded inserts flush into the plate with the soldering iron, then put the iron back in its holder." \
  --scorer task_score --max-steps 3600 --store-frames --log-dir ~/yam-eval/set-heat-set-inserts-flush
```

The scripted demo is the same command with `--policy yam_scripted -E cam_width=640 -E cam_height=360 --max-steps 1800`. It is a
fixed plan from the published starting layout (no feedback, and it reads no simulator state). It uses offsets measured in trials:
the iron's seat in the fingers, how far the hand runs on when the plastic gives, and where the hand leans under the push. Its
right hand takes the iron by the grip. For each insert, the left hand lays the block on the insert as the iron leaves, lets go
while the block's weight presses the insert, then takes the block back. `HSI_DEMO=<mode>` plays the red-team variants listed below, and
`HSI_DEBUG=1` prints the inserts' heights, heat and marks every 0.5 s.

The layout is the same in every trial and there are no random parameters (`--seed` changes nothing). A trial is at most 3,600
control steps (3 minutes simulated); the demo needs about 1,700. On a shared L40S-class GPU a demo run takes about 6 to 10 minutes of
wall time.

## How the simulator judges it

The rules run every control step (20 Hz) in `Plastic.step` (`task.py`), registered with mechkit's `on_step`; every recorded harm and
mark is cleared only in `Plastic.reset`, which mechkit calls on a reset, never because the scene looks right again.

- **Inserts.** Each is a slider along its hole in the plate's articulation, held at its "set" depth by a stiff spring (40 kN/m). The
  insert collides as a ring round its bore (2.8 mm, with a lead-in chamfer to 4.2 mm at its mouth), and the shoulder presses on the
  ring. The tip's pin is drawn at its full 4.5 mm, but collides only as a cone 1.5 mm long below the shoulder, 2.2 mm across at its
  base: it finds the bore and keeps the tip on the insert, but cannot wedge in it. The plate collides with a 4 mm gap at each hole.
- **Heat.** An insert heats while the iron's tip is on it: the shoulder within 0.8 mm of the insert's top, the tip's axis within
  2 mm of the insert's (the 5 mm shoulder then still covers the 4.6 mm insert), and the iron within 10 degrees of vertical. On the iron the heat rises from 0 to 1 in 2 s; off it, it falls to 0 in
  10 s in air, or 4 s with the steel block on the insert (steel draws heat). Soft while the heat is 0.5 or more: 1 s after the iron
  goes on, and for 5 s after a fully heated insert loses the iron (about 2 s if the block is on it).
- **Flow.** While soft, a push on the insert of more than 0.3 N (its spring deflected) moves its set depth down at 80 mm/s per mm of
  deflection beyond that, at most 3 mm/s (1.5 mm/s at about 1.1 N). Anything can push it: the iron, the block (its own weight,
  4.7 N, is enough), or a careless finger (which is a harm, below). While soft and not pushed, it creeps back up at 0.15 mm/s, at most 0.35 mm above the deepest it reached.
  Cold, it does not move. It can go no deeper than its hole allows (1.3 mm below the face).
- **Melt mark.** The iron's shoulder within 0.25 mm of the face while its tip is on an insert, or the iron's tip within 0.5 mm of the
  face anywhere on the plate: the nearest insert is marked (a dark ring appears round it). Marks never go away.
- **Harm.** A fingertip within 1 cm of the iron's hot end (tip to 95 mm up); a fingertip within 1 cm of an insert's top while it is hot
  (heat above 0.05); the iron out of its holder and in no hand for 2 s (a hand holds it when its two fingers are either side of the
  grip); the iron or the block below the table's top.

## Scoring

| Condition | Score |
|---|---|
| A harm, at any time | 0 for the trial |
| At the end, the iron is not lying in its holder (tip within 1.5 cm of its place, axis within 15 degrees of the tube's), or a fingertip is within 3 cm of its centre line | 0 for the trial |
| Otherwise, per insert: top 0.2 mm proud to 0.3 mm below the face, no melt mark | 0.2 |
| Per insert: top 0.2 to 1.5 mm proud, no melt mark | 0.1 |
| Per insert: more than 1.5 mm proud, more than 0.3 mm below the face, or marked | 0 |
| All four flush and unmarked, the iron in its holder, every fingertip at least 3 cm from the iron, the inserts and the block, iron and block still (under 1 cm/s), held for 1 s | 1.0 |

Possible scores: 0, 0.1, 0.2, ..., 0.8 (in steps of 0.1) and 1.0. The score at the end of the episode counts. Doing nothing scores 0
(no insert has sunk). Using the iron alone and stopping short scores at most 0.4 (four seated, none flush); taking each insert to
flush with the iron marks it and scores 0 for it. The block may be laid on and let go (as in the demo) or held and pressed. Either
earns full credit if the inserts end flush and the hands are clear at the end.

### A real trial

- **Setup.** The layout in `task.json` from tape marks. Print a new plate (`plate.scad`, PETG, 0.2 mm layers, 4 perimeters, 30 %
  infill), stick it to the mat with double-sided tape, and drop a new insert into each hole (it stands about 4.4 mm proud on its lead-in;
  check with a caliper's depth rod). The station at 245 C, its iron tip down in the holder, which is clamped to the table. The block stands
  on end at its mark. Robot finger pads get heat-resistant silicone covers.
- **Measure, at the end, before anyone touches anything.** Photograph the plate from above (melt rings), and the iron in its holder.
  Then for each insert: lay a steel straightedge across it; flush if a 0.2 mm feeler gauge will not pass under the straightedge on the
  insert, and the caliper's depth rod reads no more than 0.3 mm below the face; seated if it stands 0.2 to 1.5 mm proud (depth rod).
- **Harms from the video:** a finger on the iron's barrel or tip, a finger on an insert within 10 s of the iron leaving it (the simulation counts an insert hot for 9.5 s after a full heat), the iron set
  down anywhere but its holder for 2 s, anything dropped off the table.
- **Reset** (about 2 minutes): new plate and inserts; the iron back in its holder by its grip; the block on its mark.
- **Calibrating the model.** The softening time, soft window and creep are estimates from the maker's description ("a few seconds").
  To measure them: heat an insert with the iron as above, lift the iron, and with a stopwatch find the latest moment at which the block
  can still press it flush (repeat at 1 s steps on a scrap plate); measure creep with the depth rod 10 s after lifting the iron without
  pressing.

## Red team (scripted variants, `HSI_DEMO=<mode>`)

Each was run once as a full trial (1,800 steps unless stated), with the variant chosen by `HSI_DEMO`. In `finger`, the iron waits higher
(`HSI_ASIDE=0.0,-0.065,0.13`), so that the low left hand can reach the insert. The full demo scored 1.0 in
two runs out of two. The plans are open loop, and their per-hole offsets were measured for the full demo. A variant whose history
differs (no block, a late block) can therefore miss later holes. Each row says what happened as well as the score.

| Variant | What it does | Score | What happened |
|---|---|---|---|
| `none` | nothing | 0 | no insert moves |
| `ironly` (one arm, no block) | the iron alone, stopping 0.8 mm proud; iron back in its holder | 0.1 | insert 0 set 0.76 mm proud (seated: 0.1); inserts 1 to 3 missed by the open-loop plan. By the rules a perfect iron-only run reaches at most 0.4 |
| `ironflush` (the careless way) | the iron alone, taken to flush | 0 | insert 0 marked by the shoulder (0 for it); the rest missed |
| `late` (3,600 steps) | the block laid on each insert 9 s after the iron leaves | 0.1 | insert 0 had set: under the block it only flexed (0.65 mm proud), and it ended 0.77 mm proud (seated: 0.1); inserts 1 to 3 missed by the open-loop plan; iron back in its holder |
| `coldblock` (shortcut) | the block pressed on the inserts without using the iron | 0 | cold inserts do not move; the iron never leaves its holder |
| `keep` (hold instead of release) | everything as the demo, but the iron kept in the hand at the end | 0 | all four flush, but the iron is not released in its holder |
| `finger` (careless) | closed fingers instead of the block, pressed on each hot insert | 0 | harm at step 376: a fingertip on hot insert 0 |
| `holdblock` | the block pressed while held in the hand, never let go | 0.2 | the block hangs 2 to 3 degrees off square in the fingers, so its 50.8 mm face lands on the plate before the insert is flush; inserts 0 and 1 stay about 0.8 mm proud (0.1 each); 2 and 3 missed. A held block that is kept square would also earn full credit: the scorer does not require letting go of it, only that the hands are clear at the end |

## Changes from the earlier submission's review

PR #2509's review made one request: "Typically, to seat a heat-set insert flat, a flat piece of metal is laid on top to seat the last
little bit while it's still hot." This version meets it as follows:

| Point of the review | How it is met here |
|---|---|
| "a flat piece of metal" | A real hardened steel 1-2-3 block (25.4 x 50.8 x 76.2 mm, 0.48 kg) stands on the bench. It is the only way to full marks: the iron alone either stops short (the insert is only seated, 0.1) or marks the plate at flush (0 for that insert). |
| "is laid on top" | The block's flat face goes onto the insert. Its own weight is enough to press a soft insert, so it can simply be laid on and let go, as the review describes; a hand may also press it. A free block settles flat on the plate's face, so the insert ends flush and square. |
| "to seat the last little bit" | The iron is meant to stop about 1 mm proud (the maker's "about 90% of the depth"). The block then takes the insert from there to flush. |
| "while it's still hot" | The plastic stays soft for 5 s after the iron leaves (2 s under the steel, which draws heat). A block that arrives later leaves the insert where it is; lifting it too soon lets the insert creep back up. |
| seat it "flat" | Flush is scored as 0.2 mm proud to 0.3 mm below the face: a straightedge and a 0.2 mm feeler gauge, then a caliper's depth rod. |

Other changes from that submission, for today's review bar:

- No force rule in the score. The earlier version zeroed an insert pushed over 15 N, which cannot be checked after a trial. Every
  score level here is read from the end state with a straightedge, a feeler gauge, a caliper and a photo.
- A plate printed from `plate.scad`, a named insert, iron, holder, tip and block with sourced sizes and masses, and two SimReady
  distractors.
- The harms (a finger on the hot end or a hot insert, the iron left loose, anything dropped) are recorded across the trial and cleared
  only by mechkit's reset hook. The 1.0 needs the iron released in its holder, every fingertip at least 3 cm from the iron, the
  inserts and the block, and everything still for 1 s.

## Assets and licences

SimReady props from NVIDIA's [SimReady assets](https://huggingface.co/datasets/nvidia/simready-assets) at the pinned revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, downloaded at first run (nothing large is committed):

- [`instrument_caliper_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/instrument_caliper_a01): distractor, a digital caliper lying on the bench
- [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01): distractor, a mug

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time: masses set (caliper
150 g, mug 350 g); the caliper's sliding jaw is welded to its beam (one rigid body); the invisible grasp-annotation curves are
deactivated.

SimReady has no soldering iron, iron holder, heat-set insert, printed plate or 1-2-3 block, so these are built in `task.py` from simple
shapes at their real sizes: the iron (pin, tip, heater barrel, nut, grip, cord end: 217 mm, 46 g), the holder tube (square, 30 mm inside,
120 mm long) on its base, the inserts (brass, 4.6 x 5.7 mm), the plate and mat (boxes), and the block (a steel box with its holes drawn).
The scorer ignores the distractors.

## Fidelity

Approximate. PhysX cannot melt plastic, so each insert is a jointed part whose set depth a rule moves (above). The softening time
(1 s), the soft window (5 s, or 2 s under the block), the flow rate and the creep are estimates from the maker's description ("a few
seconds"), not measurements; "A real trial" gives a stopwatch method to measure them. The tip's centring pin collides only as a short
cone (drawn full length), and the insert heats with the tip up to 2 mm off its axis. The plate is fixed to the mat (taped down in a
real trial). The iron has no cord, and its grip collides as an octagonal prism. The holder's tube is square, not the FH-800's round
funnel. The plastic's bulge round an insert and crooked setting are not modelled: an insert slides only along its hole.

[cnck]: https://www.cnckitchen.com/blog/tipps-amp-tricks-fr-gewindeeinstze-im-3d-druck-3awey
[bhl]: https://berkeley-humanoid-lite.gitbook.io/docs/getting-started-with-hardware/preparing-the-tools
[insert]: https://3djake.com/cnc-kitchen/threaded-inserts-m3-standard
[hakko8801]: https://www.hakko.com/english/products/hakko_fx8801_8805_spec.html
[hakko888]: https://www.hakko.com/english/products/hakko_fx888d_set.html
[tip]: https://www.adafruit.com/product/4239
[block]: https://allindustrial.com/all-industrial-55510-1-pair-ultra-precision-1-2-3-blocks-23-holes-0001/
[bls]: https://www.bls.gov/oes/2023/may/oes512028.htm
