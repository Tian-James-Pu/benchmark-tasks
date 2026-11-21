# Put a five-piece wooden matryoshka back together, pressing each snug top onto its own bottom with its face lined up

A five-piece wooden nesting doll set has been taken apart on a living-room table. The two largest dolls are open:

- each bottom half stands open end up, its painted apron toward the arms;
- each top half stands on its rim, turned a quarter turn away from its apron.

The third doll stands closed, with the two smallest inside it.

The robot must put the set back together and let go:

- the third doll into the second doll's bottom half;
- the second doll closed with its own top, face over apron;
- the closed second doll into the first doll's bottom half;
- the first doll closed with its own top, face over apron, standing on the table.

Instruction given to the policy:

`Put the nesting dolls back together, each one closed and facing the same way as its body, with the smaller ones inside the largest.`

**The knack.** A matryoshka's top is turned to a snug friction fit over a short collar (the spigot) above the bottom half's shoulder. Set
on its collar, a top does not fall into place: it stops partway down, with a visible gap. Here the first doll's top stops 6.9 mm up and the
second's 5.1 mm up. A top closes only when it is pressed down square, or pressed and twisted, against the fit:

- about 10 N for the first doll (it cocks and jams if the push is off-square);
- about 5 N for the second.

So the robot has to do these things in order, and none can be skipped:

- close each doll by pressing its top down square while the bottom stands on the table;
- turn the top so that its face lines up with the apron, either before it goes on or by twisting it against the fit;
- close the inner doll before it goes into the next one;
- carry the closed second doll by its head (the fit holds its bottom half and the third doll inside) and lower it into the first doll's
  bottom half;
- only then close the first doll over it.

**The careless way fails.** If a top is set on and let go, it stays up on its collar. The doll is not closed, and a doll carried by such a
top drops its bottom half. If a top is pressed on facing the wrong way, it is closed but wrong: the face is 90 degrees off the apron.

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: the dolls (turned shells built in code), the fit, `score()`/`success()`, and the scripted `demo()` with its red-team modes |
| `mechkit.py` | The mechanism kit (parts written as USD text, sprung joints, hooks, fingertip points), unchanged |
| `simready.py` | Loads the SimReady distractor props, unchanged |
| `calib_fit.py` | A calibration run, not the task: pushes a top onto its collar with a growing force, then pulls it off (see "The fit, measured in the simulator") |
| `task.json` | The task's details for the board |

## An independent implementation of a lapsed idea

The idea of reassembling a set of nesting dolls was first submitted by another team in PR #3078. Its review was "Not accepted", and its 48-hour
hold lapsed with no revision. This task is our own implementation: the geometry, fit model, scorer, demo and text were all written from
scratch here.

The review raised two points, and its revision request asked for representative real dolls and a feasible demo. Each is addressed here:

| Review point | How this task meets it |
|---|---|
| Construct validity: "the simulated tops seat under their own weight with 1.5 mm clearance, removing the seating force of the stated snug real-world fit" | Each collar carries four sprung pads that stand proud of the top's bore (0.6 mm for doll 1, 0.5 mm for doll 2), so the bore always grips them. The fit is that radial load, acting through ordinary contact friction. A top set on its collar rests 6.9 mm (doll 1) or 5.1 mm (doll 2) above seated, measured with no hands in `calib_fit.py`. It seats only under a push of about 10 N or 5 N. Seated, it holds 9 N or 4 N of pull. The scorer reads "closed" from the rim on the shoulder (within 1 mm), which only a press reaches. |
| Feasibility: "the reported demonstration uses gravity-seated tops and does not demonstrate closing the stated snug real-world halves" | The demo grips each top's head, lowers it onto its collar and drives the hand 30 to 35 mm below seated. This presses the top down against the fit, and while still pressing it twists the top 8 degrees each way and back. The demo then lets go and moves away. It reaches 1.0 at step 1,136 of the 2,400 allowed. The red-team mode `no_press` sets the tops on and lets go, and it does not close them. |
| Use representative real dolls; name a product with dimensions | The largest doll is 150 mm tall and 71 mm across, as in a named, purchasable five-piece set (see "The product"), and 150 mm with five dolls is the standard Soviet-era size. The smaller dolls are this outline scaled to nest, with real wall and floor thicknesses. Masses come from linden. |
| Fix feasibility | See the second row. Two consecutive demo runs scored 1.0, under "Scores". |

## The product

- **The set.** The model is a five-piece turned-wood matryoshka with a largest doll 15 cm tall and 7.1 cm wide and a smallest, solid doll
  of about 3.3 cm. Spire sells a set of this size as
  ["Nesting Doll Set 5 pc - 15 cm"](https://letsplayspire.com/products/nesting-doll-set-5-pc-15-cm). We read those dimensions from
  search-engine text quoting the listing; the page itself refused automated requests when this was written.
- **Other sets of the same size and kind:**
  - G. DeBrekht's handmade linden "Mother Love" set of 5 is
    ["Approximate size: 6.5 x 3.25"](https://1800flowers.com/marketplace/p/mother-love-matreska-nesting-doll) inches.
  - Wikipedia gives the standard Soviet-era set as six inches tall with five dolls, and says the dolls are "ordinarily ... crafted from
    linden wood" ([Matryoshka doll](https://en.wikipedia.org/wiki/Matryoshka_doll)).

  Any lacquered five-piece set with a largest doll of 140-160 mm will do for real trials.
- **Dimensions used.** These are the numbers the scene is built from. Only doll 1's outer size is from a listing.
  - Doll 1: 150 mm tall, 71 mm across, split at the waist 62 mm up.
  - Doll 2: 103.5 mm by 49 mm, split 42.8 mm up.
  - Doll 3: 63 mm by 30 mm, closed, with dolls 4 and 5 inside it.
  - The size steps between the dolls are our choice, so that each nests in the next with 1.8-2 mm of radial play at the narrowest
    point (the collar's inside). This is an inference, not a measurement.
- **The fit is real, and it is made on purpose.** In "How Products Are Made", the bottom half's "ring fitting the bottom to the top" is
  turned last. The top's inset for that ring is then cut to match, which "tightens the ring to its upper fitting so the halves of the doll
  will close securely" ([madehow.com, Matryoshka Doll](http://www.madehow.com/Volume-6/Matryoshka-Doll.html)).
- **The fit can be stiff.** A doll seller's opening guide warns that halves can stick "due the fact that the wood has slightly expanded
  since your nesting doll was first created" ([The Russian Store](https://therussianstore.com/blogs/nesting-dolls/how-to-open-your-nesting-dolls)).
- **The closing forces are estimates.** The forces here (10 N and 5 N to close, 9 N and 4 N to pull off) are our estimates for a snug,
  lacquered fit. They are not measurements of a particular set. "Real trials" gives a kitchen-scale check to match a real set.

## Importance and sources

**Category: Societal relatability.** Nesting dolls are among the most familiar toys and souvenirs, and putting a set back together after a
child (or a visitor) has opened it is an everyday tidy-up.

- The matryoshka is a mass-market toy and ornament, made since 1892 and sold worldwide
  ([Wikipedia: Matryoshka doll](https://en.wikipedia.org/wiki/Matryoshka_doll)).
- Early-childhood programmes give toddlers nesting dolls for "developing fine motor skills" and "differentiation of size", from "18 months
  and up" ([Montessori Academy: Russian Nesting Dolls](https://montessoriacademy.com.au/montessori-at-home/preschool-resources/russian-matryoshka-nesting-dolls/)).
  The adult who tidies the set away does exactly this task.
- *Inference, not sourced:* that "most adults" have opened and closed such a set themselves. We found no survey, so the claim rests on how
  common the toy is.

The difficulty is physical, not visual. Matching tops to bottoms by size is easy to see. The work is in the manipulation:

- pressing a light, round-headed shell square onto a 10 mm collar against about 5-10 N, without cocking it;
- lining up its face, by turning it before it goes on or twisting it against the fit;
- carrying a closed doll by its head, relying on the fit, and lowering it into a bore with about 2 mm of play;
- doing it in the only order that works.

**Distinct from the nearest board tasks:**

| Task | What it tests | Why this task differs |
|---|---|---|
| Close a rigid telescoping-lid box over its product without wedging the lid (TK-WQ8VQ6, #2264) | Lowering a sliding lid square over a box so that it doesn't wedge | One lid on one box, closed by gravity and alignment. Here there are two nested closures with a friction fit that needs a real press, a face to line up, and an order that matters (the inner doll must be closed and inside before the outer one closes). |
| Paint-can lid close (TK-CYLBEJ, #2337) | Pressing a lid into a rim | A press-in lid with no orientation and no nesting. Here the press is onto a round, light shell held by its head, then turned to a painted face. |
| Put a CD back in its jewel case: press it onto the hub (TK-GTGGNW, #2342) | Pressing a disc onto a hub, then shutting a lid | A flat disc on a hub. Here there are two coaxial shells that must be pressed together square, then nested. |
| Separate nested bowls and lay out two breakfast settings (#2214, closed) | Separating nested bowls | The reverse direction, with no fits and no closures. |
| The original PR #3078 (lapsed) | The same idea | See the section above. |

We also checked prior benchmarks. RoboARC's 106 standard tasks have no nesting-doll task and no
press-fit closure of a hollow shell. We did not do a full search of the LIBERO, RoboCasa or BEHAVIOR task lists for matryoshka tasks. Their
lid and stacking tasks that we know of are pick-and-place without a friction fit.

## Run a trial

From the repository root, in the Isaac Lab-Arena environment (`source ~/IsaacLab-Arena/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/reassemble-nesting-dolls/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Put the nesting dolls back together, each one closed and facing the same way as its body, with the smaller ones inside the largest." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/reassemble-nesting-dolls/dream
```

To run the scripted demo, use `--policy yam_scripted` with `--max-steps 1300` and `-E cam_width=640 -E cam_height=360`. The red-team modes
use the same command with `TASK_DEMO=<mode>` set in the environment: `none`, `no_press`, `unturned`, `hold` or `outside`.

To run the calibration of the fit, set `FIT_DOLL=1` or `FIT_DOLL=2` and `FIT_LOG=<file>`, and pass `-E task=tasks/reassemble-nesting-dolls/calib_fit.py`
with `--max-steps 760`.

- **Runtime:** a 2,400-step trial takes about 3 to 6 minutes on one GPU, depending on load. The demo ends at success, after 1,136 steps.
- **Seeds:** one fixed layout (`object_jitter=0`). Nothing else is random, and the scripted demo uses no seed.

## How the simulator models the set

- **Shells.** Each half is a rigid turned shell. Its outline is a classic matryoshka profile (flat foot, full belly, waist split, shoulders,
  neck, round head) with the dimensions above, drawn as a lathed mesh.
  - **Colliders:** convex pieces. There are 16 sectors round each wall, in 3 to 6 tiers following the outline, and 24 sectors round each
    collar and bore. The floors and crowns are solid.
  - **Walls:** 3.5 mm (doll 1) and 3 mm (doll 2).
  - **Floors:** 5 mm and 4 mm.
  - **Masses:** from the shells' volumes at 500 kg/m3 (lacquered linden; linden is about 420-530 kg/m3):
    - bottom 1: 32 g; top 1: 20 g;
    - bottom 2: 13 g; top 2: 8 g;
    - doll 3, with 4 and 5 inside it: 10 g. This last is an estimate, at about 70% of a solid doll of doll 3's size.
- **The collar and bore.**
  - Each bottom's collar stands 10 mm (doll 1) or 8 mm (doll 2) above the shoulder. Its wall is 2.5 or 2.2 mm thick, and its top edge is
    chamfered 1 mm.
  - The top's bore clears the collar by 1 mm radially, and its mouth is chamfered 0.8 mm.
  - The fit itself comes from four pads, set in slots at 90-degree spacing round the collar. Each pad is 8 mm wide for doll 1 and 6 mm for
    doll 2, and runs the collar's height, with a 4 mm (or 3.2 mm) lead-in ramp at the top.
  - Each pad is a link on a radial slider, with a spring of 6,700 N/m (doll 1) or 5,000 N/m (doll 2) and 15 N s/m of damping.
  - At rest each pad stands 0.6 mm (or 0.5 mm) proud of the bore. A top going over the collar squeezes all four in, and their spring load,
    through contact friction (static 0.5, dynamic 0.4, lacquered wood on wood), is the grip. It resists sliding and twisting alike.
  - The pads are hidden: they stand for the wood's own springiness, which a rigid shell lacks.
  - Nothing is triggered by distance or state. A top seats or doesn't by contact and friction alone.
- **The fit, measured in the simulator** (`calib_fit.py`). A top is set square on its collar with no hands. A downward force on its axis then
  grows by 1 N/s. Afterwards the bottom is held down, and an upward pull grows by 1 N/s.

  | | Rests, unpressed | Starts down | Cocks partway | Seated at | Pulls off at |
  |---|---|---|---|---|---|
  | Doll 1 | 6.9 mm above seated | ~1 N (slides down its ramp to 6.2 mm by 6 N) | 3.2 mm up, tilted 5.6 degrees, at 7-9 N | 10 N (rim on the shoulder, 0.0 mm) | 8.5-9 N |
  | Doll 2 | 5.1 mm above seated | ~1 N | 3.0 mm up, tilted 7.5 degrees, at 2-4 N | 5 N | 3.5-4 N |

  Both pull-off forces hold far more than the weight below a top. This is why the closed second doll (21 g below its top, with doll 3
  inside) can be carried by its head.
- **The third doll** (closed, with dolls 4 and 5 inside) is one rigid body: two convex hulls, the body and the head.
- **Paint.** The face, cheeks, shawl, apron and flower are visual only.
- **The scorer** reads poses: each top's pose in its own bottom's frame, the inner dolls' poses in the next bottom's frame, and doll 1 on
  the table. For the hands-off check it also reads the arms' fingertip points. It keeps one record across steps, whether any piece has
  fallen off the table, and only a reset clears it (mechkit's `on_reset`). The stillness window also starts again whenever any condition
  fails.

## Scoring

The terms are defined as follows. All are read at the end of the trial.

- **Seated.** A top's rim is within 1 mm of its own bottom's shoulder (it cannot go lower: the shoulder stops it), centred on the bottom's
  axis within 1.5 mm, and square to it within 3 degrees.
- **Facing.** The top's face is within 15 degrees of the apron below it (about the dolls' axis).
- **Inside.** A doll stands on the floor of the next bottom half, within 2 mm of it, within 10 mm of that half's axis, and within 8 degrees
  of its upright.

The score is the highest level reached:

| Score | Condition |
|---|---|
| 0.25 | Doll 3 is inside doll 2's bottom half. |
| 0.5 | That, and doll 2's own top is seated and facing. |
| 0.75 | That, and the closed doll 2 is inside doll 1's bottom half. |
| 1.0 | That, and doll 1's own top is seated and facing, and doll 1 stands on the table (within 2 mm, upright within 5 degrees). Also, every gripper fingertip is at least 30 mm from every piece, and no piece moves more than 1 mm or 1 degree. All of this holds together for 1 s, which ends the trial. |
| 0 | Anything else. A top on the wrong doll never counts. |
| 0 for the trial | Any piece falls off the table (ends up more than 30 mm below its top) at any time. Only a reset clears this; putting things back does not. |

The levels are cumulative: a closed but empty doll 1 earns nothing for being closed, because its contents come first. Every level reads
the end state, so credit does not stick after things come apart. A fully assembled set that is still held or still moving scores 0.75.

## The demo and the red team

**The demo** (`demo()`, open loop from the published layout) uses both arms.

The right hand does the second doll:

1. It pinches the third doll's head, lowers the doll into the second doll's bottom half and lets go 12 mm above the floor. The doll drops
   in.
2. It pinches the second doll's top by its head, turns it a quarter turn while carrying it over, and lowers it onto its collar.
3. It drives 30 mm below seated, twists the top 8 degrees each way and back while still pressing, eases off and lets go.
4. It pinches the closed second doll's head, lifts the whole doll by its fit, lowers it into the first doll's bottom half and lets go
   10 mm up.

Meanwhile the left hand does the first doll's top. It picks the top up and turns it a quarter turn, then waits clear of the second doll's
path. It carries the top over high enough to clear the second doll's head, lowers it, presses 35 mm below seated with the same twist, and
lets go.

**Grips.** Each pinch closes to the head's width less 4 mm, not fully. A full close lets the stronger finger shove a light half along the
table before the other finger arrives.

**Simulator state.** The demo uses no simulator state at all: it is a fixed plan from the published layout.

**Red team** (`TASK_DEMO=<mode>`, each run once on the GPU):

| Mode | What it does | Expected | Scored |
|---|---|---|---|
| `none` | Nothing | 0 | 0 |
| `no_press` | The careless way: each top is set on its collar and let go, as if it would fall into place. The second doll is then carried by its unpressed top. | 0.25 at most (doll 3 is in; no top closes) | 0.25: both tops stayed up on their collars, and the second doll's bottom half stayed behind when its top was lifted |
| `unturned` | Both tops pressed fully on without being turned: closed, but each face 90 degrees off its apron | 0.25 | 0.25 |
| `hold` | Everything done, but the left hand keeps holding the first doll's top | 0.75 | 0.75 (all assembled, never released) |
| `outside` | The second doll closed but left beside the first, which is then closed empty | 0.5 | 0.5 |

**One arm.** The task does not claim to need two hands: each closure is pressed down against the table, so one arm could do the whole job
where it can reach. The demo uses both arms for reach and time.

## Scores

- Scripted demo: 1.0 in two consecutive GPU runs of the final code, each ending at success after 1,136 steps (56.8 s) of the 2,400
  allowed.
- Red team: as in the table above (one GPU run each).
- Dream-1 run (2,400 steps): 0.0. It moved its arms among the pieces on its right for the two minutes and closed no doll.

## Real trials

**Build.**

- One five-piece lacquered linden matryoshka with a 150 mm largest doll (see "The product"), about $20-30.
- A table, tape for the marks, a kitchen scale (1 g), a 1 mm plastic card, and a phone camera.

**Fit check** (once per set and per day, since the wood swells with humidity):

1. Stand each bottom half on the kitchen scale and zero it.
2. Set its own top on square, and press down on the head with one finger until the top seats.
3. Read the peak.

The set suits this task if:

- doll 1 needs 0.7-1.5 kg (7-15 N) and doll 2 0.3-0.8 kg;
- each top, set on unpressed, stops with a visible gap of 3 mm or more.

If a set is too loose, a thin strip of paper tape inside the bore is the usual fix. If it is too tight, leave the set a day in dry air.

**Setup and reset** (about 2 minutes):

1. Open dolls 1 and 2. Doll 3 stays closed, with 4 and 5 inside.
2. Stand the halves on tape outlines at the positions in `task.json`, with the bottoms' aprons toward the arms and the tops turned as listed.
3. Check that each top, set on its own collar unpressed, still shows its gap. If not, open and reseat it.
4. Put the distractors on their marks.

**Measurement** (end state first, then opening):

1. **Seated:** the 1 mm card must not enter under any part of the top's rim.
2. **Facing:** take a photo from above the head. Lay a protractor card on it, or mark the centre of each face and apron with a paint-pen
   dot and check the two dots are within 15 degrees.
3. **Inside:** after the photos, open doll 1 and then doll 2, and confirm what stands on each floor. Opening can only fail a reading; it
   cannot create one.
4. **Fallen piece:** any piece on the floor during the trial scores 0. The video shows it.

**Rig safety.** The pieces are smooth lacquered wood, under 35 g each, with no edges, heat or liquids. Grip pads need no protection.

## Assets and licences

**SimReady props.** The distractors are NVIDIA SimReady props from
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, © NVIDIA, CC-BY 4.0:

- [`decor_candle_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/decor_candle_a01)
  (pillar candle);
- [`kitchenware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/kitchenware_mug_a01)
  (mug);
- [`stationery_pen_metal_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/stationery_pen_metal_a01)
  (pen).

Changes made at load: masses set (350 g, 300 g, 20 g), convex-hull colliders. `simready.py` downloads them into `~/.cache/simready` the
first time the scene is built, which needs network access. Nothing large is committed.

**The dolls.** SimReady has no nesting doll. The dolls are generated in `task.py` from the outline and dimensions above. They are not a
third-party asset.

**Code.** `mechkit.py` is copied unchanged from this repository's `tasks/heat-shrink-lug/mechkit.py`, another team's open-source kit.
`simready.py` is adapted from `tasks/cutters-off-pegboard/simready.py`, with its loader unchanged.
