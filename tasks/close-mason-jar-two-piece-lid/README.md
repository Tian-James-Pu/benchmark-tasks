# Screw a canning band onto a filled pint Mason jar over its loose flat lid

**Instruction:** *Screw the band on over the lid, fingertip-tight, ready for the canner.*

A filled Ball regular-mouth pint jar stands on the counter with its flat lid (the disc with the sealing compound) already set on the
rim, centred, as home-canning guides direct. Its screw band stands flange-up on the counter beside it. The robot has to lift the band,
bring it down square over the loose disc, start it on the jar's continuous glass thread, and turn it down, regripping, about one and a
quarter turns until the flange meets the disc, then on to fingertip-tight: no further than a third of a turn past that first contact. Then it lets go.

## Response to review

| Review request | What changed |
|---|---|
| "Define a repeatable acceptable tightness range for the specified jar and band" | Fingertip-tight is now **0 to 120 degrees of band turn past first contact** (the point of first resistance), for the Ball regular-mouth pint (MFR #61000) and its band. 0 degrees is Ball's current "just until a point of resistance is met"; 90 degrees is NDSU's and Clemson's "then turn the band one-quarter turn more"; the 30 degrees on top is the mark-reading tolerance. It is an angle, not a torque, so it repeats with a marker and a printed protractor. Over 120 degrees is over-tight. See *Scoring* (`task.json` `scoring`, `level()` in `task.py`). |
| "add a practical real-trial check that distinguishes fingertip-tight from overtightened" | *Real trial*: a paint-pen first-contact mark on the band and on the jar's shoulder, made once per jar, band and lid (the method Clemson gives for beginners). After the trial, a top photo against a printed protractor ring gives the turn between the two marks. A caliper on the band's top says which turn it is on. No force is measured. |
| Construct validity: "awards full credit to an overtightened band" | An over-tight band now scores **0.5**, not 1.0. Red team `overtight` turned the band 202 degrees past contact and scored 0.5 (see *Validation*). |
| "update the simulated scoring or its documented approximation accordingly" | The simulator now has the closure's give. The rim's sealing surface is a sprung link, so a band can be turned on past contact as a real one can, by compressing the compound and flexing the lid and flange. Before, it was rigid, and fingertip-tight and cranked-down looked alike. The scorer reads the band's angle past its first-contact angle, the quantity the real trial reads from the marks. It does not read the spring or any force. `task.json` `scoring` is the text of `level()` and `Trial.step()`, and `fidelity` now describes the give and its estimated stiffness. |
| "specify how to check the disc's final centring and flatness when its edge is covered by the band" | The check is made through the band's 58 mm opening, with the band left on. A paper centre target (concentric rings 1 mm apart) is stuck centred on the lid at preparation. The top photo gives the lid's offset from the band, from the gaps between the opening's edge and the target's outer ring, and the band's offset from the jar, from the gaps between the skirt and the shoulder; together they give the lid's offset from the jar's axis. Flatness comes from the caliper's depth rod onto the lid's face through the opening at three points. Nothing is unscrewed before it is read. |

The scene layout, objects and instruction are unchanged. The only scene change is inside the jar: the sprung sealing surface, which the
band reaches only once it is threaded on. The Dream-1 run never lifted the band off the counter, so its score (0.0) stands.

## Importance and sources

Closing a jar with a two-piece canning lid is the step every home-canning reference spells out, and a familiar one in any kitchen
that puts up jam or pickles. The guidance is consistent:

- North Dakota State University Extension, *Put a Lid On It! Best Practices for Using Closures for Home-based Canning* (FN2028):
  "Place the metal screw band over the flat lid and apply fingertip tight", that is, "turn it just until you feel resistance, then turn
  the band one-quarter turn more", and "Screw bands that are applied too loosely or too tightly can cause jars to not seal properly."
  https://www.ndsu.edu/agriculture/extension/publications/put-lid-it-best-practices-using-closures-home-based-canning
- Clemson Cooperative Extension, Home & Garden Information Center, *Canning Tips: How tight should your lids be?*: place the lid on
  the filled jar, centre it and hold it with the fingers, then screw the band down fingertip-tight without force; "If the screw band is
  too tight, air cannot escape", and buckled lids may not seal.
  https://hgic.clemson.edu/canning-tips-how-tight-should-your-lids-be/
  The same page gives the practical check this task's real trial uses: "it may help to mark the band and lid with a marker at the point
  of first resistance and at the point that represents an additional quarter turn".
- Ball's own current wording, as quoted by Healthy Canning (*What does fingertip tight actually mean?*, which notes Ball's older "until
  resistance is met, then 1/4 turn more"): "Using just your fingers, screw bands down evenly and firmly just until a point of resistance
  is met – fingertip tight. Do not over tighten bands by using the full strength of your hand. Over tightening bands can prevent air from
  venting out of the jars, resulting in buckled lids or a failed seal."
  https://www.healthycanning.com/what-does-fingertip-tight-actually-mean/
- Penn State Extension, *Ensuring a Good Seal on Canned Goods*: "If bands are too tight, air cannot vent during processing and lids will
  buckle; if bands are too loose, vacuum will be low and seals may fail now or later."
  https://extension.psu.edu/ensuring-a-good-seal-on-canned-goods
- The product: Ball 16 oz pint regular-mouth canning jar with lid and band, MFR #61000, 12 per case, about $17 a case
  ($1.42 a jar). Listed dimensions: height 5 1/8 in, top (mouth) diameter 2 3/4 in, maximum diameter 3 1/8 in.
  https://www.webstaurantstore.com/16-oz-regular-mouth-glass-canning-jar-with-silver-metal-lid-and-band-case/52761000.html

**The knack.** The band is a 74 mm thin ring that has to start square on a continuous glass thread while a loose 6 g disc sits under
it. Brought down tilted, its thread climbs over the glass thread on one side (cross-threads), and the band cocks the disc or pushes it
off the rim. Started square, it needs about one and a quarter turns, and a parallel gripper turns it at most about 150 degrees before
it must let go, turn back and grip again, each time without lifting the band off its thread. The second hand holds the jar, as people
do. Then it must stop: fingertip-tight is from first resistance (Ball) to a quarter turn past it (NDSU, Clemson); cranked on further,
the band traps air and the lid buckles. Fingertip-tight here is a band turned 0 to 120 degrees past first contact (the guides' quarter
turn plus 30 degrees of tolerance for reading the marks); past that it is over-tight and scores 0.5.

**Distinct from** the nearest tasks we know of:

| Nearest | What it tests | Difference here |
|---|---|---|
| Flashlight tail cap screwed back on (PR #281, accepted) | a small cap on a tube, after placing a battery | a 74 mm band over a separate loose disc that it must not disturb; regrips through 1.25 turns |
| Spin-on oil filter threaded on by hand (PR #1405, accepted) | a heavy canister onto a nipple under a car | a light ring, a two-piece closure, kitchen context |
| Oxygen humidifier onto a flowmeter (PR #3222, pending) | a bottle threaded onto a fixed outlet | the threaded part is a ring that clamps a separate loose lid |
| E-stop locking ring onto a panel (PR #3180, pending) | a nut-like ring run down a fixed button body behind a rigid panel | the ring has to pass over and clamp a loose 6 g disc without cocking it, on a free-standing filled jar |
| Canning jars: find the unsealed lid by pressing (PR #3326, pending) | inspecting sealed jars | closing the jar before processing |

## Run and score

From the repository root, in the Python environment that has `yam_arena` and `inspect-robots`:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/close-mason-jar-two-piece-lid/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Screw the band on over the lid, fingertip-tight, ready for the canner." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/close-mason-jar-two-piece-lid
```

The scripted demonstration is closed loop (it reads the band's and jar's poses, never the scorer), so it runs through `demokit.py`
from the task folder:

```bash
cd tasks/close-mason-jar-two-piece-lid
python demokit.py task.py ~/yam-eval/close-mason-jar-two-piece-lid/demo --mode full                     # writes full/demo.mp4, setup.png, result.json
python demokit.py task.py ~/yam-eval/close-mason-jar-two-piece-lid/redteam --no-record --mode none,onearm,careless,placed,overtight,contact,short
```

`result.json` holds `task_score`. The layout is fixed (`object_jitter=0`); there are no seeds or random choices. The demo takes 1,047
control steps (52 simulated seconds); the run limit is 2,400 steps. A GPU-rendered run took 6 to 10 minutes of wall time on a shared
GPU (a CPU `--fast` run, no cameras, about 1 minute).

## How the simulator models it

All in `task.py`; nothing is triggered by proximity, every state change is contact.

- **Jar** (free articulation: a rigid glass body and its sprung sealing surface, 0.75 kg filled): the glass body is the convex hull of a lathe profile at the Ball pint's published size
  (79.4 mm widest, 69.9 mm across the thread), the neck a cylinder at the thread's root, the rim a 66.6 mm sealing surface 127 mm up.
  The glass thread is a continuous right-hand helix, 6 per inch (4.23 mm pitch), 1.25 turns, made of 30 convex segments of a trapezoid
  section (2.2 mm wide at the root, 1.2 mm at the crest, 1.6 mm deep), running out to its root over the last 29 degrees at each end, as
  a moulded thread does. Glass-on-counter friction 0.5.
- **Band** (free, 11 g): flange (58 mm opening) and skirt (73.6 mm outside, 13.5 mm tall) as 48 annular segments, and its rolled
  internal thread, the same helix, 0.875 turn, 21 segments, also run out at its ends. It stands on the counter turned 125 degrees from
  where its thread would line up with the jar's. Turning it clockwise seen from above draws it down the glass thread until
  the flange meets the disc (first contact); turning on loads the closure's give.
- **Disc** (free, 6 g, 67.2 mm by 1.5 mm with its compound ring): loose on the rim. Its edge clears the band's thread crest by 0.6 mm,
  so a square band passes over it; a tilted or off-centre band lands on it.
- **The closure's give** (jar link `seal`): a real band can be turned on past first contact because the lid's sealing compound
  compresses and the lid and the band's flange flex. That give is lumped into the rim's top 4 mm, the sealing surface, which slides down
  the jar's axis against a 6 N/mm spring (up to 4 mm). At 6 per inch a quarter turn past contact is 1.06 mm of travel, 6.4 N on the
  spring; 200 degrees is 2.35 mm. The stiffness is our estimate, not a measurement: it only sets how hard the hand has to work to
  over-tighten, and the score reads the band's angle, never the spring. The rim's visual stays with the glass: the 1-2 mm it moves is
  under the band.
- **Not modelled:** torque and force (nothing is scored on them), the band's knurl, heat. The thread's turn count and section are
  estimates for a GPI 70 mm continuous-thread finish, not a measured drawing.

## Scoring

**Turn** is how far the band has been turned clockwise (seen from above) past the angle at which it first touched the lid: the angle
from the first-contact mark on the jar's shoulder to the band's witness mark (see the real trial), negative if the band is short of
first contact. The marks give it within a turn; the band's height (4.23 mm a turn) says which turn. In the simulator the same angle is
the band's turn about the jar's axis less its turn at first contact (`YAW_CONTACT`, found the way a real trial makes its mark: the band's
angle when its flange first touched the lid, -30.5 to -32.9 degrees in three runs), with the height picking the turn. In the
demo and over-tight runs, the demo counted the band's turn since contact itself, and its count matched this reading to about
1 degree: 62.4 against 61.3, and 201.5 against 201.7.

- **0** if at any time the jar tipped more than 25 degrees, or the disc left the rim (its centre more than 12 mm off the jar's axis, or
  its underside more than 5 mm below the rim). This is recorded each control step and cleared only at a reset.
- **1.0 (fingertip-tight):** turn 0 to 120 degrees; band within 2 degrees of the jar's axis and centred within 2 mm; disc flat within
  2 degrees and centred within 2.5 mm; jar within 5 degrees of upright and within 3 mm of the counter; nothing moving (no origin more
  than 0.5 mm, the band not more than 0.5 degree, between control steps); every fingertip point at least 3 cm from jar and band; all for
  1 s (20 steps). Credit lapses if the state that earned it is later undone.
- **0.5:** band within 2 degrees and 2 mm of centred, disc within 5 degrees of flat, turn at least -90 degrees. This is the level for an
  **over-tight** band (turned more than 120 degrees past first contact), for a band up to a quarter turn short of contact, and for a
  fingertip-tight band without the rest of 1.0 (hands not clear, not held 1 s).
- **0.25:** band within 4 degrees and 3 mm of centred, its top at most 2.5 mm above its first-contact height: threaded on. An
  unthreaded band resting on top of the glass thread stands at least 3.7 mm up (one pitch, less the 45 degrees of spare thread), so it
  cannot earn this.
- **0** otherwise: the band left on the counter, set on unthreaded, or cross-threaded.

Possible scores: 0, 0.25, 0.5, 1.0.

Why 0 to 120 degrees: Ball's current wording is "just until a point of resistance is met" (0 degrees); NDSU and Clemson say resistance,
then a quarter turn more (90 degrees). The upper limit adds 30 degrees for reading the marks (about 20 mm of arc at the band's edge,
far more than the reading error). No published source gives an angle for "over-tight"; the guides define fingertip-tight and warn
against going further with the hand's full strength, so anything past that tolerance scores as over-tight.

## Real trial: preparation, reset and measurement

- **Parts:** a Ball regular-mouth pint jar with its new lid and band (MFR #61000), filled cold with jam or water to 1/2 in headspace;
  a kitchen counter or table; a paint pen; a printed protractor ring (a paper ring, 82 mm inside, 5-degree ticks) taped to the counter
  as the jar's base mark; a printed paper centre target (a 50 mm sticker of concentric rings 1 mm apart) on the lid; a caliper. No heat,
  no canner, no sharp edges: safe for the arms' pads.
- **First-contact mark (once per jar, band and lid; redo it with a new lid):** stand the jar in the ring, wipe the rim, set the lid on
  it centred (equal overhang all round). Spin the band down with one fingertip on its knurl until it stops coming down: the point of first
  resistance. Draw a short radial paint-pen tick on the band's top and one on the jar's shoulder directly outside it (Clemson's "mark
  the band and lid with a marker at the point of first resistance"; the shoulder is used because the lid is hidden under the band). The
  shoulder shows from above in the 3 mm ring between the band (73.6 mm) and the jar's widest part (79.4 mm). Caliper the band top's
  height above the counter at three points: the mean is the first-contact height. Then unscrew the band and stand it flange-up on its
  tape mark, 18 cm from the jar.
- **Centre target:** stick the target on the lid, centred on it with a printed 67 mm template. It is paper (0.1 mm), inside the band's
  58 mm opening, so the band never touches it.
- **Measurement, in this order, before anyone touches the jar:**
  1. *Top photo* (camera on a stand straight over the jar):
     - **Turn:** the angle, clockwise about the jar's centre, from the shoulder tick to the band's tick, read against the protractor
       ring. 0 to 120 degrees is fingertip-tight. Over 120 degrees is over-tight. The ticks show a turn modulo 360 degrees; step 3 tells
       a band short of contact from one turned round past it.
     - **Band centring:** the gap between the band's skirt and the jar's shoulder edge, at four points 90 degrees apart (2.9 mm
       nominal); half the difference of opposite gaps is the band's offset in that direction.
     - **Disc centring:** through the band's opening, the gap between the opening's edge and the target's outer ring, at the same four
       points (4 mm nominal); half the difference of opposite gaps is the lid's offset from the band. Add this to the band's offset
       (as vectors) to get the lid's offset from the jar's axis: within 2.5 mm for 1.0.
  2. *Video:* a tip, a dropped lid, the hands clear for the last second and nothing moving.
  3. *Caliper (the jaws only touch the band's top):* the band top's height at three points 120 degrees apart. The mean less the
     first-contact height is the rise. It says only which turn the marks are on (4.23 mm a turn), never the angle within a turn: a loose
     band can drop up to about 2 mm through its thread play and sit on the lid (the `short` red-team run did), so its height does not
     show how far it is short; the marks do. The spread over the 66 mm across the band gives the
     band's tilt (1.15 mm of spread is 1 degree).
  4. *Disc flatness:* the caliper's depth rod through the band's opening, onto the lid's face just inside the opening, at three
     points 120 degrees apart. The spread over the 55 mm between them gives the lid's tilt: 1.9 mm of spread is 2 degrees.
     The rod rests lightly on a lid clamped under the band, so it cannot move it.
- **Reset (about 2 minutes):** unscrew the band, stand it on its mark, re-centre the same lid. Replace the lid with a new one (and redo
  the first-contact mark and the target) if it fell to the counter, or if the band was turned more than 120 degrees past contact (its
  compound may be crushed). Jar back in its ring.

## Assets and licences

The jar, disc and band are authored meshes (SimReady has no canning jar or band; the thread is the mechanism under test). The four
distractors are NVIDIA SimReady props, downloaded at first run from the pinned revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`
of `nvidia/simready-assets` on Hugging Face, © NVIDIA, CC-BY 4.0:

- [food_produce_cucumber_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/food_produce_cucumber_a01) (two copies): mass set to 0.15 kg each.
- [dishware_bowl_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_bowl_a01): mass set to 0.35 kg.
- [cleaning_cloth_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/cleaning_cloth_a01): mass set to 0.05 kg.

Nothing large is committed. `simready.py` is the loader from `tasks/cutters-off-pegboard/` (unchanged apart from its docstring), `mechkit.py` the mechanism kit
from `tasks/heat-shrink-lug/` (unchanged apart from a credit line) and `demokit.py` the closed-loop demo runner from
`tasks/heat-shrink-lug/` (unchanged): other teams' open-source files in this repository.

## Validation

All runs are on the revised `task.py`, with GPU physics. Demos were rendered; red-team runs used `--no-record`. Contact sheets and
the demo video were checked by eye: the band is carried clear over the jar, comes down square and is turned in four strokes, and both
hands end clear. The demo stops at a target angle past first contact. It finds contact from the band's height reaching the
first-contact height, which is what a person sees and feels as the flange landing on the lid, and then counts the band's own turn. It
never reads the scorer or the spring. Turn below is the scorer's reading from the band's angle.

| Run | Score | Steps | What happened |
|---|---:|---:|---|
| demo (`full`), run A | 1.0 | 1,047 | threaded and turned down in 4 strokes, to 61 degrees past contact (give taken up 0.5 mm); band square within 0.1 degree, disc centred 1.4 mm |
| demo (`full`), run B | 1.0 | 1,047 | the same |
| demo (`full`), run C | 1.0 | 1,047 | the same, on the final file (only the `short` mode's stopping rule changed after A and B); its video and screenshot are the submitted media |
| overtight | 0.5 | 1,328 | as the demo, but turned on to 202 degrees past contact: over-tight, 0.5 |
| contact | 1.0 | 938 | stopped at first resistance, 10 degrees past contact (Ball's "just until resistance"): fingertip-tight |
| short | 0.5 | 1,028 | stopped about a quarter turn before contact and let go. The loose band dropped onto the lid through its thread play (rise 0.0 mm), but its marks read 83 degrees short of contact: 0.5 |
| none | 0.0 | 143 | band left on the counter |
| placed | 0.0 | 583 | band set square on the jar and let go unturned: rise 4.6 mm (unthreaded) |
| careless | 0.0 | 752 | band brought down tilted 10 degrees: it cocked the disc and pushed it off the rim (harm), then fell off |
| onearm | 1.0 | 916 | the right arm alone, jar not held, 59 degrees past contact: the 0.75 kg jar did not turn on the counter, so one hand suffices here |

The one-arm run is an honest result, not a cheat: the scorer does not require the second hand, and in the simulator the filled jar's
friction on the counter holds it against the band's turning. People usually steady the jar; we do not claim it is necessary.
Distractor drift was logged on every run: the bowl and cloth moved under 1 mm and the cucumbers rolled about 2 cm as they came to rest
after spawning, by the same amount in every run; nothing touched them.

**Dream-1 run (2,400 steps): 0.0.** The model brought the right gripper down to the band and nudged it about on the counter, but never lifted it; the band stayed on the counter for the whole run, the jar and disc untouched.
