# Peel the ring-pull lid off a cat-food can

An unopened 85 g (3 oz) single-serve cat-food can stands on a counter. The instruction is only: **"Open the can of
cat food: take its lid right off and put the lid down on the table away from the can, without spilling any food."**

The can is a full-aperture easy-open can: a ring-pull tab riveted to a scored steel panel. Lifting the ring's end a few
millimetres levers the tab's nose down onto the score and cracks it (the "pop"); pulling the ring up and back then
tears the score round the panel until the whole panel comes away. The can is light: filled, it weighs about 105 g,
while the pop takes about 15 N at the ring. Held only by its ring, the can comes up with the lid or tips over and the
food spills. So one hand holds the can down while the other works the ring, and the lid is then set down out of the
way (its edge is sharp).

## Run and score

From the repository root, in the Python environment that has `yam_arena` and `inspect-robots`:

```bash
# scripted demonstration (640 x 360 frames)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/peel-ring-pull-lid-off-cat-food-can/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "Open the can of cat food: take its lid right off and put the lid down on the table away from the can, without spilling any food." \
  --scorer task_score --max-steps 760 --store-frames --log-dir ~/yam-eval/peel-ring-pull-lid-off-cat-food-can

# model run: --policy dream, 320 x 176 frames, the run limit of 1,200 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/peel-ring-pull-lid-off-cat-food-can/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the can of cat food: take its lid right off and put the lid down on the table away from the can, without spilling any food." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/peel-ring-pull-lid-off-cat-food-can
```

The run prints `task_score`. The layout is fixed (`object_jitter=0`) and nothing depends on the seed. The scripted
demo takes about 33 s of simulated time (660 steps); a trial takes about 1 to 2 minutes of wall time on one GPU
once shaders are compiled.

## The scene

Robot frame: x forward from the arm bases, y to the left, z up; the arm bases are at x 0, y +-30.5 cm; the tabletop
is z 0.7475 m in the simulator and stands in for a kitchen counter.

| Object | Model | Size, material, mass | Where |
|---|---|---|---|
| Cat-food can | Purina Fancy Feast 85 g (3 oz), size 211 x 108, full-aperture easy-open end | 6.8 cm over the top seam, 6.5 cm body, 3.8 cm tall; tinplate steel; 105 g filled | upright, axis at x 30 cm, y +3 cm |
| Removable panel (lid) | the end's panel inside the score line | 5.7 cm across, 4.5 mm below the top of the seam; steel, 4 g | in the can |
| Ring-pull tab | riveted to the panel | ring 2.0 cm wide, 2.7 cm from rivet to ring end, 1.5 mm thick (curled edges); steel, 2 g | rivet 1.45 cm in from the score on the +y side; the ring points at the panel's centre (-y), formed 6 degrees upward |
| Cat bowl (distractor) | SimReady `dishware_bowl_a01` | 16.3 cm across, 5.5 cm tall; ceramic; 350 g | centred at x 52 cm, y +17 cm |
| Spoon (distractor) | SimReady `dishware_plastic_spoon_a01` | 15.7 x 3.5 cm; plastic; 6 g | lying at x 20 cm, y +25 cm, turned 29 degrees |
| Fork (distractor) | SimReady `dishware_plastic_fork_a01` | 15.8 x 3.0 cm; plastic; 6 g | lying at x 53 cm, y -20 cm, turned -23 degrees |

The distractors sit at least 10 cm from the can, outside the arms' working paths; scoring ignores them.

## How the simulator models the can

The defining mechanism (tab, score, tear) is modelled with joints and real resistances. `task.py` holds every
dimension; `mechkit.py` provides the meshes, colliders, articulation and per-step hooks.

- **The tab** is a link on a hinge at the rivet (the axis across the ring). Before the pop it is held flat by a
  spring of 2.9 N m/rad: lifting the ring's end 8 degrees on the rivet (about 4 mm at its end) takes 0.40 N m, about
  15 N at the ring's end. That deflection is the pop. After it, the tab folds freely on its rivet.
- **The seal and the tear.** The panel is a separate rigid body joined to the can by two joints written after each
  reset: a fixed joint (the uncracked score) and a hinge along the far side of the score, at the inside top edge of
  the seam, where a real peel finishes. The pop deletes the fixed joint. From then on the hinge's drive holds the
  panel at its tear front: pulling the panel past the front tears on (the front moves up), but the front never moves
  back, so a torn panel does not re-seal or spring shut. The tear takes 0.30 N m for the first 20 degrees (round the
  rivet end of the score), then 0.22 N m. At 105 degrees the last segment of score holds until it is pulled 0.34 N m
  hard, then breaks: the hinge is deleted and the panel is free.
- **The can** is a free rigid body, 105 g with its centre of mass 1.7 cm up, so the pull on the ring can lift or tip
  it. Nothing in the simulator holds it down except the robot.

This is how the rest of the scene is simplified (also in `fidelity`): the panel is rigid and turns about a line
instead of curling as a real lid does, so the angle it is turned through stands in for how far the score is torn;
the pop and tear values are set by hand to the usual order of magnitude, not measured on this can; there is no food,
and tipping or lifting the can stands in for spilling it; all contacts use one rubber-like friction.

## Scoring

`score()` returns what a per-step tracker (`Trial`) found. The tracker reads the can's and the panel's poses, the
tab's joint and the fingertip positions; the record of a spill is cleared only in mechkit's reset hook.

- **0.0** if at any moment the can tips more than 30 degrees from upright or its base rises more than 2 cm off the
  table (food spilled, or the can came up with the lid and fell), whatever happens after. Otherwise 0.0 if the score
  never pops (the ring was never lifted 8 degrees on its rivet).
- **0.25** the score has popped, but less than half the score line is torn.
- **0.5** at least half the score line is torn: in the simulator, the panel has been turned 90 degrees or more about
  the far side of its score (the tear never heals, so this is the end state of the score line).
- **0.75** the panel has come fully away from the can, but the full-credit conditions are not all met.
- **1.0** the panel is off and lies on the table (its centre at most 12 mm above the tabletop) with its centre at
  least 9 cm from the can's axis (about 3 cm clear of the seam); the can is upright within 5 degrees with its base
  within 3 mm of the table; every fingertip point is at least 3 cm from the can and from the panel and its ring; the
  can and the panel each move less than 1 cm/s; and all of this has held together for 1 s.

Possible scores: 0, 0.25, 0.5, 0.75, 1.0. Doing nothing scores 0. Lifting the can by the ring, or peeling without
holding the can, lifts or tips it and scores 0 even if the lid then comes off.

## Real-trial preparation, reset and measurement

- **Preparation.** Use a new, unopened Fancy Feast 85 g can for each trial (about $1; any 211 x 108 easy-open
  pet-food or tuna can of the same end works). Check it with calipers: 68 mm +-2 mm across the seam, 38 mm +-2 mm tall,
  ring flat on the panel and pointing at its centre. Stand it on a dry counter mat at the marked spot, axis at x 30
  cm, y +3 cm, with the rivet on the +y side (left, seen from the robot). Put the bowl, spoon and fork on their tape
  marks.
- **Rig protection.** The panel's torn edge is sharp: put thin nitrile finger covers on the finger pads, and lay a
  sheet of kitchen paper under the can to catch gravy.
- **Measurement**, at the end of the trial, before anyone touches the scene: from the video, the largest tilt of the
  can (a protractor overlay on a side-view frame, 30 degrees) and whether its base left the mat by 2 cm (a 2 cm tape
  mark on the side camera's background); whether the score popped (a visible crack along the score at the tab's
  nose); the torn length of the score line with a flexible tape (half is about 90 mm of the 179 mm line); whether the
  panel is off; with a ruler, the distance from the can's centre to the panel's centre (9 cm) and that the panel lies
  on the table; with a level card, that the can stands upright; and from the video, that both hands were 3 cm clear
  and nothing moved for the last second.
- **Reset** in about 2 minutes: bin the opened can and its lid (sharp edge), wipe the mat, set out a new can on the
  marks.

## Distinct from

| Nearest task | What it tests | Why this one differs |
|---|---|---|
| #1873, drink can stay-on tab (not accepted) | levering a stay-on tab's tail to push a vent panel into a beverage can | that tab stays on and the panel folds into a heavy can; here the ring pops a full-aperture score, the whole lid is torn off and removed, and the light can must be held down against the pull |
| #3271, peel a cup-noodle lid back halfway (pending, another team) | peeling a heat-sealed paper lid off a foam cup by its tab and stopping at the middle | a glued paper lid peeled part way; here a riveted ring cracks a steel score, the tear is a metal score line round the whole panel, and the panel is removed and set aside |
| #1851, rotary hand can opener (accepted) | turning a cutting wheel round a can's rim with a key | a tool cuts the lid; here there is no tool, the scored end is opened by its own tab |
| #922, lever a paint tin's lid off (accepted) | prising a friction-fit lid off with a screwdriver | a press-fit lid on a heavy tin; here a scored lid torn off a light can by hand |

These come from a search of the board's full task list for can, tin, lid, tab, ring pull, peel, tuna and pet food.
The external task sets (LIBERO, RoboCasa, BEHAVIOR and others) have not been checked for this task.

## Importance and sources

Opening a ring-pull can is part of feeding a pet every day, and of opening tuna, sardines and many other foods.

- The American Pet Products Association's 2025 Dog and Cat Report: "Forty-nine million (49) US households own a
  cat, up from 40 million in 2023."
  https://americanpetproducts.org/news/the-american-pet-products-association-appa-releases-2025-dog-cat-report
- Pull tabs spread from beverage cans "to the canning of various food products, such as pet food and nuts"; food-can
  tabs are "bent slightly upwards, creating a larger surface area for easier finger access"; US can sizes such as
  211 x 108 are given in inches and sixteenths of the outside diameter and height (2 11/16 in by 1 8/16 in).
  https://en.wikipedia.org/wiki/Tin_can

## Assets and helpers

The can, its panel and its ring-pull tab are authored meshes at the dimensions above (SimReady has no easy-open
can; its `container_can_tin_open` props are 10 x 20 cm open tins). The distractors are NVIDIA SimReady props,
downloaded at first run from the pinned revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [dishware_bowl_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_bowl_a01): mass set to 0.35 kg; collides as convex hulls.
- [dishware_plastic_spoon_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_plastic_spoon_a01): mass set to 6 g.
- [dishware_plastic_fork_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_plastic_fork_a01): mass set to 6 g.

© NVIDIA, CC-BY 4.0. Nothing large is committed: the loader fetches the pinned packages. `simready.py` is adapted
from the open-source `cutters-off-pegboard` loader in this repository, and `mechkit.py` is the shared mechanism kit
copied unchanged from `heat-shrink-lug` in this repository.

## Validation status (work in progress, not ready to submit)

The build was stopped at the time limit without a working demonstration. What was established in the simulator:

- The scene, the joints written between can and panel after each reset, the pop, tear and last-segment logic and the
  scorer load and run; a trial that does nothing scores 0.
- The right gripper can pinch the real-size ring across its 20 mm width: with the fingers parted 30 mm and lowered
  until their 15 mm-wide flat tips rest on the panel, they close and stop on the ring's sides. Pinching it wide open
  straddles the whole can instead, and pinching the ring nearer its rivet with the fingers leaning puts the pads above
  the ring.
- Lifting the pinched ring has not yet popped the score: the ring turns only 3 to 4 degrees on its rivet before the
  whole can comes up with it (the spill rule then scores 0). The left hand's grip on the can body does not hold it
  down: its pads stop on the 1.5 mm overhang of the top seam rather than squeezing the body.

Open work: a pin that holds the can (for example the left hand pressing down on the seam, or a grasp low on the body
clear of the seam), then the pop, the peel arc, the set-down and its 1 s hold, two 1.0 runs and the red-team runs
(do nothing, one arm, pop only, panel dropped next to the can).
