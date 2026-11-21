# Roll five dice from a felt-lined dice cup into a folding dice tray, then stand the cup back on the table

Every turn of Yahtzee starts the same way: the player shakes five dice in the cup and rolls them out onto the table or
into a dice tray. Here a felt-lined dice cup holding five dice stands on the table beside a small folding dice tray. The
instruction asks for the dice to be rolled into the tray and the cup put back down. It does not say how.

The hand skill is to take the round, light cup by its side, keep the fingers off the loose dice, and roll them into a
small tray with 3.2 cm walls. The five dice have different colors so each one can be identified in high-speed video.
For each die, the scorer checks that its upper-facing number changes after it leaves the cup, then checks that it
settles flat inside the tray. A low pour that lets the dice slide out without turning does not count as a roll. The
cup must then be set back down upright outside the tray and released.

The task is [`task.py`](task.py), a `yam_arena` task file containing the scene, the scorer and the scripted demo.
[`mechkit.py`](mechkit.py) builds the dice, cup and tray, and [`simready.py`](simready.py) loads the distractor props.

## How a roll is distinguished from a pour

The scorer uses an observable rotation check, not drop height. For each color-coded die, it records the upper-facing
number when the die's centre first leaves the cup. That die counts only if a different number becomes uppermost at
least once after exit, before it settles. A 240 fps oblique video frames the cup mouth and the full tray, so the color
identifies each die and its pips show the face change. A low pour that lets a die slide out without turning fails this
check. The same per-die rule is used in simulation and in a real trial.

## Importance and sources

Category: **Societal relatability**. This task models the opening roll of a familiar tabletop game, using the same
basic equipment sold for play: five dice and a shaker cup. Hasbro's current product page says each turn involves
rolling dice and lists five dice and a shaker in the game; the published rules say to place the dice in the cup, hold
it up, and roll them out ([Hasbro, Yahtzee product and instructions](https://instructions.hasbro.com/en-us/instruction/yahtzee),
[Hasbro, rules PDF](https://www.hasbro.com/common/instruct/yahtzee.pdf)). These sources establish the real activity
and equipment. They do not establish how many people play it, so this task makes no prevalence claim.

The benchmark isolates the physical roll and containment: invert the cup over a small target from enough height for
the dice to tumble, keep all five within the low walls, and set the cup back down. It is a leisure activity rather
than an occupational task; its relevance is the recognizable everyday game action, not a claim about professional
work or serious harm.

**The products.** The real objects below are what the scene is modelled on. They were sized from the listings
given; we have not measured them ourselves.

- Dice cup: a felt-lined leather dice cup sold with five dice, for example the Brybelly "Professional Dice Cup with
  Five Dice" (seller SKU GDIC-303), about 5 to 12 USD. One retailer describes it as having a "Dark, stitched leather
  exterior" and a "Red, soft plush interior", with "Felt padding [that] dampens the sound of rolling dice"
  ([North 40 listing](https://north40.com/brybelly-professional-dice-cup-with-five-dice), fetched 21 November 2026, 4.99
  USD). That page gives no size. We take 3 3/4 in (95 mm) tall and 3 1/4 in (83 mm) across from these sources:
  - the same cup's [Poker Chip Lounge listing](https://pokerchiplounge.com/products/professional-dice-cup-with-five-dice)
    (11.36 USD);
  - an [eBay listing](https://www.ebay.de/itm/177139967168) titled "Bicast ... Dice Cup w/ Red Felt Lining - 3 3/4"
    tall with 3 1/4" with 5 dice".

  Both of those sites refused direct fetches, so the two figures come from their text as shown in a web search index
  on 21 November 2026. Measure the cup before a trial. The 80 g mass is our estimate for a leather cup on a plastic
  shell. It is not a listed value.
- Dice: five opaque, 16 mm (5/8 in), six-sided acrylic dice with white pips and rounded corners, one each green,
  pink, black, blue and red. The colors make the individual dice trackable. A 100-count assorted pack is sold in
  these colors by Hobby Monsters ([product listing](https://www.hobbymonsters.com/100-count-16mm-assorted-dice-10-colors/));
  select one of each named color. Each die is 4.5 g in the model; this is an estimate from acrylic density
  (1.18 g/cm3 x 4.1 cm3, less the pips).
- Dice tray: a folding PU-leather and velvet dice tray that snaps up to "9" x 6" x 1.25" when in action mode;
  8.5" x 11" when prone", sold by GM Dice ([listing](https://www.gmdice.com/products/folding-dice-tray), read
  through a search index on 21 November 2026). Assembled, it is 22.9 x 15.2 cm outside with 3.2 cm walls. The 6 mm
  wall and 4 mm floor thicknesses are our estimates.

## Distinct from

| Nearest | What it does | Why this is a different ability |
|---|---|---|
| Controlled pouring of small items into a narrow target | A slow pour transfers contents without requiring a throw | A die scores only if its upper-facing number changes after it leaves the cup, so a slide-out with no post-exit turn does not count. All five must then land flat inside the 3 cm-walled tray, and the cup must be put back upright. |
| Pouring and sorting mixed small items | Pouring contents out and selecting them by type | No sorting or recognition is required here. The scored skill is the dynamic inversion and keeping all five bouncing dice inside the small target. |
| Tipping loose items out of a container | Contents fall out of an open container | Here each die must visibly change its upper-facing number after it exits the cup, then land flat inside a small target; the cup must be set back down upright outside the tray. |

External sets: LIBERO, RoboCasa, BEHAVIOR-1K, SimplerEnv and RoboARC have pouring and pick-and-place of containers,
but as far as we found, none has rolling dice from a cup into a tray. This is a manual check of their task lists. We
did not search them exhaustively.

## The scene

Positions are in cm from the midpoint between the two arm bases (x forward, y to the robot's left), measured on the
table top.

1. **Dice tray** (fixed: it lies flat on the table, and in a real trial it is taped down). The PU-leather walls are
   0.6 cm thick and 3.2 cm tall. The velvet floor is 0.4 cm thick. Outside it measures 15.2 cm along x and 22.9 cm
   along y, so the inside is 14.0 x 21.7 cm. It is centred at x 28.5, y 6.5.
2. **Dice cup**, standing upright, centred at x 30.0, y -17.0. It is leather over a plastic shell with a red felt
   lining: 8.3 cm across, 9.5 cm tall, a 0.3 cm wall and a 0.4 cm base, 80 g.
3. **Five dice** in one layer on the cup's floor. Four sit in a square at x +-0.95, y +-0.95 cm from the cup's centre
   (3 mm apart) and the fifth at x -2.8, y 0. Their faces are along x and y. Tops: 1 on the front-left die (+x, +y),
   3 back-left, 5 front-right, 6 back-right, and 4 on the fifth. They are 16 mm acrylic with 1.5 mm chamfers, 4.5 g each.
4. **Distractors**, which the scorer ignores:
   - a score pad (paper notebook, 22.9 x 17.0 x 1.0 cm, 120 g) at x 17, y 30, long side along x;
   - a ceramic mug (11.4 x 9.0 x 8.0 cm, 300 g) at x 43, y 25;
   - a glass tumbler (8.0 cm across, 8.5 cm tall, 250 g) at x 45, y -17.

Both arms start at the ready pose with the grippers open. The layout is the same in every trial
(`-E object_jitter=0`).

## Scoring

Every control step (20 Hz), `Trial.step` in `task.py` reads the pose of the cup and of each die, and the finger
sample points of both grippers (mechkit's `fingertips`, spread over each finger's face as a 5 x 4 grid).

- **Left the cup and turned.** A die leaves the cup at the first step its centre is outside the cup's inside: above
  the rim plane, below the base, or outside the wall. The scorer records the number on the upper-facing die face at
  that step. The die is marked as turned if a different number becomes uppermost at any later step. This record is
  kept for the rest of the trial. In the real-trial video, the five colors identify each die and the white pips show
  its upper-facing number.
- **Touched.** A die is touched if any finger point ever comes within 6 mm of its box (straight-line distance). The points sit 4.5 mm
  behind the pad surface, so this is about 1.5 mm of real clearance. A die resting against the cup's inner wall
  while a finger grips the outside is 7.5 mm away, so it is not counted as touched. This record is also kept for the
  rest of the trial.
- **A die counts** when all of these hold:
  - it left the cup and its upper-facing number changed at least once afterwards;
  - it was never touched;
  - now it **lies flat on the tray floor**: its lowest point is within 3 mm of the floor, one face is within 8 deg of
    level (so it is not cocked against a wall or another die), and its centre is inside the walls;
  - it is still: under 1 cm/s and 0.3 rad/s.
- **The cup is set down** when it stands upright (axis within 10 deg of vertical) with its base within 3 mm of the
  table, its footprint is wholly outside the tray's outline, every finger point is 3 cm or more from it, and it is
  still.
- **Score.** 0 dice count: 0. 1 or 2: 0.25. 3 or 4: 0.5. All 5: 0.75. All 5 and the cup set down: 1.0 (success,
  which ends the trial once it has held for 3 s).
- Possible scores: 0, 0.25, 0.5, 0.75, 1.0. Doing nothing scores 0.

The per-die records (exit face, post-exit turn, touched) are cleared only in mechkit's `on_reset` hook, never because
the scene looks like its start again. The score is recomputed every step from the current state plus those records, so
a die knocked out of the tray later stops counting.

`--scorer task_score` reports 1.0 if success held for 3 s, and otherwise the score at the last step.

## A real trial: setup, measurement, reset

You need the dice cup, one each of the five listed colored dice, the folding tray snapped up and taped to the table,
tape marks for the tray's corners, a taped circle for the cup's base, one phone recording the cup and tray at 240 fps
from an oblique angle, and an overhead photo at the end.

- **(a) Post-exit turn.** In the 240 fps video, follow each die by its color. When its centre first clears the cup,
  note the upper-facing number; before it stops, a different number must become uppermost. If a face is unreadable
  at exit, use the first clear frame after it emerges. A die with no visible post-exit face change does not count.
- **(b) Untouched.** In the video, no gripper finger contacts any die. The fingers stay outside the cup, and a finger
  that reaches into the mouth or pushes a die is plainly visible.
- **(c) End state, from the overhead photo** taken once nothing is moving:
  - each die lies on one face inside the tray. A die leaning on a wall or another die is cocked and does not count,
    which the usual "would a die set on top slide off" test settles.
  - the cup stands upright on the table with no part of its base on or over the tray.
  - the gripper is at least 3 cm from the cup.
- **Reset** (about 1 minute):
  - put the five dice back in the cup in the starting layout: the square of four, then the fifth beside it, tops 1,
    3, 5, 6, 4;
  - stand the cup on its circle;
  - the tray stays taped down.

The rig is safe to run. The dice and cup are light, and nothing is sharp, hot or breakable except the glass tumbler.
It stands outside the arm's path, and a plastic tumbler can replace it.

## Run and score a trial

Requirements:

- [yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) on Isaac Lab-Arena with Isaac Sim, Linux and
  an NVIDIA RTX GPU (the program's Brev launchable installs everything, including the Dream-1 policy);
- network access the first time, to download the three SimReady distractor props into `~/.cache/simready`.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/roll-five-dice-from-cup/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Roll the five dice from the cup into the dice tray, then put the cup back down on the table." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/dice-dream
```

The scripted demo:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/roll-five-dice-from-cup/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=640 -E cam_height=360 \
  --instruction "Roll the five dice from the cup into the dice tray, then put the cup back down on the table." \
  --scorer task_score --max-steps 500 --store-frames --log-dir ~/yam-eval/dice-demo
```

Videos: `for log in ~/yam-eval/dice-demo/*.json; do inspect-robots video "$log"; done`.

`DICE_DEBUG=1` prints the scorer state every 5 steps: the number of dice that count, whether the cup is set down,
the cup height and tilt, and for each die its upper-facing number at exit, `R` if a different face later became uppermost,
`T` if a finger came within the touch threshold, and `G` if it currently counts. It also prints a line whenever a die is first touched.

### The scripted demo

The demo is open loop from the known layout and uses only the right arm; the left arm holds its ready pose.

1. The gripper comes from behind the cup, pointing forward and 10 deg down. It closes across the cup 6 cm up its side,
   with the TCP 12 mm behind the cup's axis.
2. It lifts the cup 9 cm and rattles it with four quick 1.5 cm side moves.
3. It carries the cup over the near half of the tray, with the grip 20 cm above the table, and winds the wrist back
   25 deg.
4. It rolls the wrist 175 deg about its pointing direction in 0.8 s, sweeping 6 cm along the tray. The mouth turns down
   towards the tray, and the dice tumble toward the tray.
5. It rolls the cup back upright, carries it back, sets it down on its taped circle, opens and backs away.

Earlier demo runs reached 1.0 under the drop-height scorer. They are not evidence for this revised face-change scorer;
rerun the demo and controls before publishing current scores.

Dream-1, run once for 1,200 steps with the instruction above, scored **0.0**. Its right gripper reached the cup and
moved around it, but it never lifted or turned the cup, and no die left the cup.

### Red-team runs

The earlier control results used a drop-height scorer and do not apply to the current post-exit face-change criterion. Rerun all controls against this scorer before publishing red-team scores.

A variant that pressed a die inside the cup with a top-down gripper could not reach it: the gripper met the cup's rim
first and only tipped the cup a few degrees. With the cup this size, reaching in to place dice by hand is impractical,
and any finger contact with a die is caught anyway (the rake run).

These variant files were run and then removed, as was the tuning code.

## What is modelled and what is not

- Rigid bodies throughout.
  - Each die is a 16 mm cube with 1.5 mm chamfered edges and corners (one convex collider), with the pips drawn on.
  - The cup's wall is 24 flat panels on a disc base, with the felt drawn on.
  - The tray's floor and walls are fixed boxes. The velvet and padding do not deform, and the tray does not flex.
- Friction (static / dynamic, estimates): dice 0.40 / 0.35, tray 0.50 / 0.45, cup 0.80 / 0.70 (leather outside and
  felt inside share one value). PhysX averages the two touching materials.
- Restitution (estimates): dice 0.35, tray 0.20, cup 0.10. mechkit writes 0 for every part; `task.py` sets these
  values in each part's physics material as the file is written (no change to `mechkit.py`).
- The cup's 80 g mass, the dice's 4.5 g and the tray's wall and floor thicknesses are estimates. The cup's and tray's
  outside sizes are from the listings above.
- There are no force or contact sensors. Touch is judged from finger geometry, as described under Scoring.

## Runtime per trial

The scripted demo runs 432 control steps (21.6 s of simulated time). It reaches full score at step 372 and ends
after the 3 s hold. Each run took about 2.5 to 3 minutes of wall time at 640 x 360, frames stored and videos rendered,
on a shared NVIDIA RTX-class GPU with other simulation jobs. A Dream-1 trial of 1,200 steps at 320 x 176 takes several
minutes more. Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed. With `-E object_jitter=0` the layout is fixed, and the scripted demo is open loop from it.

## Assets and licences

- `mechkit.py` is copied unchanged from `tasks/heat-shrink-lug/mechkit.py` in this repository (another team's
  open-source task). `simready.py` is copied from `tasks/cutters-off-pegboard/simready.py` (likewise).
- The dice, cup and tray are built in `task.py` from the dimensions above. SimReady has no dice, dice cup or dice tray,
  so these are custom meshes and colliders. No third-party geometry is used for them.
- Distractors, from NVIDIA's SimReady library (`nvidia/simready-assets` on Hugging Face, revision
  `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`), © NVIDIA, CC-BY 4.0. They are downloaded at first run and nothing is
  committed here. At load time each is given the mass stated above and is a single loose rigid body.
  - [`misc_notebook_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/misc_notebook_a01)
    (score pad);
  - [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01)
    (mug);
  - [`dishware_glass_tumbler_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_glass_tumbler_a01)
    (glass).
