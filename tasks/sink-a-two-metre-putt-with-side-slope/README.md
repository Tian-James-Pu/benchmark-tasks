# Hole a 2 m sidehill putt in one stroke

A golf ball sits on an indoor putting green, 2.00 m straight down the green from a regulation hole. The green falls
2 degrees across the line of the putt, towards the robot's right. A right-handed 35-inch blade putter lies face down
beside the ball. The instruction is only: **Hole the golf ball with one stroke of the putter.**

Nothing in the instruction mentions the slope. Struck straight at the hole, the ball curves away below it. To hole the
putt the robot has to pick up the putter, stand it on its sole behind the ball with the face square to a start line
well above the hole, and swing it so that the ball leaves at the right pace: about 1.6 m/s off the face, dying to
well under 1.3 m/s at the cup. Too firm and the ball lips out or runs past; too soft and it falls away short and below.
One stroke only: tapping the ball along, or finishing it with the hand, scores 0.

## Run and score

From the repository root, in the Python environment that has `yam_arena` and `inspect-robots`:

```bash
# scripted demo (fixed plan, played open loop from the published layout)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/sink-a-two-metre-putt-with-side-slope/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "Hole the golf ball with one stroke of the putter." \
  --scorer task_score --max-steps 760 --store-frames --log-dir ~/yam-eval/putt-demo

# model run
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/sink-a-two-metre-putt-with-side-slope/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hole the golf ball with one stroke of the putter." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/putt-model
```

The run prints `task_score`. Nothing is randomised: every seed gives the same layout, and `object_jitter=0` keeps it
fixed. A demo trial takes about 2 minutes of wall time on one GPU once shaders are compiled.

## Scene

| Object | What it is | Size, material, mass | Where (x forward, y left, from the midpoint between the arm bases) |
|---|---|---|---|
| Putting green | platform: two 38 x 89 mm pine rails on edge, end rails, 18 mm plywood, 10 mm putting turf | 3.05 x 0.61 m, 117 mm to the turf at the low rail | x 0.08 to 3.13 m, y 0.05 to 0.66 m |
| Shim | pine strip under the high rail | 21 mm (0.61 m x tan 2 degrees) | under the rail at y = 0.66 m |
| Hole and cup | 108 mm hole through the turf, white plastic cup | cup 102 mm deep below the turf | centre x = 2.44 m, y = 0.35 m |
| Second table | same height as the robot table | 2.30 x 1.20 m top | centre x = 2.07 m, y = 0.35 m |
| Golf ball | NVIDIA SimReady `misc_golfBall_a01` | 42.67 mm, 45.93 g | x = 0.44 m, y = 0.35 m, on the turf |
| Putter | right-handed blade, 35 in, 70-degree lie | head 105 x 25 x 24 mm steel 350 g; shaft 10 mm steel 120 g; grip 27 mm rubber, 267 mm, 65 g | face down on the turf, shaft along x, head towards the arms; shaft point level with its centre of mass (14 cm up from the heel) at x = 0.34, y = 0.205 m |
| Mug (distractor) | SimReady `dishware_mug_a01` | ceramic, 0.32 kg | (0.62, -0.30) m on the robot table |
| Paper cup (distractor) | SimReady `dishware_cup_paper_a01` | 8 x 11 cm, 12 g | (0.78, -0.12) m on the robot table |
| Golf towel (distractor) | SimReady `consumable_microfiber_towel_folded_small_b01` | folded microfibre 20 x 21 cm, 60 g | (0.45, -0.52) m on the robot table |

The green's low long rail stands on the robot table at y = 0.05 m; the far rail stands on the 21 mm shim, so the turf
falls 2.0 degrees towards -y, across the putt. The far 2.2 m of the green rests on the second table. The distractors
are off the green, out of the putt's path and away from the left arm's work; the scorer ignores them.

The camera is moved for this task (the accepted shower-chair task does the same): it stands beside the green at
(1.42, -1.45, 2.00) m and looks at (1.38, 0.40, 0.84) m, so that the arms, the ball, the whole 2 m line and the hole
are in one view. The model and the demo get the same view.

## How the simulator judges each rule

All of it is in `task.py`: `_roll` (the turf) and `Trial.step` (the record) run after every control step, and
`score()`/`success()` only read the record.

- **The ball and the turf.** The ball collides as an exact 42.67 mm sphere with a solid ball's inertia. Real turf
  slows a rolling ball at a nearly constant rate; here a force of 0.0785 x the ball's weight opposes its rolling
  whenever it rolls on the turf, which gives a rolling ball the 0.55 m/s^2 deceleration of a green that reads 10 ft on
  a stimpmeter (a ball released at 6.00 ft/s rolls 10 ft). Below 4 cm/s the same turf holds the ball, as real turf holds
  a ball on a 2-degree slope (the force needed, 0.035 x its weight, is under that limit). The 2-degree slope itself is
  real geometry: gravity curves the ball. Over the hole and in the cup no turf force acts.
- **The hole.** The hole and cup are geometry (24 convex wedges round the hole, a 24-piece cup wall and a bottom).
  Whether a ball drops, lips out or skips over is decided by contact and gravity alone.
- **Holed.** The ball's centre is within 54 mm of the hole's axis and more than 2 mm below the turf's plane, so the whole
  ball is below the surface; it is still (under 5 mm/s) for 1 s, and every finger point is at least 5 cm from it.
- **Partial credit.** The ball at rest (under 5 mm/s) on the turf, within 30 cm (0.5) or 60 cm (0.25) of the hole's
  centre, measured in the turf's plane.
- **Fouls, recorded for the rest of the trial.** A finger point within 12 mm of the ball's surface (the hand touched the
  ball); or, once the ball is more than 30 cm from its spot, the ball's surface within 4 mm of the putter's head, shaft
  or grip (a second stroke). The record is cleared only in the reset hook, never because the ball comes back.
- The ball's speed is its centre's movement over each 50 ms step.

## The demo

`demo()` is a fixed plan for the left arm, played open loop by `yam_scripted` from the published layout; it reads no
simulator state. It pinches the shaft from above where it is level with the putter's centre of mass, lifts it, tips the
head down in two stages so that the putter turns upright in the hand (fingers forward, closing across the shaft), soles
it 4 mm above the turf with the face square to a start line 8 degrees left of the hole, takes a 15 cm backswing and
swings through in one smooth 0.45 s stroke that is fastest at the ball. Results are under "Validation".

## Real trial

**Build and prepare.** Rip a 2440 x 1220 mm sheet of 18 mm plywood in half and join two 1525 mm lengths end to end on
four 38 x 89 mm rails on edge (a 3.05 x 0.61 m deck); glue 10 mm putting turf on top. Cut the hole with a 108 mm
(4-1/4 in) hole saw 2.69 m from the near end, centred across the width, and fit a regulation plastic practice cup so
that its rim is 25 mm below the turf and its bottom 102 mm below. Set the deck on the robot table and a second table of
the same height, its near end 8 cm in front of the arm bases. Put a 21 mm strip under the far rail along its whole
length and check 2.0 degrees across the turf with a digital level at three places. Measure green speed with a
stimpmeter or a 20-degree ramp: a ball released at 1.83 m/s should roll 3.0 m (10 ft) on the level part of a test
strip; adjust the turf (or choose a turf rated 10 ft) until it does. Mark the ball's spot with a pencil dot 2.00 m
from the hole's centre along the deck, and tape the putter's outline face down beside it.

**Reset (about 1 minute).** Fish the ball out of the cup, set it on its dot, lay the putter face down on its tape
outline, and wipe the turf if anything was dragged across it. Nothing is consumed.

**Measure.** Video the trial from beside the green. A holed ball is visible in the cup; a ball short or long is
measured with a tape from the hole's centre to the ball's centre once it has stopped. Fouls are read from the video:
any touch of the ball by a gripper, or any touch by the putter after the ball has rolled 30 cm. The robot's pads touch
only the putter's shaft; nothing on the green is sharp, hot or wet.

## Distinct from

| Nearest task | What it tests | Why this is different |
|---|---|---|
| #2589 pool-bridge-shot (pending) | a cue slid through a bridge hand into a struck ball on a level table, aimed through a contact point | a pendulum stroke of a 0.9 m club whose head must stay level just above the turf, on a slope that bends the ball; the pace has to die at the hole (a capture window), where a pocket takes a firm ball |
| #3095 air-hockey return (revising) | striking a moving, near-frictionless puck into a goal | the ball is at rest and the turf slows it: distance control and break reading, not interception |
| #1687 tabletop curling (rejected) | a stone slid and released by hand | a struck ball, a held tool, and a slope |
| #2583 table-football pass (accepted) | rods turned by their handles | free tool use: the putter is picked up, stood on its sole and swung |
| #3419 golf ball on a tee (rejected) | balancing a ball on a tee | shares only the ball |

No putting task appears among the board's titles (searched for golf, putt, putter, cue, pool, curling, hockey, ball);
LIBERO, RoboCasa, SimplerEnv and BEHAVIOR have no golf putt. The dynamic-game suites we know of (the Robot Air Hockey
Challenge, Fancy Gym's beer pong, embodied-olympics-fun's paddle bounce) test interception, throwing and bouncing; we
could not read embodied-olympics-fun's full list of 79 tasks, so a putt there cannot be ruled out.

## Importance and sources

- Golf is played by tens of millions of people: the National Golf Foundation counts more than 48 million Americans
  playing golf in 2025, on courses and off them
  ([SGB summary of the NGF figures](https://sgbonline.com/exec-golf-participation-achieves-another-record-high-in-2025/)).
  Every round, and every mini-golf hole, ends with putts.
- The hole is 4.25 in (108 mm) across, and *break* is "the tendency of a putted ball to roll left or right of a straight
  line" ([glossary of golf](https://en.wikipedia.org/wiki/Glossary_of_golf)). The Rules of Golf also require the hole to
  be at least 4 in (101.6 mm) deep.
- A conforming ball is at least 1.680 in (42.67 mm) across and at most 1.620 oz (45.93 g)
  ([golf ball](https://en.wikipedia.org/wiki/Golf_ball)).
- Green speed: a stimpmeter releases the ball at 6.00 ft/s, and the distance it rolls in feet is the green's speed; the
  USGA's medium speed for club greens is 10 ft ([stimpmeter](https://en.wikipedia.org/wiki/Stimpmeter)). That sets the
  turf model here.
- The putter is a generic right-handed 35-inch blade with a 70-degree lie and a 350 g head, the common specification of
  entry-level blade putters (about $30 to $60). Any such putter can be used; the simulated one is drawn to those
  numbers.

## Assets and helpers

The golf ball and the three distractors are NVIDIA SimReady props, downloaded at first run from the pinned revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd` of `nvidia/simready-assets` (© NVIDIA, CC-BY 4.0); nothing is bundled.

- [misc_golfBall_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/misc_golfBall_a01):
  mass set to 45.93 g; its SDF mesh collider is switched off and an exact 21.335 mm-radius sphere collides instead, with
  a solid sphere's inertia and a physics material (friction 0.4/0.35, restitution 0.75).
- [dishware_mug_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01): mass set to 0.32 kg.
- [dishware_cup_paper_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_cup_paper_a01): mass set to 12 g.
- [consumable_microfiber_towel_folded_small_b01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/consumable_microfiber_towel_folded_small_b01): mass set to 60 g.

The green, cup, second table and putter are authored primitives (SimReady has no putter or putting green).
`simready.py` is the open-source loader from `tasks/cutters-off-pegboard/`, unchanged; `task.py` wraps its `_prepare`
for the ball only. `mechkit.py` is the open-source mechanism kit from `tasks/heat-shrink-lug/`, unchanged; it supplies
the fixed and free parts, the per-step and reset hooks, the finger points and the applied turf force.

## Validation

NOT READY: build stopped at the 4-hour limit; do not submit. Status of the internal build notes (remove this section before submitting):

- Works: scene, green slope, hole geometry, turf hold (ball stays within 0.5 mm of its spot), pick from face-down, turning the putter upright in the hand, address, and a backswing and stroke that strike the ball once without a foul.
- Missing: the stroke reaches only about 0.87 m/s of head speed even when commanded to 1.3 m/s (the arm's tracking caps it). The ball leaves at about 0.75 m/s and 21 degrees, not the planned 8 degrees, and stops about 0.65 m from its spot. Ball speed is about equal to head speed, so the 0.75 restitution does not seem to apply: check the material binding, the combine mode and PhysX's bounce threshold.
- Not run: demo 1.0 (twice), the red-team variants, and removing the `PUTT_DEBUG` logging and the `PUTT_*` environment overrides in `task.py`.
