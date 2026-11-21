# Push a loaded toddler bucket swing up to 25 degrees, catching the seat only after it turns at the back of its arc

Pushing a toddler on a swing is one of the most common things a parent does. The child sits in a full-bucket seat, and
the adult stands behind it and gives one push each time the seat comes back. The robot here stands where the parent
stands. A full-bucket toddler seat hangs still in front of it from a backyard A-frame swing stand, loaded with a 15 kg
sandbag in place of a three-year-old. The instruction gives only the goal: **"Push the toddler's swing up to 25
degrees."**

Pushing a swing is a matter of timing, not strength:

- **One shove can't do it.** The loaded seat weighs 17.2 kg and swings with a 2.13 s period. A 25-degree swing holds
  about 17 J. The push from rest in the demo, up to about 45 N for a fifth of a second, gives only 5 degrees (0.7 J).
- **Each push must catch the seat just after it turns.** Energy is added by pushing on a seat that is already moving
  away and following it out. The window is short: the seat turns at the back of its arc, and less than half a second
  later it is past the bottom, moving at 1 m/s or more.
- **The turning point moves every swing.** It moves back 5 to 10 cm each time the arc grows, so the hands must wait a
  little further back, and a little higher, each time.
- **Meeting the seat early is a strike.** A hand that waits where the seat will turn, or reaches for it before it
  has turned, meets a 17 kg seat coming at it. In a real backyard, that is the child struck by the swing.

The robot must therefore watch the swing's phase, push in the right fraction of a second, and keep its hands out of
the seat's way the rest of the time.

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: the swing stand and loaded swing, the scorer (`score`, `success`) and the scripted `demo()` |
| `mechkit.py` | The mechanism kit (an articulation written as USD text, step/reset hooks, fingertip points), unchanged |
| `simready.py` | Loads the SimReady distractors, downloading them once from Hugging Face, unchanged |
| `task.json` | The task's details for the board |

## Run a trial

From the repository root, after `source <ARENA_DIR>/yam.env`:

```bash
# a model (Dream-1 shown), 60 s at 20 Hz
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/push-toddler-swing-in-phase-to-25-degrees/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Push the toddler's swing up to 25 degrees." --scorer task_score --max-steps 1200

# the scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/push-toddler-swing-in-phase-to-25-degrees/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=640 -E cam_height=360 \
  --instruction "Push the toddler's swing up to 25 degrees." --scorer task_score --max-steps 700 --store-frames
```

Nothing is randomised, so every seed gives the same scene. The scripted demo earns full credit after 277 control steps
(13.85 s of simulated time). On a shared GPU a trial ran at about 2 to 4 steps a second, so a 1,200-step trial takes up
to about 10 minutes of wall time.

**Model run:** in its full 1,200-step run, Dream-1 scored 0: it kept both arms folded near their bases, never reached
the seat, and the swing hung still throughout.

## The scene

All positions are in the station frame: x forward from the arm bases, y to the left, heights above the floor. The
robot's table top is 76 cm up.

- **Swing stand:** a blue powder-coated steel A-frame straddling the table. Its top beam (230 x 9 x 14 cm) runs along y
  at x 82, with its underside 213 cm (7 ft) up. Four 6 cm tube legs splay to feet at x 2 and 162, y +-127, off the
  table. The pivot is 208 cm up, inside the 47 to 96 inch range the CPSC gives for swing pivots.
- **Swing:** two hangers 40 cm apart carry two green plastisol-coated chains (16 mm). Each chain splits 45 cm above the
  seat into a two-leg bridle clipped to a side wall of the seat.
- **Seat:** a green moulded plastic full-bucket toddler seat, 31 x 31 cm outside, back 31 cm high, sides 22 cm, front
  13 cm with a pommel between the legs, 1.5 kg.
  - It hangs still, its underside 82 cm up (6 cm above the table) and 126 cm below the pivot.
  - Its back faces the arms, 66.4 cm in front of their bases, centred between them.
- **Load:** a canvas bag of dry sand, 22 x 26 x 17 cm, 15.0 kg, strapped into the seat. 15 kg is the weight of a
  typical three-year-old.
- **Loaded swing:** 17.2 kg with 0.7 kg of chain. Its centre of mass is 110.5 cm below the pivot, and it swings with a
  period of 2.13 s (2.15 s measured at 20 degrees).
- **Distractors** (SimReady, loose, on the table, outside the seat's path and the arms' sweep):
  - sunglasses at (52, 47);
  - a paper coffee cup with lid at (70, -46);
  - a banana at (78, 44).

The swing hangs over the robot's own table, as it would in a real trial: the robot stands on its bench behind the
swing, where the parent stands. The camera is moved back and to the right, as in
`tasks/reset-shower-chair-short-leg/`, so that the swing's arc and both arms are in view. The station builds the
camera's stand from the same pose.

## Scoring

Scoring reads two things: the swing's angle from vertical at the hangers, and how far the hands are from the swing.

- **Free swing:** one full swing, from one rear turning point to the next, in which nothing of either arm touches the
  swing and every fingertip point stays at least 5 cm from the seat, the sandbag and the chains.
- **Height of a free swing:** the smaller of its forward turning angle and its closing rear turning angle.
- **Score, by the highest free swing so far:**

| Highest free swing | Score |
|---|---|
| 25 degrees or more | 1.0 |
| 20 to 25 degrees | 0.5 |
| 15 to 20 degrees | 0.25 |
| below 15 degrees | 0 |

Two rules score 0 for the whole trial; they are cleared only by mechkit's reset hook:

- **Strike:** any touch while the seat's back is not moving away from the arms at 5 cm/s or more. The speed is taken
  at the push point, 17 cm above the seat's underside and 1.10 m from the pivot. The one exception is a swing hanging
  still (within 2 degrees of vertical and slower than 5 cm/s), which may be pushed from rest.
- **Shove:** a hand closing on the swing faster than 0.6 m/s in the 0.05 s before it touches.

Possible scores: 0, 0.25, 0.5, 1.0. `success()` is true once a free swing reaches 25 degrees without a strike or shove,
and the trial ends 1 s later.

How the simulator judges each rule (`Trial.step` in `task.py`, once per 20 Hz control step):

- **Angle:** the hinge's joint angle. Turning points are where its rate changes sign.
- **Touch:** a fingertip point (`mechkit.fingertips`: the tip and 3 cm up each finger, at its middle and both edges)
  within 1 cm of the seat's, sandbag's or chains' collision shapes. A touch also counts when the swing's speed departs
  from free swinging by more than 3 N m. That check predicts each step's speed from the swing's own pendulum motion and
  hinge damping, and catches contact by any part of an arm, not only the fingers. In free swinging this difference
  stays under 0.2 N m. Each push in the demo gives 15 to 80 N m.
- **Moving away:** the hinge's angular rate times 1.10 m.
- **Shove:** the fall in the closest fingertip's distance to the swing over the step before the touch.

The cheap routes fail on these rules. Holding the hands still where the seat will come back is a strike. So is pulling
the seat back by hand: it moves towards the arms while touched. Pumping without watching the swing meets it on its
way back.

## The demo

`demo()` is open loop (the scripted policy can't read the scene). It plans every hand position at 20 Hz on a model of
the swing:

- **The swing:** a damped pendulum with the mass properties above. Its damping was measured from free swings in the
  simulator: 1.3% of the amplitude is lost per swing.
- **The hands:** springs that trail their commands by the arm's lag. The spring constant (300 N/m per hand), lag
  (0.09 s) and force limit (25 N per hand) were fitted to the swing's response to pushes in earlier runs.

The plan, with both hands closed and level, either side of the back's centre (y +-7 cm):

1. **Push from rest.** Both hands close in behind the still seat and push it out (about 5 degrees).
2. **Wait.** Before each rear turning point, both hands wait 3 cm behind where the model says the back will turn.
3. **Push.** Once the seat has turned, the hands follow its path out, closing to a few centimetres past its back over
   0.3 s. The command leads the arm by its lag, so the hands keep up with the seat. The hands stop before they run out
   of reach, while the seat swings on.
4. **Size the pushes.** Each push is sized so that no turning point is planned past 21.5 degrees until the last push.
   The last push is sized for 26.5 degrees, leaving room for the 1.3% loss per swing.
5. **Park.** The hands go to the sides of the stand (y +-33 cm), outside the seat's width, while the swing makes its
   free swing.

The model's pushes are only approximate, so the plan resumes from the swing's measured state (angle and rate) after
each push in a tuning run (`ANCHORS` in `task.py`). This is how any open-loop scripted demo is tuned. It is a reference
that the task can be done, not a policy.

Results:

- **Final demo:** pushes at the rear turning points of 5.0, 9.8 and 17.5 degrees (one from rest plus three). The swing
  turned at 27.0 degrees after the last push.
- **Free swing:** 26.6 degrees with both hands parked.
- **Touches:** every touch closed at 0.05 to 0.25 m/s, with no strike or shove.
- **Score:** 1.0 in each of three identical runs of the final files, and in the tuning run before them.

The demo uses 3 of the 4 pushes the swing could take before 25 degrees with these arms. Pushes from turning points
past about 23 degrees would need the hands to wait too close to the arm bases to reach.

Red team (each a variant `demo()`, run through the same harness and scorer, then deleted):

| Variant | What it does | Score |
|---|---|---|
| Do nothing | Both arms stay at their ready poses | 0 (the swing never moves) |
| Careless | Pumps the seat's back at a steady 1.4 s rhythm from where it hangs, without watching the swing | 0: a strike at 4.35 s, the pump meeting the seat on its way back |
| One arm | The demo's pushes with the left hand only | 0: with half the push the swing grew less than planned, so the hand waited too far back and closed on the seat too fast, a shove at 8.85 s |
| Cheapest shortcut | Pushes from rest, then hooks both hands over the top of the seat's back at its first rear turning point, meaning to draw it back slowly and let go | 0: the seat met the hands while still swinging back (flagged in the 0.05 s before the touch), and the hook did not hold |

## Distinct from

| Nearest task | What it tests | How this differs |
|---|---|---|
| PR #2190 (accepted): Rock a pram smoothly | Continuous gentle rocking of a wheeled pram on the floor | Here the robot adds energy to a free pendulum in discrete pushes timed to its phase. The danger is the seat coming back, not jerkiness. |
| PR #949 (rejected): Regulate a pendulum by moving its bob | Changes a pendulum's length to set its period | Never pumps the pendulum or touches it while it swings |
| PR #1746 (rejected): Stop a desk pendulum on its third swing | Removes a small pendulum's energy at a counted swing | The opposite job (adding energy), at full scale with a 17 kg load, and with a strike hazard |

We found no swing-pushing or pendulum-pumping task among the board's accepted and pending tasks (`swing`, `pendulum`,
`push`, `rock`, `pump` and `toddler` searched in every PR title). We found none in LIBERO, RoboCasa, SimplerEnv,
BEHAVIOR or RoboDojo either: their tasks are quasi-static manipulation of objects resting on surfaces.

## Importance and sources

- U.S. Consumer Product Safety Commission, *Public Playground Safety Handbook*, Publication 325,
  https://www.cpsc.gov/s3fs-public/325.pdf . It is the source for these points:
  - Full-bucket seat swings are "intended for children under 4 years of age to use with adult assistance". Pushing
    one is the everyday adult job this task tests.
  - Swing pivot points "should be more than 47 inches but no more than 96 inches above the protective surfacing".
    This task's pivot is 82 inches up.
  - The handbook recommends limits on swings per bay "to minimize the likelihood of children being struck by a moving
    swing". It recommends lightweight rubber or plastic seats "to help reduce the severity of impact injuries". The
    strike rule scores exactly that event.

The seat follows the common moulded full-bucket toddler seat sold for backyard swing sets by Swing-N-Slide and Gorilla
Playsets (about 12 inches each way, rated to about 50 lb). Their retail pages could not be fetched to confirm while
this was written, so no product link is given here; the dimensions used are in the setup above.

## Real trials

**Rig:**

- A toddler full-bucket swing on a 7 ft A-frame swing stand, its chains adjusted so the seat's underside hangs 82 cm
  above the floor.
- The robot's bench behind it, its top at 76 cm, with the arm bases 66.4 cm behind the seat's back.
- A 15 kg bag of dry sand (a reusable stand-in) strapped into the seat.
- A safety tether from the seat's front to a floor stake, set to go taut at 35 degrees back, so an over-pushed seat
  cannot reach the arm bases. At 25 degrees the bottom of the seat's back comes to 15 cm from them (11 cm at the
  demo's 27 degrees).
- Pad the seat's back edge with a strip of foam pipe insulation to protect the grippers.

**Measuring:**

- Film from the side at 60 fps with a protractor card fixed behind a hanger.
- Alternatively, put floor tape under the seat's push point at its 15, 20 and 25 degree positions: 28, 36 and 45 cm
  from where it hangs.
- A free swing's height is read from the frames at its turning points.
- The seat's speed at a touch is read from its position over the 12 frames (0.2 s) around the touch. A strike is a
  touch while it moves away by less than 1 cm over those frames.
- A shove is a hand closing more than 1 cm per frame in the frames before the touch.

**Reset (about a minute):**

1. Stop the swing by hand, let it hang still at its tape mark, and check the seat is level.
2. Check the sandbag is strapped in against the back.
3. Put the distractors back on their marks.

Nothing is consumed.

## Assets and code

- **Distractors:** from the Hugging Face dataset
  [`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision
  `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:
  - [`accessory_eyeglasses_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/accessory_eyeglasses_a01) (sunglasses, 30 g)
  - [`food_beverage_coffeeCup_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/food_beverage_coffeeCup_a01) (coffee cup, 250 g)
  - [`food_banana_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/food_banana_a01) (banana, 150 g)

  They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). The only change at load time
  is the masses, which are set as listed. The files download at first run and are not committed.
- **The swing:** SimReady has no swing, seat or swing stand, so the stand, chains, seat and sandbag are custom
  primitives (`mechkit.py` meshes and box colliders).
- **Kit code:** `mechkit.py` and `simready.py` are taken unchanged from other teams' open-source tasks in this
  repository: `mechkit.py` from `tasks/heat-shrink-lug/`.
