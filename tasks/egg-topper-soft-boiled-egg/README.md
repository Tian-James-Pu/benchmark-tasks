# Top a soft-boiled egg with a drop-weight egg topper

**Instruction:** *Take the top off the soft-boiled egg with the egg topper, then stand the topper back up on the table.*

A soft-boiled large egg stands pointed end up in a porcelain egg cup at a breakfast place setting. Beside it stands a drop-weight
egg topper: a stainless bell whose open bottom is a 34 mm ring of teeth, a silicone collar to hold it by, a rod, and a 45 g steel
weight that slides 10 cm up the rod. A side plate, a teaspoon, a tea cup and a salt shaker stand around it.

The topper works the way every drop-weight topper does: the bell is set on the egg's crown and held there, the weight is lifted to
the top of the rod and let fall onto the collar, and the blow drives the teeth through the shell in a clean circle. Lifting the
topper then brings the cut top away in the bell. The robot has to pick the topper up by its collar, lower the bell onto the crown
centred on the egg's axis, keep it there and upright with one hand while the other lifts the weight the full height and lets go; then
lift the topper off the egg, the top in its bell, and stand it back up on the table.

## The knack

- **Two hands in different roles at once.** One gripper holds the bell on the crown, centred within 3 mm and upright within
  8 degrees; the other lifts the weight up the rod and releases it. Lifting the weight drags on the topper, so the holding hand has
  to keep it where it is while the other works.
- **A position window.** The ring of teeth is 34 mm across on a 43 mm egg: centred, it cuts round the crown; off-centre or tilted, the
  blow lands on one side and cracks the shell below the ring (the egg is spoiled, score 0).
- **A full drop, released, not lowered.** A drop counts only from at least 8 cm (the weight's travel is 10 cm), falling freely. A
  short drop does nothing (it can be repeated); lowering the weight by hand does nothing.
- **Several dependent stages.** Topper on, weight dropped, topper lifted off with the top, stood back up, all without knocking the egg
  out of its cup.

## Run and score

The scripted demonstration is closed loop (it reads the egg's, the topper's and the cap's poses, the weight's position on its rod
and its own arms, never the scorer), so it runs through `demokit.py` from this folder, in the simulator's Python environment:

```bash
cd tasks/egg-topper-soft-boiled-egg
python demokit.py task.py ~/yam-eval/egg-topper-soft-boiled-egg/demo --mode full        # writes full/demo.mp4, setup.png, result.json (task_score)
python demokit.py task.py ~/yam-eval/egg-topper-soft-boiled-egg/redteam --fast --mode none,short,offcentre,onearm,grabcap,lowered
```

`--fast` runs physics on the CPU without cameras (for scorer checks); the demo scores reported below come from recorded GPU runs.
A model trial uses the ordinary harness:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/egg-topper-soft-boiled-egg/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the top off the soft-boiled egg with the egg topper, then stand the topper back up on the table." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/egg-topper-soft-boiled-egg/model
```

The layout is fixed (`object_jitter=0`, nothing depends on the seed). The run limit is 2,400 control steps (120 s at 20 Hz); the
demonstration earns full credit (the 1 s hold complete) at step 843 (42 s), about a third of the limit. A recorded GPU trial took
11 to 16 minutes of wall time on a GPU shared with other trials (about 1 step/s); a `--fast` CPU run takes about 30 s.

## Scoring

`task.py`'s `Breakfast` tracker runs after every control step (mechkit's `on_step` hook) and keeps its own record; `score()` and
`success()` read it. Harm is recorded across steps and cleared only by mechkit's `on_reset` hook. `task.json`'s `scoring` says the same:

- **A counted drop.** The weight falls freely (from its highest point to its rest on the collar in at most 0.4 s) from at least
  8 cm above its rest, while the bell sits on the egg: the ring's plane within 6 mm of where it rests on the shell, its centre within
  12 mm of the egg's axis. Lowered by hand, it does not count (too slow); dropped from under 8 cm, it does nothing and may be repeated.
- **Clean ring or cracked side.** At a counted drop, the ring's centre within 3 mm of the egg's axis and the topper's axis within
  8 degrees of the egg's cut the shell in a clean ring: from then on the cut top rides in the bell. Otherwise the blow cracks the egg's side
  (recorded; score 0 for the trial).
- **Egg out of its cup** (recorded; score 0): the egg's centre more than 12 mm off the cup's axis, the egg tilted over 25 degrees or
  the cup over 20 degrees.
- **Possible scores:**
  - **0**: the side was cracked, the egg ever left its cup, or the shell was never cut.
  - **0.25**: shell cut.
  - **0.5**: shell cut, and the topper and the cut top off the egg (the ring over 5 cm from its seat, the top over 3 cm from its seat).
  - **0.75**: all of that, with the topper standing on the table (within 15 degrees of upright, the bell within 1.5 cm of the table top).
  - **1.0**: all of that, with the egg in its cup, the egg and the topper moving under 1 cm/s, and every fingertip at least 3 cm from
    the egg, the topper and the cut top, all held for 1 s. Once reached it stays unless a 0 condition follows.

Until the shell is cut, the top is part of the egg: the hook writes its pose and velocity onto the lower shell's every step, so it
cannot be pulled, knocked or pried off. Pinching it lifts the whole egg out of its cup (red team `grabcap`). Once the shell is cut, the
hook carries the top in the bell, where it was when the ring was cut, as the teeth hold it in a real topper ("the top of the shell comes
off along with the metal cap when it's lifted", see the sources below).

## Real-trial preparation, reset and measurement

**What to buy.** A drop-weight egg topper of the Take 2 "Original Clack" pattern (a bell on the end of a rod, a stainless ball or
sleeve sliding on the rod; about 24 cm long, about 4 cm wide, roughly US$15-35); a porcelain egg cup with a 38 mm opening; large eggs
(57 g); a side plate. Mark the rod 8 cm above the weight's rest with a strip of tape.

**Eggs (the consumable, one per trial).** Large eggs straight from the fridge, lowered into boiling water for 6 minutes, then 1 minute
in cold water, used within 10 minutes. Any boiling recipe that gives a set white and a runny yolk will do; keep it the same across trials.

**Layout.** Tape marks for the egg cup at (30, 0) cm and the topper's bell at (36, 10) cm from the midpoint between the arm bases
(x forward, y left); plate at (27, -17); distractors as in `setup`. The egg goes in pointed end up. The topper stands on its bell with
the weight down.

**Rig protection.** Nothing is hot (the egg is cooled for a minute), nothing is sharp enough to cut a pad (the drop-weight
topper's cutter "isn't sharp", as the Yahoo review below notes), and the parts weigh 57 to 115 g. Put a tray under the egg cup to catch
yolk if the shell cracks.

**Measurement, at the end of the run, before anyone touches the scene.** A top photo and a side photo:
- Is there a cut ring round the crown, and does any crack run more than 5 mm below it (a cracked side, score 0)? A ruler against the
  side photo answers it.
- Is the egg still in its cup, upright?
- Is the cut top off the egg (in the bell or anywhere more than 3 cm from its seat)? Is the topper off the egg, standing on the table
  within 15 degrees of upright (a protractor against the side photo)?
- The video shows the drop: the weight let go above the 8 cm tape and falling freely; the frame at the drop shows the bell on the crown.
- Fingertips at least 3 cm from the egg, the topper and the top, everything still for 1 s: from the last second of video.

**Reset (about 2 minutes).** Discard the egg, set a new one in the cup on its mark, stand the topper on its mark with the weight down,
tip the cut top out of the bell. In a real trial the cut top occasionally stays on the egg when the topper is lifted; it is then still
on the egg at the end, and the trial scores 0.25 (shell cut) unless the robot lifts it off some other way (the 0.5 and higher levels need
it more than 3 cm from its seat).

## Distinct from

| Nearest | What it tests | Difference here |
|---|---|---|
| #1269 crack an egg into a bowl with both hands (pending) | a strike on a bowl rim, then opening the shell over the bowl | here the shell is never struck on anything: a tool is centred on the crown and a weight dropped down it, then the top lifted away; the contents stay in the shell |
| #3452 peel a hard-boiled egg (pending) | picking shell off a peeled egg | no peeling; a topper's ring scores the shell in one blow |
| #3263 slice a boiled egg in a wire egg slicer (revising) | laying an egg in a slicer and pressing the wires down | a free drop from a set height with the tool held centred and upright, not a press |
| `egg-and-spoon-race` (another egg task in this repository) | carrying an egg on a spoon | an egg that stays in its cup; the difficulty is the two-handed drop and the centring window |

No egg topper, drop-weight tool or soft-boiled egg appears in LIBERO, RoboCasa, SimplerEnv, BEHAVIOR, DataCenterBench or RoboDojo's task
lists as we know them.

## Importance and sources

A soft-boiled egg in an egg cup is an everyday breakfast across Europe and beyond, and getting its top off cleanly is the step
everyone fumbles; drop-weight and spring toppers exist to do it in one blow. The descriptions of how a drop-weight topper is used are
consistent:

- AOL, *Eierschalensollbruchstellenverursacher: The Egg Lover's Ultimate Kitchen Tool*: "a long, thin rod with a conical cap at one
  end that fits atop the egg", with "a sliding ball that, when dropped, generates just enough force to crack the shell in a perfect
  circle around the top"; users "slide the ball up to the top of the rod, place the cup over the top of the egg", "let go of the ball and
  let gravity do its work", and "the top of the shell comes off along with the metal cap when it's lifted."
  https://www.aol.com/eierschalensollbruchstellenverursacher-egg-lovers-ultimate-kitchen-185500458.html
- Yahoo Lifestyle, *The Best Clack Egg Openers to Change Up Your Breakfast*: the Take 2 Original Clack Egg Opener: "Pull the sliding
  ball to the top and let it drop"; its "interior eggshell cutter isn't sharp".
  https://www.yahoo.com/lifestyle/best-clack-egg-openers-change-080002880.html
- The Gadgeteer, on the spring-driven Rösle egg topper: "The spring mechanism causes vibrations, and the sharp edge in the dome
  perfectly cuts the shell", so the lid lifts off.
  https://the-gadgeteer.com/2018/04/24/easily-crack-the-top-off-soft-or-hard-boiled-eggs
- The Daily Meal, *What Is An Egg Topper And How Do You Use One?*: vibration toppers have "a handle and a dome that fits over the egg"
  and make "a clean circular crack around the top of the egg."
  https://www.thedailymeal.com/1443380/egg-topper-uses-explained/

**Why it is hard for a robot.** The two hands have different jobs at the same moment: one keeps a 22 cm tool standing on a 43 mm egg,
centred within 3 mm and upright within 8 degrees, while the other lifts a 45 g weight 8 to 10 cm up the rod (which tugs on the tool)
and lets go cleanly. Let go of the tool and it balances on the crown only precariously (red team `onearm`, below); hold it 5 mm off-centre and the egg tips in its
cup and the blow lands on its side (`offcentre`); lift the weight only 5 cm and nothing happens (`short`), and carry it back down instead
of dropping it and nothing happens either (`lowered`). Then the 22 cm topper, carrying the top, has to come straight up off the egg
without dragging it out of its cup, and be stood back up on its 4 cm bell.

## Validation

All runs in the table are of the final `task.py`, except where the `onearm` row says otherwise.

| Run | Mode | What the script does | task_score |
|---|---|---|---|
| Recorded, GPU, 640 x 360 (the submitted video) | `full` | the demonstration above | **1.0** (success at step 843) |
| Recorded, GPU, 640 x 360 | `full` | the same, second fresh run | **1.0** (success at step 843) |
| `--fast` CPU | `full` (two runs in one session) | the same | 1.0, 1.0 |
| `--fast` CPU | `none` | nothing | 0.0 |
| `--fast` CPU | `short` | the weight lifted only 5 cm and dropped; everything else as the demo | 0.0 (no ring cut; the top stays on the egg) |
| `--fast` CPU | `lowered` | the weight lifted the full height and carried back down by hand | 0.0 (not a free fall: no ring cut) |
| `--fast` CPU | `offcentre` | the bell held 5 mm off the egg's axis | 0.0 (pushing the bell off-centre tips the egg in its cup; in a development run of this mode the drop landed 19 degrees off the egg's axis and was recorded as a cracked side) |
| `--fast` CPU | `grabcap` | one arm pinches the top and pulls it off without the topper | 0.0 (the whole egg comes out of its cup) |
| `--fast` CPU | `onearm` | the bell set on the crown and let go; the same arm then lifts and drops the weight (and stops there) | 0.0 with the final file (the topper toppled off the crown). Over four earlier CPU runs with the same scene and drop rule: 0.0 twice (toppled, once taking the egg out of its cup), 0.25 twice (it balanced on the crown and the drop cut a clean ring) |

A Dream-1 run of the final `task.py` (2,400 steps, the instruction above) scored 0.0: it never set the topper on the egg or cut the shell.

The one-armed route is not ruled out by the scorer: in the simulation the bell sometimes balances on the crown on its own, and a drop
made then cuts the ring as it would in a held topper. It is a gamble (the 22 cm tool, with its weight lifted, is top-heavy on a 43 mm
dome), and people hold the topper. The demonstration holds it.

## Fidelity, approximations and sources of parameters

- The egg is two convex rigid bodies (shell below the cut, and the cap above it) of a large egg's size and mass: 43 mm wide, 57 mm
  long (26 mm blunt half, 31 mm pointed half), 57 g, with the cut where the 34 mm ring meets the shell (19.9 mm above the equator,
  cap 33 mm across, 11 mm high, 5.3 g). Shell fracture is not simulated: the drop rule above decides between a clean ring and a cracked
  side, at the moment of a counted drop. The egg's soft interior is not modelled, and a gripper squeezing the lower shell is not
  counted as crushing it.
- The topper is a custom articulation: its frame (bell, collar, rod, knob; 70 g) and the weight (45 g, 30 mm x 35 mm sleeve) on a
  prismatic joint along the rod with 10 cm of travel, gravity on, joint friction 0.005 N and damping 0.002 N s/m (nearly free). Its
  overall length (22.5 cm) and width (4 cm) follow retail listings of the drop-weight pattern; the split of mass between weight and
  frame, the travel, and the 34 mm ring are our estimates, not measured on a unit.
- The egg cup is a lathed porcelain cup (5.5 cm tall, 3.8 cm opening, 75 g), with colliders for its foot, stem, floor and a ring of
  16 wall segments; the egg rests on the rim's inner edge.
- SimReady has no egg cup or egg topper, and its egg is one rigid piece, so these are custom meshes. The four distractors (side plate,
  teaspoon, tea cup, salt shaker) are SimReady props.
- Once cut, the top is carried in the bell by the hook (it is written to its place in the bell every step), standing in for the teeth's
  grip; in a real trial it occasionally stays on the egg instead (see the real-trial section).

## Assets and credits

- `mechkit.py` is the mechanism kit from `tasks/heat-shrink-lug/` in this repository (another team's open-source task), unchanged apart from a credit line in its docstring.
- `demokit.py` is the closed-loop demo runner from `tasks/heat-shrink-lug/`, unchanged.
- `simready.py` is the SimReady loader from `tasks/cutters-off-pegboard/`, unchanged apart from its docstring.
- NVIDIA SimReady props (© NVIDIA, CC-BY 4.0), downloaded at first run from the pinned revision
  `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd` of https://huggingface.co/datasets/nvidia/simready-assets. Changes at load time: masses
  set as listed; the teaspoon and tea cup collide as convex hulls.
  - [dishware_plastic_bigPlate_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_plastic_bigPlate_a01): side plate, 0.15 kg (distractor).
  - [kitchenware_spoon_b01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/kitchenware_spoon_b01): teaspoon, 0.02 kg (distractor).
  - [dishware_cup_tea_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_cup_tea_a01): tea cup, 0.20 kg (distractor).
  - [kitchenware_shaker_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/kitchenware_shaker_a01): salt shaker, 0.12 kg (distractor).
- The egg, egg cup and topper are written by `task.py` from primitives and lathed profiles; nothing large is committed.
