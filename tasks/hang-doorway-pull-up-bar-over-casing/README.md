# Mount a doorway pull-up bar: hook it over the far-side casing, then swing it down until its pads bear on the near trim

A leverage doorway pull-up bar (the Iron Gym "Total Upper Body Workout Bar" kind: about $30, 93 cm wide, 2.15 kg, no
screws) lies in front of a doorway, its hooks pointing through it. The instruction is only the goal: **"Mount the
pull-up bar in the doorway."**

The bar holds by leverage alone. Its two hooks have to sit over the top edge of the casing (the trim) on the **far**
side of the wall, and its two foam pads have to bear on the casing's side legs on **this** side, lower down. Then the
bar's weight, and in use the user's, turns it about the hooks and presses the pads into the trim. Getting it there
takes both hands and a sequence of moves that each have a narrow window, the one the bar's users learn:

- **Thread it tilted, near end up.** The hooks are 18 cm tall and have to pass under the head jamb, then come up
  behind the far casing. The pads (on this side) and the hook legs (on the far side) are exactly one wall-plus-trims
  apart (14.5 cm) when the bar is level, so a level bar cannot get its hooks past the far casing: its pads meet the
  near trim first. Tilted near end up, the hooks reach further. With the hook feet just clearing the far casing's face,
  the bar fits between about 5 degrees of tilt (below that the pads hit the near trim) and about 29 degrees (above that
  the side rails jam on the head stop).
- **Raise both hooks over the casing's top together** (36.7 cm above the table here, 2.1 m in a real doorway) while the
  side rails stay under the head jamb: with the feet 1.5 cm over the casing, the rails clear the head stop by 2 to 4 cm.
- **Turn it nearly level before setting it down.** At the threading tilt the hooks cannot land: tilted 12 degrees, a
  hook's leg slopes back so far that its lower end meets the casing's face while the foot is still 1 mm short of
  reaching over the 15 mm top edge. The foot overlaps the edge only below about 11 degrees, by 5 mm only below about
  7. So the bar is turned down to a few degrees with the hooks still in the air, and that is where the window closes:
  at 3 degrees, with the feet 1.5 cm over the casing, the hooks' inner corners must be between 4 and 10 mm outside
  the casing's face (nearer, the legs hit the casing; further, the pads hit the near trim).
- **Set it down and level together.** Lowered and levelled in one movement, the feet come down on the casing's top,
  the legs against its face and the pads against the near trim, all at once. If one hook lands and the other misses,
  the bar hangs from one hook, twists and falls. Then let go: the bar's own weight keeps it seated.

The two arms share the 2.15 kg, 93 cm bar, holding it level side to side and setting its tilt. The bar's weight sits
on the hands throughout and the arms give under it (here by 6 to 8 cm down, 2 to 4 cm back and about 20 degrees), so the hooks have to be
steered by where the bar is, not by where the hands were sent. One arm alone cannot keep both hooks at the same height:
holding the bar at one side, the other hook hangs 9 cm low.

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: the doorway and the bar, the scorer (`score`, `success`) and the closed-loop scripted demo (`demo_script`) |
| `demokit.py` | Runs `demo_script` on the harness's embodiment and arm plans, and records the demo video, unchanged |
| `mechkit.py` | The prop kit (parts written as USD text, colliders, per-step hooks, fingertip points), unchanged |
| `simready.py` | Loads the SimReady distractors, downloading them once from Hugging Face, unchanged |
| `task.json` | The task's details for the board |

## Run a trial

Requirements: `yam-isaac-lab-arena` with Isaac Lab-Arena (`source <ARENA_DIR>/yam.env`), and network access the first
time, when `simready.py` downloads the two distractor packages into `~/.cache/simready`. From this folder:

```bash
# a model (here Dream-1)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Mount the pull-up bar in the doorway." --scorer task_score --max-steps 1800

# the scripted demonstration (closed loop): writes OUT/full/demo.mp4, OUT/full/setup.png and OUT/full/result.json,
# whose task_score is the harness's own --scorer task_score
python demokit.py task.py ~/yam-eval/pull-up-bar/demo --mode full

# the red-team variants of the same script, without video
python demokit.py task.py ~/yam-eval/pull-up-bar/checks --no-record --mode none,onearm,careless,holdon
```

`--scorer task_score` reports 1.0 when the bar has been seated and left alone for 3 s (the episode then ends), and
otherwise the score at the last step. Nothing is randomised (`object_jitter=0`; the scene has no other random
element), so every seed gives the same scene; the demo runs used the default seed.

Runtime: the demonstration takes 551 control steps (27.6 s of simulated time) to reach the 1.0, when the episode
ends; a full 1,800-step model trial is 90 s of simulated time. A recorded demo run took about 6 minutes of wall time on
a shared, busy GPU, and about 20 s with `--fast` (physics on the CPU, no cameras), which reaches the same score.

## The scene

World frame: x forward from the arm bases (y = 0 between them), y to the left, z up. Heights are above the table top
(74.75 cm).

**The doorway** is a rigid mock-up of the top of a standard interior doorway, built into a stud-wall section that
stands across the table:

| Part | Size | Where |
|---|---|---|
| Wall section (2x4 studs, 12.7 mm drywall each side) | 11.5 cm thick, 130 cm long, cut 16 cm above the opening | faces at x = 48.5 and 60.0 cm, y = -65 to 65 cm |
| Opening | 81.0 cm wide (a 32 in doorway); head jamb underside 30.0 cm above the table | y = -40.5 to 40.5 cm |
| Jambs (pine, painted) | 1.9 cm thick, 11.5 cm deep (the standard 4-9/16 in jamb for a 2x4 wall) | flush with both wall faces |
| Door stop (head and both sides) | 1.0 x 3.2 cm | x = 53.3 to 56.5 cm, on the jambs |
| Casing (trim), both faces | 1.5 cm thick, 6.4 cm wide, set back 0.3 cm from the jamb's edge | near face x = 47.0 cm; far face x = 61.5 cm; head casings' top edge 36.7 cm up |

Only the top 30 cm of an 81 x 203 cm doorway is modelled: the arms work at table height, so the table top stands in
for a plane 1.73 m above the floor. Every part the bar touches (head jamb, stop, both casings) is full size. The wall
is cut 16 cm above the opening so that the top camera can see over it to the far casing.

**The bar** is one rigid body of 25.4 mm black steel tube with foam grips and pads, 93 x 33 x 29.5 cm overall,
2.15 kg (see "Scale and mass"). In its own frame, which coincides with the doorway's frame when the bar is mounted
(origin at the far casing's outer face, the opening's centre line and the head jamb's underside):

| Part | Size | Where (bar frame, cm) |
|---|---|---|
| Two hooks (inverted L, steel with foam-lined feet) | 3 cm wide; foot 3.8 cm long x 2.2 cm; leg 2.5 cm x 18.2 cm | y = +-33; the foot's underside at z = 6.7 (on the casing's top edge), reaching back 1.3 cm over it; the leg's inner face at x = 0 (against the casing's outer face) |
| Two side rails | 25.4 mm tube | from the foot of each hook's leg at z = -8.0, under the head jamb, to a bend at x = -18, then down to the pad bar |
| Pad bar | 25.4 mm tube, 88 cm | x = -20, z = -15 (the bar's centre of mass is 1 cm above it) |
| Two pads (foam) | 5 cm across, 4 cm long | y = +-44 (on the casing's side legs), faces at x = -14.5 (on the near casing's face) |
| Two neutral handles (foam) | 32 mm | from the pad bar down to the grip bar |
| Grip bar | 25.4 mm tube, 93 cm, foam grips 32 mm at y = +-12 to +-30 | x = -29, z = -19 |

It starts lying on the table on its grip bar and its two pads, tilted 12.1 degrees from level (its near end up, the
hooks lower), hooks towards the doorway: grip bar axis at x = 27.0 cm, 1.6 cm up; pad faces at x = 42.0 cm; the hook
legs' outer top corners at x = 63.6 cm, 22.3 cm up. The hook feet are then directly under the far casing's bottom
edge, so the bar has to be pushed through about 3 cm before it can be raised.

**Distractors** (SimReady props, loose, ignored by scoring): a 0.5 L water bottle (6.8 x 6.8 x 24.5 cm, 0.52 kg)
standing at (12, -52) cm, and a folded microfibre towel (20.5 x 21.2 x 3.4 cm, 0.06 kg) at (16, 53) cm, turned
0.3 rad. Neither is in the arms' paths, and neither can help: the bar has nowhere to rest on them.

## Scale and mass

Nothing is scaled down and nothing is lightened. The bar is full size (93 cm wide, so that its pads reach the casing
of a 32 in doorway, the widest the product fits) and weighs 2.15 kg (4.7 lb), within the published "less than 5 lbs"
for the product (see Sources); the whole 2.15 kg hangs on the two grippers. The doorway's casing, jambs, stop and
opening width are standard full-size dimensions. The one liberty is height: only the top 30 cm of the doorway is
built, standing on the table, so the bar starts 30 cm below the head jamb instead of being lifted from the floor.

## Scoring

The scorer reads the bar's pose once per control step (20 Hz) and the fingertip points of both grippers. It never
reads a force.

- **A hook rests on the far casing** when its inner corner (where the foot's underside meets the leg's inner face) is
  within -2 to +5 mm of the casing's top edge in height and -2 to +10 mm of the casing's outer face, and the foot
  reaches at least 5 mm back over the casing's top. A feeler gauge or a phone photo on the far side checks it.
- **A pad bears** when its face is within 3 mm of the near casing's face and its centre is on one of the casing's
  side legs.
- **Level**: the grip bar slopes no more than 2 degrees end to end (a spirit level).
- **Fell**: the bar ever moves down faster than 1.0 m/s, the speed of a 5 cm free fall. This is recorded the moment it
  happens and cleared only when the trial resets (mechkit's `on_reset`), so a bar that was dropped and then mounted
  still scores 0.

| Score | Condition |
|---|---|
| 1.0 | both hooks rest, both pads bear, the grip bar is level, the bar is still (under 1 cm/s and 0.05 rad/s) and every fingertip point is at least 5 cm from the bar, continuously for 3 s |
| 0.75 | at the end: both hooks rest, both pads bear and the grip bar is level, but not held 3 s with the hands clear |
| 0.5 | at the end: both hooks rest on the far casing, but the pads do not both bear or the bar is not level |
| 0 | at the end one hook or neither rests on the far casing, or the bar fell at any time |

Possible scores: 0, 0.5, 0.75, 1.0. Every condition is read from the bar's end pose, so the checker cannot create it:
a person measures before touching anything. Lifting, tilting, setting the bar down again and regrasping are all
allowed; only a fall is penalised.

## Why the scoring is what it is

- **Both hooks and both pads.** A leverage bar is held by four contacts: two hooks carry the weight and two pads stop
  the turn. With the pads off the trim, the first pull swings the bar into the trim.
- **One hook is 0, not partial credit.** A bar resting on one hook hangs twisted from it, and a touch or a pull takes
  it off: that is an unsafe state, not progress.
- **Hands clear for 3 s.** The bar must stay mounted by itself, as it would for the person who uses it next. A bar
  still held by the robot is at most 0.75.
- **The fall rule.** A 2.15 kg steel bar falling from a doorway's head lands on someone's feet or face; a bar knocked
  off and then remounted has already done that.
- **No pull test.** The scorer does not hang a load on the bar; the manufacturer's own check (tug down on the grips)
  is a check of the same four contacts that the end pose already shows.

## The demo

`demo_script(ctx)` in `task.py` is a closed-loop script run by `demokit.py` on the harness's own embodiment. It reads
only what a camera-based policy could observe: the bar's pose and its own arms' joints, never the scorer's state.

Both grippers take the pad bar, which runs through the bar's balance point, one either side of the centre line
(y = +-21 cm), from above and behind (25 degrees from vertical, the fingers closing across the tube), and close fully.
From then on the script steers the bar, not the hands: every control step it reads the bar's pose, compares it with
where the plan wants the bar at that moment, and moves each hand's command by a quarter of the difference (at most
3 mm and 0.4 degrees a step). That takes out the arms' give under the bar's weight (by the time the bar is raised the
commands sit 6 to 8 cm above and 2 to 4 cm beyond where the hands are, and the grips 20 to 24 degrees over) and any
turn of the tube in the fingers, as it happens. The plan, in the doorway's frame:

1. Lift the bar 2 cm off the table at its starting tilt (12 degrees, near end up).
2. Push it 3.4 cm through, so that the hook feet are clear of the far casing's face.
3. Raise it until the hooks' inner corners are 1.5 cm above the casing's top edge (2.6 cm outside its face).
4. Turn it down to 3 degrees about the corners, bringing them back to 7.5 mm outside the casing's face.
5. Lower and level it in one 3 s movement, aiming the corners at the casing's top outer edge (2 mm into it, so that the
   hands press the hooks down) with the bar half a degree short of level: the feet come onto the casing, the legs
   against its face and the pads to within 2 mm of the trim.
6. Ease the commands back to where the hands are, so that the doorway takes the weight; open; back the fingers 8 cm
   off along their own axis, then move the hands 10 cm back and 5 cm up, clear of the bar.

The bar then settles its last millimetres by itself, and the scorer's 3 s hold runs out with the hands clear.

## Demo results

Two fresh recorded runs on the GPU (`python demokit.py task.py OUT --mode full`, 640 x 360, no frames lost) each
scored **1.0**: `success` after 551 steps (27.6 s), the script's 3 s hold with the hands clear having run out. The two
runs are step for step the same, as nothing is randomised. At the end of the set-down, before letting go, both hooks'
inner corners were 0.0 to 0.1 mm from the casing's face and 2.2 to 2.3 mm above its top (the pads and legs wedge the
bar in the doorway until the hands let go), each foot reached 12.9 to 13.0 mm back over the casing, and the bar was
0.03 degrees from level; once let go it settled onto the casing and held still. Every arm command was solved to
within 0.1 mm and 0.0 degrees of its inverse-kinematics target.

The same script's red-team modes (`--mode none,onearm,careless,holdon`):

| Mode | What it does | Score |
|---|---|---|
| `none` | nothing | RT_NONE |
| `onearm` | the left arm alone takes the pad bar and follows the same plan | RT_ONEARM |
| `careless` | lifts the bar straight up from where it lies, without pushing it through first, then lets go | RT_CARELESS |
| `holdon` | mounts it as the demo does but never lets go (the cheapest way to make it look mounted) | RT_HOLDON |

## Real trials: preparation, reset and measurement

**Build.** Buy an Iron Gym Total Upper Body Workout Bar (or an equivalent leverage bar for 24 to 32 in doorways) and
build the doorway mock-up from stock parts: a 130 cm length of 2x4 stud wall with 1/2 in drywall each side, framing an
81 cm opening with a 4-9/16 in pine jamb set, 3/8 x 1-1/4 in stop, and 9/16 x 2-1/2 in casing on both faces
(about $60 of lumber and trim). Screw it to a base board and clamp the base to the bench so that the head jamb's
underside is 30 cm above the bench. Mark the bench with tape at the bar's start: the grip bar's axis line at x = 27 cm,
the pads' contact points at x = 42 cm, y = +-44 cm.

**Each trial.** Lay the bar on its grip bar and pads on the tape marks, hooks pointing through the opening, their feet
under the far casing; stand the bottle and lay the towel on their marks. Reset takes under a minute: lift the bar off
the casing and lay it back. Nothing is consumed.

**Measuring.** Before touching anything at the end: look over the wall (or with a phone camera on the far side) to see
whether each hook's foot sits on the far casing's top edge and its leg against the casing's face (a 5 mm feeler gauge
must not pass under the foot, and the foot must cover at least 5 mm of the casing's top); a feeler gauge shows whether
each pad is within 3 mm of the near trim; a spirit level on the grip bar shows whether it is within 2 degrees. A video
at 30 fps shows whether the bar fell (more than 5 cm of free fall) and whether the hands were 5 cm clear for the last
3 s.

**Safety.** The arms carry 2.15 kg between them, about 1.1 kg each. The bar's foam covers every edge the trim meets.
Keep people out of the space under the bar during the trial: a bar that falls drops 30 cm onto the bench.

## Distinct from

| Nearest task | What it tests | How this differs |
|---|---|---|
| #2153, Refit a fridge door bin (accepted) | carrying a bin in level above two hidden supports and lowering both ends on together | here the hooks are threaded through a doorway at a tilt, raised behind a wall, turned nearly level in the air inside a 6 mm window, and the bar then turned about the hooks until its pads bear: a leverage mount, not a drop-in |
| #1775, Refit a loaded drawer on its roller slides (accepted) | carrying a drawer in above riding height so its rear hooks clear, then lowering it | no slides guide anything here; the bar's two hooks land on a 15 mm edge on the far side of a wall, out of the hands' reach, and the pads must close on the near trim at the same time |
| #2930, Club across a steering wheel (not accepted) | hooking a bar's ends round a wheel's rim and pulling the bar out to lock | no hook-over-trim seating from the far side, no rotation into bearing |

Also searched: #450 (a handle hung across two wall hooks: the hooks are on the wall, not the part), #1969 (shims in a
door's hinge jamb: the same kind of door frame, but no bar). Searches of the board (`pull-up`, `doorway`,
`door frame`, `casing`, `trim`, `fitness`, `gym`, `leverage`, `hook`, `cleat`, `bracket`) found no other doorway-bar or
over-trim mounting task. LIBERO, RoboCasa, BEHAVIOR, SimplerEnv, DataCenterBench and RoboDojo have no doorway fitness
bar or hook-and-lever mounting either.

## Importance and sources

Doorway pull-up bars are a common home exercise purchase, and the screw-free leverage kind is mounted and taken down
by its owner, often every session. It is recognisable in a few seconds of video, and the way it fails (one hook
missed, pads off the trim) is the way people drop them.

- The bar: [Iron Gym Total Upper Body Workout Bar (USA Home Gym)](https://www.usa-homegym.com/pull-up-bar/iron-gym-total-upper-body-workout-bar):
  "designed to fit residential doorways from 24 to 32 inches wide, with doorway trim or molding up to 3.5 inches
  wide"; "capable of supporting up to 300 lb"; "a combination of steel tubing and sections of foam padding to protect
  your door frame"; "leverage being used in place of any permanent fixtures".
- Its weight: [Garage Gym Reviews: Iron Gym Total Upper Body Workout Bar](https://www.garagegymreviews.com/equipment/iron-gym-total-upper-body-workout-bar):
  "Weighs less than 5 lbs" (elsewhere on the page "around 5 lbs"), "Holds up to 300 lbs", "May leave marks on door".
- Its price: US retail listings at about $26 to $33, for example
  [Walmart](https://www.walmart.com/ip/302326317).
- How it hangs: [Brit + Co: Install an Iron Gym pull-up bar](https://guides.brit.co/guides/install-an-iron-gym-pullup-bar):
  "Hang as shown, with the black rubber pad side on top of the lip of the doorway. Be sure the curve goes to the other
  side so downward force doesn't rip it off."
- What it bears on: [MEL Magazine: will a pull-up bar damage the door frame](https://melmagazine.com/en-us/story/door-frame-damage-pullup-bar):
  the bar bears on the architrave (in US terms the casing), "the decorative door lining that sits off on its own",
  which "in some cases is barely a couple centimeters thick".
- The doorway: [Bob Vila: standard door sizes](https://www.bobvila.com/articles/standard-size-for-doors/) ("Standard
  door height for interiors is 80 inches"; "The standard door width for interior doors is 32 inches"; "For standard
  interior walls constructed of 2x4s, standard jamb thickness is 4 9/16 inches") and
  [Door and More: jamb width](https://www.doornmore.com/help/jamb-width.html) ("4 9/16"" for a "2x4 frame with ½"
  drywall").

The bar's overall size (93 x 33 x 29.5 cm) and internal layout (hook height, rail path, pad positions) are our
reconstruction from the product's photographs and its stated doorway range; they are given in full in `task.py` and
above.

## Assets and code

The doorway and the bar are authored from boxes and tubes in `task.py` (no recognisable SimReady prop exists for
either). The two distractors are NVIDIA SimReady props, downloaded at first run from the Hugging Face dataset
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`food_beverage_bottledWater_b01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/food_beverage_bottledWater_b01): mass set to 0.52 kg.
- [`consumable_microfiber_towel_folded_small_b01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/consumable_microfiber_towel_folded_small_b01): mass set to 0.06 kg.

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Nothing large is committed.
`mechkit.py` and `demokit.py` are taken unchanged from another team's open-source task in this repository,
`tasks/heat-shrink-lug/`, and `simready.py` unchanged from `tasks/cutters-off-pegboard/`.

## Fidelity

Everything is rigid and acts by contact: the doorway is fixed boxes, and the bar is one rigid body made of tubes and
boxes. The hooks are square-section L shapes where the real ones are bent tube, and the foam is rigid. Friction is
uniform. The real bar's exact internal layout is not published; ours keeps its size, weight, two hooks, two pads and
grip bar. Only the top 30 cm of the doorway is built. No person hangs on the bar.
