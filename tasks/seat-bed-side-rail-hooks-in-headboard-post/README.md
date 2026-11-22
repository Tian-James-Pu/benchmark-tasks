# Hook a bed's side rail onto its headboard leg: both hooks into their slots at once, then drive it down until seated

A solid-wood queen bed is being put together, in the order its hardware makes natural: headboard first, then both side rails,
then the footboard. The headboard stands; the far side rail is already hooked onto the far headboard leg; the footboard waits
beyond the rails' far ends, and the bed's cross supports and angle irons lie on the floor inside the frame. The near side rail's
far end rests on a helper support by the footboard; its near end lies on the floor beside the near headboard leg. Rail and leg
are joined by a hook-in bed rail fastener: a steel plate on the rail's end carries two hooks, and a slotted steel plate in the
leg's face takes them. To join them, a person lifts the rail's end, lines both hooks up with both slots, pushes them in, lets the
rail drop so the hooks go down behind the plate, and taps the rail down so the hooks' tapered throats wedge the joint tight.

The instruction states only the goal: "Attach the side rail's loose end to the headboard post, fully seated."

**The knack.** The rail's near end carries about 20 N of a 4.4 kg, 2.06 m rail and is held only near its top edge, so it sags
and twists in the hands. Both 4.0 mm hooks have to meet both 6.35 mm slots (1.2 mm of play each side, 82.5 mm apart one above
the other) at the same moment, at a height where each 22.2 mm hook fits through its 25.4 mm slot (a 3.2 mm window), while the
rail is pressed towards the leg. A rail rolled by more than about 1.6 degrees puts one hook beside its slot. Then the joint has
to be driven down its last 3 mm against the wedge. One hand carries and steers near the end while the other carries further back
and keeps the rail plumb; together they lower it through the window and press it home. Done carelessly (end lifted only to its
seated height and pushed at the leg, or slid along the floor), the hooks meet solid plate or wood and never go in.

## Response to review

| Request (quoted from the review) | Change |
|---|---|
| "Make full credit require both hooks to be engaged and seated: check lateral alignment and bounded height and gap at each hook, rather than only upper bounds on height and gap." | `task.py`: `hooked()` now needs every reading at each hook inside a two-sided band (gap more than -1.5 mm and at most 7.5 mm, s more than -1.5 mm and at most 9.5 mm, sideways offset e within 2 mm of the slot's centre), and the new `seated()` needs a hook in *and* s and gap each within 1 mm of zero, either way. Full credit needs both hooks seated (the rail fully down at both), held 3 s with the hands clear and the rail still. `task.json`'s `scoring` and the Scoring section below say exactly this. |
| "Add a regression trial that slides the rail along the floor beside the post and verifies that it scores zero." | Two regressions. First, a simulated trial: `task.py` has a new red-team mode, `DEMO_MODE = "floorslide"`, in which both hands take the rail and slide its near end along the floor beside the leg, aiming 7 cm past the leg's face, then let go and back off. It scores **0**: the end stayed on the floor (s about -35 mm) 4 to 8 cm off the slots' line, so neither hook was in (red-team table below). Second, because the soft arms could slide the end only to 11 mm short of the face, `test_floor_slide.py` runs the scorer's own functions on 2,009 floor poses, beside and in line with the leg, from 5 cm short of its face to 15 cm past it. No pose has a hook in, so all score 0, and the seated pose still reads as seated. |
| "Restore recognizable full-size bed-frame context, including the headboard, footboard and side rail, while retaining the same hook-in operation and documenting the real-trial setup." | `task.py`: the whole queen frame at real size, from Woodworker's Journal's Greene & Greene-inspired queen bed plan. That means a 978 mm headboard with both legs, bottom rail, panel, battens and top rail; the far side rail hooked on; the 546 mm footboard with its slotted plates; the near side rail at its full 2.057 m and full 4.4 kg; and the plan's cross supports and angle irons on the floor. The table top is extended level all round as the bedroom floor. The hook-in fastener, its slots, the wedge fit and the robot's corner are unchanged. The setup image adds a wide view of the whole frame (`setup_view.py`) above the robot's own view. **Real trials** below now builds the frame from that plan, with sources, and gives the procedure. |

## Files

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: the scene, the fastener's working, the scorer (`score`, `success`) and the scripted `demo()` |
| `task.json` | The task's details for the board |
| `setup_view.py` | The same scene seen from a camera out at the bed's front right; it renders the setup image's wide view (top half of the screenshot; the bottom half is the robot's own overhead camera at the start) and is not a trial |
| `test_floor_slide.py` | The floor-slide regression on the scorer (pure Python, below) |
| `mechkit.py` | The mechanism kit (parts written as USD text, joints, hooks, fingertip points), unchanged from `tasks/heat-shrink-lug/mechkit.py` in this repository (another team's open-source kit) |
| `simready.py` | Loads the SimReady distractors: the loader from `tasks/cutters-off-pegboard/simready.py` in this repository (another team's open-source code), only its docstring changed |

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena (`source <ARENA_DIR>/yam.env`).
- Network access the first time the scene is built: `simready.py` downloads the three SimReady packages into `~/.cache/simready`.

## Run a trial

From the repository's root:

```bash
# a model
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/seat-bed-side-rail-hooks-in-headboard-post/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Attach the side rail's loose end to the headboard post, fully seated." \
  --scorer task_score --max-steps 1500 --store-frames --log-dir ~/yam-eval/bed-rail

# the scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/seat-bed-side-rail-hooks-in-headboard-post/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=640 -E cam_height=360 \
  --instruction "Attach the side rail's loose end to the headboard post, fully seated." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/bed-rail-demo
```

`--scorer task_score` prints the score. Nothing in the scene is randomised, so every seed gives the same start (use
`-E object_jitter=0`). Runtime: the demo reaches full credit at step 546 (27.3 s of simulated time). With cameras, on GPUs shared with other jobs,
a demo run took about 12 minutes of wall time here; a 1,500-step model trial takes from about 15 minutes to over half an hour.

`test_floor_slide.py` (pure Python, no simulator) is the regression for the review's floor-slide route. It runs the scorer's own
`hook_readings`, `hooked` and `seated` on 2,009 rail poses with the near end on the floor: from 12 cm to one side of the slots'
line to 12 cm to the other, and from 5 cm short of the leg's face to 15 cm past it. No pose has a hook in, and the seated pose
reads as both hooks seated:

```bash
python tasks/seat-bed-side-rail-hooks-in-headboard-post/test_floor_slide.py   # PASS
```

## The scene

All positions are in metres from the midpoint between the arm bases (x forward, y to the robot's left); heights are above the
table top, which stands in for the bedroom floor and is extended level all round (a 2.6 x 3.6 m platform in the table's colour)
so that the whole bed stands on it. `task.json`'s `setup` lists every object with its size, material and mass.

The bed is a queen frame (for a 60 x 80 in mattress) built to Woodworker's Journal's Greene & Greene-inspired queen bed plan,
using that plan's materials-list sizes. The robot faces the near side of the bed, at the headboard's near leg: the headboard runs
away from the arms on the left, the near side rail runs to the right, and the footboard is 2.1 m away to the right.

- **Headboard** (fixed): two legs 70 x 35 x 978 mm (the plan's 1 3/8 x 2 3/4 x 38 1/2 in), the near one at x 0.265 to 0.335,
  y 0.070 to 0.105, the far one 1.578 m further along x. Between them are a 25 x 165 mm bottom rail at the side rails' height
  (35 to 200 mm up), a 19 mm panel with five 50 mm battens, and a 35 x 76 mm top rail. The near leg is a mechanism of its own (the
  `post`), holding the fastener's slotted plate and the wedge stops; the rest of the frame is one fixed part (`bed`).
- **The fastener**: Rockler's 6 in x 5/8 in Heavy Duty Wrought Steel Bed Rail Fastener (item 28597; $19.99 for the four sets a
  bed needs). Every dimension comes from Rockler's installation drawing: both plates 5 15/16 in x 5/8 in x 9/64 in
  (151 x 15.9 x 3.6 mm); hooks 5/32 in (4.0 mm) thick, standing 7/16 in (11.1 mm) out, 7/8 in (22.2 mm) tall with a 3/8 in
  (9.5 mm) tip below the neck; slots 1/4 in x 1 in (6.35 x 25.4 mm); hook mortises behind the slots 1/4 in wide and 1/2 in deep.
  The hooks' spacing (82.5 mm) and positions on the plate are scaled from the drawing. Seated, the plates' ends line up and the
  hooks' necks rest on the slots' bottoms, as in the drawing. (The plan uses Rockler's 4 in size of the same fastener on 6 in
  rails; this bed uses the 6 in size on 6 1/2 in rails, which gives the joint two hooks.)
- **Near side rail** (free body): 28.6 x 165 mm (the plan's 1 1/8 in thickness and 81 in length; 6 1/2 in deep for the 151 mm
  plate), 2.057 m long, in poplar (450 kg/m^3), so 4.4 kg. The plan calls for mahogany; poplar is the usual paint-grade
  substitute and is what the real trial uses (below). It has the hook plate and hooks at both ends. Its far end rests on the
  helper support, so its near end carries 20 N.
- **Helper support** (fixed), 10 cm from the rail's far end: a 90 x 90 mm wood block carrying a 20 mm dowel across its top (35 mm
  up, the rail's seated height) between two guide cheeks 1.5 mm clear of the rail. It stands in for the second person or the
  block that holds a rail's far end while its near end is hooked on. The rail can pitch on the dowel, slide along it and swing a
  little, as a rail held at its far end can.
- **Far side rail** (fixed): the same rail, already hooked onto the far headboard leg, its far end on a wood block.
- **Footboard** (fixed): legs 70 x 35 x 546 mm (the plan's 21 1/2 in), with a bottom rail, panel, battens and top rail like the
  headboard's and a slotted plate in each leg. It stands 6 cm beyond the rails' far ends, to be hooked on last.
- **On the floor inside the frame**: the plan's six cross supports (3/4 x 4 1/2 x 60 3/4 in pine) stacked two high and its two
  1 1/2 in steel angle irons, all at least 0.6 m from the work.
- **Start**: the rail's near end rests on the floor 30 mm short of the leg and 25 mm towards the arms, the rail sloping 1.0
  degree down to it.
- **Distractors** (SimReady, loose, ignored by scoring): a coffee mug, a Phillips screwdriver and the assembly booklet, on the
  floor inside the frame, at least 25 cm from the work.

### How the joint works in the simulator

The hooks and the plate are real-size convex colliders, so a hook goes into its slot only where it really fits: within 1.2 mm
sideways of the slot's centre, at a height where the whole 22.2 mm hook clears the 25.4 mm slot, and with the rail's end
pressed towards the leg. Each hook's tip has its real sloped inner face, so as the hook drops, the plate's edge under the slot
draws the rail's end against the leg (the throat is 4.0 mm, 0.4 mm more than the plate is thick). The real fit is an
interference wedge over the last few millimetres, which a rigid simulator cannot model. In its place, under each hook's tip in
its mortise sits a stop 3 mm short of the seated height, on a stiff spring (20,000 N/m), which gives way only to more than 16 N
on that hook and stays where it is pushed. The rail's own weight (about 10 N a hook) leaves it there, and a firm push seats it.
When a hook comes out of its slot, its stop springs back up. The stops are contact-driven: only a hook standing on one can push
it down.

## Scoring

`--scorer task_score` gives 1.0 for success, otherwise the score at the end. The task file sets `SUCCESS_HOLD_S = 0` and counts
its own hold. Each rule below is what `score()` and `success()` compute (`Trial`, `hooked()`, `seated()` and `forced()` in
`task.py`); `task.json`'s `scoring` text says the same.

Read at each of the two hooks:
- **s**: the rail end's height above its seated line (the height it has when both hooks are fully seated); negative below it.
- **gap**: the rail's end face to the leg's face, at that hook's height; negative past it.
- **e**: the hook's sideways offset from its slot's centre. The hooks are on the rail's centre line, so in a real trial this is
  the rail end's centre from the slots' centre line, marked on the leg above and below the plate.

A hook is **in** (through its slot and dropped behind the plate) when every reading is inside a two-sided band:
- gap more than -1.5 mm and at most 7.5 mm. A hook stands 11.1 mm out of the rail's end, so this puts it 3.6 mm or more past the
  leg's face, which only its slot allows.
- s more than -1.5 mm and at most 9.5 mm. Its tip is then below the slot's bottom, behind the plate, so it cannot come straight
  out.
- e at most 2 mm either way. The slot itself allows 1.2 mm.

A hook is **seated** when it is in and s and gap are each within 1 mm of zero, either way.

- **0 for the whole trial** if at any time:
  - the rail end is **dropped**: a point near its end falls faster than 0.5 m/s (about a 1.3 cm free fall) while every fingertip
    point of both grippers is more than 2 cm from the rail; or
  - it is **forced into the leg**: its end 1.5 mm or more past the leg's face, a hook in its slot 1.5 mm or more below its
    seated height, or a hook 1.5 mm or more past the leg's face while more than 2.7 mm off its slot's centre (which only a hook
    driven into the plate beside the slot can be).

  These are recorded as they happen and cleared only by a reset (mechkit's `on_reset`).
- **1.0** (success, ends the trial): both hooks seated for 3 s running, with every fingertip point of both grippers
  (`mechkit.fingertips`: the tip and 3 cm up each finger, at its middle and both edges) at least 3 cm from the rail, and the
  rail still (under 1 cm/s and 0.1 rad/s).
- Otherwise, at the end:
  - **0.75**: both hooks in, with s 4.5 mm or less at both: hung on the wedge fit but not driven home.
  - **0.5**: both hooks in.
  - **0.25**: one hook in.
  - **0**: anything else, including doing nothing.

A rail end lying on the floor has s of about -35 mm at both hooks; one slid beside the leg also has e of more than 40 mm. Either
way neither hook is in, and the trial scores 0, wherever the end is along the leg. Lifting, tilting or regripping the rail is not
penalised, nor is setting it down gently (the drop rule needs the hands off it). Credit does not stick: the partial scores are
read at the end, and full credit needs the hold.

## The demo

`demo()` is a scripted reference for `--policy yam_scripted`, not a policy. Both hands come down from above at 45 degrees and
close across the rail's top edge: the left 22 cm from the rail's end, leaning away from the leg, with its grip turned 35
degrees about the vertical so that its wrist and forearm stay clear of the full-height leg; the right 36 cm from the end.
Then:

1. They lift the end to 18 mm above its seated height, in line with the leg and 13 mm short of it, and settle.
2. They press it towards the leg, so the hooks rest against the plate above the slots.
3. Still pressing, they lower it 16 mm over 4 s. As the hooks come level with the slots (s from 12.7 to 9.5 mm) they slip in,
   the rail's end goes to the leg, and the hooks drop behind the plate onto the wedge stops (s about 2.7 mm).
4. They drive it on down until seated, ease off, let go, back off along their fingers (still open either side of the rail), rise
   and move back towards the robot over the rail (the right hand turns its grip 30 degrees about the vertical, which its reach
   needs up there).

The YAM's arms are soft (shoulder joints 80 N m/rad, wrist joints 10 N m/rad), and under this rail they sag several
centimetres and the rail twists a few degrees in their grip. So while they carry or press the rail, the demo corrects its
commands from what the cameras would see: the rail's pose. It integrates the error between where the rail's end should be and
where it is, shifts both hands sideways to put the hooks on the slots' centre line, and rolls both grips to bring the two hooks
into one vertical line (`Servo` in `task.py`). The demo never reads the scorer's state.

Results (scripted demo through `yam_scripted` and `--scorer task_score`, GPU physics with cameras, this scene):

| run | what the script does | score |
|---|---|---|
| `full` | as above | 1.0, twice in a row, both at step 546 (27.3 s) |
| `floorslide` | both hands take the rail on the floor and slide its near end along the floor beside the leg, aiming 7 cm past its face, then let go | 0. The end went 4 to 8 cm to the side of the slots' line, on the floor (s -35 mm), and reached 11 mm short of the face plane, the furthest the soft arms could push it against the floor's friction; neither hook was in at any step. `test_floor_slide.py` covers the poses past the face |
| `careless` | lifts the end only to its seated height (below the slots' window), pushes it at the leg and lets go | 0: the hooks met the plate below the slots and never went in, and when the hands let go the end settled back to the floor |
| `onearm` | the left hand alone does everything | 0: one hand cannot lift the full rail's end |
| `nothing` | no plan: both arms stay at their ready pose | 0 |

The red-team runs used copies of `task.py` with `DEMO_MODE` set to the run's name (deleted afterwards; set `DEMO_MODE` in a copy to
repeat them).

**Model run.** Dream-1 was given the instruction above for the full 1,500-step run limit (the first command under "Run a
trial") in this revised scene. It scored 0: it raised both arms and held them folded near their bases, and never touched the rail.

## Importance and sources

- Wooden bed frames are taken apart and put back together with almost every move, and moves are common: using 2007 American
  Community Survey data, the US Census Bureau estimates that "a person in the United States can expect to move 11.7 times in
  their lifetime"
  ([Calculating Migration Expectancy](https://www.census.gov/topics/population/migration/guidance/calculating-migration-expectancy.html)).
- The bed: Chris Marshall, "Greene & Greene-inspired Bed", Woodworker's Journal (Rockler's magazine), 7 October 2016
  ([article](https://www.woodworkersjournal.com/greene-greene-inspired-bed/),
  [drawings and materials list, PDF](https://www.woodworkersjournal.com/wp-content/uploads/Greene-Greene-Bed-Project-Drawings-1.pdf)).
  It is a queen bed. Its materials list gives headboard legs 1 3/8 x 2 3/4 x 38 1/2 in, footboard legs 21 1/2 in, top rails
  1 3/8 x 3 x 62 in, side rails 1 1/8 x 6 x 81 in, six 3/4 x 4 1/2 x 60 3/4 in cross supports, two 1 1/2 x 1 1/2 x 80 in angle
  irons and four Rockler bed rail fasteners. Its joinery is these hook-type fasteners in routed mortises, with the rails' mortises
  cut deeper "so the pronged hooks will pull the side rails tightly against the headboard and footboard once assembled". Its
  photo caption: "Tap the bed rail hardware together with a deadblow mallet."
- The fastener: Rockler's Heavy Duty Wrought Steel Bed Rail Fasteners, 4-pack, 6 in x 5/8 in, item 28597, $19.99, "one pack of
  four fasteners will complete a bed"
  ([product page](https://www.rockler.com/heavy-duty-wrought-steel-bed-rail-fasteners-4-pack-select-size)), and its installation
  drawing with every dimension used here
  ([6" Bed Rail Fastener Installation Instructions, PDF](https://go.rockler.com/tech/6-bed-rail-fastener-installation-instructions.pdf)).
- Tapping a hooked-on rail down to lock it is also the step in commercial bed makers' assembly instructions, for example Room &
  Board's Hartley bed: "lift one end into footboard post by slipping connecting posts of side rail into interlocking keyhole on
  footboard bracket. Tap rails with rubber mallet to ensure locked position"
  ([Hartley Bed Assembly, PDF](https://www.roomandboard.com/images/pdf/Hartley_Bed_Assembly.pdf)).
- We did not find a commercial bed whose maker documents this mortised hook-and-slot fastener: makers' beds checked use
  lag-mounted brackets, keyhole-and-pin brackets or bolts. So the bed here is the published plan, and the real trial builds it.

## Distinct from

| Task | What it tests | How this one differs |
|---|---|---|
| #1989 Refit a car headrest at its lowest notch (accepted) | two vertical posts slid down into two sleeves, then a held release button | Here a long cantilevered rail is carried by one end, pushed sideways into a wall plate through two narrow slots at once, dropped behind it and wedged home; there is no button and nothing slides down a sleeve |
| #1775 Refit a loaded drawer on its roller slides (accepted) | carrying a drawer high so its rear hooks clear, then running it in | No slots or wedge there; here both hooks must meet slots 1.2 mm wide each side in a 3.2 mm height window |
| #2153 Refit a fridge door bin (accepted) | carrying a bin in level above two hidden supports, lowering both ends on together, pushing it home | Nothing passes through a slot or is wedged there, and the light bin is carried whole; here one end of a long, heavy rail is carried while its other end is held elsewhere |
| #1946 Install hook-in gondola brackets (not accepted) | hooking shelf brackets into a slotted upright | A light bracket hooked by its own tabs; here the hooks are on the end of a heavy rail whose far end is held elsewhere, and the joint must be wedged down |
| #2917 Move a fridge shelf up a slot (not accepted) | tipping a glass shelf so its rear hooks swing into slotted supports | That shelf is tipped so its hooks swing in and drop, with nothing to drive home; here the hands carry 20 N at the end of a rail pivoting about its far end, hold both hooks in a 3.2 mm height window while pressing sideways into the leg, then drive the joint down its last 3 mm against the wedge |
| #3584 Hang a TV on its tilt wall mount (pending) | dropping a light TV's two hidden hooks over an open wall rail, the TV carried whole | Open hooks dropped over a rail and nothing to drive home; here the hooks are on one end of a 2 m rail pivoting about its far end, pass sideways through two closed slots inside a 3.2 mm window, and are then driven down against the wedge |
| #2385 Hang a cutlery caddy on a dish rack's rail (not accepted) | dropping a light caddy's two hooks over a wire rail | Hooks dropped over an open wire; here the hooks pass through closed 6.35 mm slots and are then wedged |

No accepted or pending task on the board joins a bed's rail to its leg, and the external sets named in the guide (LIBERO,
RoboCasa, BEHAVIOR, SimplerEnv) have no hook-in rail fastener.

## Real trials

**The bed.** Build the queen frame from Woodworker's Journal's Greene & Greene-inspired queen bed plan (drawings and materials
list linked above), with two changes that the simulated bed also has. First, the side rails are 6 1/2 in deep instead of 6 in.
Second, the fasteners are Rockler's 6 in x 5/8 in size (item 28597, one 4-pack, $19.99) instead of the 4 in size, so that each
joint has two hooks. Mortise them per Rockler's drawing: the slotted plates flush in the legs' 2 3/4 in faces, centred, their
slots' bottoms 69 and 152 mm above the floor; the hook plates flush and centred in the rails' ends. Make the side rails from
5/4 poplar (1 1/8 in after planing, 6 1/2 in, 81 in), weighed at 4.4 kg each (within 0.3 kg; plane or pick boards to suit).
The headboard, footboard and cross supports can be the plan's mahogany or any hardwood, since only the rail is handled. Leave
the finish and the angle irons off the near rail for the trial; it is screwed on after both rails are hung. Lumber for the
frame is roughly $400 to $700 at a hardwood dealer.

**The room and the rig.** Stand the headboard on the floor (or on a platform level with the robot's mounting surface) at the
robot's left, so that its near leg's slotted face is where the simulated one is: 0.30 m in front of the arms' midpoint and 0.07 m
to its left, facing the robot's right. Hook the far side rail onto the far leg and set its far end on a block 35 mm high. Set the
footboard 6 cm beyond the rails' far ends. Lay the cross supports and angle irons inside the frame as listed in `setup`. Make the
helper support from a 90 mm square block 25 mm tall with a 20 mm dowel set across its top (dowel top 35 mm up) and two
12 mm cheeks screwed either side, reaching 85 mm up, 1.5 mm clear of the rail; put it under the near rail 10 cm from its far end. Screw the
headboard's legs to the floor or platform through two small angle brackets (or weight its feet), so that the push of the
hands cannot shift it.

**Preparation and the seated line.** Hang the near rail and tap it fully home once. Mark the leg at the rail's top edge (a pencil
line: the seated line) and draw the slots' centre line on the leg above and below the plate. Put tape marks on the floor for the
start: the rail's end 30 mm short of the leg and 25 mm towards the robot, its underside on the floor.

**Reset (about 1 minute).** Pull the rail's end straight up (a firm pull or a tap from below frees the wedge), draw it back, and
set its end down on the tape marks. Its far end stays on the helper support's dowel.

**Measurement, before anyone touches the rail.**
- s: with a steel rule, read the rail's top edge at its end above (positive) or below (negative) the seated line.
- gap: with a feeler gauge or rule, read the gap between the rail's end and the leg at the upper and lower hooks' heights; when
  the end is past the leg's face, read how far past.
- e: read the rail end's centre from the slots' centre line on the leg, with a rule across the rail's top edge.

A hook is in when gap is over -1.5 mm and at most 7.5 mm, s is over -1.5 mm and at most 9.5 mm, and e is within 2 mm. A hook
that missed its slot holds the rail's end at least 11 mm off the leg; a rail end left on or slid along the floor has s of about
-35 mm. Dropping is read from the video: a fall of more than 1.3 cm with no hand within 2 cm. The hands' clearance (3 cm) is
read from the video over the last 3 s.

**Safety.** The rail's near end weighs about 2 kg at the hands, and the trial runs at floor level with nothing above the arms.
Protect the grippers' pads from the plate's edges with a strip of tape on the plate's top and bottom ends if needed; the hooks
never touch the pads.

## Fidelity

Rigid bodies, 240 Hz physics, 20 Hz control. The frame is real size and complete; the headboard, footboard and far rail are
fixed (screwed down in the real trial). The near rail has its real length and mass, its far end on a helper support. The hooks'
and slots' geometry is real size; the wedge fit over the last 3 mm is a stop that gives only to more than 16 N per hook (an
estimate) rather than an interference fit. The floor is the station's table top, extended level all round. Rail twist, frame
racking and wood grain are not modelled. Friction is uniform per part: rail 0.6, leg and plate 0.4, helper support and the rest
of the frame 0.3.

## Assets and licences

- SimReady props (NVIDIA, [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/)), from the Hugging Face dataset
  `nvidia/simready-assets` at revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, downloaded at first run (nothing large is
  committed). At load time their masses are set as listed and each is made one rigid body; the screwdriver collides as convex
  hulls.
  - `dishware_mug_a01` (the mug): [link](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01), © NVIDIA, CC-BY 4.0
  - `handTool_screwdriver_phillips_ph2_a01` (the screwdriver): [link](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_screwdriver_phillips_ph2_a01), © NVIDIA, CC-BY 4.0
  - `misc_notebook_a01` (the assembly booklet): [link](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/misc_notebook_a01), © NVIDIA, CC-BY 4.0
- The bed frame, rails, fastener, helper support and floor extension are custom parts built from primitives in `task.py`, at the
  plan's and the fastener drawing's dimensions (SimReady's only beds are hospital beds; it has no wooden bed parts and no bed
  hardware).
- `mechkit.py` and `simready.py` are copied unchanged from other teams' open-source tasks in this repository, as noted above.
