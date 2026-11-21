# Hook a bed's side rail onto its headboard post: both hooks into their slots at once, then drive it down until seated

A solid-wood queen bed is being put together. The side rail's far end already sits in the footboard; its near end lies on the
floor beside the headboard post. The rail and the post are joined by a hook-in bed rail fastener: a steel plate on the rail's
end carries two hooks, and a slotted steel plate in the post's face takes them. To join them, a person lifts the rail's end,
lines both hooks up with both slots, pushes them in, lets the rail drop so the hooks go down behind the plate, and taps the rail
down so the hooks' tapered throats wedge the joint tight.

The instruction states only the goal: "Attach the side rail's loose end to the headboard post, fully seated."

**The knack.** The rail's end carries half of a 3.8 kg rail (about 19 N) and is held only near its top edge, so it sags and
twists in the hands. Both 4.0 mm hooks have to meet both 6.35 mm slots (1.2 mm of play each side, 82.5 mm apart one above the
other) at the same moment, at a height where each 22.2 mm hook fits through its 25.4 mm slot (a 3.2 mm window), while the rail is
pressed towards the post. A rail rolled by more than about 1.6 degrees puts one hook beside its slot. Then the joint has to be
driven down its last 3 mm against the wedge. One hand carries and steers near the end while the other carries further back and
keeps the rail plumb; together they lower it through the window and press it home. Done carelessly (end lifted only to its
seated height and pushed at the post), the hooks meet solid plate below the slots and never go in.

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: the scene, the fastener's working, the scorer (`score`, `success`) and the scripted `demo()` |
| `task.json` | The task's details for the board |
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
`-E object_jitter=0`). Runtime: the demo reaches full credit at step 545 (27.3 s of simulated time). With cameras, on GPUs shared
with other jobs, the demo runs here took 12 to 20 minutes of wall time (under 1 step a second); an idle card is several times
faster, so a 1,500-step model trial takes from about 5 minutes to over half an hour.

## The scene

All positions are in metres from the midpoint between the arm bases (x forward, y to the robot's left); heights are above the
table top, which stands in for the bedroom floor. `task.json`'s `setup` lists every object with its size, material and mass.

- **Headboard post** (fixed): hardwood, 76 x 76 mm, at x 0.262 to 0.338, y 0.070 to 0.146, cropped 212 mm up, with a stub of the
  headboard's bottom rail running away from the arms. Its face at y 0.070 carries the slotted plate, flush.
- **The fastener**: Rockler's 6 in x 5/8 in Heavy Duty Wrought Steel Bed Rail Fastener (item 28597; $19.99 for the four sets a
  bed needs). Every dimension comes from Rockler's installation drawing: both plates 5 15/16 in x 5/8 in x 9/64 in
  (151 x 15.9 x 3.6 mm); hooks 5/32 in (4.0 mm) thick, standing 7/16 in (11.1 mm) out, 7/8 in (22.2 mm) tall with a 3/8 in
  (9.5 mm) tip below the neck; slots 1/4 in x 1 in (6.35 x 25.4 mm); hook mortises behind the slots 1/4 in wide and 1/2 in deep.
  The hooks' spacing (82.5 mm) and positions on the plate are scaled from the drawing. Seated, the plates' ends line up and the
  hooks' necks rest on the slots' bottoms, as in the drawing.
- **Side rail** (free body): poplar, 25.4 x 165 mm (a 1 in x 6 1/2 in rail, deep enough for the 151 mm plate), 2.03 m long for a
  queen bed, 3.8 kg at 450 kg/m^3. Only its near 0.62 m is shown. It has the whole rail's mass, with its centre of mass halfway
  between its end and the cradle's bar, so its end carries 18.6 N, as the whole rail's end does, and its effective inertia at
  the end (a third of the mass) is the whole rail's too.
- **Footboard end** (fixed): a steel cradle standing in for the footboard and the rest of the rail. The rail lies on a round bar
  (its top 35 mm up, the rail's seated height) between two cheeks 1.5 mm clear of it, so it can pitch, slide along its length
  (up to an end stop 25 mm behind its cut end) and swing a few degrees, as a rail hooked into a free-standing footboard can.
- **Start**: the rail's near end rests on the floor 30 mm short of the post and 25 mm towards the arms, the rail sloping 3.7
  degrees down to it.
- **Distractors** (SimReady, loose, ignored by scoring): a coffee mug, a Phillips screwdriver and the assembly booklet, on the
  floor at least 20 cm from the work.

### How the joint works in the simulator

The hooks and the plate are real-size convex colliders, so a hook goes into its slot only where it really fits: within 1.2 mm
sideways of the slot's centre, at a height where the whole 22.2 mm hook clears the 25.4 mm slot, and with the rail's end
pressed towards the post. Each hook's tip has its real sloped inner face, so as the hook drops, the plate's edge under the slot
draws the rail's end against the post (the throat is 4.0 mm, 0.4 mm more than the plate is thick). The real fit is an
interference wedge over the last few millimetres, which a rigid simulator cannot model. In its place, under each hook's tip in
its mortise sits a stop 3 mm short of the seated height, on a stiff spring (20,000 N/m), which gives way only to more than 16 N
on that hook and stays where it is pushed. The rail's own weight (about 9 N a hook) leaves it there, and a firm push seats it.
When a hook comes out of its slot, its stop springs back up. The stops are contact-driven: only a hook standing on one can push
it down.

## Scoring

`--scorer task_score` gives 1.0 for success, otherwise the score at the end. The task file sets `SUCCESS_HOLD_S = 0` and counts
its own hold. Each rule below is what `score()` and `success()` compute (`Trial` in `task.py`); `task.json`'s `scoring` text says
the same.

Read at each of the two hooks:
- **s**: the rail end's height above its seated line (the height it has when both hooks are fully seated).
- **gap**: the rail's end face to the post's face, at that hook's height.
- **e**: the hook's sideways offset from its slot's centre.

A hook is **in** when its gap is 7.5 mm or less, s is 9.5 mm or less and e is at most 3.2 mm either way. A hook stands 11.1 mm
out of the rail's end, so a gap of 7.5 mm or less means it is 3.6 mm or more past the post's face, which it can be only through
its slot; s of 9.5 mm or less means its tip is below the slot's bottom, behind the plate, so it cannot come straight out.

- **0 for the whole trial** if at any time:
  - the rail end is **dropped**: a point near its end falls faster than 0.5 m/s (about a 1.3 cm free fall) while every fingertip
    point of both grippers is more than 2 cm from the rail; or
  - it is **forced into the post**: its end 1.5 mm or more past the post's face, a hook in its slot 1.5 mm or more below its
    seated height, or a hook 1.5 mm or more past the post's face while more than 2.7 mm off its slot's centre (which only a hook
    driven into the plate beside the slot can be).

  These are recorded as they happen and cleared only by a reset (mechkit's `on_reset`).
- **1.0** (success, ends the trial): both hooks seated (s and gap each 1 mm or less at both hooks) for 3 s running, with every
  fingertip point of both grippers (`mechkit.fingertips`: the tip and 3 cm up each finger, at its middle and both edges) at least
  3 cm from the rail, and the rail still (under 1 cm/s and 0.1 rad/s).
- Otherwise, at the end:
  - **0.75**: both hooks in, with s 4.5 mm or less at both: hung on the wedge fit but not driven home.
  - **0.5**: both hooks in.
  - **0.25**: one hook in.
  - **0**: anything else, including doing nothing.

Lifting, tilting or regripping the rail is not penalised, nor is setting it down gently (the drop rule needs the hands off it).
Credit does not stick: the partial scores are read at the end, and full credit needs the hold.

## The demo

`demo()` is a scripted reference for `--policy yam_scripted`, not a policy. Both hands come down from above at 45 degrees, the
left near the rail's end (13 cm from it, leaning away from the post) and the right 25 cm from it, and close across the rail's
top edge. Then:

1. They lift the end to 18 mm above its seated height, in line with the post and 13 mm short of it, and settle.
2. They press it towards the post, so the hooks rest against the plate above the slots.
3. Still pressing, they lower it 16 mm over 4 s. As the hooks come level with the slots (s from 12.7 to 9.5 mm) they slip in,
   the rail's end goes to the post, and the hooks drop behind the plate onto the wedge stops (s about 2.7 mm).
4. They drive it on down until seated, ease off, let go, back off along their fingers (still open either side of the rail), rise
   and move back towards the robot over the rail (the right hand turns its grip 30 degrees about the vertical, which its reach
   needs up there).

The YAM's arms are soft (shoulder joints 80 N m/rad, wrist joints 10 N m/rad), and under this rail they sag 4 to 9 cm and the
rail twists 3 to 5 degrees in their grip. So while they carry or press the rail, the demo corrects its commands from what the
cameras would see: the rail's pose. It integrates the error between where the rail's end should be and where it is, shifts
both hands sideways to put the hooks on the slots' centre line, and rolls both grips to bring the two hooks into one vertical
line (`Servo` in `task.py`). The demo never reads the scorer's state.

Results (scripted demo through `yam_scripted` and `--scorer task_score`, GPU physics with cameras, this scene):

| run | what the script does | score |
|---|---|---|
| `full` | as above | 1.0, twice in a row, both at step 545 (27.3 s) |
| `nothing` | no plan: both arms stay at their ready pose | 0 |
| `onearm` | the left hand alone does everything | 0.75: it lifts the end and hangs the rail on both hooks, but alone it cannot drive the joint down its last 3 mm |
| `careless` | lifts the end only to its seated height (below the slots' window), pushes it at the post and lets go | 0: the hooks meet the plate below the slots and never go in |
| `nodrive` | hangs the rail on its hooks but never drives it down | 0.75 |

The red-team runs used copies of `task.py` with `DEMO_MODE` set to the run's name (deleted afterwards; set `DEMO_MODE` in a copy to
repeat them).

**Model run.** Dream-1, given the instruction above for the full 1,500-step run limit (the first command under "Run a trial"),
scored 0: it brought both grippers to the rail but got neither hook into its slot.

## Importance and sources

- Wooden bed frames are taken apart and put back together with almost every move, and moves are common: using 2007 American Community Survey
  data, the US Census Bureau estimates that "a person in the United States can expect to move 11.7 times in their lifetime"
  ([Calculating Migration Expectancy](https://www.census.gov/topics/population/migration/guidance/calculating-migration-expectancy.html)).
- The fastener: Rockler's Heavy Duty Wrought Steel Bed Rail Fasteners, 4-pack, 6 in x 5/8 in, item 28597, $19.99, "one pack of
  four fasteners will complete a bed"
  ([product page](https://www.rockler.com/heavy-duty-wrought-steel-bed-rail-fasteners-4-pack-select-size)), and its installation
  drawing with every dimension used here
  ([6" Bed Rail Fastener Installation Instructions, PDF](https://go.rockler.com/tech/6-bed-rail-fastener-installation-instructions.pdf)).
- Tapping a hooked-on rail down to lock it is the maker's own step in bed assembly instructions, for example Room & Board's
  Hartley bed: "lift one end into footboard post by slipping connecting posts of side rail into interlocking keyhole on
  footboard bracket. Tap rails with rubber mallet to ensure locked position"
  ([Hartley Bed Assembly, PDF](https://www.roomandboard.com/images/pdf/Hartley_Bed_Assembly.pdf)).

## Distinct from

| Task | What it tests | How this one differs |
|---|---|---|
| #1989 Refit a car headrest at its lowest notch (accepted) | two vertical posts slid down into two sleeves, then a held release button | Here a long cantilevered rail is carried by one end, pushed sideways into a wall plate through two narrow slots at once, dropped behind it and wedged home; there is no button and nothing slides down a sleeve |
| #1775 Refit a loaded drawer on its roller slides (accepted) | carrying a drawer high so its rear hooks clear, then running it in | No slots or wedge there; here both hooks must meet slots 1.2 mm wide each side in a 3.2 mm height window |
| #2153 Refit a fridge door bin (accepted) | carrying a bin in level above two hidden supports, lowering both ends on together, pushing it home | Nothing passes through a slot or is wedged there, and the light bin is carried whole; here one end of a long, heavy rail is carried while its other end is held elsewhere |
| #1946 Install hook-in gondola brackets (not accepted) | hooking shelf brackets into a slotted upright | A light bracket hooked by its own tabs; here the hooks are on the end of a heavy rail whose far end is held elsewhere, and the joint must be wedged down |
| #2917 Move a fridge shelf up a slot (not accepted) | tipping a glass shelf so its rear hooks swing into slotted supports | That shelf is tipped so its hooks swing in and drop, with nothing to drive home; here the hands carry 19 N at the end of a rail pivoting about its far end, hold both hooks in a 3.2 mm height window while pressing sideways into the post, then drive the joint down its last 3 mm against the wedge |
| #2385 Hang a cutlery caddy on a dish rack's rail (not accepted) | dropping a light caddy's two hooks over a wire rail | Hooks dropped over an open wire; here the hooks pass through closed 6.35 mm slots and are then wedged |

No accepted or pending task on the board joins a bed's rail to its post, and the external sets named in the guide (LIBERO,
RoboCasa, BEHAVIOR, SimplerEnv) have no hook-in rail fastener.

## Real trials

**Build.** One headboard post (or a 3 x 3 in hardwood post on a weighted base, as a stand-in cropped above the joint), one side
rail at least 6 1/2 in deep, and one set of the Rockler fasteners, mortised per Rockler's drawing: the slotted plate flush in
the post's face, its slots' bottoms 69 and 152 mm above the floor, the hook plate flush and centred in the rail's end. The rail's
far end goes into the footboard (or onto a cradle like the simulated one: a 20 mm round bar 35 mm up between two cheeks, which
lets it pitch and slide). Clamp the post's base to the floor so it cannot tip. Cost: the fasteners ($19.99) plus lumber; a
spare bed works as it is.

**Preparation and the seated line.** Hang the rail and tap it fully home once. Mark the post at the rail's top edge (a pencil
line) and put tape marks on the floor for the start: the rail's end 30 mm short of the post and 25 mm towards the robot, its
underside on the floor.

**Reset (about 1 minute).** Pull the rail's end straight up (a firm pull or a tap from below frees the wedge), draw it back, and
set its end down on the tape marks.

**Measurement, before anyone touches the rail.** With a steel rule, read s at the rail's end (its top edge above or below the
seated line). With a feeler gauge or rule, read the gap between the rail's end and the post at the upper and lower hooks'
heights. A hook is in when its gap is 7.5 mm or less and s is 9.5 mm or less (a hook that missed its slot holds the rail's end
at least 11 mm off the post). Dropping is read from the video: a fall of more than 1.3 cm with no hand within 2 cm. The hands'
clearance (3 cm) is read from the video over the last 3 s. Protect the grippers' pads from the plate's edges with a strip of
tape on the plate's top and bottom ends if needed; the hooks never touch the pads.

## Fidelity

Rigid bodies, 240 Hz physics, 20 Hz control. The rail is cropped at 0.62 m and rests on a cradle in place of the footboard; it
has the whole rail's mass and end load but not its twist (a real 2 m rail twists a little along its length). The hooks' and
slots' geometry is real size; the wedge fit over the last 3 mm is a stop that gives only to more than 16 N per hook (an
estimate) rather than an interference fit. Friction is uniform per part (rail 0.6, post and plate 0.4, cradle 0.3). The
footboard's own movement is not modelled.

## Assets and licences

- SimReady props (NVIDIA, [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/)), from the Hugging Face dataset
  `nvidia/simready-assets` at revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, downloaded at first run (nothing large is
  committed). At load time their masses are set as listed and each is made one rigid body; the screwdriver collides as convex
  hulls.
  - `dishware_mug_a01` (the mug): [link](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01), © NVIDIA, CC-BY 4.0
  - `handTool_screwdriver_phillips_ph2_a01` (the screwdriver): [link](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_screwdriver_phillips_ph2_a01), © NVIDIA, CC-BY 4.0
  - `misc_notebook_a01` (the assembly booklet): [link](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/misc_notebook_a01), © NVIDIA, CC-BY 4.0
- The post, rail, fastener and cradle are custom parts built from primitives in `task.py` (SimReady's only beds are hospital
  beds; it has no wooden bed parts and no bed hardware).
- `mechkit.py` and `simready.py` are copied unchanged from other teams' open-source tasks in this repository, as noted above.
