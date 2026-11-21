# Take the pin out of a closed door's bottom hinge with a nail set and a hammer

An interior door stands closed and latched in a painted partition wall, hung on three 3-1/2 in. loose-pin butt hinges.
The bottom hinge's pin is painted in and loaded sideways by the door's weight, so it will not pull out by hand. A nail
set and a claw hammer lie on the floor. The instruction is only the goal: **"Take the pin out of this door's bottom
hinge."**

This is the first step of taking any door off (to move furniture through, paint it, plane it or rehang it), and it is
done the way the guides say: drive the pin up from below with a nail set and a hammer, then pull it out by its head.
Family Handyman: "With the door closed and latched, remove the hinge pins by tapping on the bottom of the hinge pin with
a nail and hammer." ... "After pins pop up an inch or so, try pulling them free with your fingers."

The difficulty is where the barrel is. On a real door the hinge barrel sits against the door casing (a 3/16 in. reveal
leaves its edge 6.4 mm from the pin's axis) and in front of the door's face, and the pin's lower end is flush with the
bottom of the barrel. Two hands work in different roles. One hand keeps the nail set's 2.4 mm tip on the 5.94 mm pin's
end, pressing up so that it does not drop away, and leans it about 10 deg out of the corner. The other hand swings the
hammer upward into the nail set's head. If the nail set is held straight up, the 26 mm hammer face has no room: it
strikes the casing or the door before it reaches the nail set, which is the failure a hinge-pin-tool patent describes
("It is also easy to miss the nail and strike the molding around the doorframe, or even strike the door itself"). Once
the pin is loose the same hand that held the nail set pinches the pin's 10.8 mm head from the open side, away from the
casing, draws the pin 9 cm straight up out of the five knuckles, and lays it on the floor.

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: the scene, the pin model (`Trial`), `score()`, `success()` and the scripted `demo()` |
| `mechkit.py` | Props and parts written as plain-text USD (copied unchanged from `tasks/heat-shrink-lug/mechkit.py`, another team's open-source kit) |
| `demokit.py` | Closed-loop demo runner (copied unchanged from `tasks/close-mason-jar-two-piece-lid/demokit.py`); here it only replays `demo()` for CPU debug runs |
| `simready.py` | Loads NVIDIA SimReady props (copied from `tasks/cutters-off-pegboard/simready.py`, another team's open-source loader) |
| `task.json` | The task's details for the board |

## Run and score a trial

    source ~/IsaacLab-Arena/yam.env
    cd ~/benchmark-tasks      # this repository

    # the scripted demo (reaches 1.0)
    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=tasks/tap-door-hinge-pins-out-with-nail-set/task.py -E object_jitter=0 -E control_hz=20 \
      -E cam_width=640 -E cam_height=360 \
      --instruction "Take the pin out of this door's bottom hinge." \
      --scorer task_score --max-steps 1100 --store-frames --log-dir ~/yam-eval/hinge-pin-demo

    # a model run (Dream-1 or any policy), with the run limit
    inspect-robots run --embodiment yam_arena --policy dream \
      -E task=tasks/tap-door-hinge-pins-out-with-nail-set/task.py -E object_jitter=0 -E control_hz=20 \
      -E cam_width=320 -E cam_height=176 \
      --instruction "Take the pin out of this door's bottom hinge." \
      --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/hinge-pin-dream

`task_score` is 1.0 when `success()` has held for 3 s (the episode then ends); otherwise it is the last step's
`score()`. The scene is the same in every trial (`object_jitter=0`) and nothing in it is random, so no seed is needed.
The demo's timing is not measured yet (see "Results"). `run_limit` is 2,400 steps (2 minutes at 20 Hz), about 2.5 times the demo's time.

Environment variables (for checking only, never needed for a trial): `HINGE_DEBUG=1` prints the model's state every 10
steps and every blow. `HINGE_MODE` swaps the scripted demo for a red-team variant: `none`, `pull`, `straight` or
`onearm` (results below).

## Scale and mass

Nothing is scaled. The wall, the door, the casing, the baseboard, the hinges and the tools are full size. Only the
mounting is a stand-in: **the station table is the floor**. The partition, the door (bottom edge 1/2 in. above the floor)
and the trim stand on the table top exactly as they stand on a floor, so the relative geometry a person meets (the
bottom hinge 11 in. above the door's bottom edge, the casing beside the barrel, the floor under it) is the real one. At
that height the bottom hinge is in the arms' reach; the two upper hinges (7 in. below the door's top and centred) are
modelled for looks only and are out of reach, which is why the task is the bottom pin.

| Item | Real | Modelled |
|---|---|---|
| Door | 30 x 80 x 1-3/8 in. hollow-core flush slab, about 10 kg | same size; fixed (it is latched and hangs on two more hinges, so it does not move) |
| Hinge | 3-1/2 x 3-1/2 in., five knuckles, 0.085 in. steel leaves, 0.234 in. pin | same; barrel 10.8 mm across (estimated from the gauge round the pin) |
| Pin | 5.94 x 88.9 mm steel, button head, about 23 g | same, 23 g |
| Nail set | 3/32 in. tip, 4 in. long, 3/8 in. body, about 50 g | same, 50 g |
| Hammer | 16 oz claw hammer, 0.57 kg | SimReady tool_hammer_a02, 304 mm, its authored 0.567 kg |

No mass was reduced. The door's weight acts on the pin only through the model's sideways load (below).

## The scene

Positions are in cm from the midpoint between the two arm bases: x forward, y to the robot's left. Heights are above
the table (the floor).

| Object | What it is | Where |
|---|---|---|
| Partition | painted drywall over studs, 11.4 cm (4-1/2 in.) thick, 2.3 m high (fixed) | face at x 47.0 |
| Door | 30 x 80 x 1-3/8 in. (76.2 x 203.2 x 3.5 cm) hollow-core flush door, white, closed and latched, 1.27 cm off the floor (fixed) | y -76.4 to -0.16, swing face flush with the wall |
| Jambs | 11/16 in. pine, white; 1/8 in. gaps to the door | hinge jamb face at y 0.16 |
| Casing | 2-1/4 x 11/16 in. (5.7 x 1.75 cm) pine, white, 3/16 in. reveal | hinge side y 0.64 to 6.36, face at x 45.25 |
| Baseboard | 3-1/2 x 1/2 in. pine, white | along the wall either side of the casings |
| Hinges | three 3-1/2 x 3-1/2 in. five-knuckle steel butt hinges, satin nickel | bottom one: axis x 45.41, y 0.0, from 29.21 to 38.10 cm up; the others 7 in. below the door's top and centred |
| Pin | steel, 5.94 x 88.9 mm, 10.8 x 4.8 mm button head, 23 g; lower end flush with the bottom knuckle | in the bottom hinge |
| Nail set | 3/32 in. nail set: 102 mm, 9.5 mm knurled body, 25 mm taper to a 2.4 mm cupped tip, 50 g | on the floor, tip toward the hinge at (30.5, -12.6), head at (23.2, -19.8) |
| Hammer | 16 oz claw hammer, 304 mm, 0.567 kg (SimReady tool_hammer_a02) | on its side, handle near the head at (21.5, 21.5), face toward the wall |
| Distractors | multi-purpose oil can 5.2 x 10.4 x 27.2 cm, 0.40 kg; retractable utility knife 16 x 6 cm, 0.12 kg | (40, -33); (40, 36) |

The distractors are loose and the scorer ignores them. Neither can stand in for the nail set or the hammer.

## How the simulator judges it

**The bound pin.** Until it has been driven up 4 mm the pin is held where it is: its pose is written back after every
240 Hz physics step, so no pull, push or lever on it moves it. It moves only when it is struck through the nail set. A
blow counts when the hammer's face meets the nail set's head moving toward it along the nail set's axis at 0.3 m/s or
more (a new blow once the face has drawn 5 mm back), with the nail set's tip on the pin's lower end (within 2.0 mm of
the pin's axis, between 1.0 mm into and 1.5 mm below its end) and its axis within 20 deg of the pin's. Each blow raises
the pin by E / F_BIND, with E = 0.5 m v^2 of the hammer (m 0.567 kg, v that closing speed) and F_BIND = 60 N for the
paint and the door's weight. A steady push never drives it. The nail set's taper is real: tilted, its tip can follow the
pin only a few millimetres into the 6.1 mm bore before the taper meets the barrel, which is about what it takes to break
the pin loose.

**The loose pin.** Once it is up 4 mm the pin is an ordinary rigid body in the knuckles' 6.1 mm bore, pressed sideways
toward the jamb by 2 N (the door's weight on it). Friction keeps it where it is left, and a pull of a few newtons draws
it out. The sideways load stops once its lower end has left the top knuckle.

**Damage.** Any part of the hammer (its head and handle, as boxes) touching the door, the casing, the wall, the
baseboard or the hinge while moving at 0.10 m/s or more is a blow on the door or its trim. It is recorded for the rest
of the run and cleared only by mechkit's `on_reset` hook.

**Score** (`task.json` "scoring" says the same; r is how far the pin's head stands above the top knuckle):

| Score | When |
|---|---|
| 1.0 | the pin out of the hinge and lying on the floor (its middle within 12 mm of it), at least 3 cm from the hinge's axis, at rest, both grippers' fingertips at least 5 cm from it, no blow on the door or trim; held 3 s |
| 0.75 | the pin out of all five knuckles (its lower end at least 2 mm above the top knuckle, or its middle at least 15 mm off the hinge's axis) |
| 0.5 | r at least 30 mm |
| 0.25 | r at least 4 mm (the pin broken loose) |
| 0 | otherwise; doing nothing leaves r at 0 |

A blow on the door or its trim caps the score at 0.25 for the rest of the run. The levels are read from the state at
every step; none of them sticks once the state is gone, except the damage record.

## Results

Not validated yet; this folder is a work in progress and is not ready to submit. Three CPU-physics debug runs of the
plan (`python demokit.py task.py OUT --fast --mode full`, which plays `demo()` through `demo_script`) show the nail set
seated on the pin's end (tip within 1 mm, 11 deg lean) and one blow registered by the model (0.40 m/s, 0.045 J, pin up
0.75 mm). Still open: the SimReady hammer twists in the left gripper on the turn and on the blows, its handle end grazes
the left baseboard while it is brought in (a recorded blow on the trim), the measured blow energy is about a quarter of
what F_BIND was set for, and the pull and lay-down have not been reached. No GPU run, no full-score demo and no red-team
run have been made.

## Distinct from

| Nearest task | What it does | Why this one differs |
|---|---|---|
| TK-NVTZYW (#3094, pending) | takes a small cabinet door (30 x 20 cm) off two loose-pin hinges: one pin already out, the other pulled by its ball tip while the door is held | no tools and no stuck pin; here the pin is bound, the door is a full-size passage door that stays latched, and the work is driving the pin from below with a nail set and hammer in the corner between casing and door, then pulling it |
| TK-W437WY (#2206, accepted) | starts a nail with fingers and drives it down into a board | a downward blow onto work that supports the nail; here the strike is upward, the nail set must be pressed up against gravity, and it has to lean out of a corner so that the hammer clears the trim |
| TK-4KAT4L (#2612, pending) | sets a drop-in anchor in concrete by striking its setting tool downward | the same contrast: a tool resting on the work and struck from above, in open space |

## Importance and sources

**Category: Societal relatability.** Taking a door off its hinges is an ordinary household job: to move a sofa or a
mattress through a doorway, to paint or plane a door, or to rehang it. The guides all start with the bottom of the pin,
a nail or nail set and a hammer:

- Family Handyman, "How To Take a Door off the Hinges"
  (https://www.familyhandyman.com/doors/how-to-remove-a-door): "With the door closed and latched, remove the hinge pins
  by tapping on the bottom of the hinge pin with a nail and hammer." "After pins pop up an inch or so, try pulling them
  free with your fingers." Tools listed: "nail set or long nail".
- US patent 7,100,255, "Apparatus for removing door hinge pins" (https://patents.google.com/patent/US7100255): "A common
  technique for removing a door hinge pin is to hold a nail to the hinge pin slot on the bottom side and then hit the
  nail with a hammer to knock the hinge pin out of the hinge." "It is also easy to miss the nail and strike the molding
  around the doorframe, or even strike the door itself".
- Hinge layout: Rustica, "Standard Hinge Placement Explained" (https://rustica.com/rustica-standard-hinge-placement-explained)
  places the top hinge of a 1-3/8 in. door 7 in. below its top and the bottom hinge 9 in. above its bottom; layouts of
  10 and 11 in. are also common. This scene uses 11 in., which leaves the room under the barrel that the hammer needs.

## Real trial

**Setup.** Any interior partition, or a mock-up: a 30 x 80 in. hollow-core flush door (for example a primed hardboard
slab, about $60 to $90) hung in a 4-1/2 in. jamb on three 3-1/2 in. five-knuckle loose-pin butt hinges (Hager RC1842 or
any residential equivalent, about $5 each), with 2-1/4 in. casing at a 3/16 in. reveal and a 3-1/2 in. baseboard, the
door closed and latched. Put the robot so that the bottom hinge's axis is 45 cm in front of the arm bases' midpoint, on
the line between them, with the floor at the arm bases' level (or raise the door frame on a platform so that its floor
is at the table top). Tape marks for the nail set (3/32 in., 4 in.) and the 16 oz claw hammer as in the scene table.

**Preparing the bound pin.** Seat the pin, then brush one coat of latex trim paint over the barrel's seams and the
pin's head, and let it dry 24 hours. Check before each trial that the pin does not move when its head is pulled up by
hand with 20 N (a luggage scale on a loop of thread under the head). For repeated trials without repainting, a reusable
stand-in is a pin wrapped with tape until it is a push fit in the barrel; it qualifies only if it passes the same 20 N
hand-pull check before each trial.

**Reset (about 2 minutes).** Push the pin back down until its head sits on the top knuckle (repaint the seam or re-wrap
the stand-in pin), wipe the floor, and lay the nail set and hammer back on their tape marks.

**Measurement.** With a steel rule: how far the pin's head stands above the top knuckle (r), whether the pin is out of
the knuckles, and where it lies on the floor (at least 3 cm from the hinge's axis, measured on the floor). Look at the
casing, the door's face and the hinge for fresh hammer marks (dents or paint chips): any one is a blow on the door or
its trim. From the video: the grippers 5 cm clear of the pin and the pin still for 3 s.

**Rig safety.** The nail set and hammer are blunt; the YAM's pads touch only the nail set's knurled body, the hammer's
handle and the pin's head. Nothing is hot, sharp or wet. Protect the floor under the hinge with a sheet of hardboard.

## Assets and licences

- NVIDIA SimReady (`nvidia/simready-assets` on Hugging Face, pinned revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`),
  © NVIDIA, CC-BY 4.0, downloaded at first run into `~/.cache/simready`:
  - `tool_hammer_a02` (claw hammer), https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/tool_hammer_a02
    : authored mass 0.567 kg kept; loose; laid on its side.
  - `consumable_lubricant_oil_multi_purpose_a01` (oil can, distractor), https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/consumable_lubricant_oil_multi_purpose_a01
    : mass set to 0.40 kg; its two bodies welded into one.
  - `handTool_knife_retractableBlade_a01` (utility knife, distractor), https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_knife_retractableBlade_a01
    : mass set to 0.12 kg; its three bodies welded into one.
- The doorway (wall, door, jambs, casing, baseboard, hinges), the pin and the nail set are built in `task.py` from
  mechkit meshes to the dimensions above: SimReady has no door with loose-pin hinges and no nail set.
- `mechkit.py` and `simready.py` are other teams' open-source files from this repository, credited above.
