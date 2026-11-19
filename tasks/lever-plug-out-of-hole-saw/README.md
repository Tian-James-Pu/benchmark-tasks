# Lever a jammed wood plug out of a 2-1/8 in hole saw through its side slots, turning the saw between pries

A 2-1/8 in bi-metal hole saw with slotted sides (modelled on the Milwaukee Hole Dozer 49-56-9668 and its Plug Jack slots)
has just bored through a 17.5 mm (11/16 in) pine board, and the pine plug is jammed in the cup, flush with the teeth. The saw
stands on its arbor in a hole in a 4x4 offcut screwed to the bench; a flat screwdriver lies beside it. The robot must get
the plug out.

The plug can only be reached through the two slots in the saw's sides. Each slot has a low step and a high step: the
screwdriver's shaft rests on a step's ledge as a fulcrum, the blade goes under the plug, and pressing the handle down
lifts the plug. Lifting it from one side only tips it crosswise in the cup and wedges it (in the model: 6 mm of net tip).
It walks out square only if the pries alternate between the two slots, so one hand holds the saw and turns it half a turn
between pries while the other levers. Once the plug is high in the cup, the high step's ledge is the one that still
reaches under it.

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: scene, the jam, scoring (`score`, `success`) and the closed-loop demo (`demo_script`) |
| `mechkit.py` | Mechanisms and custom parts for a task file (the saw, plug and post), with the per-step and reset hooks |
| `simready.py` | Loads SimReady props into a task file (`SimReady(...)`), downloading them once from Hugging Face |
| `demokit.py` | Runs a task file's closed-loop `demo_script` on the harness's own embodiment |
| `task.json` | The task's details for the board |

## Requirements

`inspect-robots` with the `yam_arena` embodiment (Isaac Lab Arena), a GPU, and network access the first time (the
SimReady props download into `~/.cache/simready`).

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The wood plug from the last hole is stuck in this hole saw. Get it out so the saw is ready to cut again." \
  --scorer task_score --max-steps 2400
```

Nothing is randomised (`object_jitter=0`), so every seed gives the same scene. Runtime: the scripted demo reaches full
credit after 101.75 s of simulated time (2,035 steps). On a shared GPU with cameras a step takes 0.1 to 0.8 s, so a
2,400-step trial takes 5 to 30 minutes of wall time.

## The demo

```bash
python demokit.py task.py OUT                                     # the full-score run, with video (GPU)
python demokit.py task.py OUT_checks --no-record --mode none,onearm,noturn   # the failure checks
```

`demo_script` is closed loop. It reads the screwdriver's pose, the saw's pose and the plug's height (all visible to a
camera), never the scorer's state.

1. The right hand picks the screwdriver up by its handle, with the fingers across it.
2. The left hand takes the saw by the band between the slots' tops and the teeth and turns it half a turn in the post.
3. Pry: aiming in the saw's own frame (the saw sits a few degrees off square in its post), the right hand points the blade
   along the slot facing it and brings the blade tip to just outside the slot facing it, at the step the plug can still be reached
   from (the low step while the plug's back face is below the high ledge, then the high step), slides it in under the plug
   beside the pilot bit, rests the shaft on the ledge, and swings the handle down about the ledge in 1-degree steps until
   the plug has come out to this pry's goal (3, 11, 19 and 26 mm) or 60 degrees. It then swings back, lifts the blade
   off the ledge and draws it straight out. Before each pry it levels the blade again.
4. Between pries the left hand lets go, turns back, takes the saw again and turns it half a turn, so the other slot faces
   the screwdriver.
5. After four pries the plug is more than 23.5 mm out, off the pilot bit. The right hand heads back to put the
   screwdriver down and the left hand lets go of the saw and slides off sideways, then both move clear. (In the recorded
   runs the blade caught in the slot on the way out of the last pry and the screwdriver was left resting in it; the
   score doesn't depend on where the screwdriver ends up.)

Results (this revision; `full` from the final command above on GPU physics with cameras, the checks with `--fast`, CPU physics):

| run | what the script does | score |
|---|---|---|
| `full` | as above | 1.0, twice (plug 30 mm out; full credit at step 2,035, 101.75 s) |
| `none` | does nothing | 0.0 |
| `onearm` | right hand only: four pries through the slot that faces it, never holding or turning the saw | 0.0 (plug 3 mm out) |
| `noturn` | the left hand holds the saw but never turns it: four pries through the same slot | 0.25 (plug 6.8 mm out, then wedged) |

## Scoring

The scorer runs every control step (`Trial.step`, a mechkit `on_step` hook) and keeps its own state, cleared only by
mechkit's `on_reset` hook.

- **1.0**: the plug's travel (how far its front face stands out past the tooth tips) is at least 23.5 mm, so its back face
  has reached the pilot bit's tip and it lifts free, with every fingertip point of both grippers (each finger's tip and
  3 cm up it, from `mechkit.fingertips`) outside the plug's cylinder grown by 2 cm all round, for 2 s running (40 control
  steps). Once reached, the score stays 1.0.
- Otherwise the travel at the end: **0.75** at 17.5 mm or more (the back face out past the teeth), **0.5** at 11 mm or more,
  **0.25** at 6 mm or more, **0** below 6 mm.

The saw, the screwdriver and the distractors are not scored.

## The model

- **Saw** (`saw`): one free articulation of 280 g (cup, back plate, teeth, pilot bit, arbor body and hex shank, built
  from boxes and cylinders, with hull colliders for every wall segment so the slots are real openings). It stands in the
  post's hole and can turn, tip and be lifted out.
- **Plug**: a link on a slider along the saw's axis (`slide`, 0 to 30 mm) and a small hinge (`plug`, plus or minus 4
  degrees) that shows how far it is tipped. It has no collider: its joint guides it, and fingers pass through it.
- **The blade's push** is a contact written in code, each control step, because PhysX's GPU contact between the held
  screwdriver and the light jointed plug let the blade pass through it at a few tens of newtons. Wherever the top of the
  screwdriver's blade or shaft is under the plug's back face (inside its 51 mm circle), the overlap d gives a force
  min(30 N/mm x d, the jam's hold) along the saw's axis, applied down on the screwdriver at that point and, equal and
  opposite, up on the saw (the plug's grip on the cup). So the robot has to push the blade up against the full force, and
  the saw feels the reaction, as with a real contact.
- **The jam** is a friction law on that contact: when the push passes the hold, the plug slides out (at most 30 mm/s)
  until the push is back at the hold. The hold is 60 N x (length still in the cup / 17.5 mm) + 1.5 N (the pilot bit);
  past 23.5 mm (off the bit) it is 0.3 N.
- **Tipping**: while the screwdriver's blade is inside the cup, new travel adds to the plug's tip on the side the handle
  comes from. Once the net tip reaches 6 mm, travel pushed from that side is held with up to 150 N (wedged) while
  the plug is still in the bore; travel pushed
  from the other side moves it again. The hinge shows the tip.
- **Post**: a fixed pine block with a square 14 mm hole.
- **Screwdriver**: the SimReady prop's mesh for the looks; it collides as four boxes fitted to that mesh (the grip a 3.4 cm
  square bar, where the real handle is round and 3.8 cm at its widest; the neck; the 5 mm shaft; the blade 7.8 mm wide and 3.5 mm
  thick, where the real one tapers from 4.4 mm to 1.4 mm). Its mesh collider gave different contacts on CPU and GPU physics,
  and a round grip rolls between the finger pads.

## Real trials

### The build

- A Milwaukee 49-56-9668 2-1/8 in Hole Dozer hole saw with arbor and its 1/4 in pilot bit.
- A pine 4x4 offcut, 10 cm long, with a 14 mm hole 32 mm deep drilled in one end grain face, screwed to the bench from
  below with two 3 in screws at (24, 8) cm.
- A 6.5 mm flat screwdriver about 22 cm long, a speed square, a utility knife (blade retracted) and safety glasses on
  their marks.
- Pine boards 17.5 mm (11/16 in) thick for the plugs.

### Preparation

Bore a hole straight through a board with a drill, stopping as soon as the plug breaks free, so the plug stays in the cup flush with the teeth (press it flush with a wood block if it stands proud). Take the saw off
the drill with its arbor and stand the shank in the post with a slot facing the right arm. Check that the plug stays in
when the saw is held teeth-down and shaken; bore another if it falls out.

### Measurement

Measure the travel with a steel rule standing on the tooth tips beside the plug (or from 60 fps side video against a
millimetre scale), and the hands' clearance from the video. The 2 cm clearance can be judged against the plug's 51 mm
diameter.

### Safety

The robot works with a flat screwdriver and a hole saw that is off the drill: nothing is powered. The teeth are sharp:
nobody's hands go near the saw while the arms move, and the operator resets the rig with gloves.

## Assets

The screwdriver and the distractors come from the Hugging Face dataset
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`handTool_screwdriver_slotted_65mm_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_screwdriver_slotted_65mm_a01)
- [`measurement_ruler_speedsquare_a02`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/measurement_ruler_speedsquare_a02)
- [`handTool_knife_retractableBlade_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_knife_retractableBlade_a01)
- [`accessory_eyeglasses_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/accessory_eyeglasses_a01)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time: masses
are set (screwdriver 120 g, speed square 150 g, utility knife 120 g, glasses 30 g), and the screwdriver's mesh collider is
switched off and replaced by fitted boxes (see "The model"). The files are not committed;
`simready.py` downloads them into `~/.cache/simready` the first time the scene is built.

The hole saw, its arbor and pilot bit, the plug and the post are primitives built in `task.py` (SimReady has no hole saw).
`mechkit.py` and `demokit.py` are copied unchanged from other open tasks in this repository (`tasks/heat-shrink-lug/` and
`tasks/start-car-with-jammed-steering-lock/`).

## Importance and sources

- The saw: Milwaukee's page for the [2-1/8 in Hole Dozer bi-metal hole saw with arbor, 49-56-9668](https://www.milwaukeetool.com/products/details/2-1-8-hole-dozer-bi-metal-hole-saw-with-arbor/49-56-9668)
  gives the 2-1/8 in diameter and 1-5/8 in cutting depth and describes "Plug Jack, an all access slot design for fast
  plug removal".
- Why plugs jam and how they are pried out: the background of US patent [9,186,731 B2, "Apparatus for removing plugs"](https://patents.google.com/patent/US9186731B2)
  says the plug "remains tightly bound within the interior of the hole saw", that the slots "may also be used to insert
  a screwdriver, nail or the like to wedge into the plug and force it out", and that the process "can be very time
  consuming".
- Tradespeople on the job: the Fine Homebuilding forum thread [stuck holesaw plugs](https://finehomebuilding.com/forum/stuck-holsaw-plugs)
  describes prying through the slots ("Pry about 3 times and the plug pops right out"), prying from opposite slots,
  and warns that pushing on one side only means "you are just walking the wedge from side to side".
- The occupations: the US Bureau of Labor Statistics counts about 889,700 jobs for
  [carpenters](https://www.bls.gov/ooh/construction-and-extraction/carpenters.htm) and about 821,000 for
  [electricians](https://www.bls.gov/ooh/construction-and-extraction/electricians.htm) (2025). Both bore holes with hole
  saws as part of regular work (door hardware, boxes, cable and conduit runs).

What the sources do not give: the slot steps' exact sizes, the force that holds a jammed plug, and the 6 mm of tip that
wedges it. Those are this model's estimates.
