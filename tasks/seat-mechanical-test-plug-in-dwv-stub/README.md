# Plug a 3 in drain stub-up with a wing-nut test plug

Before a new drain, waste and vent (DWV) system is covered up, the plumber plugs every open stub and fills the system
with water or air for the rough-in test. The plug for an open pipe end is often a mechanical "gripper" plug: a rubber
O-ring between two plastic plates on a carriage bolt, with a wing nut on top. Turning the wing nut draws the plates
together and squeezes the O-ring out against the bore until it grips.

The catch is that the wing nut's thread drags the whole plug round with it. A plug sitting loose in the pipe resists only
by its lip's friction on the pipe end, and that is less than the thread's drag. Turned on its own, the plug spins in the
pipe and never tightens. One hand has to hold the plug still while the other runs the nut down, about three turns, and
the plug has to stay seated square on the pipe end while the rubber takes hold. Once the O-ring bites, the plug holds
itself and both hands can let go.

The instruction is only: **Plug this drain pipe for the pressure test.**

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: scene, the plug's mechanism and thread, scoring (`score`, `success`) and the scripted `demo()` |
| `task.json` | The task's details for the board |
| `mechkit.py` | Mechanism kit (plain-text USD parts, joints, hooks), copied unchanged from `tasks/heat-shrink-lug/` |
| `simready.py` | Loads NVIDIA SimReady props, copied unchanged from the repository's standard loader (originally `tasks/cutters-off-pegboard/`) |

## Run and score a trial

From the repository root, in the Python environment that has `yam_arena` and `inspect-robots`:

```bash
# the scripted demonstration (two arms, open loop)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/seat-mechanical-test-plug-in-dwv-stub/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "Plug this drain pipe for the pressure test." \
  --scorer task_score --max-steps 1400 --store-frames \
  --log-dir ~/yam-eval/seat-mechanical-test-plug-in-dwv-stub/demo

# a model trial
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/seat-mechanical-test-plug-in-dwv-stub/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Plug this drain pipe for the pressure test." \
  --scorer task_score --max-steps 2400 --store-frames \
  --log-dir ~/yam-eval/seat-mechanical-test-plug-in-dwv-stub/model
```

The log reports `task_score`. The layout is fixed (`object_jitter=0`) and nothing depends on the seed. The scripted demo
reaches 1.0 and ends the trial after 1,226 control steps (61.3 simulated seconds: 58 s of work, then the 3 s check with
both hands away); the model gets 2,400 steps (2 minutes), about twice that. A demo trial took about 23 minutes of wall
time on a GPU shared with other jobs; it was not timed on an idle GPU.

## The scene

The table stands in for the support under a framed bathroom floor. Coordinates: x forward from the arms' bases, y left,
from their midpoint; heights above the table top.

| Object | What, size, material | Where |
|---|---|---|
| Subfloor | 23/32 in plywood, 60 x 84 x 1.8 cm | x 10 to 70 cm, y -42 to 42 cm |
| Stub-up | 3 in Schedule 40 PVC DWV pipe, OD 88.9 mm, bore 77.9 mm (3.068 in), white | centre x 30 cm, y 0; end face 45 mm above the plywood |
| Stud wall | spruce 2x4 (38 x 89 mm): bottom plate 80 cm long, two studs 42 cm tall at 16 in centres, a cross block 30 cm up | plate face at x 59.2 cm: the stub's centre is 30.5 cm (12 in) from the finished wall face |
| Test plug | Cherne 270237 3 in End-of-Pipe Gripper plug (an Oatey brand), see below | standing on its lower plate at x 21, y -15 cm, wings along 135 degrees |
| Distractor | 10 in groove-joint pliers, steel, 24.6 x 6.5 x 1.4 cm, 0.28 kg (SimReady) | x 47, y -30 cm, yaw 70 degrees |
| Distractor | retractable utility knife, 16 x 6.1 x 2.6 cm, 0.12 kg (SimReady) | x 48, y 27 cm, yaw 23 degrees |
| Distractor | thin blue marker, 10.9 cm, 0.01 kg (SimReady) | x 42, y 17 cm, yaw 69 degrees |

The distractors are loose, lie outside both arms' paths, and are ignored by scoring.

### The plug

| Part | Model | Source |
|---|---|---|
| Overall length | 63 mm (lip underside to wing tops 34 mm; body below the lip 29 mm) | listed 2-1/2 in long ([WebstaurantStore, Cherne 270237](https://www.webstaurantstore.com/cherne-270237-3-end-of-pipe-gripper-plug/835270237.html)) |
| Plates | yellow glass-reinforced ABS; upper plate with a lip 84 mm across and 6 mm thick over a 74 mm body, eight radial stiffening ribs on its top (3 mm wide, 4 mm tall, from 28 to 41.5 mm out); lower plate 74 mm with a chamfered edge | material from the same listing; "End-of-Pipe design won't fall in" ([Oatey](https://oatey.com/products/mechanical-test-plugs/gripper)): the lip is wider than the 77.9 mm bore and narrower than the 88.9 mm OD, sizes estimated |
| O-ring | black natural rubber, 75.5 mm OD relaxed (1.2 mm clear of the bore), 9 mm section | natural-rubber O-ring from the listings; size estimated |
| Bolt | galvanised 3/8-16 carriage bolt (1.5875 mm a turn); its end starts 8.0 mm down inside the nut's hub and rises as the nut draws it up | carriage bolt from the listings; thread size estimated |
| Wing nut | zinc, hub 19 mm, two wings 4 mm thick spanning 51 mm, 28 mm tall | "extra large rust-resistant zinc wing nut" from the listing; sizes estimated |
| Mass | about 0.18 kg | estimated (the 2 in model ships at 0.18 lb) |
| Rating | 17 psi / 40 ft of head back pressure; "for use inside pipe only, not for use inside the fitting (hub portion)" | [PEX Universe, Cherne 270237](https://www.pexuniverse.com/cherne-270237-gripper-end-of-pipe-plug) |

## How the simulation works

Everything that touches is a real collider: the pipe wall is 24 convex segments of the annulus, so the plug's plates and
O-ring sit in a real bore and its lip on a real end face.

- **The plug** is a free articulation. Its frame is the two plates (the lip included). Its links are the wing nut (an
  unlimited hinge about the plug's axis, hub and wings with their own colliders), the bolt end (a slider along the axis,
  visual only), and the O-ring as four quarter-rings, each on a radial slider with a 20 kN/m spring standing in for the
  rubber's give. The ribs on the upper plate are real colliders.
- **The thread.** The nut's joint drive is a friction clutch. Every control step its target is reset to where the nut
  is, and its torque limit is the thread's drag torque at the present draw-up: 0.20 N m rising to 0.25 N m while the
  free O-ring is squeezed, then 0.25 N m more for each further millimetre once it presses on the bore (about 1.1 N m at
  the demo's final draw-up of about 5.4 mm). Turning the nut therefore drags the plug round with up to that torque. Only rotation of the nut
  relative to the plug feeds the thread: draw-up = turns x 1.5875 mm, up to 6.5 mm (the O-ring squeezed solid), and
  the O-ring's rest radius grows by 0.6 mm for each millimetre of draw-up. The O-ring meets the bore after 2.0 mm of
  draw-up (1.3 turns); from about 2.3 mm its four sprung segments press hard enough on the bore that friction holds the
  plug against the nut without help. Turning the nut back past finger-snug just spins it loose.
- **What holds a loose plug.** The plug weighs about 1.8 N, so its lip's friction on the pipe end (friction 0.6) resists
  about 0.04 N m, a fifth of the thread's 0.20 to 0.25 N m: a plug nobody holds turns with the nut. A fingertip pressed on the
  lip between two ribs stops it (the demo's left hand), as a thumb on the plate does; pushing hard down on the plug also
  adds friction at the pipe end.
- **Not modelled:** water, pressure or leakage; force or torque limits of the hands (nothing is scored on force); the
  extra friction under the nut's face when someone pushes down on the nut (this makes a one-handed push-and-turn
  slightly easier here than in reality); rubber hysteresis. Friction is 0.6 static, 0.5 dynamic on the plug and the
  pipe (ABS and rubber on PVC; one material per part), the simulator's default on the robot's fingers.
- **Contact jitter.** With the O-ring squeezed, the solver's contacts make the plug's reported velocity flicker
  (about 0.02 m/s) while its pose does not change by a measurable amount, so "at rest" is judged from the pose over the
  last second, as a person watching would judge it.

## Scoring

The tracker in `task.py` reads the scene once per control step; `score()` and `success()` only return what it found.

- **1.0:** all of the following, continuously for 3 s:
  - the plug's body (its O-ring plane) is inside the stub-up: within 10 mm of the pipe's axis and more than 5 mm below
    its end;
  - expanded: the bolt end has risen at least 4.2 mm from its finger-snug start (about 2.6 turns of the nut, the
    O-ring then 1.3 mm larger than the bore when free);
  - seated square: at each of 12 points under the lip, 40 mm from the plug's axis, the lip is no more than 1.5 mm
    above the pipe's end face (and no more than 1.5 mm into it, a solver tolerance), and the plug tilts 2 degrees or
    less;
  - at rest: over the last 1 s no point of the lip has moved 1 mm, and the nut has turned less than 1 degree;
  - hands clear: every fingertip point on both grippers is more than 3 cm from the plug (a cylinder of the lip's 42 mm
    radius over the plug's full length).
- **0.5:** the body is inside the stub-up and the bolt end has risen at least 4.2 mm, but the 1.0 conditions are not
  all met (crooked or proud on the pipe end, still held, or moving).
- **0.25:** the body is inside the stub-up and the bolt end has risen less than 4.2 mm (pushed in but not expanded,
  including a plug that spun with its nut).
- **0:** anything else, including the plug left out of the pipe.

Possible scores: 0, 0.25, 0.5, 1.0. Credit is not latched: the score is the present state (the 3 s hold restarts if
anything is disturbed). Nothing that happened earlier needs remembering, since a plug that spun, or was set crooked,
shows it in its end state. An O-ring expanded outside the pipe (more than 2 mm of draw-up) is larger than the bore and
cannot be pushed in, so that route stays at 0.

## Distinct from

| Nearest task | What it tests | Why this is different |
|---|---|---|
| #2002 Dry-fit a slip coupling across a drain-pipe repair (accepted) | sliding a fitting along a pipe | no expanding plug, nothing turned; here a plug inside the pipe end must be stopped from turning while its nut is run down |
| #1574 Seat a tri-clamp and draw up its wing nut (rejected) | a wing nut drawing a clamp band round two flanges | the clamp cannot spin with its nut; here the threaded part sits loose in a bore and turns with the nut unless held |
| #2604 Pressure-test a water-filled pipe circuit (rejected) | reading a gauge and isolating a pump | the test itself, not the plugging; no mechanical plug |
| #2614 Reseal sparkling wine with a two-clamp stopper (rejected) | levering clamps over a bottle lip | a lever closure, not a threaded expansion seal |

Searched the board's list of all task PRs (accepted, pending and rejected) for test plug, plug, drain, stub-out, DWV,
wing nut, pressure test, pipe and gasket: none has an expanding pipe plug or a part that must be held against its own
nut's drag. LIBERO, RoboCasa, BEHAVIOR and SimplerEnv are household and kitchen sets with no plumbing test task that we
know of.

## Importance and sources

- **Occupation.** Plumbers, pipefitters and steamfitters, O*NET-SOC 47-2152.00: 510,600 workers in 2025. One of the
  occupation's task statements is "Fill pipes or plumbing fixtures with water or air and observe pressure gauges to
  detect and locate leaks" ([O*NET OnLine](https://www.onetonline.org/link/summary/47-2152.00); job count also in the
  [BLS Occupational Outlook Handbook](https://www.bls.gov/ooh/construction-and-extraction/plumbers-pipefitters-and-steamfitters.htm),
  which says they "test pressure to ensure that a pipe system is airtight and watertight").
- **Why every stub gets plugged.** The model code's drainage and vent water test (IPC 312.2, quoted in the North
  Carolina 2018 Plumbing Code interpretation) says that for a whole-system test "all openings in the piping shall be
  tightly closed, except the highest opening", and for sections "each opening shall be tightly plugged"
  ([NC Office of the State Fire Marshal](https://ncosfm.gov/plumbing/03122-drainage-and-vent-water-test/open)). A house's
  rough-in has a stub for every fixture, and each is plugged before the test and unplugged after it.
- **The product.** Cherne 270237 3 in End of Pipe Gripper plug: glass-reinforced ABS, natural-rubber O-ring, galvanised
  carriage bolt, extra-large zinc wing nut, 2-1/2 in long, 17 psi, $8.39
  ([WebstaurantStore](https://www.webstaurantstore.com/cherne-270237-3-end-of-pipe-gripper-plug/835270237.html)); "position
  the plug inside the pipe and rotate the metal wing nut to expand the rubber gasket"
  ([PEX Universe](https://www.pexuniverse.com/cherne-270237-gripper-end-of-pipe-plug)); Oatey's Gripper line:
  "End-of-Pipe design won't fall in" ([Oatey](https://oatey.com/products/mechanical-test-plugs/gripper)).
- **Why the knack matters.** A plug turned without being held only spins, so the test cannot start; one set crooked or
  proud on the pipe end seals on a tilt and can blow out under the 10 ft head the test puts on it. The model's thread
  drag and rubber stiffness are engineering estimates, not measured values (see "How the simulation works").

## Real trial: preparation, reset and measurement

- **Build (under $40):** a 3/4 in plywood offcut on the bench with a 3 in Sch 40 PVC offcut through a 3.6 in hole,
  fixed from below with a pipe clamp so its deburred end stands 45 mm proud; two 2x4 offcuts screwed to a 2x4 plate as
  the wall, its face 30.5 cm from the pipe's centre; a Cherne 270237 (or Oatey 3 in Gripper) plug; the three distractors.
  Tape marks for the plug's footprint and wing direction and for each distractor.
- **Preparation and reset (about 2 minutes):** take the plug out, back the wing nut off until the O-ring is relaxed and
  the nut just finger-snug, and check with a depth gauge that the bolt end is 8.0 mm below the top of the nut's hub
  (record the actual reading if a different plug differs). Stand the plug on its mark, wings along the tape. Nothing is
  consumed; nothing sharp or hot is near the grippers.
- **Measurement, before anyone touches the scene:** the plug is in the pipe; depth gauge from the hub's top to the bolt
  end reads at least 4.2 mm less than at preparation; a 1.5 mm feeler gauge does not enter under the lip anywhere round
  it; a phone level on the lip reads 2 degrees or less; from the video, both grippers were more than 3 cm from the plug
  and the plug and nut still for the last 3 s.

## The scripted demonstration

`demo()` is open loop (it never reads the scene). The right arm pinches the wing nut's two wings above the hub, lifts the
plug, carries it slowly over the stub-up (it hangs from a pinch, so a fast carry swings it out of line), lowers the
lower plate into the bore and lets go 8 mm up, so the plug drops in and its lip seats on the pipe end. The left arm then
presses its closed fingers on the lip at the +y side, tilted 45 degrees out so its fingers stay clear of the wing nut and
of the right hand; the fingertips sit between two ribs. The right arm closes on the wings again and runs the nut down in
seven half-turn strokes, opening, lifting 12 mm and turning back between strokes. Both arms let go and move away, and the
plug is checked for 3 s.

## Validation

Two demonstration trials on the final `task.py`, each with the scorer's own end-state check and the contact sheet looked
at by eye:

| Trial | Score | Control steps | Wall time |
|---|---:|---:|---:|
| demo 1 | 1.0 | 1,229 | about 31 min (shared GPU) |
| demo 2 | 1.0 | 1,229 | about 33 min (shared GPU) |

An earlier trial with a weaker thread (0.08 to 0.12 N m before the O-ring meets the bore) also scored 1.0 in 1,226 steps.
Its log showed a 5.45 mm draw-up, the lip within 0.06 mm of the pipe end all round, and a 0.03 degree tilt. The plug
turned 15 degrees under the pressing fingers before a rib caught them, then did not turn again.

Checks of the failure routes:

| Route | What it does | Score |
|---|---|---:|
| Nothing | both arms stay away | 0 (plug left on the subfloor) |
| One arm, earlier thread | the right arm inserts the plug and turns; nobody holds it | 0.25 when stopped after two strokes: the plug turned about 150 degrees with each 180-degree stroke, draw-up 0.21 mm (1.59 mm if held) |
| Careless, earlier thread | the holding hand lets go after the first half-turn | 0.25: the plug then spun with the nut and the draw-up reached only 2.0 mm in seven strokes |

The one-arm and careless checks ran with the earlier, weaker thread drag (0.08 to 0.12 N m). The final drag (0.20 to
0.25 N m) couples a loose plug to its nut more tightly, so it should spin at least as readily. The one-arm,
careless and push-while-turning checks on the final file were queued but did not get a GPU in time, and have not been
run.

## Assets and licences

The plug, pipe, subfloor and wall are authored in `task.py` with mechkit primitives (no SimReady prop of a test plug or
a DWV pipe exists). The three distractors come from the Hugging Face dataset
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`handTool_pliers_groove_joint_10in_b01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_pliers_groove_joint_10in_b01): mass set to 0.28 kg; its hinged handles are welded into one rigid body at load time.
- [`handTool_knife_retractableBlade_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_knife_retractableBlade_a01): mass set to 0.12 kg.
- [`stationery_marker_thin_blue_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/stationery_marker_thin_blue_a01): mass set to 0.01 kg.

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Nothing large is committed:
the files download at first run. `mechkit.py` is another team's open-source mechanism kit from `tasks/heat-shrink-lug/`
in this repository, used unchanged; `simready.py` is the repository's shared SimReady loader, used unchanged.
