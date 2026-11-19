# Reassemble a washed desk fan: blade onto the shaft pin, reverse-threaded knob, clip-on front grille

A 12 in oscillating desk fan lies on its back on the table after its grilles and blade have been washed and dried.
The rear grille is already back on. The robot has to put the fan back together: push the blade onto the motor
shaft so the shaft's cross pin drops into the slot across the back of the blade's hub, screw the blade knob onto
the shaft's reverse-threaded end, and fit the front grille onto the rear grille and close its four grille clips.
The instruction gives only the goal: "Put the fan back together."

Every one of those steps is a physical contact in the simulation, not a rule:

- **Shaft, pin and hub.** The 8 mm motor shaft is a colliding cylinder on a free, lightly dragged bearing (an
  unlimited hinge). Its 2 mm iron cross pin is a colliding cylinder. The blade's hub is built from 32 colliding
  sectors around an 8.8 mm bore, with the 2.6 mm x 4 mm cross slot cut out of its back layer. Put down with the slot
  turned off the pin, the hub rests on the pin 4 mm high; it seats only when it is turned until the slot meets the
  pin. Nothing places it.
- **Reverse thread.** The shaft's end and the knob's bore are real left-hand thread forms (1.5 mm pitch) kept
  concave as signed-distance-field colliders. The knob goes down only by being turned anticlockwise (seen from the
  front). The shaft turns freely, so turning the knob also turns the shaft, and with it the blade (the pin is in the
  slot), unless the blade is held.
- **Front grille and clips.** The front grille's locating skirt fits inside the rear grille's rim band. Each of the four
  grille clips is a hinged link on the front grille's rim with a colliding arm and hook. A clip closes only when a
  finger swings it 75 degrees in, and its hook then passes under the rear grille's lip, which it can do only when the
  front grille is seated: on a grille that is not seated the hook strikes the lip. The clip's plastic snap is an
  over-centre spring (see Fidelity): it pushes back against the finger until it is past 40 degrees and then pulls the
  clip home against its stop.

## Files

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: scene, scoring (`score`, `success`) and the scripted `demo()` |
| `task.json` | The task's details for the board |
| `mechkit.py` | Mechanism kit (articulated parts, colliders, joints, fingertip points, step and reset hooks) |
| `simready.py` | Loads NVIDIA SimReady props into a task file, downloading them once from Hugging Face |

`mechkit.py` is taken unchanged from another team's open-source task in this repository (`tasks/heat-shrink-lug/`);
`simready.py` is adapted from `tasks/cutters-off-pegboard/`.

## Run a trial

On the YAM Brev launchable, from the repository root, with the Arena environment active:

```bash
# model run (Dream-1)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/reassemble-table-fan-after-washing/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Put the fan back together." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/fan-dream

# scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/reassemble-table-fan-after-washing/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=640 -E cam_height=360 \
  --instruction "Put the fan back together." \
  --scorer task_score --max-steps 1850 --store-frames --log-dir ~/yam-eval/fan-demo
```

The run prints `task_score` at the end.

## Runtime and seeds

DEMO_RUNTIME

`object_jitter=0` keeps the layout fixed; nothing is randomised, so no seed is needed. The scripted demo is open loop
and deterministic.

## The scripted demo

A fixed plan from the published starting layout (open loop; it reads nothing from the simulator):

1. The left gripper takes the blade by its boss, carries it over the fan and lowers it with its slot deliberately
   turned 8 degrees anticlockwise of the pin, so that the counterbore in the hub's back goes round the shaft's end.
   It lets go there; the blade slides down the shaft and comes to rest on the pin, 4 mm high.
2. The left gripper, fingers closed, comes down behind the blade that points left and pushes it clockwise round the
   shaft until the slot drops over the pin. It then stays there, stopping the blade from turning anticlockwise.
3. Meanwhile the right gripper has picked up the knob. It lowers the knob onto the shaft's end and screws it on
   anticlockwise in five 210-degree strokes, letting go and turning back between strokes, until the knob is down
   on the hub's boss. The left gripper's fingers keep the blade, and so the shaft, from turning with the knob.
4. Both hands move away; the right gripper lifts the front grille by its centre boss, lowers it onto the rear grille
   so the skirt goes inside the rim, and lets go.
5. Each gripper presses two grille clips shut with its closed fingers, and both arms move clear.

The demo position offsets `CARRY_DX/DY` and `KNOB_DX/DY` (about 1 to 4 mm) compensate the arms' measured tracking
error when carrying a part to the shaft. They were measured in earlier runs of this same plan.

DEMO_RESULT

## Scoring

Checked every control step by a tracker in `task.py`; the score is the final step's.

- **Blade on:** the hub's back face within 1.5 mm of its seated height (8.9 cm above the table), its centre within
  1 mm of the shaft axis, tilted less than 3 degrees. A blade resting on the pin with its slot turned away sits 4 mm
  higher and does not count.
- **Knob down:** the knob's open end within 0.6 mm of the top of the blade's boss (12.3 cm above the table), centred
  within 1 mm, tilted less than 3 degrees. The thread is the only way down there.
- **Grille on:** the front grille's rim within 1.5 mm of its seated height on the rear grille's rim, centred within
  2 mm, tilted less than 2 degrees.
- **Clip shut:** within 3 degrees of fully shut (75 degrees from open).

| State at the final step | Score |
|---|---|
| Blade not on | 0.0 |
| Blade on | 0.25 |
| Blade on and knob down (front grille absent, or on with no clip shut) | 0.5 |
| Blade on, knob down, grille on, n clips shut (n = 1 to 4) | 0.5 + 0.1 n |
| All four clips shut, every condition above holding, all three parts still (under 1 cm/s and 0.2 rad/s) and both grippers clear (every fingertip point more than 20 cm from the shaft axis or more than 3 cm above the front grille), continuously for 2 s | 1.0 (and it stays 1.0) |

The tracker keeps no violation record; its only state is the 2 s hold counter and whether full credit has been
reached, and both are cleared only by mechkit's `on_reset` hook, that is by a reset of the scene, never because the
parts look right again. The clip springs are physics, not scoring, and live in a separate hook (`Snap`).

## Fidelity

- Rigid bodies in PhysX. Every part that the assembly depends on collides: the shaft, the cross pin, the hub's sectors
  and slot, the bore, both thread forms (signed-distance-field triangle meshes, so the thread is concave), the rear
  grille's band and lip, the front grille's skirt, rim, wires, rings and badge, and each clip's arm and hook.
- The shaft is an unlimited hinge with a small bearing drag (joint friction 0.004 and damping 0.004, our estimate:
  enough that the blade's own weight on the pin does not drag the shaft round, while a hand turns it easily).
- **Simplified:** the thread has no preload or tightening torque: "screwed down" means run down until the knob's
  open end is on the hub's boss. Plastic flex is not modelled: each clip's snap is an over-centre spring (a joint
  spring whose rest point is switched from open to shut when the clip passes 40 degrees, and back if it is pushed
  back past 40 degrees). That switch is the only state-dependent physics in the task; it never moves a part that a
  finger has not already pushed past the snap point, and the hook still has to pass under the rear grille's lip.
- The fan body, its rear grille and nut are one fixed body; the pillar and base are simple shapes behind it. Grilles
  are 24 wires and 3 rings each, not the real wire count.
- Sizes: only the 12 in blade is published; the others are estimates (see Importance and sources).
- Uniform plastic friction (0.35 to 0.6) is used for the parts; there is no force sensing.

## Importance and sources

Category: **Societal relatability**, a household job that owners of a desk fan do themselves.

- **How common the appliance is.** The US Energy Information Administration's 2020 Residential Energy Consumption
  Survey, Table HC7.1 (data released March 2023), counts 54.39 million of 123.53 million US homes (44%) using
  "floor or window fans": [EIA RECS 2020, HC7.1](https://www.eia.gov/consumption/residential/data/2020/hc/pdf/HC%207.1.pdf).
  *Inference:* the survey has no separate desk-fan row, so we take this portable-fan category as the closest measure
  of how many households own a fan of this kind; it is not a count of desk fans.
- **The real operation, from the manufacturer.** The owner's manual of the fan modelled here, the Utilitech TP30-1A
  12 in oscillating desk fan (Lowe's item 4891682), says under Care and Maintenance: "Disassemble grilles and blade.
  Grilles and blade may be cleaned with mild soap and water ... Reassemble fan after grilles and blade have dried
  completely." Its assembly steps are the ones this task asks for: "Push the Fan Blade (D) onto the motor shaft,
  lining up the blade with the iron pin on the motor shaft's end. Tighten the Blade Knob (C)", then "Mount the Front
  Grille (A) to the Rear Grille (F) by positioning the Grille Clips (B) on the Front Grille over the Rear Grille.
  Secure the two grilles with the Grille Clips". Its parts list has four grille clips:
  [Utilitech TP30-1A manual (PDF)](https://pdf.lowes.com/productdocuments/799cbd8f-86f7-4a26-9d18-a290c56f235f/68696390.pdf).
- **The reverse thread and the pin slot, on a comparable fan.** The TP30-1A manual does not give the knob's thread
  direction. The Comfort Zone CZ121 12 in table fan's manual does: "Thread blade cap onto threaded end of motor shaft.
  Note: Threads are reversed at this connection. Tighten=Counter-clockwise", and "the pin on motor shaft should fit
  into slots of fan blade": [Comfort Zone CZ121 manual (PDF)](https://images.homedepot-static.com/catalog/pdfImages/56/5696792a-3273-43b3-a7fa-6015a152397b.pdf).
  *Inference:* we give the TP30-1A's blade knob the same reversed thread. (The CZ121 itself uses three clips on its
  rear grille; the task follows the TP30-1A's four clips on the front grille.)
- **Product data used.** The 12 in (30.5 cm) blade is the model's stated size. The manuals give no other dimensions,
  so the shaft (8 mm), pin (2 mm), slot, bore, thread pitch (1.5 mm), knob, grille (34.8 cm) and masses are our
  estimates for a fan of this size, labelled as such in Fidelity.
- **Why it is physically hard** (our reasoning, not a sourced claim). The pin is hidden under the hub once the blade is
  on the shaft, so the slot has to be found by turning the blade while it rests on the pin; the shaft turns freely, so
  a hand that only turns the knob tends to turn the shaft and blade with it unless the blade is held; the knob's
  reversed thread has to be started square on an 8 mm shaft end; and each of four clips must be swung shut after the
  grille is seated, from both sides of a 35 cm grille.

## Assets

[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, © NVIDIA, CC-BY 4.0. Both are distractors, which scoring ignores:

- [`cleaning_sponge_b01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/cleaning_sponge_b01):
  kitchen sponge, mass set to 20 g, convex-hull collider.
- [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01):
  ceramic mug, mass set to 300 g, convex-hull collider.

Files download at first run into `~/.cache/simready`; nothing large is committed. The fan (body, rear grille,
shaft, blade, knob, front grille and clips) is built in `task.py` from generated meshes, because SimReady has no
desk fan whose blade, knob, shaft and clips are separate working parts (its fans, such as `climate_shopFan_pedestal_a01`,
are single rigid props).

## Relation to an earlier submission

This is an independent implementation of an idea first submitted in PR #2189, whose hold lapsed without a revision.
Its scene, mechanisms, scoring, demo and text are our own. The review of that submission asked for the following, and
this task addresses each point:

1. **Domain** (first review: not established as core work for cleaners). This task claims Societal relatability, makes
   no occupational claim, and supports the household job with the manufacturer's own care instructions and a
   household survey (Importance and sources).
2. **Construct validity: shaft, pin, notch and clips were visual-only, and state rules seated the blade and closed the
   clips.** Here the shaft, pin, hub slot and bore, the threads, the grille rims and the clips are all colliders (see
   the list at the top and Fidelity). No code places the blade, turns the knob, seats the grille or closes a clip:
   the blade seats only when its slot meets the pin, the knob goes down only by turning on the thread, and a clip
   shuts only when a finger swings it past its snap point with its hook clearing the lip. The red-team runs below
   show what happens otherwise (the blade left on the pin, clips closed before the grille is fitted).
3. **Resistance.** The clips push back (their spring holds them open until past 40 degrees); the knob needs about two
   and a half turns down a 1.5 mm thread; the free shaft turns with the knob unless the blade is held.
4. **Feasibility: a full-score demo of the physical assembly within the action limit.** The scripted demo above
   performs every step through contact and reaches 1.0 within the 2,400-step limit (see The scripted demo).
