# Pin a shaft to a clevis: drive a started spring pin home with a pin punch and hammer

A steel clevis lies in a V-block with a 12 mm shaft in its hub. Hub and shaft are cross-drilled 4 mm, and a 4 x 22 mm
slotted spring pin has been started 3 mm into the hole, standing 19 mm proud. A 5/32 in roll-pin punch and a Vaughan
TC308 8 oz ball-peen hammer lie on the bench. The instruction is only the goal: **"Pin the shaft to the
clevis."**

The pin is held in its hole only by the spring of its own split wall, so it moves only when it is struck. Driving it
home takes two hands in different roles. One hand keeps the punch's pilot in the pin's open end and the punch square to the
hole. The other strikes the punch's head with the hammer, a hand's breadth above the work. Each blow drives the pin a
little further, and the punch has to follow it down. A blow on a punch that leans more than 3 deg, or that is half off
the pin's end, cocks the pin so that it binds. The hammer alone cannot finish the job: its 22 mm face cannot follow the
pin into the hole. Once the pin is nearly flush, the face lands on the hub and dents it. The job is done when neither
end of the pin stands proud of the hub and both hands are clear.

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: the scene, the pin model (`Trial`), `score()`, `success()` and the scripted `demo()` |
| `mechkit.py` | Props and mechanisms written as plain-text USD (copied unchanged from `tasks/heat-shrink-lug/mechkit.py`, another team's open-source kit) |
| `simready.py` | Loads NVIDIA SimReady props (copied from `tasks/cutters-off-pegboard/simready.py`, another team's open-source loader) |
| `task.json` | The task's details for the board |

## Run and score a trial

    source ~/IsaacLab-Arena/yam.env
    cd ~/benchmark-tasks      # this repository

    # the scripted demo (does NOT yet reach 1.0: see Status)
    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=tasks/drive-spring-pin-flush-with-starter-punch/task.py -E object_jitter=0 -E control_hz=20 \
      -E cam_width=640 -E cam_height=360 \
      --instruction "Pin the shaft to the clevis." \
      --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/drive-spring-pin-demo

    # a model run (Dream-1 or any policy), with the run limit
    inspect-robots run --embodiment yam_arena --policy dream \
      -E task=tasks/drive-spring-pin-flush-with-starter-punch/task.py -E object_jitter=0 -E control_hz=20 \
      -E cam_width=320 -E cam_height=176 \
      --instruction "Pin the shaft to the clevis." \
      --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/drive-spring-pin-dream

`task_score` is 1.0 when `success()` has held for 3 s (the episode then ends). Otherwise it is the last step's
`score()`. The scene is the same in every trial (`object_jitter=0`), and nothing in it is random, so no seed is needed.
The demo's strokes start at about 19 s; it does not yet reach 1.0 (see Status). A trial takes 5 to 10
minutes on a shared L40S: the pin model reads the hammer and punch after every 240 Hz physics step.
`run_limit` is 2,400 steps (2 minutes at 20 Hz), about 3.6 times the demo's time.

Environment variables (for checking only, never needed for a trial): `PIN_DEBUG=1` prints the model's state every 10
steps and every blow. `PIN_MODE` swaps the scripted demo for a red-team variant: `none`, `onearm`, `press` or
`careless` (results below).

## The scene

Positions are in cm from the midpoint between the two arm bases: x forward, y to the robot's left. Heights are above
the table.

| Object | What it is | Where |
|---|---|---|
| V-block | steel, 7.5 (x) x 6 x 6 cm, 90 deg V 4 cm across with a 4 mm relief slot; clamped (fixed) | x 26.3 to 33.8, y -3 to 3 |
| Clevis | steel; hub 24 mm across, 30 mm long, 12 mm bore; fork of two 6 mm ears with an 8 mm clevis pin | hub in the V, its axis along x 5.7 cm up; fork toward the robot |
| Shaft | steel, 12 mm, in the hub's bore, running 12 cm out of the hub along +x | to x 41.5 |
| Cross-hole | 4 mm, vertical, through hub and shaft | x 28.0, y 0.0; hub top 6.9 cm up |
| Spring pin | steel slotted spring pin, ISO 8752, 4 x 22 mm, 1.4 g; started 3.0 mm, 19.0 mm proud | in the cross-hole |
| Punch | 5/32 in roll-pin punch: 4 mm point 40 mm long ending in a rounded pilot 1.8 mm across, 9.6 mm body, 100 mm long, 36 g, crowned head | lying with its point toward the left arm at (10.8, 18.7), head at (17.6, 11.3) |
| Hammer | Vaughan TC308 8 oz ball-peen: 85 mm steel head, 22 mm face, hickory handle 15 x 24 mm, 298 mm, 0.37 kg | on its side, head at (12.2, -11.9), handle to (33.4, -31.4) |
| Distractors | cotton shop rag 26 x 23 cm, 50 g; multi-purpose oil can 5.2 x 10.4 x 27.2 cm, 0.45 kg; M16 hex nut, 30 g | (47, 25); (50, -27); (44, 8) |

The hub's proportions follow SPIROL's design rules for pinning a hub to a shaft. The hub wall is at least 1.5 times the
pin's diameter (6 mm for 4 mm), and the hole is at most a third of the shaft's diameter (4 mm in 12 mm). The 22 mm pin
in the 24 mm hub leaves 1 mm of recess at each end when it is centred, so "home" is a 2 mm band, not a single depth.
The distractors are loose and the scorer ignores them. None of them can stand in for a punch or a hammer.

## Importance and sources

**Category: Impact on jobs.** O*NET lists 1,621,800 general maintenance and repair workers (49-9071.00, 2025). One of
their core tasks is: "Repair machines, equipment, or structures, using tools such as hammers, hoists, saws, drills,
wrenches, or equipment such as precision measuring instruments or electrical or electronic testing devices."
([O*NET 49-9071.00](https://www.onetonline.org/link/details/49-9071.00)). Spring pins are one of the commonest ways of
fixing a hub, gear, collar or clevis to a shaft. SPIROL's white paper on pinning a shaft and hub assembly is a design
guide for exactly this joint ([SPIROL](https://uk.spirol.com/assets/files/cldp-wp-how-to-properly-pin-a-shaft-and-hub-assembly-us.pdf)).
Taking such a joint apart and pinning it again is the "repair ... using hammers" that O*NET names. Team assemblers
(O*NET 51-2092.00, 1,403,500 in the 2025 figure for that group) fit the same pins on assembly lines
([O*NET 51-2092.00](https://www.onetonline.org/link/details/51-2092.00)). The BLS notes that "most assemblers and
fabricators are classified as team assemblers"
([BLS Occupational Outlook Handbook](https://www.bls.gov/ooh/Production/Assemblers-and-fabricators.htm)).

The doubt about this claim is stated here. Neither O*NET task list names spring pins. The claim rests on hammer work
being a named core task of a 1.6-million-worker occupation, and on pinning a hub to a shaft being one of the most
frequent forms that work takes.

**The knack.** A started pin stands on 3 mm of engagement and moves only when struck. One hand keeps the punch's
pilot in the pin's 2.4 mm open end and the punch within 3 deg of the hole's axis, while the other swings a 0.37 kg
hammer onto a 9.6 mm head. The holding hand has to lower the punch after every blow as the pin advances. A blow on a leaning punch, or
one half off the pin, cocks the pin, and it binds. Near the end, the hammer face alone would dent the hub, so the punch
is what carries the pin the last millimetres. The job is a different ability from striking a nail, which needs no
intermediate tool: the robot both holds a tool on a moving workpiece and strikes it, and has to stop with the pin
inside a 2.5 mm band.

**Products.** The punch is a 5/32 in roll-pin punch: its point ends in a rounded pilot that sits in the hollow end
of a spring pin and keeps the point on it. An example is the Mayhew 42055 pilot punch, 5/32 in x 4-1/4 in with a ball
tip and a 1/4 in hex shank, $5.46 ([distributor listing](https://www.nextind.com/itemdetail/12482-42055)). The model's
body is 9.6 mm round and 100 mm long, the size of a Starrett 565D 5/32 in punch
([Starrett catalogue page](https://www.starrett.com/dms/flipbooks/PTA%20and%20Hand%20Tools%20Flipbook/inc/html/96.html)),
not the Mayhew's thinner hex shank. The point's 40 mm length is an estimate.
Vaughan TC308 8 oz ball-peen hammer, 11 3/4 in, hickory handle, 0.81 lb, about $19
([retailer listing](https://homelectrical.com/8-oz-forged-steel-commercial-ball-pein-hammer.vau-tc308.1.html)). The
4 x 22 mm ISO 8752 slotted spring pin is a standard catalogue part sold in packs of 100 to 200 for a few cents each. Its
retailer pages could not be fetched to verify, so no link is given. The clevis hub and shaft are a turned and
cross-drilled pair to the dimensions above (any machine shop, or a 24 mm steel collar on 12 mm ground rod).

## Distinct from

| Nearest task | What it tests | Why this task is different |
|---|---|---|
| TK-W437WY, "Start a nail by holding it while tapping it in, then drive it home" (accepted) | One hand holds a nail upright on its point while the other taps it; the holding hand then lets go and the nail is struck directly into wood | No intermediate tool. Here the holding hand never lets go: it keeps a punch on a pin end that moves away after every blow, and the hammer never touches the work. A nail finishes flush on its own head; this pin has to end inside a 2.5 mm band that only the punch can reach. |
| TK-D9RUH2, "Set a drop-in expansion anchor in concrete with its setting tool and a hammer" (pending revision) | A setting tool sits in an anchor's bore and is struck until its shoulder meets the anchor: a self-stopping depth | This is the closest one. The setting tool locates itself inside the anchor and the shoulder stops the drive. Here a punch has to be kept on the end of a 4 mm pin by hand, the pin travels 20 mm through three parts, and nothing stops it: one blow too many drives it out of the far side. |
| TK-88C465 / TK-U9EC7D, "Hold a nail upright while another robot arm hammers it in" (revising) | Two agents: one holds a nail, the other hammers it | Same difference as TK-W437WY, and here both arms belong to one agent. |
| TK-RP86Y7, "Drive a bushing to its depth ... choosing the stepped driver over the plain drift" (rejected) | Choosing the right driver for a press-fit bushing | No tool choice here. The test is holding the punch square and controlling depth, blow by blow, on a friction-held pin. |

Searched: the titles of every task pull request on the board, for nail, hammer, punch, drift, pin, clevis, anchor,
flush and dowel; and the RoboARC task list (no hammer or pin-driving task). We know of no punch-and-pin task in LIBERO,
RoboCasa, BEHAVIOR-1K or RoboTwin 2.0, whose hammer task strikes a block once; those lists were checked from memory,
not searched for this task. The two cotter-pin clevis tasks (TK-T6U364, TK-P2A4ZK)
push and bend a cotter pin by hand. Nothing in them is struck.

## How the simulator judges each rule

The clevis, V-block, shaft and pin are one fixed `mechkit` mechanism. The pin is its only joint: a slider along the
hole, locked at its depth by a stiff drive (200 kN/m). Only the model below moves the pin's set point. The punch and the
hammer are free rigid bodies with their real masses.

- **A blow.** After every physics step (240 Hz) the model checks whether the hammer's face has reached the punch's head
  (or the pin's end) while moving toward it along the axis at 0.3 m/s or more. The face must lift 5 mm before the next
  blow counts. Slower contact is a push, and a push never moves the pin.
- **How far a blow drives it.** The bores hold the pin with 5 N per mm of pin engaged in them, and a blow drives it
  until that friction has used up the hammer's kinetic energy (0.5 m v^2, m = 0.37 kg). A full stroke of the simulated
  arm reaches the punch at about 0.85 m/s (about 0.13 J). With that energy, the first blow drives the pin about 4.5 mm,
  and blows near the end about 1.1 mm. The demo needs eleven blows. The friction is scaled to the simulated swing: a
  person's blow carries several times more energy, and the real pin resists it more. The real pin should take a similar
  number of firm blows. That is an estimate, not a measurement.
- **The punch on the pin.** The punch's point must be within 1.6 mm of the pin's centre (at least half of the 4 mm
  point on the 4 mm pin's end) and between 1.0 mm into and 1.5 mm above the pin's end. A blow on that punch drives the
  pin, unless the punch leans more than 3 deg from the hole's axis: then it cocks the pin. A blow with the punch's
  point 1.6 to 3 mm off centre (less than half of it on the pin's end) cocks it too. A cocked pin never moves again, and the score is capped at 0.25.
- **Blows on the part.** Each of these is recorded and caps the score at 0.25 for the rest of the run:
  - any part of the hammer (face rim, head, ball, handle) within 1 mm of the hub, shaft or fork while moving at
    0.1 m/s or more;
  - a blow on the punch with its point resting on the hub beside the pin;
  - a direct face blow that would take the pin's upper end below 0.5 mm proud. The face stops there, on the hub.
- **The record.** Every record is kept in the `Trial` object and cleared only by `mechkit`'s `on_reset` hook, never
  because the scene looks right again.
- **The score** is read from the pin's measured joint position each control step:

| Score | Condition |
|---|---|
| 1.0 | Neither end proud by more than 0.25 mm (d 21.75 to 24.25 mm); no blow on the part; pin not cocked; every fingertip of both grippers at least 5 cm from the top of the hole; the punch's point at least 3 cm from the pin; no part of the hammer within 2 cm of the hub; the pin at rest. All of this held for 3 s. |
| 0.75 | Neither end proud as above, but the hands or tools not yet clear |
| 0.5 | Through the shaft and at least 2 mm into the hub's far wall (d at least 20 mm), but an end still proud |
| 0.25 | Through the near wall and at least 2 mm into the shaft (d at least 8 mm). Also the cap after a blow on the part or a cocked pin |
| 0 | Otherwise. Doing nothing leaves d at 3 mm |

Here d is how far the pin's lower end is below the top of the hub. The upper end stands 22 - d mm proud and the lower
end d - 24 mm proud of the underside. Possible scores: 0, 0.25, 0.5, 0.75, 1.0.

## The demo

`demo()` is open loop: one fixed plan from the published starting layout, with no reading of the scene. Each step is
below, and the numbers marked "measured" were measured in this simulation.

1. The left gripper takes the punch from the bench by its body, 60 mm from the point, straight down. It lifts the
   punch high, short of the work, and turns it point-down there. The turn happens away from the work so that the
   swinging point cannot touch the clevis.
2. Over the hole, the left gripper sets the punch's point on the pin's end and holds it firmly. After each blow it
   lowers the punch by the depth the model gives for the measured blow, to 0.5 mm short of where the plan has the
   pin's end; the blow itself pushes the punch down through the fingers after the pin. Lowered, the arm's joints lag
   about 0.2 mm sideways for each mm, so each lowered target is sent that much the other way (measured). Without that
   correction the point walked off the pin's end by the third blow, and a light, still hold let the punch lean more
   than 3 deg (both measured).
3. The right gripper takes the hammer by its handle, 36 mm from the head, and turns it face-down. Each stroke is a
   short downward drive of the whole arm: the face starts 5 cm above the punch's head and is sent 1.5 cm below it.
   The arm is sent back up at once, with no follow-through: leaning on the punch after a blow tipped it in the
   fingers (measured).
4. The strokes are sent 11 mm short along the arm's heading and 2.5 mm along the handle, and turned 4 deg back.
   Without that correction the face lands long and tipped (measured). The punch's head is crowned (10 mm radius, as
   on a dressed punch), so the face's push stays on the punch's axis.
5. Eleven blows at the measured mean of about 0.12 to 0.13 J put the pin at about 23.3 mm in the plan (centred is
   23.0). The hammer lifts away, the left gripper closes on the punch and lifts it off, and both tools are held clear.

## Red team

Run with `PIN_MODE=<name>`, same scene and scorer:

| Variant | What it does | Score | Why |
|---|---|---|---|
| `none` | both arms stay parked | 0 | the pin stays 3 mm in |
| `onearm` | no punch: the right arm strikes the pin's end directly with the hammer's face, 24 blows | 0.25 | the face drove the pin to 11.3 mm, then met it 3 deg off square and cocked it (capped at 0.25); later strokes touched the hub. Square blows alone could reach 0.5 at most: the face stops 0.5 mm proud |
| `press` | punch set on the pin, then the hammer's face lowered slowly onto it and leaned on for 3 s (no blow) | 0 | a push never drives the pin (the punch slid off the pin's end, and the pin stayed 3 mm in) |
| `careless` | the full demo plus six extra blows after the pin is home | not run | needs a working demo first |

## Status (work in progress, not ready to submit)

The scene, the pin model, the scorer and the text are complete; the scripted demo is not. In 14 simulator trials the
right arm's hammer strokes became reliable (about 0.8 to 0.9 m/s at the punch, face square within about 2 deg and
centred within about 3 mm), and the first blow drives the pin as modelled (3.0 to 7.3-7.5 mm). The left arm's
open-loop hold has not kept the punch's point on the pin's end past the second or third blow. After a blow the arm
either sags onto the punch or lags 1 to 2 mm sideways as it is lowered, and the point slides off the 4 mm pin end
(more than 1.6 mm off centre, which the model scores as a cocked pin). Tried without success: firm and light holds,
a still arm with the punch sliding through the fingers, lowering with and without a sideways lag correction, and a
pilot-pointed punch seated in the pin's open end (the pilot changed how the punch hung in the fingers by about 1.5 deg).
Red-team runs so far: do nothing 0; hammer only (one arm) 0.25; pressing instead of striking 0. The careless
(over-driving) variant has not been run.

## Real trials: preparation, reset and measurement

**Build (about $90 excluding the robot).**
- Steel V-block with a clamp: any 60 mm machinist's V-block set.
- A clevis with a 24 mm hub and 12 mm bore on 12 mm ground steel rod, or a 24 mm steel shaft collar on the rod. Drill
  hub and rod together 4.0 mm through, square to the axis, 15 mm from the hub's end.
- A box of 4 x 22 mm ISO 8752 slotted spring pins.
- A 5/32 in roll-pin punch with a pilot point (for example the Mayhew 42055) and the Vaughan TC308 hammer.
- Clamp the V-block to the bench on tape marks at the positions above. Clamp the shaft in it with its hole vertical. A
  4 mm drill blank through the hole sets this.

**Preparation, each trial (about 2 minutes).**
1. Brush blue layout fluid on the hub's top and underside and on the shaft around the hole, and let it dry (30 s).
2. Line up the hole with the drill blank and remove it.
3. Start a new pin by hand with its slot facing the shaft's axis. Tap it once lightly until it stands 19.0 mm proud,
   measured with a depth gauge (within 0.3 mm).
4. Lay the punch and hammer on their tape marks.

The pin is the only consumable, a few cents each.

**Measurement, at the end, before anyone touches the scene.**
- Depth gauge or steel rule and straightedge across each end of the hole. The upper end's protrusion (or recess) and the
  lower end's (seen from below through the relief slot) give d.
- Bright marks in the layout fluid on the hub or shaft show a blow on the part.
- A pin that leans in its hole (checked with a square against its end) or that has a flattened edge is cocked.
- The video shows where the grippers and tools ended up and whether anything moved in the last 3 s.

**Reset.** Drive the pin out downward through the V-block's relief slot with the same punch. The relief slot exists for
this. Then repeat the preparation. A pin that has been driven is not reused.

**Rig safety.** The hammer's head is 8 oz and the strokes are a few centimetres long. Keep people outside the arms'
reach while the arms move. The punch and hammer have no edges. Wrap the hammer's handle where the gripper takes it in
one turn of cloth tape, which protects the finger pads and is shown in no score. Spring pins can fly when struck off
square: wear safety glasses near the rig.

## Fidelity and simplifications

- **The pin's grip is a model, not contact friction.** The pin moves only through the blow model above. Its resistance
  (5 N per mm engaged) is scaled down with the simulated hammer's energy. That number is an estimate to be measured on
  the real parts by counting blows.
- **Cocking and marking are rules.** Nothing bends and no dent is shown.
- **Primitives.** The hub's collision shape is open 3 mm around the hole so that the punch's 4 mm point can follow the
  pin into it. The punch and the hammer are built from primitives at their real sizes and masses. The SimReady hammer
  props are each one signed-distance mesh, which a gripper cannot hold and strike with reliably. No SimReady punch,
  clevis, shaft or V-block exists.
- **Friction.** Uniform friction, as in every task here.
- **The wrapper.** `task.py` wraps `yam_arena.demo.grip_rotation` so that a demo waypoint can carry a full 3 x 3 grip
  rotation (for the level grips). The accepted nail-driving task's `bodykit.allow_matrix_grips` does the same. It
  changes nothing for a model's run.

## Assets and licences

The distractors come from NVIDIA's SimReady dataset
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`. They download on the first run, and nothing large is committed.

- [`cleaning_cloth_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/cleaning_cloth_a01) (shop rag)
- [`consumable_lubricant_oil_multi_purpose_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/consumable_lubricant_oil_multi_purpose_a01) (oil can)
- [`factory_nut_m16_loose`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/factory_nut_m16_loose) (M16 nut)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes at load time:
- masses set (rag 50 g, oil can 450 g, nut 30 g);
- the oil can's two bodies (can and spout) welded into one.

Everything else (clevis, shaft, V-block, pin, punch, hammer) is built in `task.py` from `mechkit` primitives and has no
third-party asset. `mechkit.py` and `simready.py` are copied, with credit, from other teams' open-source tasks in this
repository (see the file table).
