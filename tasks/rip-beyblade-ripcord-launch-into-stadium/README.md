# Launch a Beyblade into its stadium

A child hands over a loaded Beyblade launcher: "launch it for me". The loaded launcher, a Hasbro Beyblade Burst
Supergrip Launcher with a top locked onto its prongs and its 9-inch toothed ripcord pushed in, lies on its flat side
next to a Beyblade Burst Beystadium. The robot has to pick it up, turn it upright, hold it still and level a few
centimetres above the stadium's centre, and rip the cord straight out with the other hand, fast. The cord's teeth spin
the top through the launcher's gear; when the last tooth leaves the gear the top drops off and has to keep spinning on
its tip inside the stadium for 10 seconds.

Instruction: *"Launch the Beyblade so it spins in the stadium for 10 seconds."*

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: scene, the launcher's gear and release, scoring (`score`, `success`) and the scripted `demo()` |
| `beykit.py` | The stadium's and the top's geometry, a triangle-mesh collider for the stadium, and the top's spin-ready rigid body |
| `mechkit.py` | Mechanism kit (articulated parts from USD text, hooks), copied unchanged from `tasks/heat-shrink-lug` |
| `demokit.py` | Only its grip helpers are used: waypoints whose gripper is tilted (here: level, to hold the launcher upright) |
| `simready.py` | Loads the NVIDIA SimReady distractors, downloading them once from Hugging Face |
| `task.json` | The task's details for the board |

## Run a trial and score it

From the repository root, with the `yam_arena` environment active:

    # scripted demo (reaches 1.0)
    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=tasks/rip-beyblade-ripcord-launch-into-stadium/task.py -E object_jitter=0 -E control_hz=20 \
      -E cam_width=640 -E cam_height=360 \
      --instruction "Launch the Beyblade so it spins in the stadium for 10 seconds." \
      --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/beyblade

    # a model: the same with --policy dream (or another policy) and -E cam_width=320 -E cam_height=176

`--scorer task_score` reports the trial's score: 1.0 once `success()` has held for `SUCCESS_HOLD_S` (2 s), otherwise
the last step's `score()`. The layout is fixed (`object_jitter=0`) and the simulation is deterministic: the default seed
is all a trial needs. The demo releases the top at 12.6 s and is scored 1.0 at 22.6 s; the episode ends at about 24.6 s
of simulated time (about 490 control steps). A trial takes about 2 to 4 minutes on one L40S including start-up.
The run limit is 1,200 steps (60 s at 20 Hz), about 2.4 times the demo.

## Scoring, and how the simulator judges each rule

Spin time runs from the release until the first control step at which the top is not spinning on its tip inside the
stadium, or a gripper comes near it.

| Rule | In the simulator (every control step, 20 Hz) | In a real trial |
|---|---|---|
| Release | The substep at which the cord has been pulled 10.6 cm, so its last tooth passes the gear | The video frame in which the top leaves the prongs |
| Inside the stadium | The top's centre of mass within 16.05 cm of the stadium's centre (the inner wall is at 18.5 cm, the top 4.9 cm across) | Overhead video: the whole top inside the inner wall |
| On its tip | The top's axis within 30 degrees of vertical (its forge disc's rim reaches the floor near 39 degrees) | Side video: only the tip touches the floor |
| Spinning | Turning about its own axis at 30 rad/s (290 rpm) or more | A marker dot on the layer moves between 60 fps frames |
| Hands clear | Every fingertip point (each finger's tip and 3 cm up it, both arms) 5 cm or more from the top's centre | Video against the top's own 4.9 cm width |
| Ring-out | After the release, the top's centre more than 20.7 cm from the stadium's centre, or more than 1 cm below the table top | The top ends outside the stadium |

- **1.0**: spin time reaches 10 s. It is then scored 1.0 for the rest of the trial (the top runs down on its own).
- **0.5**: spin time 5 to 10 s. **0.25**: 2 to 5 s.
- **0**: under 2 s, or the top was never released.
- A ring-out before 1.0 scores **0** for the whole trial (a top thrown out of the stadium is the hazard the maker warns
  about), even if it had spun for a while first.

Possible scores: 0, 0.25, 0.5, 1.0. The scorer reads only the top's pose and velocity, the launcher's cord joint and the
fingertip points. Nothing it records is cleared except at a reset (`Trial.reset`, run from mechkit's `on_reset`).

## The mechanism

- **Rack and pinion with a one-way clutch.** The ripcord is a slider in the launcher (a free articulation of 70 g).
  Each physics substep (240 Hz) the code sets the prong ring's spin to the cord's speed out of the slot divided by
  2.9 mm: one turn per 18.2 mm of cord. Pushing the cord in, or slowing down, leaves the ring coasting.
- **The pull is felt.** The prongs' and top's inertia, seen at the cord through the gear, is the cord joint's armature:
  1.25 kg while the top is on, 0.18 kg once it is off. Accelerating the cord takes real force from the pulling hand, and the
  same force reaches the launcher through its slot, so the holding hand has to keep it still and level.
- **Prongs and release.** While the cord's teeth are in mesh, the top (a free rigid body) is locked to the prong ring:
  its pose and velocity are written each substep from the launcher's, turned by the ring's angle. The substep the last
  tooth passes the gear, the top is let go with the ring's spin plus the launcher's own motion. From then on it is an
  ordinary PhysX body: gyroscopic forces on, the angular speed cap raised, and an explicit inertia
  (9.0e-6 kg m2 about its axis, 6.0e-6 kg m2 across, centre of mass 2.3 cm above the tip).
- **Air drag** is a torque about the top's own axis only, -0.03 x I x spin, every substep.
- **Contact.** The stadium is a triangle-mesh surface of revolution; plastic on plastic friction is 0.30 static and 0.25
  dynamic. The top's tip is a 3 mm ball.

What the simulation does with it, measured while building the task (free tops dropped spinning into the stadium):
upright tops at 400 rad/s stayed up beyond 20 s; at 200 rad/s one fell after about 18 s, as its spin drained to near
Maxwell's limit for a sleeping top (about 120 rad/s for these numbers); one at 60 rad/s fell in 0.5 s. Tops let go tilted or
moving sideways bounced and fell, or wandered out over the rim.

## Results (scripted demo and red-team variants)

All runs at 20 Hz with the trial runner, the scripted policy and the layout above. Red-team variants are copies of
`task.py` with a different `demo()`; they were run and deleted.

| Run | What it does | Score |
|---|---|---|
| Demo, run 1 | Pick up, turn upright, hold the top 3 cm over the centre, rip 17 cm in a 0.25 s command | 1.0 |
| Demo, run 2 | The same | 1.0 |
| Do nothing | Arms stay parked | 0 (nothing is released) |
| One arm | The left hand alone rips the cord out of the launcher lying on the table | 0 (the unheld launcher is dragged with the cord, which comes out 2 mm; nothing is released) |
| Slow pull | The full demo with a 1.5 s pull | 0 (released at 59 rad/s; on its tip for 0.3 s, then falls) |
| Angled pull | The full demo, the cord ripped downwards at 45 degrees to its slot | 0 (the cord binds after 5.3 cm and drags the launcher; the top is never released) |

In both demo runs the cord left the gear with the prongs at 296 rad/s (2,830 rpm); the launcher was within 1.9 degrees of level, the top's tip 2.9 cm above the bowl and 3 mm off its centre. The top stayed within 3 degrees of vertical, reached 10 s of spin (score 1.0) at 22.6 s, and was still spinning on its tip at 204 rad/s when the episode ended 2 s later. The two runs were identical: the simulation is deterministic for a fixed layout. While building the task, the same launch from 2 cm instead of 3 cm also scored 1.0 but tilted to 28 degrees by 11 s; and with sharper tips (1.2 and 2 mm) or more spin (a 2.5 mm gear radius), three of four launches bounced on landing and fell within a second. The configuration reported here is the one the demo uses.

Dream-1 run (1,200 steps): **0**. The model moved both arms around above the table but never grasped the launcher: it stayed on its side where it started, the cord was never pulled and the top was never released.

## Distinct from

| Nearest task | What it tests | Why this one differs |
|---|---|---|
| TK-S37FV6 (#3166, pending): spin a tin humming top by pumping its plunger | One hand stands a top and pumps a screw plunger in strokes, then lets go | Here the top is never touched: it is launched off a hand-held launcher. The skill is two-handed and dynamic: one hand turns the launcher upright and holds it level over a target against the yank, the other rips a cord out in a quarter of a second; a slow or angled pull cannot be made up by more strokes |
| TK-3YGQXN (#2746, accepted): send a pull-back toy car through a gate | Winds a clutch by dragging the car on the floor, then lets go | No launcher held in the air, no second hand pulling, no free spinning body that must stay upright after release |
| TK-626KKS (#2817, pending): pinball plunger launch | Pulls and releases a sprung plunger fixed in a machine | The launcher here is free and must be held level and still over a target by the other hand while the cord is ripped |
| TK-EPGLAJ (#3080, pending): flick a prize wheel | A flick that coasts a fixed wheel to a stop | The launched body is free, has to land on its tip and keep its balance by its own spin |

The usual task sets (LIBERO, RoboCasa, SimplerEnv, BEHAVIOR, RoboTwin) have no ripcord launch or spinning top that we
know of; their dynamic tasks are throws, pours and pushes, not a rip whose speed becomes a free body's stability.

## Importance and sources

Beyblade has been a mass-market toy in the West since Hasbro began selling it in 2002; more than 5 million tops had been
sold in the United States by 2003, and Hasbro has distributed the Beyblade Burst line in the United States since
January 2017 ([Wikipedia: Beyblade](https://en.wikipedia.org/wiki/Beyblade)). Every battle starts with a ripcord or string
launch; the maker's instructions have each player load a top into a launcher and pull the ripcord over the Beystadium
([Hasbro, Beyblade Burst Supergrip Launcher instructions, B9510](https://instructions.hasbro.com/en-us/instruction/beyblade-burst-supergrip-launcher),
[instruction sheet PDF](https://instructions.hasbro.com/api/download/B9510_en-us_beyblade-burst-supergrip-launcher.pdf)).
Hasbro's instructions page above gives the Supergrip launcher's 9-inch ripcord and an age grade of 8 and up, so a parent or older
sibling launches for younger fans. It is recognisable in a 10 s clip.

Product and sizes. Launcher: Hasbro Beyblade Burst Supergrip Launcher, B9510, with a 9-inch ripcord (Hasbro's page above).
The launcher's head and grip sizes, its 70 g mass, the gear ratio and the cord's 18.4 cm strip plus 4.5 cm handle are
estimates matched to that 9-inch length. Stadium: a Hasbro Beyblade Burst Beystadium; retail listings give it as about
15.5 inches (39.4 cm) across; its rim height and dish profile here are estimates. Top: a Beyblade Burst top (layer, metal
forge disc, ball tip), 4.9 cm across, 3.75 cm tall, 40 g: estimates. The real-trial section says how to measure each.

## Real trial: preparation, reset and measurement

- **Rig.** Put the Beystadium on the bench with its centre 30 cm in front of the arms' base line, midway between them,
  on a rubber mat so it cannot slide; mark its outline and the launcher's resting place with tape. Hasbro's sheet warns
  against leaning over the stadium while tops are in play and against using it on raised surfaces: keep people out of
  a 1 m circle during the launch, and stand a clear acrylic ring, 60 cm across and 15 cm tall, round the stadium as a guard for the room
  (it is outside the 20.7 cm ring-out line and does not change the score).
- **Before the first trial, measure and record:** the launcher's turns per centimetre of cord (pull the cord 10 cm
  slowly and count the prong ring's turns with a marker dot; the simulation assumes 5.5 turns), the top's mass on a
  kitchen scale, and the stadium's rim height.
- **Preparation, each trial (under 2 minutes):** push the ripcord fully in, teeth towards the gear, until it clicks;
  lock the top onto the prongs; lay the launcher on its flat side on its tape marks, grip end away from the robot,
  handle towards the stadium. Check the top's tip for wear and the stadium floor for dust.
- **Measurement:** a phone at 60 fps above the stadium and a second at rim height. Spin time is counted in frames from
  the frame the top leaves the prongs to the first frame where the top's body touches the floor, any part of it is
  outside the inner wall, a fingertip comes within one top-width (5 cm) of it, or it stops turning. A ring-out is
  the top ending outside the stadium.
- **Rig protection:** a Beyblade's metal disc spins at up to a few thousand rpm: the grippers stay clear of it by the
  scoring rule, and the pads only touch the launcher's grip and the cord's handle. Nothing is consumed.

## Fidelity and what is simplified

- PhysX rigid bodies at 240 Hz; the robot's grippers are position-controlled.
- The stadium, launcher, cord and top are meshes and boxes built here: SimReady has no Beyblade parts. The stadium is
  fixed to the table and its three pockets are left out, which makes a ring-out a little less likely than on the real one.
- The gear is code, not contact: the cord's speed sets the prongs' spin, one-way. Its inertia is the cord joint's
  armature. The top is held to the prongs by code until the last tooth passes the gear; the real launcher lets the top go
  when the gear stops driving it, at the same point of the pull.
- Air drag is a spin-only torque, -0.03 per second of spin.
- After the release the strip, which is weightless, stays on its slider line beside the head; the demo's right hand lets
  go of it.
- The top in simulation is less forgiving than a real Beyblade: a release with more than a degree or two of tilt or a few
  centimetres per second of sideways speed often makes it bounce and fall within a second, where a real top would
  usually wobble and recover. That makes the simulated task stricter than the real one, not easier.

## Assets and credits

Distractors from the Hugging Face dataset [`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets),
pinned to revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, © NVIDIA, CC-BY 4.0. Masses are set at load time; nothing
else is changed. Files download on first run; nothing large is committed.

- [`dishware_plastic_cup_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_plastic_cup_a01): a child's plastic cup, 30 g
- [`food_produce_apple_b01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/food_produce_apple_b01): an apple, 180 g
- [`misc_notebook_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/misc_notebook_a01): a notebook, 120 g

`mechkit.py` and `demokit.py` are copied unchanged from `tasks/heat-shrink-lug` (another team's open-source kits, shared
by many tasks in this repository); `simready.py` is the repository's shared loader.
