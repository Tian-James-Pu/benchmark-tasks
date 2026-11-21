# Take a hidden aerator out of a faucet's spout with its key, then screw it back in flush

A hidden ("cache") aerator sits inside the end of a faucet's spout. Nothing of it stands proud, so it can be turned only with its
plastic service key, whose lugs fit the ribs on the aerator's face. Limescale builds up on the thread and seizes it, and the aerator
has to come out to clear its screen. This task is that job on a single-lever lavatory faucet whose mounting nut has worked loose:

- **Hold the spout.** The scale bond takes twice the torque to break (0.2 N m) that the loose mounting nut holds (0.1 N m). Turned
  with one hand, the key turns the whole faucet on the deck instead, and the aerator stays put. The other hand holds the spout while
  the key turns.
- **Keep the key seated while turning it.** The key is held from below by its grip block and pressed up into the ribs. A wrist turns
  only so far, so the aerator comes out in strokes: turn, drop the lugs out, turn back, press them in again, turn again.
- **Unscrew it until it leaves the thread** (it drops 4 mm onto the key), then **turn it back in to its seat**, and take the key away.

The instruction gives only the goal: "Take the aerator out of the faucet's spout with its key, then screw it back in flush."

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: the scene, the thread and mount model, the scorer (`score`, `success`) and the scripted `demo()` (open loop) |
| `faucet_colliders.json` | Convex collider hulls for the faucet, taken from the SimReady faucet's own mesh |
| `mechkit.py` | The mechanism kit (articulations written as USD text, hooks, fingertip points), copied unchanged from `tasks/heat-shrink-lug/` (another team's open-source kit, MIT licence) |
| `demokit.py` | Shared helpers for the scripted demo's gripper orientations (`tilted`, `down`), copied unchanged from the same kit (MIT licence) |
| `simready.py` | Loads SimReady props and fetches the faucet's files, copied unchanged from this repository's shared copy |
| `task.json` | The task's details for the board |

## Run a trial

Scripted demo (scores 1.0; it needs 2,300 steps at most):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/remove-hidden-faucet-aerator/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "Take the aerator out of the faucet's spout with its key, then screw it back in flush." \
  --scorer task_score --max-steps 2300 --store-frames --log-dir ~/yam-eval/remove-hidden-faucet-aerator/demo
```

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/remove-hidden-faucet-aerator/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the aerator out of the faucet's spout with its key, then screw it back in flush." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/remove-hidden-faucet-aerator/dream
```

The run prints `task_score` at the end. The scorer also prints a line every 2 s (`[aerator] ...`) with the aerator's turns out, its
drop, the faucet's angle on the deck and the records.

## Runtime and seeds

- Demo: 1,973 steps (98.7 s simulated), about 12 minutes on an L40S including start-up.
- Dream-1: up to 2,400 steps (2 minutes simulated). In its run (score 0.0) the right arm reached over the faucet early on without picking up the key, then both arms drew back near their bases for the rest of the run; the key, the aerator and the faucet never moved.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop from the published layout and
  deterministic.

## Scoring

Every step the scorer reads the aerator's angle from the mark on its face (unwrapped, so whole turns count), the faucet's angle on
its deck, and the key's and every fingertip's distance from the centre of the aerator's face. Two records are harms, kept from the
moment they happen until the trial is reset (they are cleared only in the reset hook):

| Record | Condition | Effect |
|---|---|---|
| Faucet twisted | the faucet turned 5 degrees or more on its deck, either way | score 0 |
| Aerator forced | the aerator turned 15 degrees or more past its seat (tightened into its washer) | score 0 |

Otherwise the score is the highest of:

| Score | Condition |
|---|---|
| 1.0 | the aerator was out of its thread (1.1 turns or more out, dropped 4 mm) at some point, and is now within 15 degrees of its seat, with the centre of the key's lug face and all 24 fingertip points at least 3 cm from the centre of its face, all held for 2 s (40 steps) |
| 0.5 | the aerator has been out of its thread |
| 0.25 | the aerator has been turned 30 degrees or more out: the scale bond is broken |
| 0 | anything else, including doing nothing |

## Real trials: preparation and reset

The bond and the loose mount are set with ordinary tools before every trial, so each trial starts the same:

- **The loose mount.** Tighten the faucet's mounting nut until a spring scale hooked at the spout's end (9.5 cm from the faucet's
  axis) and pulled sideways turns the faucet at about 100 g (0.1 N m). Then turn the faucet back square, spout at the robot.
- **The bond.** A limescale bond, once broken, cannot be restored, so a real rig stands it in with torque: clean and dry the thread,
  then screw the aerator in with its key and a torque screwdriver set to 0.2 N m, with the spout held so the faucet does not turn. The mark on its face points at the robot.
- **The key** stands back on its taped outline on the deck, lugs up.

On a real rig the same readings come from a mark on the aerator's face (a protractor card under the spout), a mark on the deck at
the faucet's base, and a ruler.

## The model

`task.py` declares the mechanism in code:

- **The faucet on its deck** is a hinge about the faucet's own axis. The loose mounting nut holds it by friction up to 0.1 N m;
  past that it turns and stays turned (a clutch).
- **The scale bond** holds the aerator at its seat until 0.2 N m is reached through it, then breaks for the rest of the trial.
  Turned one-handed, the key's torque goes into the faucet first, and the faucet slips on its deck at 0.1 N m before the bond
  breaks: the other hand has to hold the spout.
- **The thread**: once free, the aerator turns against a 0.04 N m running torque, and its height follows its turns at 1 mm per turn
  for 1 turn. Past that it leaves the thread and drops 4 mm onto the key. Turned back, it re-enters the thread and runs up to its
  seat, a stiff stop it tightens against.
- **The key** is a free 25 g part. Its four lugs turn the aerator only by contact with the aerator's four ribs, and its centring pin
  sits inside the ribs' inner ends: nothing couples key and aerator in code.

The torques are low estimates, scaled down to what the simulated arm's wrist can apply through a gripped key; a real limescaled
aerator can take several times more, and a real faucet nut holds more. What the task keeps is their order: the bond needs more
than the loose mount holds. The thread has fewer engaged turns than a real one, and the key is taller than a real one so the gripper
fits under the spout. The `fidelity` field lists the rest.

## Importance and sources

Hidden ("cache") aerators are fitted to many lavatory faucets. They thread straight into the spout and disappear into it, so the
only way to turn them is a plastic service key matched to their size, and in hard water they calcify in place. Cleaning or
replacing one is routine household and maintenance work: unscrew it with the key, clear the screen, screw it back flush. A robot
doing home or building maintenance meets it as a small tool-use job with a real bimanual reason: the key's torque must be reacted
by holding the spout, or the faucet turns on its deck instead.

- Neoperl, maker of Caché aerators, on freeing a calcified one with its service key:
  https://www.neoperl.com/us/en/company/blog/remove-stuck-cache-aerator
- Neoperl's Caché range, thread sizes M24 x 1 to M16.5 x 1 ("Tom Thumb", the size modelled here):
  https://www.neoperl.com/us/en/oem/products/faucet-aerators/threaded-aerators/cache
- A retail set of Caché service keys, one per size: https://www.homedepot.com/p/NEOPERL-Cache-Hidden-Aerator-Keys-37-0298-98/207143875
- A Tom Thumb (M16.5 x 1) Caché aerator sold with its key:
  https://www.homedepot.com/p/NEOPERL-1-2-GPM-Tom-Thumb-Size-M16-5-x-1-PCA-Cache-Water-Saving-Laminar-Aerator-with-Key-2-Pack-97315-05/316884058

We found no task on the board, nor in LIBERO, RoboCasa, SimplerEnv or BEHAVIOR, that turns a recessed threaded part with a keyed tool
while the other hand reacts its torque.

## Assets

From the Hugging Face dataset [`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to
revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`fixture_faucet_singleLever_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/fixture_faucet_singleLever_a01): the faucet. Shown as authored (body, lever, pop-up rod); its own bodies, joints and
  colliders are left out, and it collides through convex hulls taken from its mesh (`faucet_colliders.json`). Mass set to 0.9 kg.
- [`cleaning_soapDispenser_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/cleaning_soapDispenser_a01): soap dispenser (distractor), mass set to 350 g.
- [`dishware_glass_tumbler_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_glass_tumbler_a01): glass tumbler (distractor), mass set to 250 g.
- [`cleaning_barSoap_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/cleaning_barSoap_a01): bar of soap (distractor), mass set to 120 g.

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). The files are not committed: `simready.py`
downloads them into `~/.cache/simready` the first time the scene is built, which needs network access.

The vanity deck, the aerator and the key are custom parts built in `task.py` (SimReady has no hidden aerator or service key).
