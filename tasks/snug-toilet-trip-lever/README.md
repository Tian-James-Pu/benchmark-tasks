# Snug a loose toilet handle: hold it level from outside while the other hand turns the tank's left-hand locknut

A toilet's flush handle has worked loose. The plastic locknut that clamps the trip lever assembly to the inside of the tank's
front wall has backed off a quarter turn, so the whole assembly (handle, escutcheon, threaded bushing and trip arm) turns in its
hole, and the handle hangs 15 degrees below level. The fix is the one plumbers and householders use:

- **Hold the handle level from outside.** The nut runs on the bushing's thread, so turning the nut drags the loose assembly round
  with it unless the handle is held.
- **Turn the locknut inside the tank counterclockwise** (seen from inside the tank: it has a left-hand thread) until it is snug
  against the spacer.
- **Don't overtighten it.** Kohler: "Do not overtighten the locknut, or the trip lever will not move freely."

The instruction states only the goal: "The toilet's flush handle is loose and has dropped out of level. Fix it so it is level and
firm."

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: the scene, the thread and clamp, the scorer (`score`, `success`) and the scripted `demo()` |
| `mechkit.py` | The mechanism kit (articulations written as USD text, hooks, fingertip points), copied from the shared open-source copy (`tasks/heat-shrink-lug/mechkit.py`), with a provenance line added |
| `demokit.py` | Grip helpers, unchanged from `tasks/heat-shrink-lug/demokit.py`; `task.py` uses it only so `demo()` waypoints can carry a full grip rotation |
| `simready.py` | Loads SimReady props (the distractors), downloading them once from Hugging Face, unchanged |
| `task.json` | The task's details for the board |

## Importance and sources

A loose, drooping flush handle is among the most common toilet faults that householders fix themselves, and the standard
home-repair guides describe exactly this repair: lift the lid, find the nut inside the tank, and tighten it counterclockwise,
because it is reverse-threaded, without cranking it down.

- Kohler, trip lever replacement instructions (087220): "The locknut has left-hand threads. Do not overtighten the locknut, or the
  trip lever will not move freely. If the assembly binds, loosen the locknut slightly until free movement is achieved."
  https://resources.kohler.com/webassets/kpna/catalog/pdf/en/087220-9.pdf
- Kohler, trip lever installation guide (1143141): "Secure trip lever with locknut. DO NOT OVERTIGHTEN. NOTE: Turn locknut
  counterclockwise to tighten." https://techcomm.kohler.com/techcomm/pdf/1143141-2.pdf
- This Old House, how to fix a toilet handle: https://www.thisoldhouse.com/bathrooms/how-to-fix-a-toilet-handle
- HomeTips, replace a toilet handle: https://www.hometips.com/repair-fix/replace-toilet-handle.html

The physical difficulty is robotic, not a decision: one hand has to hold a part that a second hand's turning would drag round,
the turn is about a horizontal axis just behind a tank wall (the nut sits 1.7 cm inside the front wall, its axis 4 cm below the
rim), and the turn has to stop at a snug seat, neither short of it (the handle stays loose) nor far past it (overtightened).
Done one-handed, the nut's drag swings the loose handle round instead of the nut advancing on its thread, and the handle drops back when let go.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena (`source <ARENA_DIR>/yam.env`).
- Network access the first time the scene is built: `simready.py` downloads the SimReady packages into `~/.cache/simready`.

## Run a trial

From this folder:

```bash
# a model (Dream-1)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The toilet's flush handle is loose and has dropped out of level. Fix it so it is level and firm." \
  --scorer task_score --max-steps 2400

# the scripted demo (task.py's demo())
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "The toilet's flush handle is loose and has dropped out of level. Fix it so it is level and firm." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/snug-toilet-trip-lever
```

`--scorer task_score` prints the score. Nothing is randomised, so every seed gives the same scene. Runtime: the scripted demo reaches full credit at step 424 (21.2 s of simulated time), when the run stops. On a shared GPU that took about 4 minutes of wall time including scene build; a full 2,400-step model trial can take 15 to 20 minutes on a busy card.

## Scoring

The task file sets `SUCCESS_HOLD_S = 0` and counts its own hold; `--scorer task_score` prints the result.

- The nut is **seated** when its angle on the thread is within 3 degrees of snug on the spacer, or past it (in the start layout it
  is 90 degrees, a quarter turn, short of snug). Seated, it clamps the trip lever assembly in its hole at whatever angle the handle
  then has (the assembly's joint is locked); backed off below that, it frees it again.
- **1.0** once the nut is seated, the handle is within 5 degrees of level and every fingertip point of both grippers
  (`mechkit.fingertips`: the tip and 3 cm up each finger, at its middle and both edges) is at least 3 cm from the handle, the
  escutcheon and the nut, all held for 2 s (40 control steps in a row). It stays 1.0 from then on.
- **0.5** if, at the end, the nut is seated and both grippers are 3 cm clear, but the handle is more than 5 degrees from level
  (firm but crooked) or has been level for less than 2 s.
- **0** otherwise: the nut not seated (the loose handle then drops back onto its stop, 15 degrees below level, when let go), or a
  gripper still within 3 cm.
- **0 for the whole trial**, even after full credit, if at any time the nut was more than 30 degrees past snug for 3 control
  steps running (overtightened: past snug a 2.5 N m/rad spring stands in for the washer, so this takes about 1.3 N m on the nut),
  or was turned 700 degrees loose (off the end of the thread: dropped in the tank). These records are cleared only at a reset
  (mechkit's `on_reset`).

## The demo

`demo()` is a fixed open-loop plan from the published starting layout (it reads nothing from the simulation).

1. The right hand comes down over the handle 7 cm out from the hub, closes across its 8 mm thickness and lifts it 15 degrees to
   level about the hole's axis, then holds it there.
2. The left hand comes down inside the front wall, its fingertips leaning 30 degrees towards the wall so that the fingers' wide
   upper part clears it, either side of the locknut. Once the handle is level it closes across two of the nut's flats and turns
   it counterclockwise (seen from inside the tank) 60 degrees, opens, turns back, regrips and turns 45 degrees more. The nut
   slips a little in the grip and ends about 90 degrees on, snug on the spacer.
3. The left hand lets go and leaves the tank; then the right hand lets go of the handle and leaves. The handle stays clamped
   at about 2.5 degrees below level.

All runs below were made with `inspect-robots run --policy yam_scripted` on the GPU (640 x 360 frames, 900-step cap). The
red-team variants are copies of `task.py` with a different `demo()` (not kept in the folder).

| run | what the script does | score |
|---|---|---|
| `full` | as above | 1.0 at step 424, three runs in a row (the last on the final source; the first two differed only in the 0.5 branch of the scorer and in debug prints) |
| `none` | both arms wait at their ready poses | 0.0 |
| `onearm` | the left hand turns the nut the same way; the right hand never holds the handle | 0.0: the nut's drag swings the loose handle up to about 20 degrees above level instead of advancing; the nut gets 25 degrees round, and the handle drops back when let go |
| `careless` | as `full`, but keeps turning: 60, 60 and 40 degrees | 0.0: the nut reached 124 degrees past its start, 34 past snug, so overtightened; without that record the final state (seated, level) would have scored 1.0 |

In its 2,400-step run Dream-1 scored 0.0: both arms left their ready poses and the right gripper hovered by the sponge in front of the tank, but neither arm reached the handle or the locknut, so the nut stayed a quarter turn loose.

In the simulation the washer spring pushes an overtightened nut back to snug once it is let go; on a real tank the nut would
stay tight. The overtightening record is what scores that case.

## Real trials

A real trial uses a real two-piece toilet tank (vitreous china, lid off, empty, or with its water at least 4 cm below the hole) with a
Kohler-type trip lever. Set the tank into a cutout in the bench (or stand it on a lower shelf beside it) so that its rim is 12 cm above
the bench top and the trip lever hole 8 cm above it, its front face along the line y = 0 facing the right arm and the hole's centre at
x = 20 cm, as in the setup; clamp the tank so it cannot shift. To prepare it, paint a line on the locknut's end face, hold the handle level, turn the locknut counterclockwise
(seen from inside the tank) until it first meets the spacer, mark that snug position on the bushing beside the nut's paint line, then
back the nut off a quarter turn clockwise and let the handle drop onto its stop. Put a strip of masking tape on the tank's front face round the hole and
mark the handle's level line and lines 5 degrees above and below it (a protractor or phone inclinometer). Mark the bushing a second time
30 degrees past the snug mark (a printed protractor disc held on the nut's axis). Film the handle and, from above the open tank, the nut, at 60 fps. The handle angle, the
nut's angle past its snug mark and the grippers' 3 cm clearance from the handle, escutcheon and nut are read from the video; a
nut dropped off the thread is seen in the tank.

## Assets

The distractors come from the Hugging Face dataset [`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets),
pinned to revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`cleaning_barSoap_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/cleaning_barSoap_a01)
- [`cleaning_sponge_b01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/cleaning_sponge_b01)
- [`consumable_paperTowel_roll_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/consumable_paperTowel_roll_a01)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time: masses are set
(soap 110 g, sponge 20 g, paper towel roll 200 g); the files authored none. Nothing large is committed: the files download at
first run.

The tank, the trip lever assembly and its locknut are built from meshes generated in `task.py` (SimReady has no toilet tank or
trip lever). `mechkit.py`, `demokit.py` and `simready.py` are copied from other open-source tasks in this repository
(`tasks/heat-shrink-lug/`).
