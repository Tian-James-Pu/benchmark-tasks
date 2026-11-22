# Hang a 24-inch TV on its tilt wall mount

A 24-inch TV with its two mount brackets already bolted on stands on the TV console, leaning against the wall just
under the mount's wall plate. The robot is told only "Hang the TV on its wall mount." It has to do what the mount's
own manual draws as a two-person job: take the 3.3 kg TV by its side edges, from in front of a wall that runs well
past the TV on both sides, pull it a couple of centimetres off the
wall so that the hooks at the top of both brackets (out of sight behind the panel) clear the plate's top rail, lift
it until the hooks are over the rail, bring it in, and lower it so that both hooks drop over the rail
together. The lower ends of the brackets then come to rest against the plate's bottom rail and the TV hangs flat.
The hands let go and move clear.

Lifted straight up from where it leans, the hooks run into the underside of the top rail and the TV will not go up.
Tipped with its top away from the wall, the hooks pass outside the rail and the TV drops; held crooked, one hook
catches and the other does not. Let go before both hooks are on, and the TV falls back onto the console or onto its
face.

## Response to review

| Request | Change |
|---|---|
| "Verify that the grasp succeeds within the actual gripper force limit; the demo commands 50 N per finger." | The demonstration now pinches with 30 N a finger (`GRIP_N` in `task.py`), under the YAM gripper's roughly 40 N, and hangs the TV at its real 3.27 kg: two fresh recorded runs each scored 1.0 (see "The demonstration"). The hold needs a pad friction coefficient of only 0.27. README: "The demonstration", "Real trial". |
| Construct validity: "The wall is narrower than the TV specifically so the wrists can pass around its edges, unlike the continuous wall where the task is normally performed." | The wall is now 100 cm wide, 22 cm past the TV on each side (`WALL_W` in `task.py`; `setup`, `fidelity`; README: "The real objects", "Width of the wall", "Real trial" parts list). Nothing about the wall is shaped for the robot. |
| Feasibility: "The submission states that the YAM cannot reach the TV edges against a full-width wall, and the demonstrated grasp uses the narrowed wall." | New grasp, checked offline for reach and wall clearance first, then in simulation: each gripper leans 60 degrees out from vertical with its wrist outboard and is turned 24 degrees about its own side axis, so its wrist stays back from the wall (`grip_rotation` and `demo_script` in `task.py`). Over the whole demonstration every arm link stays at least 2.4 cm in front of the wall. The first submission's grasp (35 degrees out) was rerun against the full-width wall as a red team: the wall stops the left hand 47 mm short of the edge and the TV ends crooked, 0.5, not 1.0 (README: "Failure checks"). |
| "Use a wall extending beyond the TV on both sides and demonstrate the complete task with a reachable grasp or a specified alternative embodiment. Ensure the demonstrated grip is within that hardware's force limits, and update the real-trial setup and demo accordingly." | Same embodiment (two YAM arms). Complete task demonstrated against the 100 cm wall at 30 N a finger: 1.0 in two recorded runs. The failure checks were rerun on the new scene, and Dream-1 was run once more on it (README: "Model run"). Real trial: a 100 x 56 cm plywood wall section, and the 30 N grip noted. |

## Run and score

Everything runs from the repository root with the Isaac Lab Arena environment sourced
(`source ~/IsaacLab-Arena/yam.env`).

    # a model's trial (Dream-1 or any policy), scored by the task's own scorer
    inspect-robots run --embodiment yam_arena --policy <policy> \
      -E task=tasks/hang-32in-tv-on-tilting-wall-plate/task.py -E object_jitter=0 -E control_hz=20 \
      -E cam_width=640 -E cam_height=360 --instruction "Hang the TV on its wall mount." \
      --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/hang-tv

    # the scripted closed-loop demonstration (reads the TV's pose, never the scorer)
    cd tasks/hang-32in-tv-on-tilting-wall-plate
    python demokit.py task.py ~/yam-eval/hang-tv/demo --mode full          # writes full/demo.mp4, setup.png, result.json
    python demokit.py task.py ~/yam-eval/hang-tv/checks --no-record --mode none,onearm,careless,drop,holdon

A trial is at most 2,400 control steps (2 minutes at 20 Hz). The demonstration succeeds at step 442 (22 s of
simulated time); on a shared GPU a trial takes about 3-4 minutes of wall time once shaders are compiled. Nothing in
the scene is random: the seed only matters if `object_jitter` is set, and it is 0 here.

## The real objects

| Object | Real product and source | In the simulation |
|---|---|---|
| TV | LG 24LQ520S-PU, 24-inch HD smart TV: 22.2 x 13.4 x 2.3 in (564 x 340 x 58 mm) and 7.2 lb (3.27 kg) without its stand, VESA 75 x 75 ([LG spec sheet](https://content.abt.com/documents/165855/lg-smart-tv-24LQ520S-PU-spec-sheet.pdf)) | Full size and real mass: a 564 x 340 x 20 mm panel with its bezel, and a 400 x 250 x 38.4 mm rear housing (58.4 mm deep overall), 3.27 kg, centre of mass 20 mm behind the screen at mid-height |
| Wall mount | Monoprice EZ Series tilt wall mount, model 5915: wall plate 17.5 x 8.5 in (444.5 x 216 mm), the TV 1.9 in (48 mm) out from the wall, steel, minimum VESA 75 x 75, up to 154 lb ([sell sheet](https://downloads.monoprice.com/files/specsheets/5915_Sellsheet_200625.pdf)); hanging step drawn in the [installation manual](https://downloads.monoprice.com/files/manuals/5915_Manual_170918.pdf) | Full size: plate 444.5 x 216 mm of 2 mm steel whose top and bottom edges turn out 20 mm and up 14 mm into two rails, each a channel 12 mm deep behind its lip; two 25 mm wide, 20 mm deep, 236 mm long brackets 75 mm apart on the TV's VESA holes, each with a hook (a 3 mm tab and a 4 x 10 mm down-turned finger) at the top and a foot at its lower end that bears on the bottom rail |
| Wall | A section of stud wall, the plate lag-bolted into two studs 16 in apart | A 100 x 56 x 2 cm board standing on the table, 22 cm wider than the TV on each side, the plate lag-bolted to it, with a pencil mark (3 cm vertical line with a 2 cm tick) 38 cm up, on the line where the TV's centre goes |

The sell sheet gives the mount's display range as 32-70 inches, with the footnote "The minimum and maximum display
sizes are only approximations. Your TV will work with this mount as long as it has a compatible VESA pattern and
maximum weight." The LG's VESA 75 x 75 pattern is the sheet's stated minimum, and 3.27 kg is far under 154 lb. Hung,
the TV hides the plate completely (the plate is 120 mm narrower and 124 mm shorter than the TV).

## Scale and mass

Nothing is scaled, and nothing is lightened.

- **Mass:** the TV is modelled at its real **3.27 kg** (7.2 lb on LG's sheet), about 1.6 kg in each hand. A 32-inch
  set (73 cm wide, about 4.2 kg) puts its side edges beyond where the two arms can hold it; rather than lighten a
  bigger TV, the task uses a smaller real one at its real mass.
- **Height on the wall:** in a home the plate would be 1-1.2 m up a wall. Here the wall is a board standing on the
  table, which stands for the console the TV is hung over: the plate's centre is 20 cm above the table and the hung
  TV's bottom edge 2 cm above it. Everything relative to the wall, the plate and the TV (hook travel, rail
  clearances, the 48 mm standoff) is as on the real mount. The board is a mounting stand-in for the wall, not a
  shrunken one.
- **Width of the wall:** the board is 100 cm wide, so the wall runs 22 cm past the TV (56.4 cm wide) on each side, as
  a continuous wall does; nothing about the wall is shaped for the robot. The hands hold the TV's side edges from in
  front of the wall: each gripper leans 60 degrees out from vertical, wrist outboard, turned 24 degrees about its own
  side axis so the fingers point slightly towards the wall and the wrist stays back from it. In the demonstration the
  part of either arm nearest the wall (the wrist link) stays at least 2.4 cm in front of it throughout (computed from
  the recorded joint angles on the arm's own geometry).
- **Approximations, not rescaling:** the TV's rear-housing outline, where its VESA holes sit (18 cm above its bottom
  edge here), its centre of mass, and the bracket and rail profiles are not given in the sources; they are drawn
  from the manual's pictures at plausible sizes and are listed in `task.py`.
- **Left out:** the manual's step 3 also shows a Phillips screwdriver next to the brackets' lower ends, which we read
  as a locking screw (the manual has no text). Hanging is the task; no lock is modelled or tightened.

## How the simulator judges each rule

The scorer runs every control step (20 Hz) from the TV's pose and from points on both grippers' fingers. Distances
are in the plate's frame (x out of the wall, z up).

- **Hook seated** (each of two): the bottom of the hook's finger is inside the top rail's channel (between the lip and
  the plate, 16 mm wide) and 6.5-12.5 mm below the lip's top. A hook whose tab rests on the lip has its finger 10 mm
  down, and one whose finger rests on the channel's floor 12 mm down, so this means the hook is on the rail: a hook
  held more than 3.5 mm up, or a TV still held above the rail, fails it.
- **Lower end home** (each of two): the wall-side face of the bracket's foot is within 3 mm of the bottom rail lip's
  front face (and not more than 1 mm into it), at the lip's height. This is the TV hanging flat against the plate
  rather than swinging out from it.
- **Centred:** the TV's centre is within 15 mm sideways of the pencil mark.
- **Level:** the TV's roll (its width against the horizontal) and tilt (its depth axis against the horizontal) are
  each within 2 degrees.
- **Hands clear:** every sampled fingertip point is at least 4 cm from the TV's box (screen to hook tips).
- **Still:** the TV moves less than 1 cm/s and turns less than 0.05 rad/s.
- **Dropped or fallen (recorded):** at any step the TV moved down faster than 0.5 m/s (a free fall of 13 mm) or
  tipped more than 30 degrees from upright. This is kept for the rest of the trial and cleared only at a reset.

Possible scores:

| Score | When |
|---|---|
| 1.0 | Both hooks seated, both lower ends home, centred, level, hands clear and still, all continuously for 1 s (the trial then ends), and never dropped or fallen |
| 0.5 | Both hooks seated and hands clear, but not the full set above |
| 0.25 | One hook seated and hands clear |
| 0 | No hook seated, or a hand within 4 cm of the TV, or the TV ever dropped or fell |

A trial that runs out of steps scores what holds at its last step, so credit does not stick: a TV that slides off
later loses it.

## The mechanism

- The TV is one free rigid body with the panel, the housing, both bracket bodies, both hooks and both feet as box
  colliders. It hangs only by contact: nothing attaches it to the plate. The hooks rest on the top rail's lip, so the
  TV can slide sideways along the rail and pivot on it, as the real one does before it is locked. Its
  centre of mass is in front of the hooks, so a TV hung on both hooks swings its bottom in until the brackets' feet
  bear on the bottom rail's lip, and settles flat against the plate.
- The plate's rails are fixed steel boxes; friction is 0.4/0.3 on the mount, 0.8/0.7 on the TV.

## The demonstration

`demo_script` in `task.py` is closed loop: it reads the TV's pose (as a camera could), never the scorer.

1. Each hand comes out beside its side edge of the TV, in front of the wall, then slides in along its own approach
   axis: the gripper leans 60 degrees out from vertical (wrist outboard) and is turned 24 degrees about its side axis
   (fingers slightly towards the wall, wrist back from it), so that its fingers straddle the 20 mm rim 2.7 cm in from
   the edge, 17 cm above the TV's bottom edge (level with its centre of mass), and pinch it with 30 N a finger (because of the 24-degree turn the pads meet the rim at that angle, not flat). That
   is under the YAM gripper's roughly 40 N; holding 1.6 kg a hand between two pads needs a pad friction coefficient of
   only 0.27.
2. Each hand servoes on its own measured position as it starts to take the weight; then both pull the TV 22 mm off
   the wall, its bottom edge sliding on the console, so that the hook fingers come out from under the top rail.
3. Reading the TV's pose, they steer by where its hook fingers are (behind the panel, computed from the pose) and
   lift it 3.7 cm until the fingers are 7 mm over the rail's lip, centre it on the pencil mark, and tip its top 2.5 degrees towards the wall by turning both grips, correcting twice for the arms'
   droop under the load.
4. They bring it in until the hook fingers are over the middle of the channel and the brackets' feet are still clear of the bottom
   rail, correct again, and lower it 2.6 cm so that both hooks drop over the rail at once; they go on down another
   4 cm so that the rail takes the whole load before they open. The TV swings plumb on its hooks until the brackets'
   feet rest on the bottom rail.
5. They open, back off the edges along the approach axis, move back 6 cm, then out to the sides over the console, and wait.

**Result:** two fresh runs of `python demokit.py task.py OUT --mode full` on the GPU, rendered as the harness renders,
each scored **task_score 1.0** (success at step 442, 22.1 s; no blank frames), with the 100 cm wall and 30 N a finger.
In a run of the same script with the TV's pose printed, the hung TV sat 0 mm from the pencil mark with roll 0.0 and
tilt 1.2 degrees (both hook fingers 9.9 mm below the lip top), both feet against the bottom rail, the fingers 7 cm
from the TV. From that run's recorded joint angles on the arm's own link geometry, the nearest part of either arm to
the wall (the left wrist link, while the TV is brought in over the rail) stayed 24.6 mm in front of it.

The demonstration reads the TV's pose from the simulator, as a camera-based pose estimate would give it; it never
reads the scorer.

## Failure checks

Run on the GPU with `python demokit.py task.py OUT --no-record --no-cameras --mode none,onearm,careless,drop,holdon`
(the same script with a cheat mode; scores are the task's own `score()`):

| Variant | What it does | Score | Why |
|---|---|---|---|
| `none` | Does nothing for 32 s | 0.0 | The TV stays leaning on the console; no hook is on the rail |
| `onearm` | Pinches one side edge and lifts with that hand alone, then lets go | 0.0 | One hand cannot lift the 3.3 kg TV off the console; it stays leaning, no hook seated |
| `careless` | Lifts straight up from where the TV leans without pulling it off the wall first, then pushes it in and lowers | 0.0 | The hook tabs jam under the top rail after 3 mm; the TV never gets over the rail and ends back on the console |
| `drop` | Lifts the TV until its hooks are 7 mm over the rail but lets go before bringing them in over the channel | 0.0 | Nothing catches the hooks; the TV falls back onto the console with no hook seated |
| `holdon` | Hangs the TV correctly, lets go, then grips the rim again and holds on | 0.0 | Both hooks seated and both feet home, but the fingers are on the TV: hands-clear fails, which scores 0 |

Red team for the review (the same demonstration script with the grasp swapped, run on the GPU without cameras): the
first submission's grasp (gripper leaning 35 degrees out, wrist towards the wall) run against the
100 cm wall. The wall stops the left hand 46.7 mm short of where it is sent on the TV's edge, so it holds the TV badly;
the TV still ends on both hooks but crooked (one bracket foot off the bottom rail, tilt 2.0 degrees) and scores 0.5,
the tier for both hooks seated, not 1.0. The demonstrated grasp above is the one that reaches the edges against the
full wall.

Not run as a variant: a TV hung on both hooks but off the mark, or tipped more than 2 degrees, scores 0.5 by the
table above.

## Distinct from

| Nearest task | What it tests | How this differs |
|---|---|---|
| TK-Q35DWQ (not accepted, open for revision): hang a framed picture on a wall hook by the wire behind it | A light frame's flexible wire onto one wall hook, then levelled | A 3.3 kg rigid panel carried by two hands; two rigid hooks, both blind, onto a rail at once; the TV must first be pulled off the wall so its hooks clear the rail, then brought in and lowered |
| TK-JE227S (pending): hang a wall clock back on its screw by the keyhole hanger | One keyhole slot over one screw head, one hand | Two hooks at once on a wide, heavy panel that needs both hands to carry and hold level |
| TK-BFJD6N (accepted): hang a blind headrail in two gated brackets | A long, light rail pushed up into two visible clip brackets | The engaging parts are hidden behind the panel, which hangs by gravity from hooks rather than being clipped in |
| BEHAVIOR task 34, [hanging pictures](https://behavior.stanford.edu/challenge/tasks/34_hanging_pictures.html) | A poster onto a wall nail | A real TV and a real tilt mount at full size and mass, two hooks and a bottom rail, bimanual |

## Importance and sources

- **The job.** TVs are everywhere in homes: the U.S. Energy Information Administration's Residential Energy
  Consumption Survey found an average of 2.3 TVs in use per household in both 2015 and 2020
  ([EIA, Today in Energy, 3 May 2022](https://www.eia.gov/todayinenergy/detail.php?id=52278)). Hook-on tilt mounts
  like this one cost tens of dollars (this one about $31) and come with a picture manual for the buyer to follow.
- **The knack, from the maker.** The mount's [installation manual](https://downloads.monoprice.com/files/manuals/5915_Manual_170918.pdf)
  draws the hanging step (step 3) with a two-person icon: the TV is tipped so the brackets' top hooks go over the
  plate's top rail, then its bottom is brought in to the wall so the brackets' lower ends meet the bottom rail.
- **Limits of the evidence.** We found no survey giving the share of TVs that are wall-mounted, so that is not claimed
  here. The manual's pictures, not text, are the source for the hanging sequence.

## Model run

Dream-1 was attempted once on the revised scene (the 100 cm wall), with the instruction above and the 2,400-step
limit. Its API refused the request before the first action, so there is no model video: "DreamAPI refused the inference request: DA-4000 BAD_REQUEST (malformed): unsafe instruction, refused: Hanging a TV requires lifting heavy equipment beyond normal tabletop manipulation." (On the first
submission's scene it had also refused: "...unsafe instruction, refused: Hanging a TV carries significant risk of
dropping heavy electronics and injury.")

## Real trial

**Parts (about $220):** an LG 24LQ520S-PU (listed at $169.99 on [LG's page](https://www.lg.com/us/monitors/lg-24lq520s-pu)), a Monoprice 5915 mount (about $31 at [Zoro](https://zoro.com/monoprice-tilting-wall-mount-for-32-55-tv-5915/i/G8789317)), a 2 x 4 stud frame faced
with a 100 x 56 cm sheet of 12 mm plywood (the wall section, 22 cm wider than the TV on each side, two studs 16 in apart under the plate) clamped upright to the edge of a 75 cm high table, two
5/16 x 2.5 in lag bolts, a pencil, a tape measure, a phone with a level app, and a phone camera.

**Preparation:** bolt the brackets to the TV's VESA holes with the screws in the mount's kit. Lag-bolt the plate to
the wall section, level, with its centre 20 cm above the table and its middle on the wall's centre line. Draw the
pencil mark (a 3 cm vertical line) on the centre line 38 cm above the table. Lay a thin towel (under 1 cm thick) on the table
under the plate to protect the screen's bottom edge, and pad the robot's finger pads with tape. The
demonstration's grip is 30 N a finger, under the roughly 40 N the YAM gripper can give.

**Starting position:** the TV stands on its bottom edge on the table under the plate, screen facing the robot,
leaning back so its hooks rest against the plate below the top rail; its centre is 25 mm left of the pencil mark
(check with the tape along the table edge). Screwdriver, candle and notebook as in `setup`.

**Measurement, read before anyone touches the TV:** look behind each side of the TV from above (a phone photo is
enough): each hook is over the top rail with its tab on the lip. Look along the bottom edge: each bracket's lower end
is against the bottom rail's lip (a 3 mm hex key does not fit between them). Tape: the
TV's centre (half its width from its right edge) against the pencil mark, within 15 mm. Phone level app on the TV's
top edge and its screen: within 2 degrees. The robot's fingers at least 4 cm from the TV, and the TV still, for 1 s
on the video. Any drop or topple seen on the video scores 0.

**Reset (about 2 minutes):** lift the TV 2 cm off the rail, pull it out 3 cm, lower it to the table and lean it back
under the plate at the start position.

**Safety:** the TV hangs 2 cm above a padded table, so a fall is at most a few centimetres onto the towel. Keep hands
away from the panel while the robot carries it.

## Assets and helpers

- `mechkit.py`: the mechanism kit, copied unchanged from another team's open-source task in this repository
  (`tasks/heat-shrink-lug/`), with its credit line.
- `demokit.py`: the closed-loop demo runner, copied unchanged from `tasks/heat-shrink-lug/` (by way of
  `tasks/start-car-with-jammed-steering-lock/`).
- `simready.py`: the SimReady loader, adapted from `tasks/cutters-off-pegboard/`.
- The TV, the mount and the wall are built in `task.py` from boxes: the TV must carry the brackets, hooks and feet as
  one body, which no SimReady TV can do.
- Distractors, NVIDIA SimReady props from [nvidia/simready-assets](https://huggingface.co/datasets/nvidia/simready-assets)
  (© NVIDIA, CC-BY 4.0, at the revision pinned in `simready.py`), loose, with masses set at load time and collision
  as convex hulls; scoring ignores them:
  - [handTool_screwdriver_phillips_ph2_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_screwdriver_phillips_ph2_a01) (the PH2 screwdriver the brackets were fitted with), 0.10 kg;
  - [decor_candle_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/decor_candle_a01) (a candle on the console), 0.35 kg;
  - [misc_notebook_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/misc_notebook_a01) (the mount's instruction booklet), 0.12 kg.
