# Clear a platform-game level with a real gamepad

A 24-inch monitor at the back of the desk shows a side-on platform game: a small character at the left, a narrow gap, a brick
step, a wide gap and a flag at the right. An Xbox Wireless Controller lies face up between the arms in a cradle taped to the desk,
its bumpers toward the monitor. The instruction is only the goal: *"Use the game controller to get the character to the flag."*

The level is built so that each obstacle needs a different piece of the knack every gamepad player has: the **left stick** runs the
character (nothing moves inside a 20 % deadzone, full speed only past 90 % of the stick's throw), and **A** jumps, but only on a fresh
press, and the jump keeps rising for as long as A is held (the variable-height jump of most platform games). So:

1. the **narrow gap** needs a jump at the right moment (a late press drops the character in);
2. the **brick step** is 150 px tall: a tap of A (0.05-0.15 s) rises only 48-123 px, so A must be **held** about 0.2 s or more;
3. the **wide gap** (300 px) can only be cleared with a held jump at **nearly full stick** (at least 65 % of the throw) from a take-off
   window about 0.35 s long; at half stick the character falls in.

A must be let go between jumps, and the character has three lives: a fall restarts it at the left, and after the third fall the game
shows GAME OVER. One hand holds the stick deflected while the other presses and holds A at the right moments: two hands, two
different jobs, both timed against what is on the screen.

| File | What it is |
|---|---|
| `task.py` | The scene, the game (one function per step, all constants in `GAME`), the console that reads the controller and draws the picture, the scorer, and the closed-loop demo (`demo_script`, with failure modes) |
| `game.html` | The same level, physics and constants as a self-contained web page, for real trials with a real controller (below) |
| `mechkit.py` | The mechanism kit (articulations written as USD text, hooks, fingertip points), unchanged |
| `demokit.py` | The closed-loop demo runner, unchanged |
| `simready.py` | Loads the SimReady props (see Assets) |
| `task.json` | The task's details for the board |

## Requirements

- The `yam_arena` embodiment for `inspect-robots` (Isaac Lab-Arena), as in the program's walkthrough.
- Network access the first time the scene is built: `simready.py` downloads the SimReady packages into `~/.cache/simready`.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Use the game controller to get the character to the flag." --scorer task_score --max-steps 1200
```

The scripted, closed-loop demonstration (it reads only what a person at the desk could see: the controller's pose, the stick's tilt,
A's travel, and the character as drawn on the screen):

```bash
python demokit.py task.py ~/gamepad-demo                    # the recorded run: ~/gamepad-demo/full/demo.mp4, setup.png, result.json
python demokit.py task.py ~/gamepad-checks --no-record --mode none,onearm,mash,tap,walk,hold,grab
```

Nothing is randomised, so every seed gives the same scene. Runtime per trial: about 3-6 minutes of wall time on one L40 (Isaac start
plus up to 1,200 steps at about 3-5 steps/s). The demo clears the level at 11.5 s and reaches full credit at step 288 (14.4 s of
simulated time), identically in both recorded runs, with no life lost; the run limit, 1,200 steps (60 s), is about four times that
and leaves time for all three lives.

## The demo

`demo_script` in `task.py`:

- **Left hand** (on the stick): nearly straight down, leaned 15 degrees outward so the two wrists stay 3-4 cm apart, its fingers
  opened 1.2 mm wider than the stick's 15.5 mm cap on either side of it. It steers the stick as a thumb does, by pushing the cap's
  side, without squeezing it. Each step it puts its fingers where the stick should be (20.5 degrees right for full speed) plus a
  sideways correction learnt from the tilt it sees, since the arm gives a few millimetres under the stick's spring.
- **Right hand** (on A): fingers closed, tilted 35 degrees inward from its own side, so that only the lowest corner of its fingertips
  can touch A (the B and X buttons are 11 mm away). Pressing puts that corner 5 mm below A's top (A bottoms out at 1.5 mm); letting
  go lifts it 4 mm above.
- First a **calibration jump** in place: the script presses A and counts the steps until the drawn character leaves the ground
  (3 steps, 0.15 s: the arm's travel plus the game's one-step input sampling). Then it holds the stick right and, for each obstacle,
  presses and holds A for 0.45 s once the standing character will reach a take-off point after that delay (front edge at 495 px for
  the narrow gap, 700 px for the step, 1,375 px for the wide gap). In the recorded runs the full-height jump over the narrow gap
  lands the character against the foot of the step, so the second jump goes straight up from there and onto it. After a fall it
  does the same again from the start.
- On LEVEL CLEAR it centres the stick, lets go of A, opens the left hand, lifts both hands off and moves them back.

Failure modes (`--mode`), each run in the batched check: `none` does nothing; `onearm` runs with the stick only, never jumping;
`mash` holds A down from the start (one jump, in place, then no fresh press is ever made); `tap` only taps A (0.15 s); `walk` runs at
half stick (10.5 degrees, axis 0.5); `hold` plays the level but keeps both hands on the controller; `grab` pinches the stick and
lifts the controller out of its cradle.

## The game

The game is a small fixed-step model, the same in `task.py` (`GAME`, `Game.step`) and `game.html`. Screen pixels, 1920 x 1080, y up;
one game step per 20 Hz control step, the controller read once per step.

| Constant | Value |
|---|---|
| Character | 72 x 96 px, starts with its bottom centre at (130, 200) |
| Stick | axis = left-right tilt / 21 degrees (+ right); speed = 450 px/s x clip((abs(axis) - 0.20) / (0.90 - 0.20), 0, 1) |
| Acceleration | 1,800 px/s^2 toward the stick's speed; 2,400 px/s^2 to a stop inside the deadzone |
| Jump | on a fresh press of A while standing: 1,050 px/s up; gravity 2,400 px/s^2; letting go of A while rising cuts the upward speed to 150 px/s |
| A switch | closed past 1.0 mm of travel, open again above 0.7 mm |
| Level | ground (top 200 px) 0-520 px, narrow gap 520-610, ground 610-1,400 with a brick step 820-1,000 (150 px tall), wide gap 1,400-1,700, ground 1,700-1,920, flag pole at 1,830 |
| Lives | 3; a fall (the character's top below the picture) restarts it at the start after 0.5 s; GAME OVER after the third |

What the numbers mean in play: a jump held to its top rises 204 px and stays up 0.85 s (382 px at full speed); taps of 1, 2 and 3
steps rise 48, 88 and 123 px; at half stick (193 px/s) a held jump covers 164 px, short of what the wide gap needs; from 65 % of the
throw it can just be cleared, from a short window.

The picture is drawn on the monitor's own screen: flat, visual-only shapes 0.5-3 mm in front of its face (the approach of
`tasks/mouse-drag`, another team's open-source task in this repository), the character's shapes moved by their USD transform every
step and the hearts, progress markers and banners shown or hidden. The demo reads the character's position back from those drawn
shapes, never from the game's state.

## Scoring

Partial credit is the furthest point the character reached in any of its lives, which the game keeps and shows as four progress
markers at the top right of the screen.

- **1.0**: the level is cleared (the character touches the flag pole; LEVEL CLEAR shows) and afterwards, for 1 s, no gripper finger is
  within 3 cm of the controller, the stick is back within 3 degrees of centre and A is up (its switch open).
- **0.75**: the character stood on the ground beyond the wide gap, or the level was cleared without that end state.
- **0.5**: it stood on top of the brick step.
- **0.25**: it stood on the ground beyond the narrow gap.
- **0**: anything else.
- **0 whatever the game shows** if at any moment the controller moved more than 1 cm from where it lay or tilted more than 10 degrees
  (lifted out of its cradle), or a gripper finger came within 5 mm of the monitor's 54.2 x 17.2 x 39.4 cm outline box (stand
  included: from 8 cm in front of the screen's face to the back of the stand, x 0.404-0.576 m, the screen's full width, up to its
  top).

How the simulator judges each rule: "stood on" is the game's own landing test (the character's box comes down onto that block's top);
"cleared" is the character's right edge reaching the pole's x. "Within 3 cm of the controller" uses mechkit's 24 fingertip points
against the controller's outline box (sticks included), in the controller's frame. The stick angle is its left-right hinge (and the
fore-aft one, both within 3 degrees); A's travel is its slider. The controller's shift is its frame's origin against its first-step
position, and its tilt is its frame's angle from vertical (it rests at 1.1 degrees on its grips and triggers). The monitor test is
the fingertips against the monitor's outline box (the SimReady prop's bounds, 54.2 x 17.2 x 39.4 cm, stand included) grown by 5 mm. A harm is recorded once and kept until the next reset.

Results of the batched failure modes (scored as written above): see the table at the end.

## The controller

The controller is the main mechanism, built from mechkit primitives at the real size and mass of an Xbox Wireless Controller (model
1914): 153 x 102 x 61 mm, 287 g with two AA cells. Its body, grips and triggers are rigid colliders resting on the cradle's floor on
the grips and the triggers. The **left stick** is a two-axis gimbal (two hinges, left-right then fore-aft), 21 degrees each way,
spring-centred at 0.036 N m/rad (0.7 N at the cap's top at full throw); its cap is 15.5 mm across and 6 mm tall, its top 11 mm above
the face, and its side is a band of a sphere, so it keeps its width between two fingers whatever its tilt. The **face buttons** A, B,
X and Y are sprung sliders: 9.6 mm caps standing 3 mm proud, 1.5 mm of travel, 400 N/m (0.4 N at the 1.0 mm switch point). The
d-pad, right stick, bumpers, triggers and the small buttons are fixed shapes. The cradle (a 3D-printed tray with a 4 mm floor and
14 mm rims, 3 mm of play round the controller, open at the back between the grips) holds it in place against the small sideways
pushes of the stick and buttons, as a controller stand or an accessibility mount does; it does not stop it being lifted.

## Real trials

### Equipment

- An Xbox Wireless Controller (model 1914, https://www.xbox.com/en-US/accessories/controllers/xbox-wireless-controller, about USD 60),
  with two AA cells, connected to the computer by USB-C cable or Bluetooth.
- A 24-inch 1080p monitor and a computer with Chrome or Edge; open `game.html` from this folder and press F11 for full screen.
  The page reads the controller through the browser's Gamepad API (standard mapping: `axes[0]` is the left stick's left-right value,
  `buttons[0]` is A) once per 1/20 s step, with the same deadzone, speed curve, jump and level as the simulation. Menu (`buttons[9]`)
  or R restarts the level. Arrow keys and Space work too, only to check the page.
- A cradle: a tray 11.6 x 16.7 cm inside with 14 mm walls (3D-printed or stiff card), taped to the desk.

**Mapping the stick:** the browser reports the stick's deflection as a fraction of its full throw (-1 to +1), which is what the
simulation computes as tilt / 21 degrees. A real stick's travel differs a little from 21 degrees, but the game only sees the fraction,
so the thresholds (20 % deadzone, full speed from 90 %, a usable wide-gap jump from about 65 %) are the same on both.

### Layout (from the arm bases: x forward, y left, origin midway between them)

- Cradle centred at x 0.25 m, y 0.00, the controller face up, bumpers toward the monitor; left stick at x 0.262, y +0.037; A at
  x 0.251, y -0.037.
- Monitor at x 0.49 (screen face at x 0.487), centred on y 0, facing the robot; its screen's bottom edge about 10 cm above the desk.
- Distractors: keyboard pushed aside at x 0.62, y +0.47; mug at x 0.46, y -0.42.

### Measurement and reset

The game measures itself: the hearts show the lives left, the four progress markers show the furthest point, and LEVEL CLEAR or
GAME OVER shows at the end. The end state is checked on the run's video (or with a ruler): both grippers at least 3 cm from the
controller, the stick back at centre and A up for 1 s. Mark the controller's corners on the cradle with tape: a corner more than 1 cm
from its mark, or the controller lifted, scores 0. Reset in under a minute: press Menu (or R) to restart the level and put the
controller back on its marks.

### Safety

The robot works on a desk with a light plastic controller; the only risk is to the monitor, which the scoring penalises; its stand's
foot is 11 cm beyond the controller's front and its screen 19 cm.

## Importance and sources

Playing a console game with a gamepad is one of the most familiar things people do with their hands: 43 % of U.S. adults often or
sometimes play video games and 39 % own a dedicated games console
([Pew Research Center, 2018](https://www.pewresearch.org/short-reads/2018/09/17/5-facts-about-americans-and-video-games/)), and
205.1 million Americans aged 5 to 90 play regularly
([Entertainment Software Association, 2025](https://www.theesa.com/annual-esa-study-reveals-video-games-universal-appeal-across-generations/)).
A platform level is the classic first game. For a robot it is a hard, unusual kind of manipulation: the hands act on small, sprung
controls (a 15.5 mm stick cap moved 6 mm, a 9.6 mm button pressed 1.5 mm between neighbours 11 mm away), and success depends on
timing against a moving picture (a 0.35 s take-off window at full speed, A held at least 0.2 s for the step but let go before the
next jump), with the two hands doing different jobs at once.

Controller size and mass: [Wikipedia, Xbox Wireless Controller](https://en.wikipedia.org/wiki/Xbox_Wireless_Controller) (6.02 x 4.01
x 2.4 in) and [LANOC](https://lanoc.org/review/input-devices/rog-raikiri-pro?start=2) (Xbox Series X|S controller, 287 g with two AA
cells). The Gamepad API's standard mapping: [W3C Gamepad](https://www.w3.org/TR/gamepad/#remapping).

## Distinct from

| Task | What it is | How this differs |
|---|---|---|
| #3073 (accepted) claw machine | Steer a claw with a spring-centred joystick, one button to drop | The claw moves at one speed past a switch and waits for you; here the stick's deflection sets the speed, and the jumps are timed and held against a game that keeps moving |
| #1840 (accepted) drag a file with a mouse | One hand slides a mouse and holds its button | Two hands with different jobs at once; held-time and timing windows rather than a pointing path |
| #2534 (accepted) pinch-zoom a map on a tablet | Two-finger gesture on a touch screen | Physical sprung controls (stick and button) on a separate device, timing against a moving picture |
| #3739 (pending) whack-a-mole | Strike lit targets with a mallet in time | Indirect control through a gamepad: deflection sets speed, press length sets jump height, presses must be fresh |

No task on the board plays a video game with a gamepad; console hardware tasks there (inserting a disc, docking a console) do not use
the controller.

## Assets

The monitor, keyboard and mug come from the Hugging Face dataset
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`computer_monitor_office_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/computer_monitor_office_a01) (the monitor; fixed in place)
- [`computer_keyboard_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/computer_keyboard_a01) (distractor; mass set to 0.55 kg)
- [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01) (distractor; mass set to 0.30 kg)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time: masses set as
listed (the monitor has none: it is fixed), the files' grasp-annotation curves deactivated, and each prop made one rigid body. The
SimReady files are not committed: they download on first use. The game's picture is drawn over the monitor's screen area (its
`unique_decal_screen` face) by `task.py`.

The controller, its cradle and the picture are custom primitives (`mechkit.py` meshes): SimReady has no game controller.

`mechkit.py` and `demokit.py` are taken unchanged from another team's open-source task in this repository, `tasks/heat-shrink-lug/`,
and the way the picture is drawn on a monitor (flat visual-only shapes moved by their USD transforms) follows `tasks/mouse-drag/`,
another team's open-source task here. `simready.py` is the loader from `tasks/cutters-off-pegboard/`, unchanged apart from its
docstring.

## Results

Scripted demo, two recorded runs (`python demokit.py task.py OUT`): task score 1.0 both times, success at step 288 (14.4 s), level
cleared at 11.5 s with all three lives left, no blank frames.

Dream-1, one run to the 1,200-step limit: 0.0. It reached toward the controller with its right arm, but the character never
left the start.

Failure modes, one batched check (`--no-record --mode none,onearm,mash,tap,walk,hold,grab`):

| Mode | What it does | Score | Why |
|---|---|---|---|
| `none` | nothing | 0 | the character never moves |
| `onearm` | full stick, never presses A | 0 | three falls into the narrow gap |
| `mash` | A held down from the start, full stick | 0 | one jump in place, then no fresh press: three falls into the narrow gap |
| `tap` | only 0.15 s taps of A | 0.25 | clears the narrow gap, can't get up the 150 px step |
| `walk` | half stick (axis 0.5), held jumps | 0.5 | up on the step, then three falls into the wide gap |
| `hold` | plays the level, keeps both hands on the controller | 0.75 | LEVEL CLEAR without the hands-off end state |
| `grab` | pinches the stick and lifts the controller | 0 | the controller moved 11 mm and tilted 10 degrees out of its cradle |
