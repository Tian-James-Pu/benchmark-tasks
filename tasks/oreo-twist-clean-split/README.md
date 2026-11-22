# Twist an Oreo open and lay both halves on the plate

An Oreo lies flat on the table beside a plate. The robot must open it into its two halves and leave both halves on
the plate, lying flat and apart, with its grippers clear. In the simulation the cookie opens only by counter-rotation:
the two wafers are turned against each other about their shared axis until the creme lets go of the top wafer.

Instruction: *"Open this Oreo into its two halves and put both halves on the plate."*

The knack: the cookie is 11 mm thick and each wafer is 3.5 mm thick, held only at its rim. One hand has to pinch just
the top wafer and lift the cookie. The other hand has to pinch just the base wafer below it, from a different side,
without touching the top wafer. Then one hand holds still while the other turns its wafer about the cookie's axis.
The creme yields at about 0.1 N m after a turn of about 10 degrees. Turning the whole cookie in one hand, or pulling
the wafers straight apart, does not open it. Each pinch must also stay gentle: more than 15 N on a wafer cracks it,
and a cracked cookie scores 0.

## Response to review

| Request (quoted) | Change |
|---|---|
| "Make wafer integrity a scored condition in real trials" | A cracked wafer scores 0 in the real trial: the end photo (from above, 20 cm, ruler in frame) must show no crack, chip or broken-off edge longer than 2 mm on either wafer. README "Real trial", measurement 2; `task.json` scoring. |
| "add a calibrated simulation rule that penalizes grip forces sufficient to break the wafers; demonstrate that the full-score run still succeeds" | `task.py` now reads each fingertip's normal force on each wafer every control step. More than 15 N on a wafer for 2 control steps in a row marks it cracked, and the trial scores 0 (recorded until reset). The 15 N is calibrated from published biscuit bend data (README "Wafer breaking force"; an estimate, labelled as such, with a check to run on the trial pack). The demo's grip peaks at 7.3 N, and two fresh full runs on the final source still score 1.0 (README "Validation", with the force log). Red team: a 40 N grip cracks the wafer and scores 0. |
| "Change the plate check to require each half to rest on the plate rather than merely satisfy an upper height bound, so a plate covering the halves cannot earn credit" | `_on_plate` now needs the plate upright (within 10 degrees) and the half's lowest point 0 to 4 mm above the plate's floor surface (1 mm tolerance below), as well as the centre within 64 mm. Red team: the halves left on the table with the plate set upside down over them score 0.5 (opened, not on the plate). The plate condition fails twice over: the plate is at 180 degrees, and the halves sit 1.6 mm below where its floor would be. Real trial: measurement 3. |
| "specify how the real-trial video verifies the required relative twist, or align the real and simulated opening criteria" | Ticks are drawn across both wafers' rims every 45 degrees. A named front camera (plus a side camera at 90 degrees) records the trial. In the last frame before the halves part, the tick offset along the rim must be at least 3.5 mm: 10 degrees at the 22 mm radius is 3.8 mm, less up to 22.5 degrees of foreshortening. This is the sim's criterion: a turn of at least 10 degrees before the creme lets go. Halves pulled apart without it score 0 in both. README "Real trial", measurement 1. |

## Importance and sources

- **Most adults have done it.** Twisting the cookie apart is the ritual the brand builds its identity on. The brand
  calls Oreo the "twistable, lick-able, and dunk-able" cookie ([oreo.com, About us](https://www.oreo.com/about-us)).
  Its owner calls the "twist, lick, dunk" ritual central to the cookie wherever it is sold
  ([Mondelez International, Oreo](https://mondelezinternational.com/our-brands/oreo)). The study below also notes the
  habit of twisting a sandwich cookie apart before eating it.
- **The physics is published.** Owens, Fan, Hart and McKinley, "On Oreology, the fracture and flow of 'milk's favorite
  cookie'", *Physics of Fluids* 34, 043107 (2022), open access, CC BY 4.0
  ([doi:10.1063/5.0085362](https://doi.org/10.1063/5.0085362);
  [MIT DSpace](https://dspace.mit.edu/handle/1721.1/145488)). The paper treats the cookie as a parallel-plate
  rheometer: the wafers are the plates and the creme is the sample. Counter-rotating the wafers shears the creme until
  it fails. Two findings matter here:
  - The torque to break an Oreo open in pure rotation is about 0.1 N m (Sec. III A).
  - In the paper's rheometer tests, failure is typically adhesive, with nearly all (95%) creme left on one wafer.
    That wafer is the one the creme was deposited on at the factory. Its orientation in a pack is consistent, so it
    can be identified from a test twist.

  The MIT press release reports the larger hand-and-device survey of more than 1,000 cookies, in which the creme
  stayed on one wafer about 80% of the time
  ([MIT News, 2022](https://news.mit.edu/2022/oreometer-cream-0419)).
- **The product.** Nabisco Oreo Chocolate Sandwich Cookies, Original. Serving size is 3 cookies (34 g), so one cookie
  is 11.3 g ([oreo.com product page](https://www.oreo.com/products/oreo-cookie.md)). The cookie is about 44 mm across
  (1.75 in) and about 11 mm thick. The wafer and creme thicknesses (3.5 mm and 4 mm) and the creme diameter (35 mm)
  are approximate values from that total, not published figures. Check them with calipers on the pack used for real
  trials.
- **Why it is a robot test.** Two grippers hold one small, light, layered object at different heights of its rim:
  one hand steadies and the other turns. There is one dependent sequence: lift by the top wafer, take the base wafer
  from below, twist, separate, then set down each half.

Evidence limits: the 10-degree failure angle is an estimate. The paper reports the failure stress and modulus; the
angle follows from a failure strain of about 0.8 at 4 mm creme height and 17.5 mm radius. The straight pull-off force
is also our own estimate, about 48 N. It uses the squeeze-flow pull-off of a yield-stress layer, 2 pi sigma R^3 / 3H,
with the paper's expected yield stress of 17 kPa. The paper itself reports that combining push or pull with the twist
did not change the creme distribution.

## Distinct from

| Nearest | What it tests | Why this differs |
|---|---|---|
| #3015 break one row off a chocolate bar (rejected) | bending a bar to snap it along a groove | no bending or snapping here; two wafers are counter-rotated about a common axis |
| #2947 split joined chopsticks (rejected) | pulling or prying two sticks apart | pulling does not open an Oreo; the opening move is a torsional shear |
| #2938 snap a banana off a bunch (rejected) | bending one fruit off a stem | one hand holds, the other turns about a shared axis; nothing is bent |
| #1436 twist an ice-cube tray (accepted) | flexing a tray by twisting its ends | here the two parts are separate rigid wafers held at their rims, and the twist separates them |

Board check: no task on the board (accepted, pending or rejected) names an Oreo or a sandwich cookie.
We found none in LIBERO, RoboCasa, SimplerEnv, BEHAVIOR, DataCenterBench or RoboDojo.

## Run and score

Environment: `source ~/IsaacLab-Arena/yam.env`. From this folder:

    # model run (any policy), scored by the task's own scorer
    inspect-robots run --embodiment yam_arena -E task=tasks/oreo-twist-clean-split/task.py \
      -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Open this Oreo into its two halves and put both halves on the plate." \
      --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/oreo

    # the scripted closed-loop demonstration (writes OUT/full/demo.mp4, setup.png and result.json with task_score)
    cd tasks/oreo-twist-clean-split && python demokit.py task.py OUT --mode full

    # red-team modes of the same script
    python demokit.py task.py OUT --fast --mode crush,none,onearm,pull,careless

The start layout is fixed (no object jitter). Seeds are not used. A demo trial runs about 32 s of simulated time, 640
control steps, and ends when the 1 s hold is met. Wall time is about 3-7 minutes on a GPU with cameras.

## How the simulator models it

- **Wafers** (`mechkit.py`): each is one rigid body, a 16-sided disc 44 mm across and 3.5 mm thick, built from eight
  boxes. The base wafer carries the creme disc (35 mm across, 4 mm high) as part of the same body, 7.4 g. The top
  wafer is 3.9 g.
- **Creme**: a twist bond between the wafers. It is a revolute joint about the cookie's axis on a torsion spring of
  0.57 N m/rad. Once the relative turn passes 10 degrees (0.1 N m) the joint is switched off for good, and the wafers
  are then separate bodies that only touch. The joint is rigid in pull and bending. Pulling, prying or squeezing the
  cookie does not open it, and turning the whole cookie in one hand does not either.
- **Contacts**: the wafers have a 6 mm torsional friction patch, so that a fingertip pinch on the rim can carry a
  twist. Free kit parts are solved with 64 position iterations. These are the two local changes to `mechkit.py`; both
  are marked "local change" in the file.
- **Wafer integrity** (the wafers are rigid, so cracking is scored rather than simulated): every control step, each of
  the four fingertips' normal force on each wafer is read as its gripper's finger-drive force (the force the
  fingertips push with; in a pinch both fingertips carry it) whenever a point of that fingertip's pad is within 4 mm
  of that wafer. More than 15 N on a wafer for 2 control steps in a row (0.1 s) marks the wafer cracked for the rest
  of the trial, and the trial scores 0. The threshold is the estimate below.
- **What the score reads**: wafer poses, the joint's state, the plate's pose, the fingertip positions of both
  grippers and their finger-drive forces. Nothing is triggered by proximity alone.

## Wafer breaking force (estimate)

We found no published breaking force for an Oreo wafer: texture-analyser studies of cookies report other products,
and Owens et al. test the creme, not the wafer. So 15 N per fingertip is our estimate from comparable biscuit data,
not a measurement:

1. **Biscuit strength.** Thermo Fisher application note V293 (Oldörp, "Crunchiness of biscuits: texture analysis on
   a rheometer", 2021) breaks fresh commercial biscuits in three-point bending over a 50 mm span: peak force
   5.95 ± 0.54 N (5 biscuits). The note does not give the biscuit's width or thickness. Taking 40-55 mm wide and
   4-6 mm thick, the flexural strength 3FL/(2bh²) is about 0.2-0.7 MPa, about 0.36 MPa for 50 x 5 mm.
2. **How a gripper breaks an Oreo wafer.** The fingertips press on the 3.5 mm rim. A wafer squeezed across its whole
   diameter would not split until far higher forces (Brazilian test, F = σπDt/2, about 87 N at 0.36 MPa). The weak
   point is the rim crumbling under the fingertip. Taking the crushing stress equal to the flexural strength (lower
   than real crushing strength, so this errs low), over a 3.5 mm x 8-12 mm contact patch of a padded fingertip, the
   rim gives way at about 10-29 N. The central value, 0.36 MPa over 3.5 x 12 mm, is 15 N.
3. **Rule.** BREAK_N = 15 N, held for 2 control steps. The demo pinches with 6-7 N (force log below), less than half
   that. The YAM's full grip, about 37-40 N a finger, is well over it.

How to check the estimate on the trial pack (not yet done by us): press 10 cookies edge-on onto a kitchen scale with
a 10 mm-wide rubber-tipped dowel until the rim cracks, and record each reading. The sim threshold should be at or
below the pack's lowest reading.

## Scoring

Opened = the two wafers have been turned at least 10 degrees against each other, and the creme has let go. In a real
trial the same is read from video (see below). Cracked = a fingertip pressed on a wafer with more than 15 N for 2
control steps in a row; in a real trial, the end photo shows a crack, chip or broken-off edge longer than 2 mm on
either wafer. On the plate = resting on its floor: the plate upright (within 10 degrees), the half's centre within
64 mm of the plate's centre, and the half's lowest point 0 to 4 mm above the plate's floor surface (1 mm tolerance
below, for contact). A half under an upside-down plate, on the other half, or held above the plate is not on it.

| Score | Condition |
|---|---|
| 0 | never opened; or a wafer cracked; or a half ever fell off the table (centre over 3 cm below the top); cracking and falling are recorded and cleared only at reset |
| 1.0 | opened; both halves on the plate; each flat within 20 degrees (either face up); centres at least 45 mm apart; every fingertip at least 3 cm from both halves; both still; all held for 1 s |
| 0.75 | opened, both halves on the plate, fingertips 3 cm clear, without the rest of 1.0 |
| 0.5 | opened, otherwise (a half still held, or off the plate) |

Possible scores: 0, 0.5, 0.75, 1.0. No tier gives credit for a cracked cookie. Credit for 1.0 does not outlast the
state that earned it. The harm record is kept in mechkit's per-episode memory and is cleared only by the reset hook.

## Real trial: preparation, reset and measurement

- **Parts**: one pack of Nabisco Oreo Original (about $5); a 17 cm plastic plate; a drinking glass, a plastic spoon
  and a mug as distractors. Tape marks: a 44 mm circle for the cookie, 19 cm in front of the plate's centre, and the
  plate's outline.
- **Preparation**: twist one test cookie from the pack to see which wafer keeps the creme. Lay each trial cookie
  flat on its mark with that wafer down. Use cookies at room temperature (the paper found warm, humid storage changes
  the failure).
- **Pad protection**: food-safe nitrile finger covers on the gripper pads; wipe them between trials. The demo's grip
  is 6-7 N a finger, and there are no blades or heat.
- **Twist marks and camera** (before each trial): with a food-safe marker, draw eight short ticks across the rim's
  edge, every 45 degrees, each tick crossing both wafers and the creme. Record the trial with a fixed front camera:
  a phone or webcam at 30 fps or more, 1080p, 40 cm in front of the robot's workspace, lens level at 15 cm above
  the table, the cookie's start mark and the space above it in view. A second identical camera at 90 degrees (the
  side camera) covers a cookie turned away from the first.
- **Measurement**, after the robot has stopped and before anyone touches the scene:
  1. **Opened, with the twist.** Find the last video frame before the two halves part. In whichever camera shows a
     tick most squarely, measure the offset between the tick's top-wafer and base-wafer parts along the rim, scaling
     pixels by the cookie's 44 mm width in the same frame. Opened needs an offset of at least 3.5 mm: a 10-degree
     turn moves the rim 3.8 mm, and 3.5 mm allows for a tick up to 22.5 degrees off square. Halves pulled or pried
     apart with a smaller offset are not opened (score 0), as in the sim.
  2. **Wafer integrity.** Photograph both halves from above at 20 cm with a ruler in frame. Any crack, chip or
     broken-off edge longer than 2 mm on either wafer (the embossed pattern's own edges aside) scores 0. Crumbs on
     the plate with no matching chip do not count.
  3. **Resting on the plate.** The plate is right side up. Each half lies directly on the plate's top surface, not on
     the other half, the rim or anything else. In a side photo at plate level its lowest edge is no more than 4 mm
     above the plate's surface. With a ruler, each half's centre is within 64 mm of the plate's centre, and the
     centres are at least 45 mm apart.
  4. Check that each half lies flat (tilted no more than 20 degrees).
  5. From the video's last second, check that the grippers are at least 3 cm from both halves.
  6. Record the creme distribution (photo) for information. It is not scored.
- **Reset** (about 1 minute): remove the halves, wipe the plate, and lay a fresh cookie on the mark, ticked as above.
  Each trial uses one cookie.

## Assets and licences

- `simready.py` (loader) and the props below are NVIDIA SimReady assets from
  [nvidia/simready-assets](https://huggingface.co/datasets/nvidia/simready-assets), © NVIDIA, CC-BY 4.0. They are
  downloaded at first run, at the revision pinned in `simready.py`. Masses are set at load time; nothing is committed.
  - `dishware_plastic_bigPlate_a01` (plate, 0.15 kg)
  - `dishware_glass_8oz_a01` (distractor, 0.40 kg)
  - `dishware_plastic_spoon_a01` (distractor, 0.02 kg)
  - `dishware_mug_a01` (distractor, 0.30 kg; its triangle collider is replaced by a convex hull by PhysX)
- `mechkit.py` is the "multi-box parts, hinges, sliders and breakable bonds" kit shared by several tasks in this
  repository (for example `tasks/folding-table-leg-ring-lock/`). Its twist bond is used unchanged; the two local
  changes are listed above.
- `demokit.py` is the closed-loop demo runner shared by several tasks in this repository (for example
  `tasks/close-mason-jar-two-piece-lid/`), unchanged.
- The cookie is built from primitives, because SimReady has no sandwich cookie.

## Validation

Scripted demo (`demokit.py`, closed loop, final source, GPU, cameras): two fresh runs, each task_score 1.0 at 640
steps (32 s), with no wafer cracked. The submitted video and screenshot come from the first of these scored runs.

Force log of that run: `demo_force_log.csv` has each fingertip's normal force on each wafer at every control step. The
largest value is 7.3 N (the right gripper on the base wafer during the twist). The left gripper holds the top wafer
at up to 6.3 N. The break threshold is 15 N, and no step comes within 7 N of it. A fingertip within 4 mm of both
wafers counts on both. The second run's peak is also 7.3 N.

Red-team modes of the same script (CPU physics, `--fast`), each run on the final source:

| Mode | What it does | task_score |
|---|---|---|
| crush | as the demo, but both grippers pinch with the YAM's full grip (40 N commanded) | 0.0 (36.7 N on the top wafer at the first pinch: cracked, recorded for the trial) |
| plate upside down over the halves (a temporary variant file, since deleted) | twists the cookie open and lays both halves on the table; the plate is then set down upside down over them (pose written by the script, since the arms cannot flip a plate) | 0.5 (opened; neither half on the plate: the plate is at 180 degrees, and the halves sit 1.6 mm below where its floor would be) |
| none | does nothing | 0.0 |
| onearm | the left hand lifts the cookie by the top wafer, twists it 30 degrees each way, and sets it on the plate | 0.0 (the whole cookie turns; it does not open) |
| pull | both hands hold, then the left pulls 3 cm straight up without turning | 0.0 (not opened; relative turn 1.7 degrees) |
| careless | twists it open, then both hands let go over the plate without lifting the wafers apart | 0.5 (opened; the halves slid off the plate). In one run the turn reached only 4.4 degrees, so the cookie did not open: 0.0. There the stacked top half, 9 mm above the floor, did not count as on the plate. |
| one-hand press-and-twist (an earlier temporary variant, since deleted) | presses the cookie onto the table and turns the top wafer 25 degrees with one hand | 0.0 (the whole cookie turned on the table) |

Model run: Dream-1, one run of 2,400 steps (scene and instruction unchanged by this revision), scored 0.0: it carried
the whole cookie onto the plate without opening it.
