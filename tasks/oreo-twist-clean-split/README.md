# Twist an Oreo open and lay both halves on the plate

An Oreo lies flat on the table beside a plate. The robot must open it into its two halves and leave both halves on
the plate, lying flat and apart, with its grippers clear. In the simulation the cookie opens only by counter-rotation:
the two wafers are turned against each other about their shared axis until the creme lets go of the top wafer.

Instruction: *"Open this Oreo into its two halves and put both halves on the plate."*

The knack: the cookie is 11 mm thick and each wafer is 3.5 mm thick, held only at its rim. One hand has to pinch just
the top wafer and lift the cookie. The other hand has to pinch just the base wafer below it, from a different side,
without touching the top wafer. Then one hand holds still while the other turns its wafer about the cookie's axis.
The creme yields at about 0.1 N m after a turn of about 10 degrees. Turning the whole cookie in one hand, or pulling
the wafers straight apart, does not open it.

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
    python demokit.py task.py OUT --fast --mode none,onearm,pull,careless

The start layout is fixed (no object jitter). Seeds are not used. A demo trial runs about 32 s of simulated time, 640
control steps, and ends when the 1 s hold is met. Wall time is about 1 minute on a free GPU.

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
- **What the score reads**: wafer poses, the joint's state, and the fingertip positions of both grippers. Nothing is
  triggered by proximity.

## Scoring

Opened = the two wafers have been turned at least 10 degrees against each other, and the creme has let go. In a real
trial: the two halves are apart.

| Score | Condition |
|---|---|
| 0 | never opened; or a half ever fell off the table (centre over 3 cm below the top), recorded and cleared only at reset |
| 1.0 | opened; both halves on the plate (centre within 64 mm of the plate's centre, at most 12 mm above its underside); each flat within 20 degrees (either face up); centres at least 45 mm apart; every fingertip at least 3 cm from both halves; both still; all held for 1 s |
| 0.75 | opened, both halves on the plate, fingertips 3 cm clear, without the rest of 1.0 |
| 0.5 | opened, otherwise (a half still held, or off the plate) |

Credit for 1.0 does not outlast the state that earned it. The harm record is kept in mechkit's per-episode memory and
is cleared only by the reset hook.

## Real trial: preparation, reset and measurement

- **Parts**: one pack of Nabisco Oreo Original (about $5); a 17 cm plastic plate; a drinking glass, a plastic spoon
  and a mug as distractors. Tape marks: a 44 mm circle for the cookie, 19 cm in front of the plate's centre, and the
  plate's outline.
- **Preparation**: twist one test cookie from the pack to see which wafer keeps the creme. Lay each trial cookie
  flat on its mark with that wafer down. Use cookies at room temperature (the paper found warm, humid storage changes
  the failure).
- **Pad protection**: food-safe nitrile finger covers on the gripper pads; wipe them between trials. The forces are
  under a few newtons and there are no blades or heat.
- **Measurement**, after the robot has stopped and before anyone touches the scene:
  1. Is the cookie in two halves? A photo is enough.
  2. With a ruler, check that each half's centre is within 64 mm of the plate's centre and that the centres are at
     least 45 mm apart.
  3. Check that each half lies flat on the plate (not leaning on the rim more than 20 degrees).
  4. From the video's last second, check that the grippers are at least 3 cm from both halves.
  5. Record the creme distribution (photo) for information. It is not scored.
- **Reset** (about 1 minute): remove the halves, wipe the plate, and lay a fresh cookie on the mark. Each trial uses
  one cookie.

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
steps (32 s). The submitted video and screenshot come from one of these scored runs. Red-team modes of the same
script (CPU physics, `--fast`), each run on the final source:

| Mode | What it does | task_score |
|---|---|---|
| none | does nothing | 0.0 |
| onearm | the left hand lifts the cookie by the top wafer, twists it 30 degrees each way, and sets it on the plate | 0.0 (the whole cookie turns; it does not open) |
| pull | both hands hold, then the left pulls 3 cm straight up without turning | 0.0 (not opened) |
| careless | twists it open, then both hands let go over the plate without lifting the wafers apart | 0.5 (opened; the halves slid off the plate) |
| one-hand press-and-twist (a temporary variant file, since deleted) | presses the cookie onto the table and turns the top wafer 25 degrees with one hand | 0.0 (the whole cookie turned on the table) |

Model run: Dream-1, one run of 2,400 steps on the final source, scored 0.0: it carried the whole cookie onto the plate without opening it.
