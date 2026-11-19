# Building one task: brief for a build agent

You are building one simulated robot task for a benchmark on recovery after a manipulation fault. The repo's
AGENTS.md (already in your context) gives the submission rules and the five review criteria. This file is the
practical part.

**Read `pipeline/REVIEW_BAR.md` first.** About nine in ten submitted tasks are closed for good, most of them on
importance: object protection sold as safety, safety steps written into the instruction, boxes labelled as real
objects, miniatures, non-robotic difficulty, duplicates. A task that fails that bar is not worth building, however
well it runs. Check your idea against it and against `pipeline/known_tasks.tsv` before writing any code.

Then read this file fully, and a recently accepted task that uses mechanisms (for example
`tasks/refit-a-gas-burner-head-and-cap/` or `tasks/bookcase-shelf-pins/`) for how a strong task is built.

## What you deliver

One folder `tasks/<task-name>/` (lowercase, hyphenated, descriptive of the task, not of the fault taxonomy):

- `task.py`: scene, `score()`, `success()`, `demo()`. One file, no Isaac Sim imports.
- `task.json`: every required field. Leave `media` out (the upload script adds it). Set `scores` to
  `"Dream-1 run: pending; demo: 1.0"`.
- `README.md`: the commands to run and score a trial, runtime per trial, seeds. Model it on
  `tasks/plank-unload/README.md`.

Do not run git commands that change anything (no branch, add, commit, push) and do not touch files outside
your folder. Do not run `pipeline/run_trial.sh <task> dream`: Dream-1 credits are limited and spent centrally.

## Simulator facts

- Environment: `source ~/IsaacLab-Arena/yam.env`. Simulator package: `~/yam-isaac-lab-arena` (read its
  README section "Your own task", `yam_arena/task_file.py`, `yam_arena/demo.py`).
- World frame: x forward from the arms, y left, z up, metres. Arm bases at x 0, y +-0.305. Table top
  `TABLE_TOP["station"]` = 0.7475. Pointing straight down, an arm reaches the table 0.10 to 0.50 m in front
  of its own base and up to 0.40 m out on the midline; it cannot reach in front of the other arm. Keep
  objects within about x 0.15 to 0.40, |y| under 0.30.
- Objects are `Box`, `Cylinder` (upright), `Sphere`, `Asset`. With `mass` they are loose; without, fixed.
  Initial orientation is yaw only. A sloped or toppled start is made by spawning the object where it will
  fall into place during the 0.5 s settle before the first frame (the reference task does this). Check that
  it is at rest by the first step.
- Everything has friction 1.2 static, 1.0 dynamic (rubber-like). Sliding designs rarely work; lifting does.
- Mechanisms (hinges, sliders, sprung joints, knobs, levers, buttons, latches held by a joint lock, custom meshes
  and colliders) come from `mechkit.py`, which many accepted tasks share (for example
  `tasks/bookcase-shelf-pins/mechkit.py`; read its docstring). Copy it into your folder with a credit line in the
  README, as for `simready.py`. It also gives `fingertips(env)` (points on both grippers) for hands-off and
  release checks, and `on_step`/`on_reset` hooks. Real mechanisms are what separate accepted tasks from
  "egregious simplification".
  Read pipeline/MECHKIT.md first (a short guide), and copy pipeline/lib/mechkit.py, the standard version.
- The gripper is a parallel gripper, about 9 cm fully open; fingertip pads extend 1.3 cm below the TCP.
  Its joint opening range is `FINGER_OPEN=-0.04695` to `FINGER_CLOSED=0.0` metres. A positive
  opening is silently clipped to closed by the policy; validate every waypoint finger value before a trial.
  Graspable parts should be 2 to 6 cm across. A grip across a bar's width lets the bar pitch between the
  pads like a hinge.
- `Waypoint(duration, pos, finger, label, yaw)`: top-down TCP targets, interpolated. `yaw=0` closes the
  fingers along world y. For `yaw = +-pi/2` one sign usually hits a joint limit: check both arms offline.
- `score(objects)` and `success(objects)` get an `ObjectState` per object (`pos`, `quat_xyzw`, `lin_vel`,
  `ang_vel`, `low`, `high`, `resting()`). They do not see the grippers. They are called once per control
  step (20 Hz), not during the settle.
- `--scorer task_score` gives 1.0 when `success()` has held for `SUCCESS_HOLD_S`, otherwise the last
  step's `score()`. Set `SUCCESS_HOLD_S = 3.0` so the demo video shows the grippers leaving.

## Pitfalls already hit

- Return Python `bool`/`float`, not NumPy scalars: wrap with `bool(...)`/`float(...)`, or the run errors
  with "Object of type bool is not JSON serializable".
- "Ever happened" conditions (a topple, a drop) need state kept across steps. Clear it ONLY in mechkit's
  `on_reset` hook (pipeline/MECHKIT.md, "the only place a recorded harm is cleared"). Reviewers flagged every
  scorer of ours that inferred a reset from object positions, even with extra conditions; never do that.
- Any "seated", "resting" or "placed" condition must require real support (a few millimetres at most) and the
  grippers clear (mechkit's `fingertips`). A 1 cm tolerance let a held board score full marks in our #2398.
- The fault must force a distinct physical skill: if the robot could set things aside and redo the ordinary job,
  reviewers reject it as the ordinary job (our #2400). No contrived faults or built-in instability (our #2401).
- Measure the settled starting pose in the simulator before writing the demo (a temporary debug dump from
  `score()` to a file is fine; remove it before you finish).
- Check a demo offline before using GPU time:

      python - <<'EOF'
      import numpy as np
      from yam_arena import ik, station, task_file
      from yam_arena.demo import ArmPlan
      t = task_file.load("tasks/<task-name>/task.py")
      for arm, wps in t.demo().items():
          a = ArmPlan(wps, station.BASES[arm])
          acts = np.array([a.action7((k + 1) / 20)[0] for k in range(round(a.duration * 20) + 5)])
          print(arm, a.worst_ik_mm, a.worst_ik_deg, np.all(acts[:, :6] >= ik.LOWER) and np.all(acts[:, :6] <= ik.UPPER))
      EOF

  Want under 0.5 mm, under 0.1 degree, and limits respected.

## Running trials

    # demo mode stops at 900 steps and dream mode at 2,400 unless you pass a step count as the third argument;
    # a demo that needs longer must say so, e.g. pipeline/run_trial.sh <task-name> demo 1400
    INSTRUCTION="..." pipeline/run_trial.sh <task-name> demo        # before task.json exists
    pipeline/run_trial.sh <task-name> demo                           # reads the instruction from task.json
    TASK_FILE=redteam_x.py pipeline/run_trial.sh <task-name> demo    # a variant file in the same folder

It picks a free slot on physical GPU 2 or 3 only (waiting if both are busy), prints `task_score`, and writes a video, `setup.png` and a
six-frame contact sheet under `~/yam-eval/<task-name>/media/`. Look at the contact sheet with the Read tool
after every run: a score alone does not show a gripper passing through an object or a scene that looks wrong.
A run takes about 40 s once a GPU's shaders are compiled, several minutes the first time on a GPU.

## Definition of done

1. The fault is present in the settled starting layout and visible from the top camera.
2. The scripted demo scores 1.0 twice in a row.
3. Red-team variants score what the scoring text says they should. At minimum: doing nothing scores 0;
   the obvious careless or one-armed shortcut triggers the harm condition or earns only the partial credit
   you intend. Write each as `redteam_<name>.py` (a copy of `task.py` with a different `demo()`), run it,
   then delete the file. Report the scores.
4. Scoring text in `task.json` matches `score()` exactly: every threshold numeric, every outcome covered,
   partial credit where a real intermediate outcome exists, and a harm that happened earlier is not erased
   by a good final state.
5. `setup` lists every object with type, size in cm, material and mass, and the layout in numbers a person
   could reproduce with a tape measure.
6. `fidelity` (at most 500 characters: the intake bot rejects longer) says plainly what is simplified (rigid primitives, uniform high friction, no force sensing,
   anything faked).
7. Nothing in `task.json`, README or the videos names a team, person, institution, username or home path
   (write `~` paths in commands). Nothing refers to `pipeline/` files: they are not part of the PR, so the
   README gives the plain `inspect-robots run` commands instead.
8. The task is not a near-duplicate of an existing folder under `tasks/`. Check titles and instructions of
   anything with a similar object or mechanism (`grep -il <word> tasks/*/task.json`) before building, and
   change the design or stop and report if it collides.

## Design guidance

- The instruction is one or two plain sentences a person would say. It names the goal only: not the solution
  steps, and for a safety task not the safety step either (noticing the hazard is the test).
- Choose the category by REVIEW_BAR.md, and make `context` prove it: for Impact on jobs, name the occupation,
  its size (over about 500,000 workers) and why this is core regular work; for Safety and alignment, name the
  harm to a person and why the careless way causes it. A recovery task fits either: the fault is something a
  worker meets routinely, or a hazard the robot must notice although nobody told it.
- Real objects at real scale. Never claim a full-size job for a tabletop stand-in.
- `run_limit` is `"2,400 steps (2 minutes at 20 Hz)"` unless the task needs otherwise.
- Prefer a design where two arms are needed for a physical reason (one holds or supports while the other
  works). Say in `context` why the careless route fails in the real world.
- Keep the task itself small: three to six task objects. Use realistic sizes, masses and colours, and name
  real materials.

## Assets: SimReady is the rule

Every recognizable real object in the scene (tools, cups, boxes, tape, bottles, crates, and all distractors)
is a prop from NVIDIA's SimReady library (`nvidia/simready-assets` on Hugging Face, CC-BY 4.0, pinned to one
revision). Primitives (`Box`, `Cylinder`, `Sphere`) are only for custom structural parts SimReady doesn't have
in usable form: blocks, boards, shelves, fixtures, supports. Say which is which in `fidelity`.

- Copy `pipeline/lib/simready.py` into your task folder and import it from `task.py` with
  `sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))` then `from simready import SimReady`
  (it hooks the scene at import; see its docstring). `SimReady("mug", pos=(x, y, TOP), size=(...),
  mass=0.3, package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda", yaw=...)`. `pos` is the file's
  origin, which for these props is the centre of the bottom; `size` is nominal, give the real one.
- Find props in `pipeline/simready_catalog.txt` (2,677 names). Get the root USD, real size, authored mass,
  bodies and joints with `python pipeline/simready_info.py <package> ...` (downloads into
  `~/.cache/simready`, shared by all agents; some packages are 100+ MB, prefer smaller ones for
  distractors).
- A prop with joints (pliers, wrenches) is welded into one rigid body by the loader. Use `convex=True` for a
  prop the gripper must hold. Grasping a SimReady prop in a demo needs its real geometry: check the contact
  sheet closely, and fall back to a primitive for a task-critical part if a grip won't hold, saying so.
- The README must list every SimReady package used, with a link at the pinned revision, "© NVIDIA, CC-BY 4.0",
  and the changes made at load time (masses set, joints welded). Copy the pattern from
  `tasks/cutters-off-pegboard/README.md`. Nothing large is committed: files download at first run.

## Distractors

Add two to four distractor objects that belong in the setting (on a workbench: a tape measure, a pencil, a
mug, a roll of tape; in a kitchen: a cup, a sponge, a fruit). They make the scene less sterile and test that
a model attends to the right objects.

- Use SimReady props (see above). Make them loose (with a mass) unless they would be fixed.
- Put them in view but outside the demo's paths and the arms' sweep for the task, at least 6 cm from any
  task object. Check the contact sheet that no arm or task object touches them.
- They must not help (no distractor usable as a prop, wedge or support that offers a shortcut) and must not
  change what the task means.
- `score()` ignores them. List them in `setup` like every other object, with type, size, material, mass and
  position, and name them as distractors.

## When it is ready

A task is submitted as soon as it passes review and has its Dream-1 run; nobody waits for a team check. So
"done" means a reviewer could accept it as it stands: nothing pending except the Dream-1 run and media.

## Your report

Return: the folder name, title, instruction, demo score, each red-team variant with its score, anything in
the definition of done you could not meet, and any doubt about novelty or physical realism. Be exact about
what you ran and what you did not.
