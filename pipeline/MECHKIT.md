# mechkit: a working guide

`pipeline/lib/mechkit.py` is the standard copy of the mechanism kit, taken unchanged from
`tasks/heat-shrink-lug/mechkit.py` (another team's open-source kit). Its docstring and code are the authority;
this guide quotes them and the accepted tasks that use them. Names below are `m.<name>` after `import mechkit as m`.

## Which mechkit this is

Several unrelated kits in `tasks/` are all called `mechkit.py`. This guide covers only the one whose docstring
opens "mechkit: props for a yam_arena task file, written as plain-text USD". Don't copy calls from the others:
- The `Mech`/`Joint` kit (108 KB; an older copy is 18 KB): `refit-a-gas-burner-head-and-cap` (BRIEF names it as a
  good example: read it for task design, not for API), `shut-a-clamp-meter-round-one-motor-lead`,
  `tip-a-recessed-book-off-a-packed-shelf`, `hang-a-blind-headrail-in-gated-brackets`.
- The `assembly`/`slider(name, axis, shapes, upper=...)` kit (127 KB): `tongue-groove-pliers-widest`.
- "multi-box parts, hinges, sliders and breakable bonds" (40 KB): `folding-table-leg-ring-lock` and others.
- yamkit "laws" (needs `yamkit.py`): `brake-bleed-close-before-release`, `instill-one-eye-drop`.

Versions of this kit, oldest first (each one adds to the last, so the standard copy is a superset of them all):
`chalk-line`, `tube-bender` (no `push`, no `start_quat` on `mechanism`, no unlimited joints) ->
`slit-carton` (adds those) -> `bookcase-shelf-pins`, `alaris-set-load`, `fr3-guide-swap`,
`keyboard-two-hand-dynamics` (add `Link(inertia=...)`, `mechanism(frame_inertia=...)`) -> `heat-shrink-lug`
(adds `free(com=...)`): the standard. `mouse-drag`'s copy is a side branch with `fixed(combine=...)`,
`mechanism(frame_com=...)` and `settle_after_reset()`. None of those three is in the standard copy, so
`mouse-drag/task.py` doesn't run on it.

## Importing it

    cp pipeline/lib/mechkit.py tasks/<task>/

```python
import pathlib, sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import mechkit as m  # noqa: E402  importing it hooks yam_arena; nothing else to call
```
Importing it needs only NumPy and `yam_arena.task_file`, so `task_file.load` and BRIEF's offline demo check still
work. It hooks `yam_arena.scene` (kit parts spawn from USD text it writes to a temp folder; `object_states`
reports them) and `YamArenaEmbodiment._task_step`/`reset` (the hooks below). It works with `simready.py` in the
same task file: both pass on objects that aren't theirs. In the README, credit where `mechkit.py` came from.

## Parts: fixed, free, mechanism

A kit part is a `Part` (a `Box` subclass) in `OBJECTS`, built by one of the following:
- `fixed(name, pos, parts, colliders=(), looks=None, yaw_deg=0.0, friction=(1.0, 0.8))`: static, no mass.
- `free(name, pos, parts, colliders, looks, mass, friction=(0.9, 0.8), torsional_patch=0.015, solver_iters=32,
  yaw_deg=0.0, start_quat=None, com=None)`: one rigid body. `start_quat` (xyzw) starts it in any rotation;
  `com` sets the centre of mass in the part's frame (otherwise PhysX takes it from the colliders).
- `mechanism(name, pos, links, looks, frame_parts=(), frame_colliders=(), yaw_deg=0.0, friction=(1.2, 1.0),
  solver_iters=32, start=(), free_mass=None, start_quat=None, frame_inertia=(0.01, 0.01, 0.01))`: an
  articulation, made of a `frame` body plus one `Link` per joint. The frame is welded to the world unless
  `free_mass` (kg) makes it free-standing (it then needs colliders). `frame_inertia` is in kg m^2. The
  default "suits a fixed frame, not a free slender one".

`pos` places the part's origin (in world metres; heights are `TOP + ...`). `yaw_deg` turns the whole part, links
and axes included. `parts` are `(prim name, Mesh, look name)` or `(prim name, Mesh, look name, hidden)`.
`looks` maps look names to `Look(color=m.rgb(r, g, b), metallic=0.0, roughness=0.6, opacity=1.0,
emissive=None, glass=None)`. `rgb` turns sRGB 0..1 into linear. Friction is `(static, dynamic)` per part. These
defaults differ from the 1.2/1.0 that BRIEF gives for plain primitives.

## Meshes and colliders

What you see and what collides are separate lists. In the docstring's words: "Colliders stay simple and separate
from what is seen: hidden, and they still collide." Visual meshes have no collision.

Meshes (NumPy, in the part's frame): `box(size, center)`, `cylinder(radius, height, center, axis="z",
segments=32)`, `sphere(radius, center)`, `lathe(profile, segments=32)` ((radius, z) pairs from bottom to top,
where radius 0 closes the end), `prism(outline, z0, z1)` (a counter-clockwise, star-shaped xy outline, extruded),
`tube(points, radius, sides=12)` (a closed tube along a polyline), and `join(*meshes)`. A `Mesh(v, f)` has
`.moved(t)`, `.turned(deg, axis)` and `.aimed(outward)` (turns a mesh built along +z to point at "+X", "-Y" and
so on). You can also write a `m.Mesh(np.array(verts), np.array(faces))` by hand (slit-carton's `BLADE`) or
read one from an STL (fr3-guide-swap).

Colliders (each returns USD text; names must be unique within the part):
- `col_box(name, size, center)`: axis-aligned in the part's frame.
- `col_cylinder(name, radius, height, center)`: about z only.
- `col_hull(name, mesh)`: the convex hull of any mesh. It is "the way to a collider that is not axis-aligned",
  for example a pin along y as `col_hull("pin_c", m.cylinder(r, h, c, axis="y"))` (bookcase-shelf-pins).

A hull fills in anything concave: the hull of a ring is a disc, and the hull of a tray is a block. Build concave
things from several convex pieces: one `col_box` per board (bookcase-shelf-pins `_case`, slit-carton `_frame`),
or separate hulls for each region (mouse-drag's shell). Colliders use a 4 mm contact offset and a 0 rest offset.

## Links: hinge, slider, knob, lever, button

`Link(name, kind, axis, at, lower, upper, parts=(), colliders=(), mass=0.05, spring=0.0, damping=0.2,
friction=0.02, effort=50.0, armature=0.001, gravity=False, com=None, parent=None, inertia=None)`. Make links with:
- `hinge(name, axis, at, lower, upper, parts, colliders, **kw)`: turns about `axis` ("X", "Y" or "Z" of the part's
  frame) through point `at`. Limits are in **degrees**.
- `slider(name, axis, lower, upper, parts, colliders, at=(0, 0, 0), **kw)`: slides along `axis`, limits in metres.
- `knob(name, at, outward="+Z", look="knob", mark=None, hub_r=0.016, hub_h=0.01, wing=(0.06, 0.01, 0.025),
  lower=-170.0, upper=170.0, friction=0.03, damping=0.05, spring=0.0, mass=0.05)`: a wing knob. The kit's
  description: "A gripper pinches the wing across its thickness and turns." `mark` (a look name) adds a stripe
  to read the angle by.
- `lever(name, pivot, along, axis="Y", look="lever", length=0.12, radius=0.008, ball_r=0.016, lower=-30.0,
  upper=30.0, spring=0.0, damping=0.05, friction=0.01, mass=0.08, gravity=False)`: a bar with a ball on its end.
- `button(name, at, outward="+Z", look="button", radius=0.02, height=0.015, travel=0.008, spring=300.0,
  damping=2.0, mass=0.02)`: a sprung cap. Pressing it in is a negative joint value for a "+" `outward`
  (limits (-travel, 0)) and a positive one for "-".

`knob` and `button` take their axis from `outward`'s letter, so positive is right-handed about +axis even for a
"-Z" knob. No accepted task in this repo uses them: the accepted tasks build keys, buttons and levers from
`hinge` and `slider`. Treat them as unproven, and look at a run before relying on them.

Joint behaviour:
- **Joint zero is the authored pose.** "Joint zero is the pose written in the file, whatever else is authored:
  model a mechanism at its start pose." `mechanism(start=[(link, value)])` starts a joint elsewhere at reset. Its
  values are **radians or metres** (they go straight to Isaac Lab), unlike limits, which are in degrees for a hinge.
- **Spring**: `spring` (N m/rad or N/m) is the actuator stiffness, and it pulls toward joint zero (or toward
  whatever `set_target` sets). With 0 the joint is free, "held only by `friction` (N m or N) and `damping`".
  slit-carton found that friction is PhysX joint friction, "which grows with the load through the joint":
  measure it in the simulator (its `_slider_test`).
- **effort** (default 50) caps the actuator's force or torque. Raise it for stiff or preloaded springs
  (tube-bender uses 100); keyboard-two-hand-dynamics uses 5 for its keys. **armature** (default 0.001) adds
  inertia to the joint.
- **Limits**: keep joint zero (the authored pose) inside `lower`/`upper`. `float("inf")` limits leave a joint
  unlimited ("a wheel, a roller").
- **Gravity**: "Links have no gravity unless `gravity`". A lid won't fall shut and a button won't sag unless you
  set `gravity=True` (the centre of mass is then `com`, or the mean of the mesh's points).
- **Inertia**: by default the inertia is a round guess from mass and reach. Pass `inertia=(ix, iy, iz)` "for
  slender parts turned about their own axis".
- **parent**: another link's name, for a chain. fr3-guide-swap builds a 7-joint arm, with buttons on `link7`.

## Driving joints while it runs

Call these from a hook (they need `env`):
- `set_target(env, part, link, value)`: moves the spring's rest point to `value` (radians or metres), as "a
  motor, a solenoid, a task-driven part" would. It only acts when the link has a `spring`.
- `lock(env, part, link, stiffness=50000.0, effort=2000.0)`: holds the joint where it is now, "as a latch does,
  against any force a gripper can apply". `unlock(env, part, link)` gives the joint back its own spring and
  effort and sets its target to **0**, so call `set_target` again afterwards if you need another rest point.
- `push(env, part, force, at=None, body=None)`: a steady force (N, world frame) applied until you change it.
  `body` is a link name (default: the mechanism's frame). `(0, 0, 0)` stops it, and a reset stops all.
- `set_visible(path, visible)`: shows or hides a visual prim. The path is
  `f"{m.ENV_PATH}/<part>/visual/<prim name>"` for a fixed or free part (chalk-line, heat-shrink-lug,
  mouse-drag). In a mechanism the code puts these under `<part>/frame/visual/` and `<part>/<link>/visual/`.
  Start a prim hidden with the fourth element of its `parts` tuple.
- `run_seed()`: the seed of the latest reset (tube-bender uses it to pick a tube's springback).

Before each reset the kit undoes every `set_target`, `lock` and `push`. Re-apply anything the trial must start
with in an `on_reset` hook (slit-carton locks its taped flaps there; keyboard-two-hand-dynamics preloads its keys).

## Reading state: score(), hooks, fingertips

`score(objects)`/`success(objects)` receive a `PartState` for each kit part, and `m.state(env, name)` returns the
same thing in a hook. A `PartState` has `pos` (**the part's origin**, not a bounding-box centre; `low` and
`high` are both just `pos`), `quat_xyzw`, `lin_vel`, `ang_vel` and `joints` (`{"<link>_joint": value}`, in
radians or metres). Its methods are `joint(link)`, `deg(link)`, `rotation()`, `tilt_deg()`,
`to_local(points)`, `to_world(points)` and `resting()`. A fixed part reports its authored pose. Example:
tube-bender's `distance()` returns `abs(TARGET - objects["bender"].deg("tube") * -1.0) / TARGET`. To read a
non-kit object (a `Box`, a SimReady prop) in a hook, use `from yam_arena.scene import object_states;
object_states(env, (name,))[name]`.

Hooks:
- `@m.on_step` or `m.on_step(fn)`: `fn(env)` runs after each control step's physics, before that step's
  `score()`/`success()`. It doesn't run during the reset's settle, and it never runs offline.
- `m.on_reset(fn)`: `fn(env)` runs "after every reset, once the scene has settled".

Hooks run both under `run_trial.sh demo` (yam_scripted) and under demokit. `env.unwrapped.scene[...]` and
`env.unwrapped.step_dt` work inside them. The accepted tasks built on this kit follow this pattern:
```python
class Trial:
    def __init__(self): self.reset()
    def reset(self, env=None):          # the ONLY place a recorded harm is cleared
        self.harm, self.hold, self.full, self.score = False, 0, False, 0.0
    def step(self, env):
        s = m.state(env, "bender")
        if -s.deg("lever") > 96.0: self.harm = True          # latch "ever happened"
        ok = ...                                               # this step's full-score conditions
        self.hold = self.hold + 1 if ok and not self.harm else 0
        self.full = self.full or self.hold >= HOLD_STEPS       # the tracker counts its own hold
        self.score = 0.0 if self.harm else (1.0 if self.full else ...)
TRIAL = Trial(); m.on_step(TRIAL.step); m.on_reset(TRIAL.reset)
SUCCESS_HOLD_S = 0.0
def score(objects): return float(TRIAL.score)
def success(objects): return bool(TRIAL.full)
```
The violation record (`split` in heat-shrink-lug, `too_far`/`off_bench` in tube-bender, `damage`/`cardboard`
in slit-carton) is latched in `step` and cleared only in `reset`. Never clear it because things look right again.

`fingertips(env)` returns a (24, 3) array of world points: the left arm's 12, then the right arm's. Within an
arm, one finger's 6 come first, then the other's. Each finger gives its tip and a point 3 cm up it, at its
middle and both edges, opened as the fingers are now. The finger's real underside sits about 5 mm above the
reported tip point (keyboard-two-hand-dynamics `UNDER = 0.005`). Typical uses:
- Hands-off: put the points in the part's frame and check the distance. `local = s.to_local(m.fingertips(env))`,
  then require more than 3 cm outside the part's box (bookcase-shelf-pins `Tracker.step`) or from its segments
  (tube-bender `segment_gap`), or more than 5 cm from sampled points (heat-shrink-lug `now`).
- Holding: fingers on opposite sides of a grip (heat-shrink-lug `_grasped`).
- Pressing: a fingertip's depth past a face (heat-shrink-lug's switch).

## Demos with mechanisms

- **`demo()`** (BRIEF's route, played by `pipeline/run_trial.sh <task> demo`): open loop, so it can't read
  joints. The hooks and scorer still run. Measure the settled joint values first with a temporary print from a
  hook, then script the waypoints.
- **`demokit.py`**: every accepted task built on this kit uses it. Copy `tasks/heat-shrink-lug/demokit.py`, the
  copy eight of them share. `demo_script(ctx)` is a closed-loop generator. `ctx.state(name)`, `ctx.pos(name)`,
  `ctx.joint(part, link)` and `ctx.deg(part, link)` read the scene. Each arm (`ctx.left`, `ctx.right`) has
  `move(pos, grip=, finger=, t=)`, `servo(pos, read=fn)`, `follow`, `arc(center, axis, deg)` ("a door's edge, a
  lever's end"), `turn(deg, axis)` ("a knob, a valve. A knob on +z follows `deg` with the sign of its joint"),
  `close(width, force)`, `open()`, `pick`, `place` and `wait`. `ctx.together(...)` runs both arms in lockstep
  and `ctx.until(cond, timeout)` waits for a condition. `ctx.mode` comes from `TASK_DEMO`, for red-team modes.
  Grips are `d.down(yaw_deg)`, `d.tilted(theta, phi)` and `d.level(phi)`. Run it from the task folder after
  `source ~/IsaacLab-Arena/yam.env`: `python demokit.py task.py OUT --fast --mode full,none` (CPU, no cameras),
  or `python demokit.py task.py OUT` (final: `OUT/full/demo.mp4`, `setup.png`, `result.json` with `task_score`).
  `run_trial.sh` doesn't run `demo_script`. demokit's rule: a script "reads only what a policy could observe
  ... Never a scorer's state" (never `TRIAL`).

## Solver pitfalls (from the docstring and code)

- "Author nothing inside a collider, and no link touching a collider of another part: a light link on a joint
  blows up the solver. Whatever a link's pivot sits on goes in the mechanism's own `frame_parts` and
  `frame_colliders`, where self-collisions are off. Walls that stand clear of the links go in a `fixed` part of
  their own."
- Self-collision is off within a mechanism: links pass through their own frame and each other, so **stops must
  be joint limits**. slit-carton gives its flap a limit at shut, so the flap "rests on its stop: it may not sag
  into the product".
- A blown-up joint prints `[mechkit] BLOWN UP: <part> <joint> is at ..., limits ... to ...` (more than 20% of its
  range past a limit). Every number after that is noise: grep the run log for it.
- Weightless links (the default) and the default inertias can mislead you. slit-carton had to override a free
  knife's frame inertia (0.01 kg m^2 is far too much for 150 g); now pass `frame_inertia`.
- "Check a moving part against the arms at their start pose, and with an arm left there."
- More iterations help stiff or long chains (`solver_iters`: fr3-guide-swap uses 64). keyboard-two-hand-dynamics
  splits 61 keys into two mechanisms with `solver_iters=16`.

## Recipes

1. **Sprung button or key**: a slider with a spring and limits `(-travel, 0)`. Read the press as
   `-s.joint("lbtn")`, with separate click and release thresholds (mouse-drag `_mouse`: 0.6 mm travel, 1500 N/m,
   click at 0.4 mm and release under 0.3 mm). For a hinged key with a preload, keyboard-two-hand-dynamics
   `_key_link` uses `hinge(..., "Y", pivot, -bed, 0.0, spring=k, effort=5.0)` plus `set_target` in reset. Buttons
   riding on a moving link are fr3-guide-swap's `enable`/`guide`/`teach` sliders, which have a `parent`.
2. **Drawer or blade on a slider**: put the carcass in `frame_parts`/`frame_colliders` and the moving piece as
   `slider(name, "X", lower, upper, parts, colliders, friction=...)`. The limits are the stops. slit-carton's
   `_blade_link` (20 mm travel, `friction=2.0`) is the accepted example, in a free knife (`free_mass=0.15`).
3. **Lid or flap latched shut until something frees it**: slit-carton `_flap` is
   `hinge(name, "X", hinge_at, 0.0, 200.0, ..., spring=0.3, gravity=True, com=c)` (for `flap_n`). `Trial.reset` calls
   `m.lock(env, "carton", "flap_n")`. Once the tape is cut, it calls `unlock` and then `set_target(..., radians(15))`
   so the flap springs up. Past 90 degrees it calls `set_target(..., radians(180))`, and the flap falls open.
4. **Lever with a permanent set**: tube-bender `_bender` has a `lever` hinge about Z (`spring=0.6`, `damping=0.3`,
   `effort=100`) on a free 0.4 kg base. A second `tube` hinge follows the lever and keeps its plastic set,
   through `m.set_target(env, "bender", "tube", -radians(self.tube))` every step. The scorer reads `-st.deg("lever")`.
5. **Catch, detent or brake**:
   - Sprung ramped pins that a part cams aside: alaris-set-load `_clip`, `prism` pins on Y sliders with
     `spring=CLIP_K`, plus `m.push` to hold the part while it is home.
   - A notch at each end of a slider's travel: slit-carton `_detent`, which writes a weak spring toward the
     nearest stop with the private `m._joint`/`m._tensor`. The public route is to give the link a `spring` and
     call `set_target` to the nearest notch.
   - Joints held until two buttons are pressed: fr3-guide-swap `_hold`, which calls `lock`/`unlock` on all
     seven joints, and only when the state changes.
6. **Knob turned by pinching**: `knob("valve", at, outward="+Z", mark="white", friction=0.03)` in a fixed
   mechanism (its looks need "knob" and "white"), read with `s.deg("valve")`, and turned in a demokit demo by closing on the wing and calling
   `turn(deg)`. No accepted task here uses `knob()` yet, so look at a run before trusting it.

## Nonzero joint starts: task-local reset correction, 17 November

The connector probe demonstrated that the ordinary Arena `Object` reset restored
its root but left the actual joint at USD zero despite a nonzero configured/default
joint position. An isolated no-contact probe reproduced configured TPA 5 mm versus
actual 0 mm. In the connector's local helper, assigning the jointed object's reset
event to `isaaclab_arena.terms.events.reset_articulation_pose_and_joints` restored
actual 5 mm before settling. This restores root, joint positions and velocities at
reset only. See `pipeline/pdf-screening/sr045-connector-contact/evidence.json`.
The shared helper and pending PRs have not been changed. Confirm the combined
reset API exists in the target Arena version before packaging this correction.


### Initial joint position is not a spring rest target (17 November connector fixture)

A reset that writes a joint to 5 mm does not also set its actuator target to 5 mm.
The connector's closed-retainer load fixture opened during reset settling because
its 8000 N/m spring still targeted zero. This invalidated that fixture, not the
retainer's measured locked strength. A fixture-local pose-event wrapper now calls
the normal root/joint reset and then sets only the TPA spring target to 5 mm before
settling. The lance retains its physical zero-angle rest target. Do not set every
spring target to its initial joint position: an initially deflected spring should
remain loaded. Check the first post-settle state before interpreting any trial.
