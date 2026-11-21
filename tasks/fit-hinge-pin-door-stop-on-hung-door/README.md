# Fit a hinge-pin door stop on a hung door

A hinge-pin door stop keeps a door's handle from punching into the wall. It needs no screws or drilling: it hangs on
the door's top hinge. The top hinge pin comes out, the stop's ring goes on top of the top knuckle, and the pin goes
back down through the ring and all five knuckles so that its head clamps the stop. The instruction is only the goal:
**"Fit the hinge-pin door stop on this door."**

The task starts with the door hung and nearly shut and the top pin already tapped up 40 mm. The robot must:

1. Pull the pin straight up out of the hinge. It is hand-loose but binds in its 6.36 mm bore if pulled crooked.
2. Set the stop's ring on the top knuckle, centred over the bore, and keep it there. The knuckle is 10.5 mm across
   and the stop's weight hangs about 4 cm out from the ring, so the stop tips off unless something holds it.
3. Stand the pin upright over the ring and push it down through the ring and the knuckles. The ring's hole has
   0.28 mm of clearance on the pin and the knuckle bores 0.21 mm.
4. Let go of the stop once the pin is through it, press the pin home until its head sits on the stop, and move both
   hands clear.

One hand holds the stop on the knuckle while the other brings the pin through it. That is how the makers' and
retailers' instructions describe the job (see Importance and sources). The access is tight. The pin's axis is
16 mm in front of the door's face and 6.4 mm from the casing's edge, and the door's top is 178 mm above the hinge,
so neither hand can come straight down over the pin.

## Run and score

Use the Python environment that has `yam_arena` and `inspect-robots` (`source ~/IsaacLab-Arena/yam.env`). The demo
is a closed-loop script (`demo_script` in `task.py`, run by `demokit.py`):

```bash
cd tasks/fit-hinge-pin-door-stop-on-hung-door
python demokit.py task.py ~/yam-eval/hinge-stop --mode full                 # demo: video, setup.png, result.json (task_score)
python demokit.py task.py ~/yam-eval/hinge-stop --fast --mode none,onearm,careless,placed,held   # red-team checks, CPU physics
```

A model trial (no demo script) uses the standard runner:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/fit-hinge-pin-door-stop-on-hung-door/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the hinge-pin door stop on this door." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/hinge-stop-model
```

The layout is fixed (`object_jitter=0`) and nothing is seeded. The demo takes about 1,150 control steps (58 simulated
seconds). The run limit is 2,400 steps (2 minutes at 20 Hz). The demo script reads only what a policy could see:
the poses of the pin, the stop and the door, and its own arms' joints. It never reads the scorer's state.

## Scoring

The tracker (`Trial` in `task.py`) reads the state at every control step, and `score()` returns its value.

- **Stop seated:** the ring's centre is within 1.0 mm of the hinge axis, its underside is -0.5 to +2.0 mm from the
  top knuckle's top, and the stop is tilted at most 10 degrees.
- **Pin home:** the pin's head and tip are within 1.0 mm of the axis and the pin is within 3 degrees of vertical. Its
  head's underside is -0.5 to +1.5 mm from the top of the stop's ring.
- **1.0:** stop seated and pin home, both still (no tracked point moves more than 0.5 mm between steps), and every
  fingertip point at least 3 cm from the pin and from the stop, all continuously for 1 s.
- **0.75:** stop seated and pin home, but not yet 1.0 (still held, or not yet for 1 s).
- **0.5:** stop seated, with the pin through its ring and at least the top knuckle: head and tip within 1.0 mm of the
  axis, and the tip at least 18 mm below the knuckle's top.
- **0.25:** stop seated, with the pin not through it. This state can only last while a hand holds the stop.
- **0:** everything else, including the starting state, the pin put back without the stop, and the stop off the
  hinge.

Full credit lasts only while the state that earned it does.

**Measuring a real trial.** The checks are on surfaces you can see and use ordinary tools:

- Look across the top of the hinge: the ring lies flat on the top knuckle and the pin's head sits on the ring. A
  phone level on the stop's bar reads 10 degrees or less.
- A 1.5 mm feeler leaf must not pass under the pin's head.
- Hands off: watch the hinge for 1 s.

The 0.5 level is checked with a ruler against the pin's head: the head stands at most 70 mm above the ring (88 mm of
pin, less 18 mm).

## Scale and mass

Every part is modelled at **100 % of real size**, and no mass is reduced.

| Part | Real | Modelled | Source |
|---|---|---|---|
| Door leaf | 28 x 80 in (711 x 2032 mm), 1-3/8 in (35 mm) | 711 mm wide, 35 mm thick; only the top 273 mm is drawn | standard interior door sizes (Bob Vila) |
| Door mass | hollow-core leaf, about 10 kg (estimate) | 10 kg and the whole leaf's inertia, on the hinge joint | estimate |
| Jamb | 4-1/2 in deep plus 1/2 in drywall, i.e. 4-9/16 in | 114 mm | Bob Vila |
| Top hinge | 3-1/2 x 3-1/2 in, five knuckles, 0.085 in leaves, 0.234 in pin | 88.9 mm, 5 x 17.8 mm knuckles, 5.94 mm pin, 6.36 mm bore | Hager 1741 spec sheet |
| Top hinge position | its top 7 in below the door's top | 178 mm | woodworkers' 7-and-11 rule |
| Pin | 0.234 x about 3-1/2 in steel, button head | 5.94 x 88 mm, 10.4 mm head, 22 g (steel density) | Hager 1741 |
| Stop | Ives 70 type; Everbilt equivalent 2.62 x 1.87 x 0.63 in | 70 x 33 x 12.5 mm, 35 g (estimate) | Ives catalogue E22 |

The only change is the **mounting**. The arms work at table height, so the doorway is lowered. The table top stands
in for a plane 1.75 m above the floor, and the top hinge's top knuckle is 100 mm above the table (1,854 mm above the
floor in a real door). Only the top 450 mm of the wall section and the top 273 mm of the door leaf are drawn. The
door's lower 1.75 m and its middle and bottom hinges are represented by the hinge joint, which turns about the top
pin's axis as those hinges make it do. The joint carries the whole leaf's 10 kg and inertia. All geometry the hands
and the stop touch keeps its real relative position: the door's face, the casing, the head jamb 181 mm above the
hinge, and the knuckles.

## Mechanism and physical model

- **Door** (`door`, an articulation fixed to the world). The frame holds the wall section, the jambs, the head jamb,
  the casing on this side (2-1/4 in, 11 mm thick at its inner edge, 3/16 in reveal) and the top hinge's jamb leaf
  with knuckles 1, 3 and 5.
  - Each knuckle is bored 6.36 mm for the 5.94 mm pin, built as 12 convex sectors. The top knuckle's bore is
    chamfered 0.6 mm.
  - The door leaf is a link on a hinge about the pin's axis, from shut to 170 degrees open, with 2 N m s/rad damping.
    It carries knuckles 2 and 4; they turn about the pin's own axis and so never bear on it.
- **Pin** (`pin`, a free body, 22 g): the shank as a 16-sided hull with a rounded tip, plus a button head. It has
  steel-on-steel friction (0.5 static, 0.4 dynamic). The pin is hand-loose, as after it has been tapped up. That fit
  is modelled as sliding friction along the pin's axis, of up to 3 N, while its tip is in the top knuckle's bore and
  it is on the hinge's axis. Each step the friction applies only what stops the pin within that step (class `Fit`).
  - Without it, the pin would slide down its bore under its own weight, and a real one does not.
  - The reset's settle runs no hooks, so the reset hook puts the pin back at its tapped-up height, at rest, before
    the first frame.
- **Stop** (`stop`, a free body, 35 g):
  - a zinc ring 3.5 mm thick with a 6.5 mm hole, chamfered 0.5 mm;
  - a 6 mm bar to a 12 mm-tall boss 64 mm out;
  - a rubber tip on a stud beside the ring that bears on the casing, and a rubber tip on the boss's side that bears on
    the door's face.
  
  With the stud as set, the door stops at about 103 degrees open, inside the Ives 70's 85 to 125 degree range. The
  adjusting thread is not modelled.
- Not modelled: forces (none is scored); a tight pin that needs a hammer (the robot has none); the door's sag when the
  top pin is out (a real door sags well under 1 mm when its lower two hinges hold it).

## Distinct from

| Nearest board task | What it tests | How this differs |
|---|---|---|
| #3094 TK-NVTZYW (pending): lift a cabinet door off its hinge pins | lifting a door leaf off fixed pins | Here the door stays hung: a loose pin comes out and goes back through a loose part that one hand holds on the knuckle |
| #1969 TK-Q8FKPK (accepted): shim a door's hinge jamb | driving opposed shims behind a jamb | Different object and skill. No shims, and nothing is driven |
| #2662 TK-KDQ4CB (closed): prop a door open with a rubber wedge | pushing a wedge under a door | A floor wedge, not hinge hardware, with no part captured between others |

Also checked: #3313 (rehang an enclosure door on removable pins, closed on importance as specialist maintenance),
#166 (a miniature door on a pin hinge) and the door, hinge, pin and stop titles in the task board's PR list. No board
task fits a hinge-pin stop.

## Importance and sources

Hinge-pin stops are sold at every hardware store as the drill-free way to protect a wall from a door handle. Fitting
one is a common household job: anyone with a swinging door near a wall can do it with a screwdriver and a hammer.

- [Ives residential door stops catalogue, page E22](https://s3.amazonaws.com/s3-absupply-net/pdf/ives/ives-residentialdoorstops-brochure-132-134.pdf):
  the 70, 72 and 73 hinge-pin door stops are "for use on hinged doors where the installation of a standard door stop
  is not desirable". They have 85 to 125 degree door opening adjustments, and removable bushings accommodate 1/4 to
  5/16 in hinge pins.
- [Ives 70 retailer listing](https://www.americanlocksets.com/ives-70-hinge-pin-door-stop-adjustable-85-deg-to-125):
  price $2.23 to $4.10 each, "Easy installation".
- [Hinge Outlet: Hinge pin door stops, how they work and how to install them](https://www.hingeoutlet.com/blogs/news/hinge-pin-door-stops-how-they-work-and-how-to-install-them).
  The stop "mounts onto the top hinge of your door, sharing the same hinge pin". Its steps: tap the pin up, remove it
  "just enough to slide the stop onto the hinge", "line up the holes in the door stop bracket with the hinge
  knuckles", and reinsert the pin through both.
- [Hager 1741 residential hinge specification](https://cdn.shopify.com/s/files/1/0052/5395/5651/files/0121-MjcwMTMzLzAvbWFzdGVy.pdf):
  full mortise, 5-knuckle, removable pin, 0.085 in gauge, 0.234 in pin, 3-1/2 x 3-1/2 in size.
- [Bob Vila: standard door sizes](https://www.bobvila.com/articles/standard-size-for-doors/): interior doors are
  80 in tall and 1-3/8 in thick, 28 in widths are available, and prehung jambs are 4-9/16 in.
- [WOODWEB: hinge placement rules and traditions](https://woodweb.com/knowledge_base/Hinge_Placement_Rules_and_Traditions.html):
  "7 from the top, 11 from the bottom" is the most common layout.

These sources establish the product, the operation and the dimensions. They do not measure how many households fit
such stops. The relatability claim is that the job is ordinary home hardware work, not an occupation.

## Real trial

**Build.** You need:

- A standard 28 x 80 in hollow-core prehung interior door, inswing, with 3-1/2 in five-knuckle hinges with 0.234 in
  removable pins (for example Hager 1741).
- One adjustable hinge-pin door stop (Ives 70 or an equivalent; about $5).
- A coffee mug, a notebook and a roll of tape as distractors.

Mount the door in a rough opening, or in a freestanding stud frame screwed to the floor, at full height. Do not
lower the real door: the real trial uses the full-size product, with the arms raised or on a stand so that the top
hinge (about 1.85 m up) is at the height it has above the arms in the simulation (100 mm above the arms' table).

**Set up.**

1. Shut the door, then open it 1 in at the latch edge (2 degrees).
2. Tap the top pin up from below with a nail set until a 40 mm gauge block fits under its head.
3. Lay the stop flat on a tray or shelf at the arms' table height, ring at the tape mark 7.5 cm in front of and
   14 cm to the left of the hinge axis, bar pointing at the robot.
4. Place the distractors on their marks.

**Reset** takes about 2 minutes and nothing is consumed:

1. Pull the pin and lift the stop off.
2. Push the pin home without the stop.
3. Tap it up to the 40 mm gauge block.
4. Put the stop on its mark.
5. Reset the door's angle.

**Safety.** The pin is light and the stop has rubber tips. Keep fingers away from the hinge side of the door while
the arms move, since a door pushed shut pinches at the hinge.

## Assets and helpers

The door, wall, hinge, pin and stop are authored structural and mechanical parts (`mechkit.py` meshes and colliders).
The distractors are NVIDIA SimReady props. They are downloaded at the first run from the pinned revision in
`simready.py`, © NVIDIA, CC-BY 4.0:

- `dishware_mug_a01`: mass set to 0.30 kg.
- `misc_notebook_a01`: mass set to 0.12 kg.
- `tape_clear_a01`: mass set to 0.10 kg.

No textures or meshes are bundled. `mechkit.py` is copied unchanged from `tasks/heat-shrink-lug/mechkit.py`, another
team's open-source kit. `simready.py` is adapted from the open-source `cutters-off-pegboard` loader. `demokit.py` is
copied from `tasks/heat-shrink-lug/demokit.py`.

## Validation

**Not yet validated: the demo is not reliable.** On CPU physics (`--fast`), the closed-loop demo scored 1.0 in four
runs, including one with 3 mm of object jitter, and 0.0 in two runs. On the GPU, with cameras, one recorded run
scored 0.0. In every failure the pin jammed at the mouth of the ring or the top knuckle. The demo is not yet
submittable; see the open issue below.

Red-team modes of the same script, run with `--fast` (CPU physics, no cameras):

| Mode | What it does | Score |
|---|---|---:|
| none | nothing | 0.0 |
| onearm | the stop set on the knuckle and let go before the pin goes in: it tips off the knuckle | 0.0 |
| careless | the pin pushed back without the stop | 0.0 |
| placed | the stop held on the knuckle, the pin left out | 0.25 |
| held | as full, but the hand stays on the pin's head | 0.75 |

**Open issue.** The right hand holds the pin in its fingertips, and the wrist is compliant. The servo leaves the
pin's tip 1 to 2 mm off the axis over the ring. The ring's hole has 0.28 mm of clearance and the knuckle bores
0.21 mm, so some attempts jam. Planned fix: detect the jam and back off (partly in place), add a small circular
search at the mouth, and grip the pin lower so it tilts less.

No model score is claimed.
