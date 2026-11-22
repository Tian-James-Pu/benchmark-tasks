# Straighten a leaning tabletop Christmas tree in its three-eye-bolt stand

A 3 ft (91 cm) tabletop tree stands in a small live-tree stand that holds the trunk with three eye bolts. Someone put it up with the
bolts uneven and never tightened them, so although all three bolts' steel end caps sit on the bark, the tree leans 2.5 degrees to the
robot's left (its top is about 4 cm off). The tree is a reusable stand-in for a fresh-cut one: a 1-3/4 in hardwood dowel trunk carrying
an artificial 2 ft pine top (see "The real objects"). The instruction is only:

> Straighten the little Christmas tree in its stand so that it stands upright and firm.

The trunk is caged by the three caps, so it cannot simply be pushed upright, and screwing in the bolt on the side the tree leans to
just jams against the other two. The bolt on the opposite side has to be eased out while the lean-side bolt is screwed in, by the
same amount (about 3.5 turns of each 1.41 mm-pitch bolt), until the trunk is within 1 degree of plumb. Then the bolts must be snugged
until each bites, so that the tree is firm: full credit needs every cap pressing on the bark and a firmness probe (a 5 N push on the
trunk towards each bolt in turn) that moves the treetop no more than 3 mm. The two eyes are 20 cm apart on opposite sides of the stand, and each must be turned by pinching its eye and rolling the wrist,
regripping every half turn or so, so in practice one arm turns each, with the backing-out bolt kept a little ahead.

## Response to review

| Review point | What changed |
|---|---|
| Construct validity: "Full credit permits a gap of up to 1 mm at every cap, so a tree with no cap actually gripping the trunk can score as firm." | The 1 mm allowance is gone. Full credit now needs every cap **bearing**: in the simulator the bark pushes each cap back along its thread with at least 3 N (a cap that only touches, or is 0.1 mm off, reads 0 N); in a real trial a 0.05 mm feeler blade will not enter at any cap. The start state now has the caps on the bark but not tightened, and the demonstration snugs both turned bolts until each bites (the eye stops following the wrist). Red team: the previous demonstration's end state (plumb, caps 0.0 to 0.6 mm off, none gripping), which scored 1.0 before, now scores 0.5 (`nosnug`); caps backed off to 1 mm and 0.5 mm after a full job score 0.5 (`gap1`, `gap05`). |
| Scoring: "A 1 mm feeler check cannot establish firm support." / "Add a reproducible real-world firmness check, e.g. a specified gentle lateral load and a maximum trunk or treetop deflection, and implement the matching full-credit condition in the simulator." | Added a **firmness probe** to full credit, the same in the simulator and a real trial: after the 2 s hold, a 5 N horizontal push on the bare trunk 25 cm above the butt, towards bolt A, then B, then C (1 s each), must move the treetop at most 3 mm and leave it within 1 mm once released, with the tree still plumb and every cap bearing. Real trial: a push gauge or luggage scale at a pencil ring on the trunk, and a ruler clamped level beside the treetop read from a phone video ("Real trial"). In `task.py` the scorer applies the force to the tree with Isaac Lab's wrench composer and reads the treetop from the gimbal (`Trial`, `push`). The 5 N is a fixed test load, not a force the robot must produce; the scored quantity is deflection. Gripped tree: under 0.01 mm (0.001 to 0.003 mm in both demonstrations); caps backed off to 1 mm: the push towards A moved the treetop 5.89 mm, failing. |
| Clarity: "no explanation of how to reliably source or prepare replacement fresh trees with the specified 44-46 mm trunk and bare section in a few minutes." / "Provide a practical repeatable way to source or prepare replacement trees, or use a reusable realistic tree stand-in that keeps the bolt-and-trunk manipulation." | The tree is now a **reusable stand-in**, made once: a 1-3/4 in (44.5 mm) round hardwood dowel cut to 32 cm (the bare trunk, with a spike hole in its butt), with a 2 ft artificial tabletop pine's pole epoxied into its top and fishing sinkers bringing it to 1.5 kg, with sourced links and steps in "The real objects". Nothing is replaced between trials; the trunk geometry the bolts and spike act on (45 mm round, bare for 30 cm, 91 cm tall, 1.5 kg) is unchanged, and so is the bolt-and-trunk manipulation. A fresh tree is optional. The simulated tree already looks like this (brown 45 mm trunk, conifer foliage), so the scene's appearance is unchanged. |

## Run and score

The scripted demonstration is closed loop (it reads the tree's lean, the bolts' positions and its own arms, never the scorer), so it
runs through `demokit.py` from this folder, in the simulator's Python environment:

```bash
cd tasks/plumb-tabletop-christmas-tree-in-three-screw-stand
python demokit.py task.py ~/yam-eval/plumb-tabletop-christmas-tree-in-three-screw-stand/demo --mode full          # writes full/demo.mp4, setup.png, result.json (task_score)
python demokit.py task.py ~/yam-eval/plumb-tabletop-christmas-tree-in-three-screw-stand/redteam --fast --mode none,onearm,careless,wrongway,hold,push,short,loose,nosnug,gap1,gap05
```

`--fast` runs physics on the CPU without cameras (for the scorer checks); the reported demo scores come from the recorded GPU runs.
A model trial uses the ordinary harness:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/plumb-tabletop-christmas-tree-in-three-screw-stand/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Straighten the little Christmas tree in its stand so that it stands upright and firm." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/plumb-tabletop-christmas-tree-in-three-screw-stand/model
```

The layout is fixed (`object_jitter=0`, no seed-dependent randomisation). The run limit is 2,400 control steps (120 s at 20 Hz); the
demonstration reaches full credit at step 1,159 (58.0 s), the firmness probe included. Wall time per recorded trial was about 30
minutes on a heavily shared GPU (0.6 steps/s); a `--fast` CPU run takes about 35 s of simulation and a minute of wall time.

## The real objects

- **Stand: Jack-Post Oasis 313-ST** live-tree stand, about $18. Listed as 13 in (33.0 cm) across and 7 in (17.8 cm) tall, 1.45 lb,
  green plastic, "3 eye bolts to secure tree", "steel end caps", a metal spike, a 5 in opening, a 0.75 gal reservoir, for trees 3 to
  5 ft high ([Christmas Central listing](https://www.christmascentral.com/heavy-duty-green-christmas-tree-stand-for-live-trees-up-to-5),
  [Do it Best listing, model 313-ST](https://www.doitbest.com/product/900284/oasis-5-ft-christmas-tree-stand-900284/)). The listings
  give only those overall sizes; the bowl's profile, the collar and the bolt height (15 cm) are our reconstruction inside them. The eye
  bolts are modelled as 5/16 in-18 UNC (1.411 mm a turn), 80 mm from eye to cap; measure the real stand's bolts before a trial and set
  `PITCH` and `BOLT_LEN` in `task.py` if they differ.
- **Tree: a reusable 3 ft tabletop tree stand-in**, made once and used for every trial, so no fresh tree has to be sourced or trimmed
  between trials. It keeps what the task needs from a real tree: a round 44 to 46 mm trunk bare for 30 cm above the butt (where the
  three caps and the spike bear), a tree's height and mass, and foliage above that looks like a small tree to the cameras.
  - Trunk: a **1-3/4 in (44.5 mm) x 36 in round hardwood dowel**, for example
    [Waddell 6354U at Home Depot](https://www.homedepot.com/p/reviews/Waddell-1-3-4-in-x-36-in-Round-Hardwood-Dowel-6354U/203706857/1)
    or [Woodpeckers DR-134-36](https://woodpeckerscrafts.com/products/wooden-dowel-rod-1-3-4-x-36). Cut it to 32 cm. Drill a 6 mm hole
    25 mm deep in the centre of the butt so that it seats on the stand's spike, and stain the bare part brown. Mark the probe point with a
    pencil ring 25 cm above the butt.
  - Top: a **2 ft (61 cm) artificial tabletop pine** sold in two pieces (tree and stand), for example the
    [Northlight 2' Full Pine, 97 tips, green](https://www.target.com/p/northlight-2-full-pine-artificial-christmas-tree-unlit/-/A-84825878).
    Pull its pole out of its plastic stand, drill a hole of the pole's diameter 40 mm deep into the top of the dowel, and glue the pole in
    with epoxy, so the lowest branches start 30 cm above the butt and the top is 91 cm above it.
  - Mass: weigh it, and bring it to 1.5 +-0.1 kg (the modelled mass) with fishing sinkers (sold by weight, for example two 16 oz bank
    sinkers) zip-tied to the dowel just under the lowest branches.
  - Making it takes about an hour with a saw and a drill, once. A 44.5 mm dowel does not compress or crush under the caps the way bark
    does, so it can be reused indefinitely; check it with calipers (44 to 46 mm at the bolts) if a cap has dented it.
  A fresh-cut 3 ft tabletop fir with a 44 to 46 mm, 30 cm bare handle can be used instead (for example
  [this 3 ft tabletop real tree](https://www.gardenista.com/products/3-ft-tabletop-premium-grade-real-christmas-tree/)), but it is
  optional; every number in the scoring applies to either.

## Scale and mass

Nothing is rescaled: the stand, the bolts and the tree are modelled at the real products' full size (the 3 ft tree is the smallest
the 313-ST is listed for). The stand is placed with its centre 34 cm in front of the arm bases, which puts the two front bolts' eyes
where the arms can hold a level grip along each bolt and roll the wrist through 220 degrees.

| | Real | Modelled |
|---|---|---|
| Stand | 13 in across, 7 in tall, 1.45 lb, 5 in opening (listed) | 33.0 cm, 17.8 cm, 12.7 cm opening; fixed to the table, reservoir empty |
| Tree | the stand-in above: 44.5 mm dowel trunk, 2 ft artificial top, brought to 1.5 kg on a scale | 91 cm, 45 mm trunk bare for 30 cm, 1.5 kg |
| Eye bolts | 5/16 in-18 (assumed; measure) | 1.411 mm/turn, eye 26 mm across, 6 mm wire |

The tree's mass only sets the small sideways load its lean puts on the caps (about 5 N at 2.5 degrees); the arms never carry it.

## How the simulator models it

- **Stand** (`stand`, fixed): bowl, deck, collar with three bosses and nuts, and the centre spike, as custom meshes; it collides as
  convex segments of the bowl and the collar that leave the trunk's well open.
- **Tree** (`tree`): an articulation fixed at the spike's tip, 2 cm above the table, with two hinges through that point (a gimbal), so
  the trunk pivots on the spike as a real butt does. Gravity acts on the whole 1.5 kg tree at 40 cm above the butt. The trunk collides as
  a 45 mm cylinder from the butt to the lowest branches; the foliage is seen only (real branches bend out of a hand's way). Lifting the
  tree off the spike is not modelled, and the hinges stop at 12 degrees.
- **Bolts** (`bolt_a`, `bolt_b`, `bolt_c`, one articulation each so that their caps collide with the trunk): a slider along the bolt
  carries the shank and the end cap (the cap collides as a 15 mm disc 9 mm long); a hinge about the bolt carries the eye (it collides as
  a 26 x 6 mm disc that a gripper pinches across the wire). The thread is code, run every control step (`Threads`): the eye's angle,
  counted through every turn, screws the bolt in 1.411 mm a turn (clockwise from outside screws it in). The slider is driven there with
  3e5 N/m and at most 300 N. The thread does not back-drive: a push on the tree never unscrews a bolt. When a cap is blocked (it lags
  0.3 mm, pressing about 90 N, behind where the eye has screwed it), the eye is held at the angle the cap allows by a 2 N m/rad drive,
  so a snug bolt stops turning, as a real one does when it bites the bark.
- **Firmness probe** (`Trial` and `push` in `task.py`): once the tree has stood plumb, with all three caps bearing, both grippers
  clear and the tree still for 2 s, the scorer applies a horizontal 5 N force to the tree's body at the point on the trunk's axis 25 cm
  above the butt, through Isaac Lab's wrench composer, towards bolt A, then B, then C (each ramped on over 0.25 s, held 1 s, released
  for 0.75 s), and reads the treetop (the trunk's axis 91 cm above the butt) from the gimbal's joints. The robot keeps control during
  the probe; if a gripper comes within 3 cm the probe stops and the hold starts over.
- **A cap bears** when the bark pushes it back along its thread with at least 3 N: the thread's spring (3e5 N/m) times how far the cap
  sits behind where its thread holds it. A cap merely touching, or 0.1 mm off, reads 0 N.
- Friction: the bolts' parts 0.5 static / 0.4 dynamic, the tree 0.6 / 0.5 (PhysX averages the pair in contact, so steel cap on bark
  is about 0.55 / 0.45); the stand 1.0 / 0.8.

## Scoring

Read from the trunk's tilt, whether each end cap bears on the bark, a firmness probe, and where the grippers are (`Trial` in `task.py`):

- **1.0**: the trunk within 1.0 degree of vertical, all three end caps bearing on the bark, every fingertip point of both grippers at
  least 3 cm from the trunk below the branches and from all three eyes, and the tree still (turning under 0.02 rad/s), all held for 2 s;
  then, with both grippers still 3 cm clear, the **firmness probe** passes: a 5 N horizontal push on the bare trunk 25 cm above the
  butt, towards bolt A, then B, then C (each ramped on over 0.25 s, held 1 s, then released for 0.75 s), moves the treetop at most 3 mm
  under the load and leaves it within 1 mm of where it stood once released, with the trunk within 1.0 degree and all three caps bearing
  after each release. The trial then ends with 1.0. A failed or interrupted probe starts the 2 s hold over.
- **0.5**: at the end, the trunk within 1.0 degree of vertical and both grippers 3 cm clear, short of full credit (for example a cap
  not bearing, or the probe moving the treetop more than 3 mm).
- **0.25**: at the end, all three caps bearing and both grippers clear, with the trunk within 2.0 degrees of vertical but more than
  1.0.
- **0**: anything else, including the starting 2.5 degree lean and a gripper still on the tree or a bolt.
- **Letting the tree flop scores 0 for the whole trial**: the trunk at any time more than 6 degrees from vertical (bolts backed off
  until the tree falls against the stand). This record is cleared only when the scene is reset.

How each rule is judged: the tilt is the angle of the trunk's axis from vertical, from the gimbal's two joints. **A cap bears** when the
bark pushes it back along its thread with at least 3 N (real trial: a 0.05 mm feeler blade, the thinnest in a standard set, will not
enter between the cap and the bark; the simulator's reading is the stricter one, since a cap that only touches reads 0 N). The
**probe's** 5 N is a fixed test load applied by the scorer, not something the robot must exert; what is scored is the treetop's
movement. The numbers are chosen so that a cap that is not gripping fails: with the caps 13 cm above the butt and the treetop 89 cm
above it, every 0.1 mm of play at a cap moves the treetop about 0.7 mm, so 3 mm allows at most about 0.44 mm of give at the caps in
total, while the 1 mm gap the earlier version allowed moves the treetop about 7 mm. On the gripped tree the simulator's treetop moves
less than 0.01 mm (rigid trunk and caps); a real stand's plastic collar and bolt threads will give a little, which the 3 mm leaves room
for. The grippers' clearance uses the kit's fingertip points (each finger's tip and 3 cm up it, at its middle and both edges). "Still"
reads the gimbal's joint speeds.

## Real trial

**Setup.** Table top as the station table. Stand centre 34 cm in front of the arm bases' midpoint, on the midline, with bolt C on the
far side (its eye facing away from the robot) and A and B at the front left and front right, 120 degrees either side; the base is held
to the table by two strips of double-sided carpet tape (the simulation fixes it), and the reservoir is left empty so that no water
reaches the gripper pads. Stand the tree on the spike with all three caps lightly on the bark and set it plumb
(angle gauge 0.0 +-0.1 degrees on two faces of the trunk 90 degrees apart). Then prepare the fault: turn A out and B in by equal
half-turns, C untouched, until the gauge on the trunk's face towards the robot's left reads 2.5 +-0.2 degrees with all three caps
touching the bark but not tightened (each eye still turns freely by finger). This takes about 3.5 turns of each and is the same every
trial. Clamp a steel ruler level on a stand beside the treetop, its scale just behind the top's tip, and set a phone to film it. Distractors:
candle at (50, 33) cm, mug at (48, -36) cm, notebook at (66, -24) cm, on tape marks. Both grippers open at their ready poses.

**Measurement.** Once the robot has stopped with both grippers 3 cm clear and the tree has stood still for 2 s, stop the robot and,
before anything else touches the tree:
1. a digital angle gauge held on the bare trunk (two faces, take the larger): within 1.0 degree for 0.5 and 1.0;
2. a 0.05 mm feeler blade at each cap: it must not enter at any of the three for a cap to count as bearing;
3. the firmness probe: a push gauge (or a luggage scale pulling, with a loop of string at the mark) held level on the pencil ring 25 cm
   above the butt, pushing the trunk towards bolt A, raised smoothly to 5 N and held for 1 s, then released; then the same towards B and
   towards C. Read the treetop on the clamped ruler from the phone video: before, at 5 N, and 1 s after release. It must move at most
   3 mm under the load and settle back to within 1 mm, and the angle gauge must still read within 1.0 degree with no feeler entering
   any cap afterwards;
4. the video, for the grippers' clearance and for whether the trunk ever leaned past 6 degrees (a protractor card behind the tree, read
   on the 60 fps video).

**Reset (about 2 minutes).** Repeat the fault preparation from wherever the bolts are: plumb with all caps lightly on the bark, then
A out and B in by equal half-turns to 2.5 +-0.2 degrees, caps not tightened. Put the distractors back on their marks. Nothing is
consumed: the dowel-trunk tree is reused for every trial and needs no replacement (check with calipers that the trunk still measures
44 to 46 mm at the bolts if a cap has dented it).

**Rig safety.** The eye bolts are blunt and turn under hand torque; the tree weighs about 1.5 kg and cannot fall off the spike while
any two caps bear. The 5 N probe is a gentle push by hand, made with the robot stopped. Keep the reservoir dry. Gripper pads touch only the zinc-plated eyes.

## Distinct from

| Nearest | What it tests | Why this is different |
|---|---|---|
| #2379, #2392 (not accepted, on importance) Level a builder's automatic level on its three foot screws; level an analytical balance on its two thumb-wheel feet | turning foot screws until a bubble centres | the nearest skill: screws turned against each other to tilt something. Those were specialist instrument setups; this is a household job, and here the screws cage the part from the side, so the one that holds it must be let out as the other pushes, or it jams |
| #880 (not accepted) Raise a hinged mast with its two guy lines and peg each so the mast stands plumb | tensioning two lines to stand a mast | here a caged trunk is moved by screw threads: one bolt must give way while the other drives, by matched amounts |
| #3504 (pending) Stop a wobbly stool rocking with a shim under the right foot | finding a rocking corner, inserting a shim | no shim or wedge; the support itself is re-adjusted by turning two eye bolts many turns |
| #2065 (accepted) Mark a point with a plumb bob | using a plumb line to locate a point | measuring only; nothing is adjusted |
| #2909, #2017 (not accepted) Set screw on a pulley or a boring bar | tightening one set screw | one screw, no coupled constraint, no straightening |
| BEHAVIOR `putting_up_Christmas_decorations_inside` | placing decorations next to a tree | pick-and-place round a static tree; the tree's stand is never touched |

Searches of every task pull request's title (about 3,500, all states, 21 November 2026) for Christmas, tree stand, eye bolt, plumb,
upright, level, foot screw, wing nut, thumbscrew, set screw and screw clamp, and of the BEHAVIOR activity list, found no accepted or
pending task that turns opposing screws against each other to re-align a part they cage; the two levelling tasks above are the
closest and were closed on importance, not as duplicates.

## Importance and sources

- Setting a real tree straight in an eye-bolt stand is a common, recognisable household job. The National Christmas Tree Association's
  consumer survey projects that **21.58 million real Christmas trees were purchased in the US in 2023**, by 14% of consumers surveyed
  ([NCTA, 2 November 2024](https://realchristmastrees.org/2024/11/02/what-ncta-wants-everyone-to-know-about-the-farm-grown-christmas-tree-supply-for-2024/)).
- The Real Christmas Tree Board's stand guide calls the classic eye-bolt stand the "most common stand type - and perhaps most feared",
  and says "set-up is a two-person job. One person holds the tree straight, while the other tightens the eye bolts to secure the
  trunk" ([RCTB Christmas Tree Stand Guide, PDF](https://realchristmastreeboard.org/wp-content/uploads/2022/10/RCTB-Stand-Guide.pdf)).
- The stand modelled is an ordinary three-eye-bolt stand for 3 to 5 ft trees
  ([Christmas Central listing](https://www.christmascentral.com/heavy-duty-green-christmas-tree-stand-for-live-trees-up-to-5)), and
  3 ft tabletop trees are sold as such
  ([3 ft tabletop real tree listing](https://www.gardenista.com/products/3-ft-tabletop-premium-grade-real-christmas-tree/)).

Limits of this evidence: the survey counts real trees of every size, not tabletop trees, and does not count how many households use an
eye-bolt stand; the RCTB guide says eye-bolt stands are the most common type without a number. The 1.5 kg tree mass is set on the stand-in
with ballast (a fresh tree's mass is our estimate), and the 5/16 in-18 bolt thread is our estimate, flagged above.

## Assets and helpers

The tree, the stand and the bolts are custom meshes written by `task.py` (no SimReady prop exists for a Christmas tree or a tree
stand). The distractors are NVIDIA SimReady props, downloaded at first run from the pinned revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd` (© NVIDIA, CC-BY 4.0; masses set at load time, nothing else changed):

- [decor_candle_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/decor_candle_a01): unlit pillar candle, 0.35 kg.
- [dishware_mug_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01): mug, 0.30 kg.
- [misc_notebook_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/misc_notebook_a01): notebook, 0.25 kg.

`mechkit.py` is the shared mechanism kit from `tasks/heat-shrink-lug/` in this repository (another team's open-source kit), unchanged.
`simready.py` is the SimReady loader from `tasks/cutters-off-pegboard/`, unchanged apart from its docstring. `demokit.py` is the
closed-loop demo runner shared by `tasks/heat-shrink-lug/` and other tasks, unchanged.

## Validation

Scripted demonstration (`--mode full`, recorded on the GPU, scores from the harness's `task_score`):

| Run | Score | Control steps | End of turning |
|---|---:|---:|---|
| demo 1 | 1.0 | 1,159 (58.0 s) | plumbed at 40 s: 0.25 degrees, no cap yet bearing (0.43, 0.25, 0.02 mm off); snugged at 50 s: 0.21 degrees, caps A, B, C bearing 78, 51, 49 N; probe: treetop 0.001 mm (towards A) and 0.003 mm (towards B) under 5 N, then C, all passed |
| demo 2 | 1.0 | 1,159 (58.0 s) | the same, to the logged precision |

In both, A was screwed in 1,330 degrees and B backed out 1,369 degrees in eight regrips each, which brought the tree plumb; then B was
turned back in and A further in, two pinch-turns each, until each stopped following the wrist (A to 1,492 degrees in all, B to -1,172);
C was not touched. Stopping before the snugging (the previous demonstration) now scores 0.5 (`nosnug`, below).

Failure checks, all on the final `task.py`, physics on the CPU (`--fast`, same physics settings; the demonstration also scored 1.0 this
way, at 1,161 steps). "Bearing" is how hard the bark pushes each cap back (A, B, C):

| Mode | What it does | Score | End state |
|---|---|---:|---|
| none | nothing | 0.0 | 2.53 degrees, as set; only A bears (1 N, the lean) |
| onearm | screws A in for six regrips without touching B | 0.0 | A jams after 238 degrees of turn: 2.29 degrees (outside 2.0) |
| careless | backs B out six regrips without touching A | 0.0 | tree stays on A: 2.53 degrees, B's cap 4.0 mm off the bark |
| wrongway | backs out A, the lean-side bolt | 0.0 | the tree flops past 6 degrees (6.3) |
| push | pushes the trunk towards plumb by hand and lets go | 0.0 | caged: 2.37 degrees while pushed, 2.53 after |
| hold | the full job, then keeps the left gripper on A's eye | 0.0 | 0.20 degrees and gripped (93, 79, 47 N), but a gripper on the bolt; the probe stops when it starts |
| short | stops both bolts at 1.5 degrees, then snugs them | 0.25 | 1.40 degrees, all three caps bearing (40, 73, 59 N) |
| loose | the full job, then backs A out two regrips | 0.5 | 0.14 degrees; no cap bearing, A's cap 0.45 mm off the bark |
| nosnug | straightens the tree but never snugs the bolts: the previous version's demonstration, which scored 1.0 there | 0.5 | 0.37 degrees; caps 0.66, 0.00, 0.00 mm from the bark, none bearing (0 N) |
| gap1 | the full job; then, just as the probe's push towards A starts, the test itself (not the arms) backs every cap's thread off to leave it 1 mm from the bark: the review's case | 0.5 | the tree settles onto B and C (0.86 degrees, A 2.15 mm off); the push towards A moved the treetop 5.89 mm under load and 10.3 mm after release, failing, and with no cap bearing the hold never restarts |
| gap05 | the same with 0.5 mm gaps | 0.5 | 0.44 degrees, A 0.74 mm off, no cap bearing; the push towards A moved the treetop 2.70 mm under load and 3.81 mm after release, failing on the return |

With caps backed off, the bearing rule alone already denies full credit; the probe is the second, measurable check, and on the gripped
tree in the demonstrations it moved the treetop 0.001 to 0.003 mm. The simulated stand is stiffer than a real one (rigid caps that
hold the trunk by friction once preloaded), so the 3 mm limit is set by the play a loose cap leaves, not by the simulator's figure.

Model run: Dream-1, run once with the instruction above through the harness command in "Run and score", on the previous revision of
`task.py`, scored 0.0 over the full 2,400 steps; after one early swing of the right arm out to the side it kept both arms drawn back
near their bases and never reached the stand, so the tree stayed at its starting lean (2.53 degrees). This revision did not change the
scene (the objects, their sizes, poses, masses, colliders and the thread model are the same; one helper now lists a stand collider's
triangles without scipy, from the same points), only the scorer and the demonstration, and every new rule only adds conditions, so that
end state scores 0.0 under this scorer too; the run was not repeated.

The contact sheets and videos of the recorded runs were inspected: the grippers pinch the eyes across the wire, the eyes turn with
the wrist, nothing passes through the stand or the caps, and the distractors are not touched. The arms work under the lowest branches;
the foliage, which has no collider, hides the stand from the overhead camera, so the wrist cameras are the views that show the bolts.
