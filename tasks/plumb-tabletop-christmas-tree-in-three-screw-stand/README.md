# Straighten a leaning tabletop Christmas tree in its three-eye-bolt stand

A 3 ft (91 cm) fresh-cut tabletop fir stands in a small live-tree stand that holds the trunk with three eye bolts. Someone put it up
with the bolts uneven, so although all three bolts' steel end caps touch the bark and the tree is firm, it leans 2.5 degrees to the
robot's left (its top is about 4 cm off). The instruction is only:

> Straighten the little Christmas tree in its stand so that it stands upright and firm.

The trunk is caged by the three caps, so it cannot simply be pushed upright, and screwing in the bolt on the side the tree leans to
just jams against the other two. The bolt on the opposite side has to be eased out while the lean-side bolt is screwed in, by the
same amount (about 3.5 turns of each 1.41 mm-pitch bolt), until the trunk is within 1 degree of plumb with every cap still on the
bark. The two eyes are 20 cm apart on opposite sides of the stand, and each must be turned by pinching its eye and rolling the wrist,
regripping every half turn or so, so in practice one arm turns each, with the backing-out bolt kept a little ahead.

## Run and score

The scripted demonstration is closed loop (it reads the tree's lean, the bolts' positions and its own arms, never the scorer), so it
runs through `demokit.py` from this folder, in the simulator's Python environment:

```bash
cd tasks/plumb-tabletop-christmas-tree-in-three-screw-stand
python demokit.py task.py ~/yam-eval/plumb-tabletop-christmas-tree-in-three-screw-stand/demo --mode full          # writes full/demo.mp4, setup.png, result.json (task_score)
python demokit.py task.py ~/yam-eval/plumb-tabletop-christmas-tree-in-three-screw-stand/redteam --fast --mode none,onearm,careless,wrongway,hold,push,short,loose
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
demonstration reaches full credit at step 847 (42.4 s). Wall time per recorded trial was about 10 minutes on a GPU shared with five other
trials (1.4 steps/s); a `--fast` CPU run takes about 30 s of wall time.

## The real objects

- **Stand: Jack-Post Oasis 313-ST** live-tree stand, about $18. Listed as 13 in (33.0 cm) across and 7 in (17.8 cm) tall, 1.45 lb,
  green plastic, "3 eye bolts to secure tree", "steel end caps", a metal spike, a 5 in opening, a 0.75 gal reservoir, for trees 3 to
  5 ft high ([Christmas Central listing](https://www.christmascentral.com/heavy-duty-green-christmas-tree-stand-for-live-trees-up-to-5),
  [Do it Best listing, model 313-ST](https://www.doitbest.com/product/900284/oasis-5-ft-christmas-tree-stand-900284/)). The listings
  give only those overall sizes; the bowl's profile, the collar and the bolt height (15 cm) are our reconstruction inside them. The eye
  bolts are modelled as 5/16 in-18 UNC (1.411 mm a turn), 80 mm from eye to cap; measure the real stand's bolts before a trial and set
  `PITCH` and `BOLT_LEN` in `task.py` if they differ.
- **Tree: a fresh-cut 3 ft tabletop fir** (Fraser or similar), sold as a "3 ft tabletop" tree (for example
  [this 3 ft tabletop real tree, $69.99 with a stand](https://www.gardenista.com/products/3-ft-tabletop-premium-grade-real-christmas-tree/)).
  Modelled 91 cm tall from the butt, trunk 45 mm (1.77 in) across, bare for 30 cm above the butt (tabletop trees are sold with a bare
  "handle" for the stand), foliage 43 cm across at the bottom tier.

## Scale and mass

Nothing is rescaled: the stand, the bolts and the tree are modelled at the real products' full size (the 3 ft tree is the smallest
the 313-ST is listed for). The stand is placed with its centre 34 cm in front of the arm bases, which puts the two front bolts' eyes
where the arms can hold a level grip along each bolt and roll the wrist through 220 degrees.

| | Real | Modelled |
|---|---|---|
| Stand | 13 in across, 7 in tall, 1.45 lb, 5 in opening (listed) | 33.0 cm, 17.8 cm, 12.7 cm opening; fixed to the table, reservoir empty |
| Tree | 3 ft fresh-cut tabletop fir; mass not listed by sellers | 91 cm, 45 mm trunk, **1.5 kg (our estimate)**; weigh the real tree before trials |
| Eye bolts | 5/16 in-18 (assumed; measure) | 1.411 mm/turn, eye 26 mm across, 6 mm wire |

The tree's mass only sets the small sideways load its lean puts on the caps (about 3 N at 2.5 degrees); the arms never carry it.

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
- Friction: the bolts' parts 0.5 static / 0.4 dynamic, the tree 0.6 / 0.5 (PhysX averages the pair in contact, so steel cap on bark
  is about 0.55 / 0.45); the stand 1.0 / 0.8.

## Scoring

Read from the trunk's tilt, each end cap's gap to the bark, and where the grippers are (`Trial` in `task.py`):

- **1.0**: the trunk within 1.0 degree of vertical, all three end caps bearing on the bark (gap at most 1 mm: a 1 mm feeler gauge
  will not pass), every fingertip point of both grippers at least 3 cm from the trunk below the branches and from all three eyes, and
  the tree still (turning under 0.02 rad/s), all held for 2 s. The trial then ends with 1.0.
- **0.5**: at the end, the trunk within 1.0 degree of vertical and both grippers 3 cm clear, short of full credit (for example a cap
  not bearing on the bark).
- **0.25**: at the end, all three caps bearing and both grippers clear, with the trunk within 2.0 degrees of vertical but more than
  1.0.
- **0**: anything else, including the starting 2.5 degree lean and a gripper still on the tree or a bolt.
- **Letting the tree flop scores 0 for the whole trial**: the trunk at any time more than 6 degrees from vertical (bolts backed off
  until the tree falls against the stand). This record is cleared only when the scene is reset.

How each rule is judged: the tilt is the angle of the trunk's axis from vertical, from the gimbal's two joints. A cap's gap is its
face's distance from the stand's axis minus the bark's reach towards it at the bolt's height (the trunk's offset there, its radius, and
the cap's nearest edge on the leaning trunk). The grippers' clearance uses the kit's fingertip points (each finger's tip and 3 cm up
it, at its middle and both edges). "Still" reads the gimbal's joint speeds. The thresholds are numbers a person can check with a
digital angle gauge, a 1 mm feeler gauge and a video frame.

## Real trial

**Setup.** Table top as the station table. Stand centre 34 cm in front of the arm bases' midpoint, on the midline, with bolt C on the
far side (its eye facing away from the robot) and A and B at the front left and front right, 120 degrees either side; the base is held
to the table by two strips of double-sided carpet tape (the simulation fixes it), and the reservoir is left empty so that no water
reaches the gripper pads. Stand the tree on the spike with all three caps lightly on the bark and set it plumb
(angle gauge 0.0 +-0.1 degrees on two faces of the trunk 90 degrees apart). Then prepare the fault: turn A out and B in by equal
half-turns, C untouched, until the gauge on the trunk's face towards the robot's left reads 2.5 +-0.2 degrees with all three caps on
the bark (a 1 mm feeler gauge does not pass at any cap). This takes about 3.5 turns of each and is the same every trial. Distractors:
candle at (50, 33) cm, mug at (48, -36) cm, notebook at (66, -24) cm, on tape marks. Both grippers open at their ready poses.

**Measurement.** Read the end state before anyone touches the scene: a digital angle gauge held on the bare trunk (two faces, take
the larger), a 1 mm feeler gauge at each cap, and the video for the grippers' clearance and for whether the trunk ever leaned past 6
degrees (a protractor card behind the tree, read on the 60 fps video).

**Reset (about 2 minutes).** Repeat the fault preparation from wherever the bolts are: plumb with all caps on the bark, then A out
and B in by equal half-turns to 2.5 +-0.2 degrees. Put the distractors back on their marks. Nothing is consumed; replace the tree if
it drops needles or its bark is crushed at the caps (check that the trunk still measures 44 to 46 mm across at the bolts).

**Rig safety.** The eye bolts are blunt and turn under hand torque; the tree weighs about 1.5 kg and cannot fall off the spike while
any two caps bear. Keep the reservoir dry. Gripper pads touch only the zinc-plated eyes.

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
eye-bolt stand; the RCTB guide says eye-bolt stands are the most common type without a number. The 1.5 kg tree mass and the 5/16 in-18
bolt thread are our estimates, flagged above.

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
| demo 1 | 1.0 | 847 (42.4 s) | 0.25 degrees from vertical; caps A, B, C 0.43, 0.25, 0.02 mm from the bark |
| demo 2 | 1.0 | 847 (42.4 s) | the same |

In both, A was screwed in 1,330 degrees (5.21 mm) and B backed out 1,369 degrees (5.37 mm) in eight regrips each; C was not touched.

Failure checks, all on the final `task.py`, physics on the CPU (`--fast`, same physics settings; the demo also scored 1.0 this way, at
859 steps):

| Mode | What it does | Score | End state |
|---|---|---:|---|
| none | nothing | 0.0 | 2.53 degrees, as set |
| onearm | screws A in for six regrips without touching B | 0.0 | A jams: 238 degrees of turn, 2.29 degrees |
| careless | backs B out six regrips without touching A | 0.0 | tree stays on A: 2.53 degrees, B's cap 4.0 mm off the bark |
| wrongway | backs out A, the lean-side bolt | 0.0 | the tree flops past 6 degrees (6.3) |
| push | pushes the trunk towards plumb by hand and lets go | 0.0 | caged: 2.37 degrees while pushed, 2.53 after |
| hold | the full job, then keeps the left gripper on A's eye | 0.0 | 0.37 degrees and firm, but a gripper on the bolt |
| short | stops both bolts at 1.5 degrees | 0.25 | 1.50 degrees, all caps bearing |
| loose | the full job, then backs A out two regrips | 0.5 | 0.37 degrees, A's cap 2.1 mm off the bark |

Model run: Dream-1, run once on the final `task.py` with the instruction above through the harness command in "Run and score",
scored 0.0 over the full 2,400 steps; after one early swing of the right arm out to the side it kept both arms drawn back near their
bases and never reached the stand, so the tree stayed at its starting lean.

The contact sheets and videos of the recorded runs were inspected: the grippers pinch the eyes across the wire, the eyes turn with
the wrist, nothing passes through the stand or the caps, and the distractors are not touched. The arms work under the lowest branches;
the foliage, which has no collider, hides the stand from the overhead camera, so the wrist cameras are the views that show the bolts.
