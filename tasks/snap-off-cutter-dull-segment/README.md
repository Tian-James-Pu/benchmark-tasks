# Snap a snap-off cutter's dull end segment into a pocket disposal can and put the knife down retracted

**Instruction:** "Snap the dull end segment off the cutter's blade, put it in the blade-disposal case, and put the knife down with
its blade retracted."

An 18 mm snap-off cutter lies on a cutting mat with its blade left out about 30 mm, so that two score lines stand past the steel
nose. A pocket blade-disposal can stands beside it with its door open over the snapping slot. The robot has to set the blade back
one notch-run so that only the first score line is outside the nose, hold the light can, put the tip into the slot up to the
first score line, bend the handle over until exactly one 12.5 mm segment snaps off into the can, then retract the rest of the
blade fully and leave the knife lying on the mat.

## Importance and sources

**Category: Societal relatability.** The snap-off cutter (the "box cutter" or "craft knife" with a segmented blade) is in most
kitchen drawers, toolboxes and offices, and snapping off the dull segment is its only routine upkeep: OLFA, which invented the
segmented blade, built the whole product around it. Anyone who has used one for long has snapped a segment and has had to decide
where the sharp scrap goes. The task is recognisable in a ten-second clip.

**The procedure being tested** is the maker's own. OLFA's "Blade Snapping 101" gives, for the DC-3 pocket disposal can: "open the
sliding door of the can, insert the dull tip into the can up to the break line of the blade, then push the handle down: the blade
will snap off and fall into the can", and warns to cover the end so that the tip cannot fly off
(<https://olfa.com/blogs/professional/blade-snapping-101>). OLFA's blade instructions add that the blade must not be extended so
far that a second segment line is past the end of the steel channel, and that only one segment is used at a time.

**The knack, in one sentence with numbers:** the blade starts with two score lines past the nose (tip 30 mm out), and a snap made
like that breaks at the second line and throws away 25 mm of blade, so one hand first pushes the 3 N thumb slider back two to four
notches (2.5 mm each) until only line 1 is out (tip 13-24 mm out, the demo uses 20 mm), then that hand holds the 32 g can while the
other puts the tip into a 3 mm slot to within about 1 mm of line 1 and tilts the 45 g knife about the slot's edge through about
20 degrees (about 10 degrees of slot play, then a 10 degree, roughly 0.05 N m bend) until the segment snaps and drops in; then the
slider goes back another five notches with the knife held, so that no steel is left past the nose.

Two arms with different roles are needed throughout: the knife must be held while its slider is moved (it weighs 45 g and slides
on the mat under a 3 N push), and the can must be held while the blade is levered in its slot (it is 32 g and 6.3 cm tall and
tips or slides under the bending load).

**Products (real size):**

| Item | Product | Size and mass used | Price |
|---|---|---|---|
| Knife | OLFA 18 mm slider-lock heavy-duty cutter, e.g. BN-AL auto-lock (<https://olfa.com/products/18mm-bn-al-auto-lock-heavy-duty-utility-knife>; the ratchet-lock L-1 is listed at 6 in overall, <https://olfa.com/products/18mm_l-1_ratchet_lock_utility_knife>) | handle 152 x 25 x 17 mm, 45 g with blade | about $6-10 |
| Blade | OLFA LB 18 mm silver snap-off blade (8 segments) | 100 x 18 x 0.5 mm, score lines every 12.5 mm | about $5 for 10 |
| Disposal can | OLFA DC-3 pocket blade disposal can (<https://www.zoro.com/k/olfa-dc-3/>: 2 1/2 in long, 1 1/4 in wide, 2 1/2 in high; about 1.1 oz) | 63 x 32 x 63 mm, 32 g, 26 x 3 mm slot | about $4-5 |
| Mat | any A4 self-healing cutting mat | 300 x 220 x 3 mm | about $10 |

### Distinct from

| Nearest task | What it tests | Why this one differs |
|---|---|---|
| TK of PR #921, snap a circuit board out of its panel over the break edge (accepted) | bending a panel at a V-score over an edge | no exposure to set, no captive scrap, no tool to make safe afterwards |
| TK of PR #2235, snap a swab's handle off at its breakpoint in a transport tube (accepted) | snapping a scored stick at its rim, one break point | here the break point is chosen by a detented slider first: snap as found and two segments go; then the knife must be made safe |
| TK of PR #672, retract a utility knife's blade before putting it away (accepted) | one slider move | here the slider is set twice to two different targets around a snap, with a held knife |

None of LIBERO, RoboCasa, SimplerEnv, BEHAVIOR, DataCenterBench or RoboDojo has a blade-snapping task.

## Run and score

The scripted demonstration is closed loop (it reads the knife's, the segments' and the can's poses and the slider's position, never
the scorer), so it runs through `demokit.py`, from this folder, with the simulator's environment loaded
(`source ~/IsaacLab-Arena/yam.env`):

    python demokit.py task.py ~/yam-eval/snap-off-cutter-dull-segment/demo --mode full        # writes full/demo.mp4, setup.png, result.json
    python demokit.py task.py ~/yam-eval/snap-off-cutter-dull-segment/redteam --no-record --mode none,careless,onearm,loose,extended

A model trial uses the standard command:

    inspect-robots run --embodiment yam_arena --policy dream \
      -E task=tasks/snap-off-cutter-dull-segment/task.py -E object_jitter=0 -E control_hz=20 \
      -E cam_width=320 -E cam_height=176 \
      --instruction "Snap the dull end segment off the cutter's blade, put it in the blade-disposal case, and put the knife down with its blade retracted." \
      --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/snap-off-cutter-dull-segment/dream

Runtime: the demonstration takes about 50 s of simulated time (about 1,000 steps); a trial at the 2,400-step limit takes about
2 minutes of simulated time, a few minutes of wall time on one GPU. There is no randomness: the layout is fixed
(`object_jitter=0`) and no seed changes anything.

## How the simulator models it

All state changes come from contact and joints; nothing is triggered by proximity.

- **Knife** (`knife`, a free articulation): the ABS handle (a box collider) and the steel nose (a hull whose face follows the
  score-line angle), 38 g with its real inertia. The blade carrier (`blade`, 5 g) slides along the knife on a prismatic joint and
  carries segments 3-8 and the thumb slider (14 x 5 x 10 mm, standing 5 mm proud of the spine). The slider's notches are a
  1500 N/m spring toward the nearest notch (every 2.5 mm), so it takes about 2-3 N to move it a notch and it stays where it is
  left: the auto-lock's behaviour, without its locking pawl.
- **The two end segments** are separate 2 g rigid bodies, joined to the carrier by PhysX joints authored when the scene is built:
  segment 2 is welded at score line 2, segment 1 hangs from segment 2 at score line 1 on a revolute joint across the blade, sprung
  at 0.3 N m/rad (the blade's bending stiffness at the score, estimated). They slide inside the handle with the carrier and never
  collide with the knife itself. Every control step the tracker reads the bend at line 1 from the two segments' poses. At 10
  degrees (about 0.05 N m, roughly 5 N on the tip 10 mm out) the blade breaks: the joint at the exposed score line nearest the
  nose is switched off and gravity is applied to the freed piece. With only line 1 past the nose, segment 1 comes off; with line 2
  past the nose too, the break comes at line 2 and segments 1 and 2 come off together (the bending moment grows toward the nose,
  and the steel channel supports the blade only up to its end). A score line still inside the nose cannot break.
- **Disposal can** (`case`, 32 g, free): floor, four 2 mm walls, a top with a 26 x 3 mm slot across its middle and two 10 mm
  guide plates under the slot, all box colliders. The door is drawn slid open; it is not a joint. A segment held in the slot is
  clamped by the slot's lips and the guide plates when the handle is tilted; once free it drops to the floor of the can.
- The mat is a fixed 3 mm slab. The arms' pads, the knife and the can use the simulator's rubber-like friction (0.8-1.0).

## Scoring

The tracker runs every control step; `score()` returns its value and `success()` its 1 s hold. A recorded harm (two segments, a
fall) is cleared only in the reset hook.

- **0.0** if no segment came off; if two segments came off (a break at score line 2); if the knife ever fell (its downward speed
  passed 0.8 m/s, about a 3 cm drop); or if the knife's centre is off the mat.
- **1.0**: exactly one segment off, lying wholly inside the can (every corner below the can's top face and within its walls); the
  blade retracted (the new tip, old score line 1, at least 1 mm inside the nose); the knife resting on the mat (its lowest point
  within 3 mm of the mat, moving under 2 cm/s); the can upright within 10 degrees and standing on the mat (within 3 mm); every
  fingertip point at least 5 cm from the knife and from the can; all held for 1 s.
- **0.5**: exactly one segment off, the blade retracted and the knife resting on the mat with every fingertip at least 5 cm from
  it, but the can or segment conditions of 1.0 not met (the segment snapped somewhere else and left loose, or the can knocked over).
- **0.25**: exactly one segment off, in any other state (the blade left out, the knife still held).

Possible scores: 0, 0.25, 0.5, 1.0. Doing nothing scores 0. Snapping the blade as found (two lines out) scores 0. Snapping it
against the mat (OLFA's last-resort method) can reach 0.5 at most.

## Real trial: preparation, reset and measurement

- **Preparation:** tape the mat's outline on the table and the knife's and can's outlines on the mat. Use a new LB blade with its
  cutting edge covered with a strip of polyimide tape so that the grippers' pads cannot be cut (the score lines are on the flat,
  so taping the edge does not change the snap). Extend the blade until its tip line is 30 mm past the nose (two score lines out),
  checked with a steel rule against the nose, and lay the knife on its marks. Slide the can's door open.
- **Measurement (end state, before anyone touches the scene):** a photo from above and one from the side, then a steel rule:
  the blade's length past its last remaining score line tells how many segments went (12.5 mm per segment, plus or minus 2 mm);
  the snapped segment is either rattling inside the can (shake it after the photos) or visible on the mat; retracted means no
  steel visible past the nose in the side photo; the knife's underside on the mat and the can upright are visible in the photos;
  the 5 cm hands-clear and 1 s hold are read from the video. A fall is read from the video (the knife dropping from 3 cm or more).
- **Reset (about 2 minutes):** empty the can, set the blade back to 30 mm out with the steel rule (one segment shorter each trial:
  a blade lasts seven trials, then fit a new one), put knife and can on their marks.
- **Safety:** the only hazard is the blade; its edge is taped, and the snap happens inside the can with the door open toward
  the arm, so the segment cannot fly. Forces are a few newtons.

## Assets and licences

The knife, blade, can and mat are authored meshes (SimReady has no snap-off cutter, segmented blade or blade-disposal can, and they
are the mechanism under test). The three distractors are NVIDIA SimReady props, downloaded at first run from the pinned revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd` of `nvidia/simready-assets` on Hugging Face, © NVIDIA, CC-BY 4.0:

- [dishware_mug_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01): mass set to 0.30 kg.
- [tape_clear_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/tape_clear_a01): mass set to 0.06 kg.
- [stationery_marker_thin_blue_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/stationery_marker_thin_blue_a01): mass set to 0.01 kg.

Nothing large is committed. `simready.py` is the loader from `tasks/cutters-off-pegboard/` (unchanged), `mechkit.py` the mechanism
kit from `tasks/heat-shrink-lug/` (unchanged apart from its credit line) and `demokit.py` the closed-loop demo runner from
`tasks/heat-shrink-lug/` (unchanged): other teams' open-source files in this repository.

## Validation

VALIDATION_PLACEHOLDER
