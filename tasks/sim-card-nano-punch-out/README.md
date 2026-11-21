# Push the nano-SIM out of a triple-cut SIM card, leaving the micro and mini frames in the card

A new triple-cut SIM carrier card lies chip side up on a stack of four notebooks, its cut-out end 3.5 cm out over the
stack's edge. The robot is asked: **"Push the nano-SIM out of the card, leaving the micro and mini frames in the card."**

The three SIM sizes nest in one cut-out: the 12.3 x 8.8 mm nano-SIM (4FF) hangs by breakaway tabs inside a 1.5 mm wide
micro-SIM (3FF) ring, which hangs inside the 25 x 15 mm mini-SIM (2FF) outline, which hangs in the card. A push on the nano alone
loads all three sets of tabs in series, and the nano's (the weakest) give first. A push that lands on the 1.5 mm ring, or across a
cut line, loads only the outer tabs, and the ring (with the nano still in it) or the whole mini piece tears out instead. The card
weighs 5 g and the push is a few newtons 1.6 cm beyond the stack's edge, so a card that nobody holds tips off the stack. One hand
holds the card down on the stack; the other presses inside the nano's outline, and stops once it gives.

Why the card starts out over the edge of a stack: a person holds the card in one hand, or over a table's edge, and pushes with the
other thumb, because the nano needs somewhere to go. A card lying flat on a desk can't be punched (the desk is under the nano), and
a 0.76 mm card is hard for a parallel gripper to pick off a flat desk. The notebook stack is the ordinary desk-top version of
"over the edge": the cut-out hangs free, the rest of the card has a surface for the holding hand to pin it against, as a palm
would, and the nano drops 4 cm onto the desk.

| file | what it is |
|---|---|
| `task.py` | scene, tab model, scorer, and the closed-loop demo (`demo_script`) |
| `task.json` | the submission's fields |
| `mechkit.py` | the shared mechanism kit, unchanged (see "Assets and helpers") |
| `demokit.py` | the closed-loop demo runner, unchanged |
| `simready.py` | the SimReady prop loader, unchanged |

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena (`source <ARENA_DIR>/yam.env`).
- Network access the first time the scene is built: `simready.py` downloads the SimReady packages into `~/.cache/simready`.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Push the nano-SIM out of the card, leaving the micro and mini frames in the card." \
  --scorer task_score --max-steps 1200
```

Nothing is randomised, so every seed gives the same scene. Runtime: the scripted demo reaches full credit 13.2-13.4 s into the
trial (264-268 control steps). A 1,200-step trial with cameras takes a few minutes of wall time on a shared GPU.

## The demo

```bash
python demokit.py task.py OUT                                                # the full-score run, with video (GPU)
python demokit.py task.py OUT_checks --fast --mode none,onearm,careless,hold   # the failure checks (CPU, no cameras)
```

The demo is a scripted reference, not a policy. It reads the card's pose and the parts' positions in the card (which the cameras
show: a part that gives sinks into the card), the loose nano's position once it is out, and its own arms' joints. It never reads
the scorer.

1. The left hand comes down on the card's end that lies on the notebooks, fingers closed, 2.3 cm in from the stack's edge, and holds
   the card down.
2. The right hand opens, tilts 45 degrees along the nano's length and brings one fingertip's lower corner over the middle of the
   nano. (A YAM fingertip's end is about 14 x 3.5 mm flat with rounded edges, wider than the 12.3 mm nano is long, so pressing square
   would bridge the cut lines; the tilted corner is the robot's equivalent of a thumbnail.)
3. It lowers its command 0.4 mm a step until a part of the card has sunk 0.6 mm (more than any tab bends before breaking), then
   lifts at once. The nano's tabs give at 5 N; it drops 4 cm onto the table.
4. The left hand lets go and both arms move clear. Full credit comes 1 s later.

| run | what the script does | score | what happened |
|---|---|---|---|
| `full` (GPU, recorded) | as above | see "Results" | |
| `full` (CPU, twice) | as above | 1.0, 1.0 | nano tabs gave at 5.1-5.4 N; ring and mini peaked at 5.1-5.6 N (series) and held |
| `none` | does nothing | 0.0 | nothing torn |
| `onearm` | the right hand alone, the card not held | 0.0 | the card tipped off the stack's edge; nothing torn |
| `careless` | the same press landing 5.2 mm across from the nano's centre: on the micro ring, by the nano's cut line | 0.25 | the micro ring's tabs gave at 7.1 N with the nano still in it (nano tabs 0.5 N) |
| `hold` | as `full`, but the left hand never lets go | 0.75 | nano out, frames whole, but a hand stays on the card |
| straight down (a one-off red-team copy of the script, not shipped) | the naive press: card held as in `full`, the right gripper pointing straight down with its fingers closed, on the nano's centre (turned 0 and 90 degrees) | 0.0, 0.0 | the right hand never reached the card: it stopped about 5 cm above it, against the holding hand 4 cm away (at 90 degrees it knocked the card off the stack onto the table); nothing torn |

The CPU checks above were run in one simulator session (`--mode careless,full,hold,onearm,none,full`), so they also check that a
reset restores torn tabs.

## Results

Final GPU runs with cameras (`python demokit.py task.py OUT`), current `task.py`:

- run 1: _pending_
- run 2: _pending_

## How the simulator models it

- **The card** is a free articulation: a 4.6 g PVC frame (85.6 x 54 x 0.76 mm with a 25 x 15 mm hole), and three nested links on
  sliders along the card's normal: the mini piece in the frame, the micro ring in the mini, the nano in the micro. Each is built
  from flat boxes (a ring is four), 0.1 mm inside its hole all round (the cut's kerf). Printed grey lines mark the three cut lines on
  the chip side; the nano carries its gold contact pad. Masses are PVC at 1.38 g/cm^3 (mini 0.25 g, micro 0.10 g, nano 0.12 g).
- **Tabs.** A part's tabs are one stiff spring on its slider (20 kN/m: 0.25 mm at 5 N). Because the parts nest in series, the
  slider's deflection times the stiffness is the load through that part's tabs. Past its breaking load (nano 5 N, micro 7 N,
  mini 9 N) the spring is removed and stays removed until the next reset. The breaking loads are estimates in the 5-8 N range of
  such tabs, the smallest part's tabs the weakest; they are not measured.
- **What tears depends only on where the fingertip pushes.** A push on the nano alone loads nano, micro and mini tabs equally, so
  the nano's give first. A push on the ring loads the micro and mini tabs, and not the nano's; a flat push across the whole
  cut-out loads the mini's most. Contact is PhysX contact between the fingertip and the parts' own colliders: nothing is triggered
  by proximity.
- **After a tear.** A loose frame can drop up to 3 mm out of either face, carrying whatever is still attached to it. A loose nano
  drags on its cut edges (1,000 N s/m of damping) until it is 0.96 mm out (clear of the card's face) or for 0.5 s, then carries on
  as a free body with the nano's pose and velocity (0.3 g, so that PhysX handles so small a body steadily; the real one is 0.1 g)
  and falls onto the table. The drag stands for the moment a person's thumb takes to stop once the nano clicks out; without it the
  rigid, softly driven fingertip would lunge 5 mm into the empty hole and onto the ring within a single control step.
- **Solver details.** The three tab sliders carry 3 g of joint-space inertia (armature) while their tabs hold (a chain of light
  parts on stiff drives is otherwise unsteady in PhysX), and three weightless locked stubs with no geometry hang on the card's frame
  so that PhysX roots the articulation at the frame. Neither changes what touches what.

## Scoring

Read at the end of the trial from the state of the card, the nano and the hands:

- A part is **torn** if any of its tabs is broken (sim: its tabs gave way). Torn stays torn.
- The **nano is out** once its tabs gave way and it dropped clear of the card, unless it then fell off the table (its centre more
  than 1 cm below the table top).
- **1.0**: the nano is out and lying on the table or the notebooks (its centre at most 4.3 cm above the table, moving under
  1 cm/s), neither the micro ring nor the mini frame is torn, the card lies flat (tilt at most 10 degrees) and still (under 1 cm/s
  and 3 degrees/s), and every fingertip point of both grippers is at least 3 cm from the card and from the nano, all for 1 s.
  It stays 1.0 while the nano is out and both frames are untorn.
- **0.75**: the nano is out and both frames are untorn, without the 1.0 conditions (a hand still on the card, say).
- **0.5**: the nano is out, but the micro ring or the mini frame is torn.
- **0.25**: the nano is not out, and the micro ring (with the nano still in it) or the whole mini piece is torn out.
- **0**: the nano is not out and neither frame is torn (nothing torn, or a nano that fell off the table). Also 0, whatever else happened, if the card was ever more than 2 cm below the table top (knocked off
  the table).

Possible scores: 0, 0.25, 0.5, 0.75, 1.0. The fingertip points are mechkit's `fingertips()` (tip and 3 cm up each finger, at its
middle and both edges). There are no force or impulse rules in the score: the tab loads only decide what tears, as the real tabs do.

## Real trial

**Product.** A triple-cut (2FF/3FF/4FF) SIM carrier card, ISO/IEC 7810 ID-1, 85.6 x 54 x 0.76 mm PVC, about 5 g. Any
carrier's triple-cut starter SIM works; one that can be bought in quantity is the
[RevX Systems KM1 "Triple 2FF/3FF/4FF" SIM card on DigiKey](https://www.digikey.com/en/products/detail/revx-systems-corp/KM1/9556339)
(DigiKey 1998-1007-ND, US$8.45 each, US$7.33 each for 10). The card is consumed: one per trial.

**Preparation (2 minutes).** Stack four closed A5-size notebooks (22.9 x 17 x 1 cm) 4 cm high, centred 30 cm in front of the arm
bases and 8.5 cm to the left of the midline, their right edge on a tape line at the midline. Lay a fresh card chip side up on the
stack, long side left-right, with its cut-out end 3.5 cm out past the stack's edge, on a tape outline. Check before each trial
that no tab of the card is cracked (hold it to the light). Put the mug, glasses and pen on their tape marks. Fit the grippers'
usual rubber pads.

**Reset.** Take the card and any loose pieces away; lay a fresh card on its outline.

**Measurement.** After the trial, photograph the card's chip side and the table. Torn: count each part's tabs on the photo; a
part is torn if any of its tabs is broken. The nano is out if it lies separate from the card on the table or the notebooks.
Full credit also needs, in the video, the card lying flat on the stack or the table with no gripper within 3 cm of it or of the
nano for 1 s. Nothing in the setup is sharp, hot or heavy: the card is 5 g and the pushes are a few newtons.

## Distinct from

| task | what it is | how this differs |
|---|---|---|
| TK-GJ7LSU (pending) | Pop one tablet through a blister pack into a cup | Both push a small item through a sheet. A blister tablet pushes through foil, and a press anywhere on its dome works; here three parts nest with tabs in series, and the push must land inside the 12.3 x 8.8 mm nano and not on the 1.5 mm ring round it, or the frames tear out instead. Nothing has to be spared there; here two frames must survive. |
| TK-MLJF7T (accepted) | Pop a phone's SIM tray out with the eject pin | Same object family, different operation: a pin into a hole to release a tray. Here the SIM is freed from its carrier card before it ever goes in a tray. |
| TK-PXSM9V (accepted) | Snap a swab handle at its breakpoint | A part broken off by bending at one score. Here breakaway tabs in three nested rings, separated by where the push lands. |
| TK-RDGBWC (closed) | Remove only the centre knockout from an electrical enclosure | The closest skill: freeing the innermost of nested breakaway parts. It was closed on importance (the electricians who do it regularly were not counted), not as a duplicate; this task is a 5 g card that most phone owners have handled, pressed by hand. |
| TK-5UJ7WK (closed) | Open a concentric knockout to a connector's size | Also nested breakaway parts, closed because square push-out plates stood in for real knockouts and dropped the prying, twisting and fastening. A SIM card's parts really are flat plates on breakaway tabs that are pushed out, so here the plates are the real geometry, at real size. |
| TK-TTV8LS (closed) | Press today's tablet out of a calendar blister pack | Closed because choosing the day was the hard part and the press was ordinary. Here nothing is chosen: the difficulty is physical, a press placed inside a 12.3 x 8.8 mm part with a 1.5 mm ring round it, on a 5 g card the other hand must keep from tipping, and a misplaced press does lasting damage (torn frames). |

No title on the task board punches a SIM out of a card, and we know of no such task in LIBERO, RoboCasa, SimplerEnv, BEHAVIOR or
RoboDojo.

## Importance and sources

- **Relatability.** Physical SIMs still ship on triple-cut cards: US Mobile's guide says "Our physical SIM ships as a triple-cut
  card", that "The mini, micro, and nano sizes are pre-scored into one card, so you pop out the size you need by hand", and
  "Don't toss the white plastic frames", since "you can reassemble a nano back up to a mini in seconds"
  ([US Mobile: How to cut a SIM card](https://www.usmobile.com/blog/cut-sim-card/)). That is why the frames must survive. Anyone who has
  started a new phone line or replaced a SIM has done it, and it is plain in a 10 s clip.
- **Sizes.** 4FF nano 12.3 x 8.8 mm ([ETSI press release, 1 June 2012](https://etsi.org/newsroom/news/398-news-release-1-june-2012));
  2FF 25 x 15 x 0.76 mm, 3FF 15 x 12 x 0.76 mm, ID-1 card 85.60 x 53.98 x 0.76 mm
  ([SIM card sizes, mbedded.ninja](https://blog.mbedded.ninja/pcb-design/component-packages/sim-card-sizes)). The nano is 0.67 mm
  thick on its own; on a triple-cut card it is cut from the 0.76 mm card, as modelled. The cut-out's place on the card (its outer
  edge 6.25 mm from the short end, 3 mm off the long centreline) is approximate: carrier cards put it near one short end, and it
  varies between issuers. It does not change the task.
- **Tab strengths** (5, 7, 9 N) are estimates, not sourced; they only set which tabs give first, which follows from the parts
  nesting in series.

## Fidelity and limits

- The card and its parts are rigid; nothing bends. Each part's 3-4 tabs act as one spring that tears all at once; a real part can
  tear partly (and the photo then counts it as torn).
- The loose nano's 0.5 s of exit drag and its 0.3 g mass (real 0.1 g) are simulator choices, stated above.
- The notebooks are fixed in place. Friction is the kit's defaults (card 0.5 static, 0.4 dynamic).
- Fingertips are rigid; the real rig's rubber pads are softer and would make the press more forgiving.

## Assets and helpers

The card and its parts are authored parts (mechkit meshes and colliders). The notebooks and the three distractors are NVIDIA
SimReady props, downloaded at first run from [`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets) at
pinned revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, © NVIDIA, CC-BY 4.0:

- [`misc_notebook_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/misc_notebook_a01)
  and [`misc_notebook_a02`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/misc_notebook_a02):
  the stack the card lies on, two of each, fixed in place (kinematic), convex-hull collision at load.
- [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01):
  distractor, loose, mass set to 0.30 kg.
- [`accessory_eyeglasses_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/accessory_eyeglasses_a01):
  distractor, loose, mass set to 0.03 kg (any joints welded into one body by the loader).
- [`stationery_pen_plastic_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/stationery_pen_plastic_a01):
  distractor, loose, mass set to 0.012 kg (its cap and body welded into one body by the loader).

`score()` ignores the distractors. No meshes or textures are bundled. `simready.py` is adapted from the open-source
`cutters-off-pegboard` loader; `mechkit.py` (the shared mechanism kit) and `demokit.py` (the closed-loop demo runner) are copied
unchanged from `heat-shrink-lug`, another team's open-source task in this repository.
