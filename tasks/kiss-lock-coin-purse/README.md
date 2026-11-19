# Put two coins back in an open kiss-lock coin purse and squeeze its ball clasp shut

A small coin purse on an 8.5 cm round sew-in kiss-lock frame stands open on the table, with two 2-euro coins beside it. The coins
go back in through the mouth, then the frame is closed: the two brass balls on top of the frame halves have to ride past each other
and click. The clasp is a real detent. Over the last 9.7 degrees before shut it pushes the front half back open (0.25 N m, about 5 N
at the ball). Inside the last 2 degrees it pulls the front half shut and holds it. Let go anywhere in between and the front half is
pushed back out.

The purse weighs 35 g and stands free. Pushed on the front half alone, it gives way and the clasp does not snap. Closing it takes force on
both halves at once: the back half held while the front half is swung over, as a thumb and finger squeeze a kiss lock shut. The
instruction gives only the goal: "Put the coins back in the coin purse and close it."

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: the scene, the clasp's detent, the scorer (`score`, `success`) and the scripted `demo()` |
| `mechkit.py` | The mechanism kit (articulations written as USD text, hooks, fingertip points), unchanged |
| `simready.py` | Loads the SimReady distractors, unchanged |
| `task.json` | The task's details for the board |

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena (`source <ARENA_DIR>/yam.env`).
- Network access the first time the scene is built: `simready.py` downloads the SimReady packages into `~/.cache/simready`.

## Run a trial

From the repository root:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/kiss-lock-coin-purse/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the coins back in the coin purse and close it." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/kiss-lock-coin-purse-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 900` (and `-E cam_width=640 -E cam_height=360`
for the higher-resolution video). The run prints `task_score` at the end.

## Runtime and seeds

- Demo: the clasp snaps at about 23.5 s of simulated time, full credit is reached 2 s after the hands are clear, and the run stops
  3 s later (628 steps, 31.4 s simulated). About 2 to 5 minutes of wall time on a shared L40S, including start-up.
- Dream-1: up to 2,400 steps (2 minutes simulated).
- Dream-1 scored 0.0 in its 2,400-step run: it brought both grippers down towards the coins and hovered beside them, but never picked a coin up or touched the purse.
- `object_jitter=0` keeps the layout fixed and nothing is randomised, so every seed gives the same scene. The demo is open loop and
  deterministic.

## The demo

`demo()` is a fixed plan from the published starting layout. It reads nothing from the simulator.

1. The left hand pinches the left coin across its rim, lifts it over the mouth and lets it drop in. Then it parks.
2. The right hand does the same with the right coin.
3. The right hand comes down over the frame with its fingers open, one finger outside each frame half, 2.4 cm right of the purse's
   middle (clear of the balls), with the fingertips 2.2 cm above the rivets. It closes. The front finger swings the open half over,
   the back finger holds the back half, and the balls snap past each other between them.
4. It opens, lifts and parks. The frame stays shut.

## Scoring

`score()` and `success()` read a tracker that runs every control step (`Trial.step`, on mechkit's `on_step` hook):

- **Coin inside:** its centre inside the purse's body (below the rivet line and within its tapered walls, in the purse's own frame),
  with no fingertip point within 3 cm of it.
- **Snapped:** the front half within 2 degrees of fully shut, where the detent holds it. On a real purse: the balls have passed each
  other and the frame stays shut when let go.
- **Hands clear:** every fingertip point of both grippers (`mechkit.fingertips`: each finger's tip and 3 cm up it, at the middle and
  both edges) at least 3 cm from the box enclosing the purse with its frame open.
- **1.0** once both coins are inside, the clasp is snapped and the hands are clear, all held for 2 s (40 steps); kept from then on.
- Otherwise, at the end: **0.75** if both coins are inside and the clasp is snapped (a hand still on or near it), else **0.25** per
  coin inside, whether or not the clasp is shut.
- **0 for the whole trial** if a coin or the purse ever falls off the table (more than 2 cm below its top). This record is latched
  and cleared only by the environment's reset (mechkit's `on_reset` hook), never by the scene looking right again.

Red-team runs of the same file with a different `demo()` (made as temporary copies, then deleted): doing nothing scored 0.0; closing
the purse with one coin left on the table scored 0.25; with both coins in, pushing the front half over with one closed hand from
the front only, 3 mm past its shut position, with nothing holding the back half, did not snap the clasp and scored 0.5.

## Real trials

Any small kiss-lock coin purse on an 8.5 cm round sew-in frame (the size used by the patterns below), two 2-euro coins, a pair of
glasses and a mug. Mark the purse's and the coins' places on the table with tape. Score from an overhead video: whether the coins are
inside, whether the balls have passed each other, and whether both grippers are 3 cm clear. Reset: open the clasp, swing the front
half out to 25 degrees (a card cut to 25 degrees stood against it), put the purse and coins on their marks. Nothing is consumed; the
task is safe (light, soft objects).

## Importance and sources

Kiss-lock (clasp) frames are the standard closure for coin purses, glasses pouches and evening bags, and closing one is a common
everyday manipulation: the two halves must be pressed together until the balls ride past each other, with the back half held while
the front one is swung over. A light purse pushed on one half alone gives way rather than closing.

- 8.5 cm round sew-in kiss-lock frame for a coin purse, finished purse about 3.5 x 2.5 x 3.5 in (8.9 x 6.4 x 8.9 cm):
  [Craft Passion, Crochet Kisslock Coin Purse](https://www.craftpassion.com/crochet-kisslock-coin-purse/).
- 8.5 cm (3 3/8 in) half-round sew-in purse frame with a kiss lock, "the snap-shut metal clasp":
  [So Sew Easy, Free Kiss Lock Coin Purse Pattern](https://so-sew-easy.com/how-to-sew-coin-purse-with-a-sew-in-purse-frame/).
- 2-euro coin: 25.75 mm, 2.20 mm thick, 8.50 g, white outer ring and yellow centre:
  [European Commission, Common sides of euro coins](https://economy-finance.ec.europa.eu/euro/euro-coins-and-notes/euro-coins/common-sides-euro-coins_en).

The purse's body here (9.1 cm wide, 8.5 cm tall to the arch's top, 5.6 cm deep at the bottom) matches those patterns' finished size.
The clasp torque (about 5 N at the ball) is an estimate, not a measurement.

## Assets

The distractors come from the Hugging Face dataset
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`accessory_eyeglasses_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/accessory_eyeglasses_a01)
- [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time: each prop's parts
are welded into one rigid body, and masses are set to 30 g (glasses) and 300 g (mug). The files are not committed: they download on
first use.

SimReady has no coin purse and no coin, so the purse (body, frame halves, balls, rivets) and the coins are custom shapes built with
`mechkit.py` meshes at real size.

`mechkit.py` is taken unchanged from another team's open-source task in this repository, `tasks/heat-shrink-lug/`, which is published
under the MIT licence; `simready.py` is adapted from `tasks/cutters-off-pegboard/` (loader unchanged).
