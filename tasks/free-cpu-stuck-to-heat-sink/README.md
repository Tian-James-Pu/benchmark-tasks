# Free a server processor stuck to its heat sink by the old paste

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena
0.3.1) on the station scene: [task.py](task.py).

When a server's heat sink is lifted off, old thermal paste often holds the processor to it, and the processor comes out of its socket
stuck to the heat sink's base. The task starts there: a 1U heat sink for an Intel Xeon Scalable (LGA3647) processor lies upside down on
the bench, fins down, with the processor (package 76.0 x 56.5 mm) held on its base by the paste, its gold land side facing up. The robot
has to get the processor off and into the empty pocket of a processor tray, still land side up. The service manual's way is to twist or
slide the processor to break the paste's seal, never to pry or pull it off, and to handle it by its edges without touching the
contacts. The instruction says none of that.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/free-cpu-stuck-to-heat-sink/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The processor came out of the socket stuck to the heat sink. Get it off the heat sink and into the empty tray pocket, contact side up." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/free-cpu-stuck-to-heat-sink-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 600 -E cam_width=640 -E cam_height=360`.

The run prints `task_score` at the end. The scorer counts its own 2 s hold, so `success()` is true and the score 1.0 once the processor
has rested in the pocket for 2 s with the hands away; the scripted demo ends there, at about step 430.

## Runtime and seeds

- Demo: about 430 steps (22 s simulated), about 1.5 minutes on an L40S including start-up (several minutes the first time a GPU
  compiles its shaders).
- Dream-1: 2,400 steps (2 minutes simulated), about 5 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

## Layout

Centimetres from the midpoint between the arm bases (x forward, y left). The heat sink (10.8 x 7.8 x 2.55, 455 g, aluminium) lies fins
down at x 30, y -2, its long side along y. The processor (7.6 x 5.65 x 0.55, 50 g) lies centred on the heat sink's base, long side along
y, heat spreader down on the paste and gold lands up. The processor tray (black conductive plastic, fixed) is centred at x 21, y -17,
with its 8.1 x 6.1 pocket's long side along y; the pocket is bounded by 4 mm walls, with a 4.8 cm finger cut-out in the middle of each
long side. Distractors: a Torx T30 screwdriver at x 10, y 17 and a mug at x 46, y 18.

## The model

Declared in `task.py`'s docstring and code:

- **The paste bond.** While the seal holds, an internal pair of forces presses the processor onto the heat sink's base (5 N) and pulls the
  heat sink up by the same amount, so the heat sink's weight on the bench is unchanged, and a processor lifted while bonded brings the
  heat sink with it. The processor's faces have paste friction (0.3), so it can turn or slide on the base while it stays bonded. The bond
  is removed (the seal is broken) once the processor has turned at least 3 degrees each way relative to the heat sink, or has slid 5 mm
  across it: Dell's "twist or slide". The heat sink is loose on its fins; its weight and fin friction keep it from turning when the
  processor is twisted with one hand, as in the demo.
- **Pried or pulled off:** any corner of the processor's underside more than 1.5 mm off the base while the seal holds. That ends the
  bond and scores 0 for the trial.
- **Contacts touched:** any fingertip point over the land side (more than 8 mm in from the package's edges) at the height of a finger
  resting on it. A pad gripping the package's edge reads up to about 5 mm inside the edge, so gripping by the edges is allowed. Scores 0.
- **Dropped:** the processor falling faster than 0.5 m/s or tilting past 30 degrees at any time. Scores 0.
- These three are latched each control step and cleared only in the reset hook (`Trial.reset`, registered with mechkit's `on_reset`),
  never because the scene looks right again.

## Scoring

| Outcome | Score |
|---|---|
| Pried or pulled off, contacts touched, or dropped, at any time before full credit | 0 |
| Seal not broken | 0 |
| Seal broken by turning at least 3 degrees each way, or sliding 5 mm | 0.25 |
| ... and the processor then lifted clear (underside 15 mm above the base, or its centre 7 cm from the heat sink's) | 0.5 |
| ... and it rests in the pocket: centre within 4 mm of the pocket's along each axis, square within 6 degrees (either end first), flat within 2 degrees, every underside corner within 1.5 mm of the pocket floor, still (under 1 cm/s), no fingertip within 3 cm of it, for 2 s | 1.0 |

A processor set in contact side down is tilted 180 degrees and is not seated.

## Validation runs

- Scripted demo (one arm: grip the long edges, turn 8 degrees each way and back, lift, set in the pocket, let go): 1.0, twice.
- Doing nothing: 0 (seal never broken).
- Careless: grip the processor and pull it straight up: 0. The bond lifts the heat sink with it, and the edge lifts off the base when
  the heat sink is set down on the tray (pried).
- Dream-1 (2,400 steps): 0. The left arm reached over to the processor on the heat sink and stayed there without turning it, so the
  seal never broke; the right arm hovered by the tray.

## Real-world trial

Hardware: a 1U passive heat sink for LGA3647 (for example Supermicro SNK-P0067PS), a dead or reject Xeon Scalable processor (LGA3647),
a tube of server thermal compound (use the same compound every trial), an anti-static processor tray (or a plastic tray with a pocket
81 x 61 mm and 4 mm walls), double-sided tape, a fine marker, a printed protractor, a ruler and a phone camera on a stand.

Preparation and reset: clean both faces with isopropyl alcohol, put a 4 mm dot of compound (about 0.1 ml) on the processor's heat
spreader, centre the heat sink on it, press down with about 100 N (a 10 kg weight) for 30 s, and wait 10 minutes at room temperature.
Turn it over onto the bench fins down at the layout position (mark the heat sink's outline on the bench with tape). Tape the tray down
at its position. Check the bond: lift the processor 1 cm straight up by its long edges; the heat sink must come up with it (the bond
holds at least its 455 g, as in the simulation), then set it back. If it separates, re-paste. Draw a short marker line across the
processor's edge and the heat sink's base at one long side. Wipe and re-paste after each trial.

Measurement: in the top view, the marker line's two halves show the relative turn (read against the protractor: 3 degrees each way) or
slide (against the ruler: 5 mm); a side photo against a ruler shows a lifted edge (1.5 mm) and the processor flat in the pocket; the
video shows fingertips on the gold land side, the release and the 2 s hold.

## Importance and sources

Processors are pulled from servers whenever one is replaced, upgraded, moved to another board or sent back under warranty, and every
time a heat sink is re-pasted. Old paste bonding the processor to the heat sink is common enough that server makers write it into
their service procedures: break the seal by twisting or sliding, never pry, and keep fingers off the contacts.

- Dell PowerEdge 1955 Hardware Owner's Manual, removing the heat sink and processor: "When removing the heat sink, the possibility
  exists that the processor might adhere to the heat sink and be removed from the socket" ... "If the processor is removed from the
  socket with the heat sink, twist or slide the processor off of the heat sink." ... "Do not pry the processor off of the heat sink."
  https://www.manualslib.com/manual/356873/Dell-1955.html?page=85
- Lenovo ThinkSystem SR950, "Remove a microprocessor": the seal is broken "using a twisting motion to break the processor-to-heat-sink
  seal" (there with a screwdriver on the processor's carrier, which this task's bare processor does not have); the processor is then
  held with "the processor-contact side up"; "Do not touch the processor socket or processor contacts."
  https://pubs.lenovo.com/sr950/remove_a_microprocessor
- LGA 3647 processor dimensions, 76.0 x 56.5 mm: https://en.wikipedia.org/wiki/LGA_3647
- Supermicro SNK-P0067PS, a 1U passive heat sink for LGA3647 on Xeon Scalable servers:
  https://store.supermicro.com/1u-passive-cpu-cooler-x11-snk-p0067ps.html (the page gives no dimensions; the heat sink here is
  108 x 78 mm and 25.5 mm tall, the 1U height, 455 g, approximate).

Robots that service data-centre hardware will meet this fault whenever they remove a processor; it needs the twist instead of a pull,
a grip on the package's edges only, and a placement that keeps the land side up.

## Files

| File | What it is |
|---|---|
| `task.py` | The scene, the bond and scorer (`Trial`), `score()`, `success()` and the scripted `demo()` |
| `mechkit.py` | Props and mechanisms written as plain-text USD, with step and reset hooks; the standard copy of the kit first published in `tasks/heat-shrink-lug/` in this repository (another team's open-source kit) |
| `simready.py` | Loads SimReady props into a task file (`SimReady(...)`), downloading them once from Hugging Face; the loader shared by tasks in this repository |

## Assets

The heat sink, processor and tray are built from primitives in `task.py` (SimReady has no processor, heat sink or processor tray).
The two distractors are NVIDIA SimReady props from
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, © NVIDIA, CC-BY 4.0:

- [`handTool_screwdriver_torx_t30_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_screwdriver_torx_t30_a01)
  (the Torx T30 driver used on processor heat sink nuts): mass set to 80 g.
- [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01):
  mass set to 300 g.

Nothing large is committed: the files download into `~/.cache/simready` at the first run.
