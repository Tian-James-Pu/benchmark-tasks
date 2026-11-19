# Refit a pressure cooker's float valve cap: hold the loose valve in from the top and snap the silicone cap on underneath

The lid of a 6-quart electric pressure cooker has a small stainless float valve in a hole near its handle. It rises
under pressure to show the pot is sealed, and a small silicone cap on its underside end stops it leaving the lid and
seals it. Both come off for washing, and they have to go back before the next use: without the cap the cooker cannot
come to pressure. The valve drops into its hole from the top and sits there loose. The cap then has to be pushed
over the valve's end on the underside until its lip snaps into the groove behind the end knob. Pushed on with one
hand, the cap only shoves the loose valve back out of the top, so the maker's guide has the user keep a finger on the
valve's flat top while the cap goes on. The task needs both hands: one holds the valve in from the top face while the
other lines a 12 mm cap up with a 5 mm valve end and presses it on.

Here the washed lid stands on edge in a dish rack's lid slot, its top face toward the left arm. The valve is already
in its hole, loose. The cap lies open side down on the counter next to the rack.

## Files

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: the scene, the scorer (`score`, `success`) and the scripted `demo()` |
| `task.json` | The task's details for the board |
| `mechkit.py` | The mechanism kit (articulations written as plain-text USD, hooks, fingertip points), unchanged; taken from `tasks/heat-shrink-lug/` in this repository (another team's open-source kit) |
| `demokit.py` | Grip orientations for the scripted demo (`level`, `down`), unchanged; also from `tasks/heat-shrink-lug/` |
| `simready.py` | Loads the SimReady distractors, unchanged; from `tasks/cutters-off-pegboard/` |

## The parts (all at full size)

- **Lid** (fixed in the rack slot): 250 mm across, brushed steel cover over a steel inner lid, 12 mm thick round the
  valve, with a rim 25 mm deep, a black handle across the top face, a steam release knob, an anti-block shield and a
  white sealing ring on the underside. The valve hole is 6 cm from the lid's centre, 15 cm above the table.
- **Float valve** (loose in its hole, slides along its axis, 10 g stainless): flat top 10 mm across and 2.5 mm thick,
  resting on the top face; 5 mm shank in a 6 mm hole, ending 3 mm below the underside; a groove 3.4 mm across and
  1.2 mm long; an end knob 5.4 mm across and 1.8 mm long, chamfered to 3.2 mm at the tip.
- **Silicone cap** (loose, 2 g, white): 12 mm across and 6.5 mm long with a closed end; a 1 mm lip at its open end
  with a 3.6 mm hole (the mouth chamfered to 4.8 mm), and a 6 mm bore behind it that takes the knob.
- **Dish rack lid slot** (fixed): white plastic-coated wire bar and tines on a pale blue drip tray.
- **Distractors** (SimReady, loose, ignored by the scorer): a dish soap bottle, a mug and a kitchen sponge.

The snap is a real mechanism, not a rule: the knob is made of four sprung quarters (150 N/m each) that give inward
as the cap's lip rides up their chamfer and spring back out once the lip is past them. That is how the stretch of the
silicone lip is represented with rigid bodies. A cap pushed short of the groove sits on the chamfer, held only by the
quarters' grip; a cap past it is held by the knob's back face. The valve slides freely in its hole (no spring, 0.05 N
friction), so a cap pushed against it with nothing on the valve's top pushes it out of the lid.

## Scoring

- **Seated**: the cap's open face between 0.6 mm short of and 0.4 mm past the end of the valve's shank, so its lip is
  in the groove with the knob past it (all four knob quarters back out, under 0.3 mm in), and the cap's axis within
  10 degrees and 0.6 mm of the valve's.
- **Fell out**: the valve pushed more than 10 mm out of the top face, at any time before full credit. With 12 mm of
  shank in the hole, a real valve falls out there; the simulated one stops at 12 mm, so the record stands in for the
  drop. It is cleared only at a reset.
- **Hands clear**: every gripper fingertip at least 2 cm from the cap and from the valve's top.
- **1.0** once the cap has been seated, still (under 1 cm/s) and hands clear for 1 s running, with the valve never
  fallen out. It is then kept.
- **0** if the valve fell out, whatever happens after (in a real kitchen it is on the floor or in the sink).
- Otherwise, scored on the run's last step: **0.25** if the cap is left on the valve's end but not seated (open face
  from 3 mm short of the shank's end to 0.4 mm past it, axis within 30 degrees and 1.5 mm of the valve's), still and
  hands clear; **0** for anything else (the cap on the counter or dropped, or still held).

In a real trial the same checks are made with calipers and a photo along the valve's axis: whether the cap's lip is
in the groove (the knob visible past the lip inside the cap, the cap tight against the shank's end), and whether the
valve left its hole. The lid is clamped in the rack slot for a real trial, as it is fixed in the simulation.

## The scripted demo

A fixed plan from the published starting layout (it reads no simulator state). The left gripper, closed and level,
comes in toward the lid's top face and rests its fingertips on the valve's flat top, commanded 6 mm into it so the
arm's stiffness holds the valve in. The right gripper picks the cap from above with its pads 2.5 mm off the counter,
turns it so its open end faces the lid, lines it up with the valve's end 3 cm below the underside, pushes it on over
the knob (commanded 10 mm past the shank's end, so the arm's stiffness supplies the push) until it snaps home, lets go and backs away. Then the left hand lets go of the valve and both
arms move clear.

In the Dream-1 run (score 0.0 over the 2,400 steps) the model raised both grippers near the arm bases and drifted
toward the sponge and the mug; neither gripper touched the cap or the float valve.

## Run a trial

On an instance set up by the simulation walkthrough, from the repository root:

```bash
INSTR="Put the silicone cap back on the bottom of the pressure cooker lid's float valve."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/refit-instant-pot-float-valve/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/float-valve-dream

# Scripted demo (full score)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/refit-instant-pot-float-valve/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=640 -E cam_height=360 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 900 --store-frames --log-dir ~/yam-eval/float-valve-demo
```

The scene is not randomised: every seed gives the same layout. The scripted demo reaches full credit after about 19 s
of simulated time (373 steps) and stops 3 s later; a trial takes about a minute on one GPU once its shaders are compiled.

## Importance and sources

Refitting the float valve and its cap after washing the lid is routine care for one of the most common small kitchen
appliances, done by the owner by hand. The manufacturer's instructions describe exactly this two-handed step:

- Instant Pot quick start guide (Duo-family model), "Remove & Install Float Valve": "Install: Drop float valve into
  float valve hole on top of lid. Place one finger on the flat top of float valve. Turn lid over. Attach silicone cap
  to bottom of float valve." and "The float valve and silicone cap must be installed before use to seal in pressurized
  steam. Clean after each use." https://www.qvc.com/footers/cd/pdf/K49640_quick_start_guide.pdf
- Instant Pot Duo Plus manual (warning not to operate without the float valve and silicone cap properly installed):
  https://www.williams-sonoma.com/netstorage/pdf/116080-116081-Instant-Pot-Duo-Plus-8qt-6qt-and-3qt-Manual.pdf
- iFixit, Instant Pot IP-DUO80 V2 float valve replacement ("The red float valve will fall once the white cover is
  removed"): https://www.ifixit.com/Guide/Instant+Pot+IP-DUO80+V2+Float+Valve+Replacement/103070
- Replacement float valve sold for Duo-series lids (stainless steel): https://www.instantpot.be/en/spare-parts-en/instant-pot-float-valve-duo-3-6-and-8qt/

The dimensions of the valve and cap are estimated from product photos and the parts' known fit (no published drawing
was found); the cap's push-on force is likewise an estimate.

## Fidelity

All parts are at real size. The lid is fixed in the rack slot (a real lid in a rack can rock a little when pressed
from both sides), and the cooker base is not in the scene. Everything is rigid: the silicone lip's stretch over the
knob is represented by the knob's four sprung quarters (150 N/m each), which makes the cap go on with about 1 N in
the simulation, less than a real cap takes. Because of that light grip, a cap pushed only part way on
usually drops off in the simulation when let go, where a real one tends to hang on the knob. The valve slides freely in its hole and stops at 12 mm out; past 10 mm it
is counted as fallen out. No steam or pressure is simulated. Friction is about 0.15 on the wet steel parts and 0.9 on
the silicone cap.

## Assets

The distractors come from the Hugging Face dataset
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`cleaning_dishSoap_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/cleaning_dishSoap_a01)
- [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01)
- [`cleaning_sponge_b01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/cleaning_sponge_b01)

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time:
masses set (soap bottle 400 g, mug 320 g, sponge 20 g); the soap bottle's pump and cap joints are removed and its
bodies merged into one rigid body. Nothing large is committed: the files download at first run.

The lid, the float valve, the cap and the rack are built in `task.py` from primitives and revolved profiles: SimReady
has no pressure cooker, float valve or dish rack.
