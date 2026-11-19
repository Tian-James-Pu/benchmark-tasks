# Relight a gas water heater's pilot: hold the knob in while the other hand sparks, until the status light blinks

The gas to a house was shut off for a repair and is back on. The water heater's standing pilot is out, its control knob is at OFF and its status
light is dark. The robot must light it the way the label on the heater says, and leave the knob at HOT (the 120 F mark).

The scene is the lower front of a 40-gallon atmospheric natural-gas water heater at full size, standing on the table as on a floor: a section of its
20 in jacket, the burner opening with the outer door off and the inner door's sight glass, the black-iron gas supply with its shutoff valve, and the
gas control, modelled on the Resideo (Honeywell) WV8840B that A.O. Smith fits to these heaters (trade replacement WT8840B1000), at A.O. Smith's
height for it. The knob, its push-in, the igniter button and the status light are where and how the makers show them.

| File | What it is |
|---|---|
| `task.py` | The `yam_arena` task file: scene, the control's behaviour, scoring (`score`, `success`), the scripted `demo()` and the closed-loop `demo_script()` |
| `task.json` | The task's details for the board |
| `check_score.py` | The control's model and the scorer on hand-built states, with no simulator |
| `mechkit.py`, `demokit.py` | The mechanism kit and the closed-loop demo player, unchanged copies of the ones in this repository's `tasks/heat-shrink-lug/` |
| `simready.py` | Loads the SimReady distractors, downloading them once from Hugging Face |

## Importance and sources

- **The operation is part of the job.** A standing-pilot heater is lit at the end of its installation: A.O. Smith's manual ends its start-up
  checklist with "read the Lighting Instructions and proceed with lighting the heater" ([A.O. Smith 100341468, p. 23](https://www.aosmithatlowes.com/media/1688/signature-100-series-high-recovery-100341468.pdf)).
  Pilots are relit whenever the gas has been off: "Gas Service Representatives will visit customer homes and businesses to restore service and
  relight pilot lights", and a pilot that has gone out is relit by following "the appliance manufacturer's instructions"
  ([PG&E, Gas safety](https://www.pge.com/en/outages-and-safety/safety/gas-safety.html), read 18 November 2026).
- **Who does it.** Plumbers (510,600 employed, 2025) have the tasks "Install pipe assemblies, fittings, valves, appliances such as dishwashers or
  water heaters, or fixtures such as sinks or toilets, using hand or power tools" and "Inspect, examine, or test installed systems or pipe lines"
  ([O*NET 47-2152.00](https://www.onetonline.org/link/summary/47-2152.00)). General maintenance and repair workers (1,621,800 employed, 2025)
  "Assemble, install, or repair ... pipe systems, plumbing, machinery, or equipment" ([O*NET 49-9071.00](https://www.onetonline.org/link/details/49-9071.00)).
  O*NET names neither occupation as lighting pilots, so that plumbers, building maintenance workers and utility technicians do this is our reading
  of the sources above, not a measured share of their work. Householders also light their own from the label (PG&E, above).
- **How common the appliance is.** In 2020, 59.33 million of 123.53 million US homes heated water mainly with natural gas, and 8.03 million main
  water heaters of any fuel were tankless ([EIA, RECS 2020, Table HC8.1](https://www.eia.gov/consumption/residential/data/2020/hc/pdf/HC%208.1.pdf),
  final data March 2023), so at least 51 million homes had a gas storage-tank heater (our arithmetic, 59.33 − 8.03). PG&E says "most water heaters
  have a small, continuously burning gas flame—the pilot light" (above). We found no survey of what share of gas storage heaters have a standing
  pilot rather than electronic ignition, or of how often pilots are relit.
- **The procedure modelled,** from the heater's lighting label ([A.O. Smith 100341468, p. 24](https://www.aosmithatlowes.com/media/1688/signature-100-series-high-recovery-100341468.pdf)):
  1. "Turn the gas control/temperature knob clockwise to 'PILOT'."
  2. "Press the gas control/temperature knob all the way in and hold it in. The knob should travel in about 1/4 inch (6.35 mm) if it is set to
     'PILOT' correctly. While holding the gas control/temperature knob in, click the igniter button continuously (about once a second) for up to 90
     seconds or until Status Light begins to blink."
  3. "When the status light starts blinking, release the gas control/temperature knob. Set the gas control/temperature knob to the desired setting."

  "The circuitry in this advanced gas valve requires that you wait 10 minutes between lighting attempts." The control maker's instructions for the
  trade replacement say the same: "hold the knob in until the status indicator starts to blink"
  ([Resideo 34-00013EF, p. 5](https://digitalassets.resideo.com/damroot/Original/10004/34-00013EF.pdf)). These documents establish the real
  operation and its order; they do not by themselves show how widespread it is.

## What is distinct from other hold-one-control tasks

Accepted tasks already keep a button pressed while moving a handle (a suitcase handle), hold an enabling button at its middle position while guiding
an arm, squeeze handles while turning a crank, and hold a tailstock lever down while seating a shaft. This one differs in what the held control is,
in what the other hand does and in when the hold may end:

- **A sustained push, not a grip or a momentary press.** The knob is a valve held open against its spring by the hand alone: about 15 N straight into
  an upright face, on a 29 mm knob that offers nothing to grip in the push direction, for at least 40 s (the label allows up to 90). In the model
  the pilot valve opens only in the last 0.85 mm of the 6.35 mm travel and shuts if the knob comes back past 5.0 mm, so the hold stays fully in. A
  position-controlled arm can only do this through its own compliance: the demo commands its fingertips 33 to 36 mm past first touch and keeps them
  there.
- **The other hand works a second stiff control 40 mm below the held one, on the same face.** The igniter is an 11 mm button that takes 8 N to start
  and 20 N at its snap; a press that stops short of the snap makes no spark. Both hands converge on a patch about 4 cm across from the front. With the
  YAM's grippers two level approaches collide; the demo comes onto the knob from above (60 degrees from straight down) and onto the igniter level
  from the right.
- **The release is set by the process, and an early release cannot be undone.** The status light, not a stop or a count, says when to let go. Let go
  before it blinks and the pilot goes out and the control refuses to hold it again for 10 minutes, longer than the trial. Hold the knob in with no
  flame for more than 90 s in total and the trial scores 0.
- **Then a set-point turn.** The freed knob is turned 92 degrees, by its 5 mm ridge, to the HOT mark and no further (past HOT is a scald risk).

The scoring asks for the result only: any division of the work between the arms is allowed, including a single gripper spanning both controls
(they must then be pushed to different depths: the knob 6.35 mm, the button 4 mm, from faces 9 mm apart).

## The control, checked against its makers' documents

Every control is at its factory place and works in its factory direction. Nothing was moved or enlarged for the robot.

| | The real control and heater | The model |
|---|---|---|
| Layout | The knob on the control's front face, left of centre; the igniter, a square button marked with a spark, directly below it; the status light to the right (A.O. Smith label, "Gas Control" figure; Resideo 34-00013EF Fig. 5) | The same. Scaled from the label's figure with the cover 135 mm wide (an estimate: neither maker publishes the control's outside dimensions; Resideo's Fig. 5 gives the same proportions within a few millimetres): knob axis 25.5 mm from the cover's left edge, the knob 29 mm across, igniter 40 mm below it, status light 78 mm right and 14.5 mm up. Measure a real control's cover, knob and spacings before a real trial |
| Knob turn | OFF, PILOT, VAC, LOW, HOT (120 F mark), A, B, C, VERY HOT, clockwise; "Turn ... clockwise to 'PILOT'" (label, figures A to C) | The same order and direction; angles read off the label's dial: PILOT 30, VAC 60, LOW 92, HOT 122, A 155, B 182, C 206, VERY HOT 244 degrees from OFF |
| Push-in | "all the way in", "about 1/4 inch (6.35 mm) if it is set to 'PILOT' correctly" | In along its own axis, toward the tank, 6.35 mm, only within 4 degrees of PILOT; pushed in, it does not turn |
| Igniter | "click the igniter button continuously (about once a second)" | Pushed straight in; each press past 4 mm snaps and sparks once; let out, it springs back and re-arms |
| Hold | "until Status Light begins to blink", which "may take up to 90 seconds of continuous Pilot operation" (A.O. Smith); "hold the knob in until the status indicator starts to blink", and the light "should flash after 1 minute" (Resideo) | The control holds the pilot after 40 s of continuous flame; the light then blinks once every 3 s ("LED should blink once every three seconds", Resideo) |
| A failed attempt | "wait 10 minutes between lighting attempts"; the light stays on for a while after the pilot goes out (label; A.O. Smith status chart: "LIGHT ON (SOLID) ... Thermopile is cooling down") | Let out early or turned to OFF, the pilot goes out, the light stays on while the thermopile cools (60 s), and the control will not hold again for 10 minutes |
| Height and size | A.O. Smith GCR-40: 20 in diameter (C), the 1/2 in gas connection 12-1/4 in above the floor (D) | Jacket 508 mm; gas inlet 311 mm up, the knob's axis 322.5 mm up |
| Forces | Not published | Estimates: the knob's spring 6 N at first touch, 15.5 N fully in; the igniter 8 N to start, 20 N at its snap. Measure both with a push-pull gauge (below) |

## Run a trial

With `yam-isaac-lab-arena` and Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`), from this folder:

```bash
# Dream-1 (or any policy): 3,000 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The gas is back on after a repair. Light the water heater and leave it set to HOT." \
  --scorer task_score --max-steps 3000 --store-frames --log-dir ~/yam-eval/relight-pilot-dream

# the scripted demo (demo(), played by yam_scripted): scores 1.0
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "The gas is back on after a repair. Light the water heater and leave it set to HOT." \
  --scorer task_score --max-steps 3000 --store-frames --log-dir ~/yam-eval/relight-pilot-demo

# the closed-loop reference and its failure checks (demo_script(), played by demokit)
python demokit.py task.py OUT                                   # the full-score run, with video
python demokit.py task.py OUT_checks --fast --mode none,skip,onearm,half,early,nospark,holdon,stay,hotter,handon

# the scorer and the control's model on hand-built states, no simulator
python check_score.py
```

The run prints `task_score` at the end.

**Runtime and seeds.** The scripted demo ends in success after 1,614 steps (80.7 s simulated), 6 to 11 minutes per run on a shared RTX PRO 6000
Blackwell GPU, including start-up. A model run is up to 3,000 steps (2.5 minutes simulated). `object_jitter=0`: the layout is fixed, so no seed is needed; the demo
is open loop and deterministic.

## The demos

Both demos are scripted references, not policies. Both follow the label, with the arms split as a person splits their hands.

**`demo()`** (open loop, played by the harness's `yam_scripted`). The left hand pinches the knob's ridge and turns it from OFF to PILOT, lets go,
comes down onto the ridge with closed fingertips (tilted 60 degrees from straight down) and pushes the knob all the way in, its fingertips commanded
36 mm past first touch. The right hand, closed fingertips and level, comes onto the igniter from the right and clicks it 40 times, about once a second,
each press commanded 33 mm past touch so that it snaps. It then leaves, and the left lets the knob out slowly and backs off; the light keeps blinking.
The left pinches the ridge again, turns the knob 92 degrees to HOT, lets go and moves clear. Its positions are the ones the closed-loop script settles
on from the published layout, and its hold is timed (44 s of clicking from the first press) instead of watched. It scored 1.0 in six GPU runs in a row
(1,614 steps each; the last two with this `task.py` exactly as published, the earlier four with earlier revisions of it).

**`demo_script()`** (closed loop, played by `demokit.py`). The same moves, but the left hand pushes until the knob is in (it reads the knob's travel),
each igniter press goes on until the button has snapped (it reads the button's travel), and the clicking stops when the status light is lit (it reads
whether the light's lens is drawn: what the wrist cameras show). It reads the knob's place from the published layout, its own fingertips, and the
knob's angle. It never reads the pilot's model or the scorer. It scored 1.0 on the GPU (1,715 steps) and on the CPU (1,724 steps); the light blinked
40 s after the first spark, after 39 clicks.

**Failure checks.** `demokit` modes (CPU physics), and open-loop variants of `demo()` played through `yam_scripted` on the GPU (marked GPU):

| check | what it does | score |
|---|---|---|
| `none` (both) | nothing (GPU: both arms parked clear for 30 s) | 0.0 |
| `skip` | turns the knob straight to HOT without lighting | 0.0 (never lit) |
| careless (GPU) | turns the knob straight from OFF to HOT, as on a stove, pushes it hard from above while the other hand clicks the igniter 10 times | 0.0 (the knob goes in only at PILOT) |
| `onearm` (both) | the left arm alone: knob to PILOT, pushed in for 2 to 4 s, let go, then the same hand clicks the igniter (GPU: 6 clicks, then the knob pushed in again for 4 s) | 0.0 (no gas while it sparks) |
| `half` | holds the knob in only 4 mm while clicking | 0.0 (the pilot valve stays shut) |
| `early` (both) | lets the knob out after 5 (CPU) or 8 (GPU) clicks, then turns it to HOT | 0.25 (lit, out, locked out) |
| early, then again (GPU) | lets the knob out after 15 clicks, pushes it in again and clicks 25 more times, lets it out and turns it to HOT | 0.25 (relit while held, but the control will not hold a second attempt within 10 minutes) |
| `nospark` | holds the knob in 92 s before the first click, then lights it | 0.0 (more than 90 s of unburnt pilot gas) |
| `holdon` | never lets the knob out | 0.25 |
| `stay` | leaves the knob at PILOT, lit | 0.5 |
| `hotter` | turns on past HOT, to A | 0.25 |
| `handon` | sets HOT but keeps hold of the ridge | 0.5 |

## Scoring

`task_score` gives 1.0 when `success()` holds, otherwise the last step's `score()`. The task file sets `SUCCESS_HOLD_S = 0` and counts its own
5 s hold; `success()` is true from then on, and the trial ends.

- **Kept for the whole trial, cleared only by a reset:** if pilot gas flows with no flame for more than 90 s in total (the knob held in at PILOT
  with the pilot out), the score is 0. The label gives 90 s as the limit for one lighting attempt; it is an operating limit, not a simulated flare.
- **1.0** when the pilot is lit and held by the control (status light blinking), the knob let out (in less than 1 mm), the knob within 8 degrees of
  the HOT mark, and every fingertip at least 3 cm from the knob and the igniter, all for 5 s in a row.
- **0.5** at the end if the pilot is lit and held by the control and the knob is let out, at HOT + 8 degrees or cooler (PILOT, VAC, LOW, or HOT with a
  hand still on it or for less than 5 s).
- **0.25** if the pilot was lit at some point but at the end it is out (let out before the light blinked, or turned to OFF), the knob is still being
  held in, or the knob is set more than 8 degrees past HOT (toward A to VERY HOT, a scald risk).
- **0** if the pilot never lit.

`check_score.py` runs the control's model and the scorer on 38 hand-built cases: no flame without the knob fully in, one spark per full press, held
after 40 s, out when let go early or turned to OFF, no hold after an early release, every tier, the 5 s hold, the hands-clear rule, and the unburnt-gas
record surviving the starting layout's return.

**In a real trial,** from video and a photo: the status light blinking once every 3 s (Resideo, normal operation: "1 flash every 3 seconds",
"indicates the electronics is holding the Pilot Valve open") and the flame seen through the sight glass by a camera at the burner opening; the knob's angle from a photo of the
dial (the pointer within 8 degrees of the HOT mark, read with a printed protractor laid over the photo); the knob's push-in, the unlit
time (the knob held in while the sight-glass view shows no flame) and the fingertip clearance from the video.

## Real trials: build and safety

- **Heater.** A 40-gallon atmospheric natural-gas water heater with the WV8840B control (or its trade replacement WT8840B1000) and its CS88xx pilot
  assembly, the igniter in its factory place, as delivered. Fill the tank with water before any lighting (the control senses the water through its
  immersion well; never fire an empty tank). Take the outer burner door off, as in the scene; the inner door and its sight glass stay in place, so
  the pilot and main burner flames stay inside the sealed combustion chamber.
- **Stand.** Stand the heater on a platform level with the arms' table top, at the scene's place: its jacket's front 40.6 cm in front of the arm
  bases, its axis on the arms' midline. Install it as its manual says: gas supply through a sediment trap, a manual quarter-turn shutoff valve within
  reach of the supervisor, the draft hood connected to a vent (or the stand outdoors or under an exhaust hood), a carbon-monoxide alarm at head
  height and a combustible-gas detector at the level where the supplied gas collects, and a fire extinguisher at hand.
- **Supervision.** A trained supervisor watches every trial at the shutoff valve and closes it if pilot gas flows unlit for 90 s in total, if either
  detector alarms, or if an arm does anything but work the control. The robot's hands touch only the control's face.
- **Between trials.** Knob to OFF, shutoff valve closed, and a 10-minute wait (the label's wait between lighting attempts; the status light goes out
  first).
- **Forces and sizes.** Before the first trial, measure with a push-pull gauge the knob's force at first touch and fully in (at PILOT) and the
  igniter's force at the start and at its snap. The model uses 6 and 15.5 N, and 8 and 20 N. Measure the cover's width, the knob's diameter and the
  knob-to-igniter spacing with calipers too (the model uses 135 mm, 29 mm and 40 mm).

## Model

The flame, the spark and the thermopile are bookkeeping in `task.py` (`GasControl`), not combustion, and no gas, heat or carbon monoxide is
simulated. A spark lights the pilot only while the knob is in at least 5.5 mm at PILOT (the valve shuts again below 5.0 mm). The thermopile warms over
40 s of continuous flame, after which the control holds the pilot valve and the light blinks once every 3 s; it cools over 60 s without flame. Let
out before then, or turned to OFF, the pilot goes out, the light stays on while the thermopile cools, and the control will not hold for 10 minutes.
The flame is drawn behind the sight glass while the pilot burns.

The control is one articulation (`mechkit`): the push-in and the igniter are sprung slides, the knob a hinge on the push-in, with light detents at
OFF and PILOT and a lock against turning while pushed in. The cover, knob, jacket, inner door and pipes are rigid primitives at their real sizes; the
dial's marks are drawn as strokes and the label's small print as lines. Friction is uniform and high, and the arms have no force sensing. The flame
cannot be seen from the top camera, as on a real heater, where the control sits above the burner opening; the status light can be seen from the
wrist cameras and, small, from the top camera.

## Assets and credits

The three distractors (scoring ignores them) come from the Hugging Face dataset
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned to revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [`handTool_wrench_adjustable_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_wrench_adjustable_a01): adjustable wrench, 300 g
- [`handTool_pliers_groove_joint_10in_b01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_pliers_groove_joint_10in_b01): 10 in groove-joint pliers, 280 g
- [`cleaning_cloth_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/cleaning_cloth_a01): cleaning cloth, 50 g

They are © NVIDIA, licensed [CC-BY 4.0](https://creativecommons.org/licenses/by/4.0/). Changes made at load time: masses are set to the values
above, and the jointed tools (the wrench's worm and jaw, the pliers' two handles) are welded into one rigid body each. `simready.py` downloads them
into `~/.cache/simready` the first time the scene is built; nothing large is committed.

The heater, control and pipes are generated by `task.py` with `mechkit.py`. `mechkit.py` and `demokit.py` are unchanged copies of the open-source
kits in this repository's `tasks/heat-shrink-lug/` folder.

## Sources

- A.O. Smith, *Signature 100 Series High Recovery: Installation Instructions and Use & Care Guide* (100341468), the lighting label ("For your safety
  read before lighting", p. 24, with the "Gas Control" and control-knob figures A to C) and Table 5, status light chart:
  https://www.aosmithatlowes.com/media/1688/signature-100-series-high-recovery-100341468.pdf
- Resideo, *WT8840 & WT8860 Trade Water Heater Controls, Installation Instructions* 34-00013EF-05: Table 1 (WV8840B to WT8840B1000, A.O. Smith),
  Fig. 5 (controls and connections), "Pilot Gas and Lighting Procedure", Table 4 and Fig. 6 (status light):
  https://digitalassets.resideo.com/damroot/Original/10004/34-00013EF.pdf
- A.O. Smith, *Residential Gas Water Heaters* specification sheet AOSRG47000 (GCR-40: diameter C 20 in; 1/2 in gas connection D 12-1/4 in):
  https://university.hotwater.com/wp-content/uploads/sites/2/2023/11/AOSmith-Residential-Gas.pdf
- O*NET OnLine, 47-2152.00 Plumbers, Pipefitters, and Steamfitters (https://www.onetonline.org/link/summary/47-2152.00), and 49-9071.00
  Maintenance and Repair Workers, General (https://www.onetonline.org/link/details/49-9071.00); employment 2025.
- U.S. Energy Information Administration, *2020 Residential Energy Consumption Survey*, Table HC8.1, Water heating in U.S. homes (final data March
  2023): https://www.eia.gov/consumption/residential/data/2020/hc/pdf/HC%208.1.pdf
- PG&E, *Gas safety* (pilot lights; gas restoration and relight), read 18 November 2026:
  https://www.pge.com/en/outages-and-safety/safety/gas-safety.html
