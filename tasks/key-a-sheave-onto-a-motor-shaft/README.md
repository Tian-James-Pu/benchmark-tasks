# Key a V-belt sheave onto a motor shaft and tighten its set screw over the key

A task file for the `yam_arena` embodiment (Isaac Lab-Arena, station scene): [task.py](task.py).

A NEMA 56 frame motor is bolted to the bench with its 5/8 in shaft pointing at the robot. A Browning AK30X5/8
cast-iron V-belt sheave is just started on the shaft's end (hub 2 mm on), its keyway turned 30 degrees away from the
shaft's empty keyseat. Its 3/16 in square key lies across a small wooden rest on the robot's left, and a 5/32 in hex
L-key stands in a drilled wooden holder on the right. The instruction gives only the goal: mount the sheave against
the shaft shoulder and lock it there. Doing that means pressing the key fully down into a keyseat only 0.06 mm wider
than the key, turning the sheave until its keyway lines up with the key, sliding it home over the key, and then picking
up the L-key, getting its tip into the 5/32 in socket of the sheave's set screw and turning the screw down until its
cup point bears on the key and clamps the sheave. The two hands share the work: in the demonstration the left hand
handles the key and the sheave and the right hand handles the L-key. Which hand does what is free.

## Run and score

With the simulator installed, from the repository root:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/key-a-sheave-onto-a-motor-shaft/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=640 -E cam_height=360 \
  --instruction "Mount the sheave on the motor shaft, all the way in against the shaft shoulder, and lock it there." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/sheave-demo
```

That plays the scripted demonstration (`demo()` in `task.py`). The model run is the same command with
`--policy dream -E cam_width=320 -E cam_height=176 --max-steps 2400`. The harness prints `task_score` at the end; the
episode ends as soon as `success()` is true.

- **Runtime:** the demonstration lasts DEMO_STEPS control steps (DEMO_S s simulated), about 10 minutes of wall time on an
  L40S including start-up; a model run is at most 2,400 steps (2 minutes simulated).
- **Seeds:** `object_jitter=0` keeps the layout fixed, so the seed does not matter. The demonstration is open loop
  (fixed waypoints computed from the published layout) and deterministic.
- **Debug log:** setting `SHEAVE_LOG=/path/file.txt` makes the task append one line per step (joints, key pose, screw
  turn, scorer state). It does not change the scoring.

## Scoring

`score()` and `success()` in `task.py` implement this exactly (class `Trial`); thresholds are the module's constants.

- **0.25: key home.** The key lies fully down in the keyseat: its centre within 0.3 mm of the seated position across
  and in height, its whole length inside the keyseat (to 0.2 mm), its length within 1.5 degrees of the shaft's axis,
  and no gripper within 1 cm of it (unless the sheave's hub already covers part of it).
- **0.5: and the sheave home.** The hub's inner end within 0.5 mm of the shaft shoulder, with the keyway within 2
  degrees of the keyseat (so the key is inside the keyway).
- **0.75: and the set screw bearing on the key.** The set screw has been turned at least 5 degrees past the point
  where its cup point first meets the key. A screw run down onto the bare shaft (key missing or not under it) does not
  count.
- **1.0 (success): and tight and verified.** The screw turned at least 45 degrees past first contact with the key,
  the L-key's tip at least 15 mm out of the socket, both grippers and the L-key at least 3 cm from the sheave, and the
  sheave then holds still (moving under 0.3 mm along the shaft and under 0.3 degrees about it) for 1 s while the
  scorer pulls it 20 N toward the shaft's end and twists it 1 N m about the shaft. If it moves, the test stops and
  is tried again 1 s later.
- **0** otherwise: key not fully down, cocked, lost or still held; sheave not home; nothing done.

**In the real world** the same checks take a depth gauge (key top 4.76 mm above the keyseat floor), a 0.5 mm feeler
gauge at the shoulder, a torque wrench with a 5/32 in hex bit (Browning's 110 to 130 in-lb for a 5/16 in set screw
over the key) and a spring scale on a cord for the 20 N pull. The simulation cannot reach that torque (see Fidelity),
so there "tight" is the screw's turn past first contact.

## The mechanism (`task.py`)

- **`drive`** is one articulation. Its fixed frame is the motor and its shaft, with the keyseat cut into the shaft's
  colliders (floor, two walls with 0.4 mm edge breaks, both ends closed as an end-milled keyseat). Its links are the
  sheave's slide along the shaft (stopped by the shoulder at 0 and by the shaft's end), the sheave's free turn about the
  shaft, and the set screw's feed along its own axis. The sheave's hub, web and rim are 45 convex colliders, with the
  keyway's two walls and top built round the key, 0.08 mm clear of it on each side and 0.13 mm over it.
- **The key** is a free steel body (chamfered 0.4 mm) that collides with the keyseat's and keyway's walls by contact;
  nothing holds it in the seat but those walls.
- **The set screw's thread** is code (class `Screw`): while the L-key's tip is in the socket (within 1.1 mm of the
  screw's axis, 1.8 mm or more deep, under 8 degrees of tilt), the L-key's twist about the screw's axis turns the screw,
  and the screw advances 1/18 in a turn (5/16-18 UNC) until its cup point meets what is under it: the key's top, the
  keyseat floor or the bare shaft. Turning past first contact is counted; at 90 degrees past it the screw stalls. The
  screw resists the L-key with a torque that rises from 0.02 N m running to 0.2 N m at 45 degrees past contact and
  1.5 N m stalled, applied to the L-key.
- **The clamp:** with the cup point bearing 5 degrees or more past contact, the sheave's slide and turn are locked
  (mechkit `lock`), as the screw's point bearing on the key locks the hub in the real part; backing the screw off frees
  them.
- **The L-key** is a free body of two hex prisms (5/32 in across flats), and the round socket is 0.3 mm clear of its
  corners with a 1 mm countersunk entry.

## Physical setup

Millimetres from the midpoint between the arm bases (x forward, y to the robot's left); heights above the bench top.

| Object | What it is | Where |
|---|---|---|
| Motor | NEMA 56 frame (shaft height D 3-1/2 in = 88.9, frame P 6-5/8 in = 168 across, 240 long behind the shoulder, cast base 170 x 140), fixed | shaft axis along x at y 0, 88.9 up; shoulder at x 265 |
| Shaft | steel, U 5/8 in (15.9), V 1-7/8 in (47.6) usable, end at x 217 | keyseat on top: S 0.190 in (4.83) wide, R 33/64 in (floor 13.1 above the shaft's bottom), 32.3 long, 3.2 to 35.4 from the shoulder |
| Sheave | Browning AK30X5/8, cast iron, 1 groove A/4L, OD 3.05 in (77.5), bore 5/8 in, keyway 3/16 x 3/32 in, hub 1.57 in (39.9) across, 1.19 in (30.2) overall, 445 g | on the shaft end, hub's inner end at x 219 (2 mm on), keyway turned 30 degrees toward -y |
| Set screw | 5/16-18 x 3/8 in cup point socket set screw, 5/32 in socket 0.140 in deep, alloy steel | in the hub over the keyway, its axis 7.9 from the hub's inner end; cup point 0.6 above the key's top when everything is home |
| Key | steel square key 3/16 in (4.76) x 1-1/4 in (31.75), 5.7 g | lying along x across the key rest, centred at (195, 135) |
| Key rest | wooden block 12 (x) x 40 (y) x 25 high, fixed | centred at (195, 135); the key's ends overhang it 10 mm |
| L-key | Eklind 16110 5/32 in hex L-key, 2-1/2 in long arm, 27/32 in short arm, black oxide steel, 9 g | long arm down in the holder's hole, short arm pointing +x |
| L-key holder | wooden block 40 x 40 x 40 with a 7 mm hole 35 deep, fixed | centred at (205, -150) |
| Distractor: shop rag | SimReady `cleaning_cloth_a01`, cotton, 260 x 233, 50 g | (470, 270), turned 17 degrees |
| Distractor: combination wrench | SimReady `handTool_wrench_combination_a01`, steel, 260 long, 250 g | (420, -240), along x |
| Distractor: mug | SimReady `dishware_mug_a01`, ceramic, 90 across, 80 high, 350 g | (620, -160) |

The scorer ignores the distractors; none is reached by the demonstration's paths or offers a shortcut.

## Demonstration

`demo()` is a fixed plan of waypoints for both arms, computed from the layout above (no simulator state is read).
Left arm: pinch the key end to end on its rest, carry it over the keyseat and let it go just above it, press it down
at both ends with closed fingers, pinch the sheave's rim and roll it 30 degrees to bring the keyway over the key, then
push the rim's face with the hand tilted 45 degrees until the hub meets the shoulder, and move clear. Right arm: lift
the L-key out of its holder by its long arm, bring its tip down into the set screw's socket, turn it 200 degrees
clockwise in one stroke while following the screw down, lift it out, stand it back in its holder and move clear.

## Fidelity and approximations

- The set screw's thread, its resisting torque and the clamp are modelled in code, not by contact (PhysX does not
  mesh threads). The real seating torque (110 to 130 in-lb, 12 to 15 N m) is beyond a YAM wrist, so the simulation's
  "tight" is a turn of 45 degrees past first contact against a token 0.2 N m. Turning 45 degrees advances a 5/16-18
  screw 0.18 mm, about what a cup point bites and the parts deflect when seated (an estimate, not a measurement).
- The bore's fit on the shaft is the articulation's joints (self-collision is off inside it); the key's fit in the
  keyseat and keyway is contact, at ANSI class 1 clearances.
- The socket is round (0.3 mm over the hex's corners), so the L-key's twist is read from its pose rather than from
  hex-on-hex contact.
- Uniform friction of the parts (0.35 static, 0.3 dynamic on the steel parts); no lubrication, burrs or cup-point
  marking. The motor is fixed to the bench; its winding, fan and terminal box are reduced to plain shapes.
- The key rest and L-key holder are plain wooden blocks (primitives). The motor, sheave, key and L-key are built from
  the dimensions above because SimReady has no NEMA motor, FHP sheave, machine key or single hex L-key.

## Importance and sources

- **Occupation and size.** General maintenance and repair workers: 1,621,800 US jobs in 2025, BLS Occupational
  Outlook Handbook, [General Maintenance and Repair Workers](https://www.bls.gov/ooh/installation-maintenance-and-repair/general-maintenance-and-repair-workers.htm).
  Industrial machinery mechanics, machinery maintenance workers and millwrights: 547,300 jobs in 2025, BLS
  [Industrial Machinery Mechanics, Machinery Maintenance Workers, and Millwrights](https://www.bls.gov/ooh/installation-maintenance-and-repair/industrial-machinery-mechanics-and-maintenance-workers-and-millwrights.htm)
  (446,900 industrial machinery mechanics).
- **That this is their regular work.** O*NET's task list for general maintenance and repair workers includes
  "Perform routine maintenance, such as inspecting drives, motors, or belts ..." and the work activities "Replace
  worn, damaged, or defective mechanical parts" and "Install machine or equipment replacement parts"
  ([O*NET 49-9071.00](https://www.onetonline.org/link/details/49-9071.00)); for industrial machinery mechanics,
  "Reassemble equipment after completion of inspections, testing, or repairs"
  ([O*NET 49-9041.00](https://www.onetonline.org/link/details/49-9041.00)). *Inference:* fitting a sheave to a
  motor shaft, when a belt drive's motor or sheave is replaced, is part of that drive work; O*NET does not name the
  sheave.
- **The real operation.** Browning's FHP variable-pitch sheave instructions: "Install shaft key. Slide the sheave onto
  the motor shaft ... Secure the sheave to motor shaft by tightening the setscrew over the key to the proper torque
  (110-130 in. lbs. torque for 5/16" setscrews ...)"
  ([MasterDrive/Browning VL-VP FHP sheave installation instructions](https://d20xmcaxe3n3e3.cloudfront.net/userfiles/documents/solveindustrial/technical-resources/masterdrive%20vl-vp%20sheave%20installation%20instructions.pdf)).
  *Inference:* we apply the same set-screw torque to the fixed-pitch AK sheave, which those instructions do not cover.
- **The sheave.** Browning AK30X5/8 (part 1026186): 5/8 in finished bore with keyway 3/16 x 3/32 in, OD 3.05 in, cast
  iron, single groove 3L/A ([distributor data sheet](https://www.eindustrialsolutions.com/browning-ak-1026186-sheave-finished-with-keyway-bore-5/8-in-dia-bore-21/32-in-w-face));
  0.98 lb ([MROSupply listing](https://www.mrosupply.com/products/2276227/as-markdown/)). Its profile, hub, bore and the
  5/16 in tapped hole (radius 0.1562 in) are read from the manufacturer's 3D model linked from that listing
  ([IGES](https://documents.mrosupply.com/product_documents/AK/AK30X5/8/regalpts_EMERSON-AK_Sheave_Master1-AK30X5-8_DTMx8mQ.IGS)).
  The same sheave size is catalogued by Martin ([FHP sheaves catalog](https://martinsprocket.com/docs//catalogs/power%20transmission/6_sheaves%20catalog/fhp-sheaves.pdf), AK30).
  *Inference:* the rim's inside radius under the groove (0.95 in) is ours; the model's arcs do not give it.
- **The motor and keyseat.** NEMA 56: D 3-1/2 in, U 5/8 in, keyseat R 33/64 in and S 3/16 in
  ([ABB NEMA quick-reference poster](https://library.e.abb.com/public/cf3a80c5c7fd40aab5af7a89fbabbc33/NEMA%20Wall%20Poster_24x36.pdf)).
  The 0.190 in keyseat and 0.191 in keyway widths are our ANSI class 1 allowances on the 3/16 in nominal (an
  inference, not from these sources).
- **The set screw.** 5/16 in cup point socket set screw: 5/32 in key, socket depth 0.140 in, cup diameter 0.203 in max,
  minimum length 3/8 in, recommended seating torque 165 in-lb, axial holding power 1,500 lb
  ([Unbrako engineering guide](https://wp.optics.arizona.edu/optomech/wp-content/uploads/sites/53/2016/08/UnBrako-fasteners.pdf)).
- **The L-key.** Eklind 16110: 5/32 in, 2-1/2 in long arm, 27/32 in short arm
  ([Zoro listing](https://zoro.com/eklind-sae-plain-hex-key-532-tip-size-2-12-in-long-2732-in-short-10-pk-16110/i/G4395571)).

## Earlier submission of this idea

This is an independent implementation of an idea first submitted by another team in PR #2035, whose hold lapsed with
no revision. Its review asked for four things, met here as follows:

1. *A realistic key-to-keyway fit for the real pulley:* a real catalogue sheave (Browning AK30X5/8) with its published
   5/8 in bore and 3/16 x 3/32 in keyway, on a NEMA 56 shaft with its standard keyseat; the key is 3/16 in square, and
   the side clearances are ANSI class 1 (0.06 mm in the keyseat, 0.08 mm a side in the keyway), not millimetres of play.
2. *A real threaded set screw and a tightening tool the robot handles:* a 5/16-18 cup point set screw that advances
   1/18 in per turn and is turned only by a 5/32 in hex L-key whose tip the robot must get into the screw's socket;
   there is no knob or detent.
3. *A reproducible tightening check:* the screw must be turned at least 45 degrees past first contact with the key,
   standing in for Browning's 110 to 130 in-lb, which a person checks with a torque wrench.
4. *Scoring must verify that the screw clamps the pulley:* full credit needs the screw bearing on the key (not on the
   bare shaft) and a holding test in which the sheave must not move under a 20 N pull off the shaft and a 1 N m twist
   for 1 s, with hands and tool clear.

## Credits and assets

- `mechkit.py` and `demokit.py` are copied unchanged from the open-source kit in `tasks/heat-shrink-lug/` of this
  repository. `simready.py` is the repository's SimReady loader.
- SimReady props, from [`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets) at revision
  `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, © NVIDIA, CC-BY 4.0, downloaded at first run (nothing large is committed):
  - [`cleaning_cloth_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/cleaning_cloth_a01)
  - [`handTool_wrench_combination_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/handTool_wrench_combination_a01)
  - [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01)

  Changes at load time: each is made one rigid body with the mass given above.
- Every other mesh is generated in `task.py` from the dimensions above.
