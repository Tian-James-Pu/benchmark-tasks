# Fly escape detector: build and wiring

The detector does in the real rig what `task.py` does in the simulator: once anything comes within 150 mm of the fly, the
fly leaves 100 ms later, even if whatever came near has pulled back since. Thirteen VL53L1X time-of-flight sensors set
flush in the wall round the fly do the "within 150 mm". An Arduino Nano runs the timer (`detector.ino`). A 12 V solenoid
then pulls the inked fly pad 2.5 mm behind the wall face. Parts and links are in `PARTS.md`; the check that a build works
is in `CALIBRATION.md`; `coverage.py` computes the thresholds and where the detector fires.

## The sensor puck

The fly sits at the centre of a puck 127 mm (5 in) across and 12 mm thick, the same thickness as the panel. The puck is printed
in PLA or cut from MDF. It is press-fitted into a 127 mm (5 in) hole-saw hole in the panel, centred on the fly's spot (y 49,
14 cm up). Its face is sanded flush with the panel (within 0.3 mm: a straightedge across it shows no light) and painted
white. A flush face lets the swatter's head lie flat on the fly.

Coordinates are in the wall's plane, measured from the fly's centre: v along the wall towards the robot's +y (to the left
as the robot sees it), w up. Elevation is the tilt of a sensor's axis away from straight out of the wall; azimuth is the
direction it tilts in (0 = +v, 90 = up, 180 = -v, 270 = down).

| Sensor | Window centre (v, w) mm | Axis elevation | Azimuth | Threshold in `detector.ino` (mm from its window) |
|---|---|---|---|---|
| S0 | (9, -9) | 0 (straight out) | - | 149 |
| S1 | (22, 0) | 45 | 0 (+v) | 134 |
| S2 | (0, 22) | 45 | 90 (up) | 134 |
| S3 | (-22, 0) | 45 | 180 (-v) | 134 |
| S4 | (0, -22) | 45 | 270 (down) | 123 (15 mm short of the table top in its cone) |
| S5 | (25, 25) | 70 | 45 | 116 |
| S6 | (-25, 25) | 70 | 135 | 116 |
| S7 | (-25, -25) | 70 | 225 | 116 |
| S8 | (25, -25) | 70 | 315 | 116 |
| S9 | (40, 0) | 75 | 0 (+v) | 111 |
| S10 | (0, 40) | 75 | 90 (up) | 111 |
| S11 | (-40, 0) | 75 | 180 (-v) | 111 |
| S12 | (0, -40) | 75 | 270 (down) | 85 (15 mm short of the table top in its cone) |

- A threshold is the range at which a target on that sensor's axis is 150 mm from the fly's centre: 150 mm minus how far
  the window sits out along the axis.
- Each sensor carrier (13 x 18 mm) sits in a pocket cut at its elevation. S0's window is 1 mm below the puck face; the
  tilted sensors' windows are level with it, so that the face does not cut into their cones.
- Each tilted pocket has a sloped relief cut in front of the window, sloped at the sensor's elevation, so its 27 degree cone
  clears the puck face: 10 x 14 mm for 45 degrees, 10 x 18 mm for 70 and 75 degrees (S9-S12's cuts end 3 mm inside the
  puck's rim).
- The centre has an 8 mm through-hole for the pad's rod, inside a 14 mm felt ring glued flush with the face.
- The solenoid is screwed to a 12 mm MDF backer block behind the panel, its plunger on the hole's axis. The fly pad (black
  felt cut to a fly's outline, 7 x 4 mm and 3 mm thick, on a 3 mm rod) stands 3 mm proud of the face when the solenoid is
  off. Energised, the 5.5 mm stroke pulls it 2.5 mm behind the face.
- The sensor cables leave through the back of the puck to the Arduino, which is clipped to the panel's foot behind the wall.

```
  wall face, seen from the robot (puck 127 mm across)        section through S11, S3, S1, S9 (seen from above)

                    w (up)                                     room side            face of the wall (x 42)
                     o S10                                      \ 75  \ 45     |     45 /  75 /
                     |                                           \     \       |       /     /
           S6 o      o S2     o S5                          ______\[S11]\[S3]__|__[S1]/[S9]/______
                     |                                      panel  pockets    pad    pockets
   -v  S11 o----S3 o-+-o S1----o S9  +v                     12 mm  ___________ || ___________
                   (fly)                                           backer      [SOL]  12 mm MDF
           S7 o   S0 o o S4   o S8                                     solenoid, plunger on the hole's axis
                     |
                     o S12
```

## Wiring

All sensors share the I2C bus and power. Each sensor's XSHUT has its own Arduino pin, so that the sketch can wake the
sensors one at a time and give each its own address. The sketch needs an Arduino Nano: it uses A6 and A7, which an Uno
does not have.

| Arduino Nano pin | Goes to | Notes |
|---|---|---|
| 5V | VIN of all 13 VL53L1X carriers | the carriers regulate to 2.8 V and level-shift SDA/SCL |
| GND | GND of all carriers, MOSFET source, LED cathodes, button, CAL jumper, 12 V supply minus | one common ground |
| A4 (SDA) | SDA of all carriers | keep the bus under 50 cm in all, twisted with GND |
| A5 (SCL) | SCL of all carriers | |
| D2, D3, D4, D5, D6, D7, D8, D9, D10, D12, A0, A1, A2 | XSHUT of S0, S1, ..., S12 in that order | XSHUT is not 5 V tolerant: the sketch only drives it low, or leaves it floating (the carrier pulls it up) |
| D11 (PWM) | MOSFET gate through 220 ohm; 100 k from gate to GND | IRLB8721 logic-level N-MOSFET; its drain to the solenoid's minus lead |
| D13 | red ESCAPED LED anode through 330 ohm | latches on when the fly leaves; the Nano's own LED lights with it |
| A3 | green ARMED LED anode through 330 ohm | |
| A6 | RESET push button to GND; 10 k from A6 to 5V | A6 is analog input only, read with analogRead |
| A7 | CAL jumper to GND; 10 k from A7 to 5V | fitted: calibration mode, solenoid off |
| VIN | +12 V | the Nano runs from the same 12 V supply |

```
                    +5V ──┬─────────┬───── ... ─────┬──────────┐
                          │         │               │          │
                      ┌───┴───┐ ┌───┴───┐       ┌───┴───┐      │
     SDA (A4) ────────┤  S0   ├─┤  S1   ├─ ... ─┤  S12  │   VL53L1X carriers: VIN, GND, SDA, SCL, XSHUT
     SCL (A5) ────────┤       ├─┤       ├─ ... ─┤       │
                      └─┬──┬──┘ └─┬──┬──┘       └─┬──┬──┘
                  XSHUT │  │GND   │  │GND         │  │GND
     D2  ───────────────┘  │      │  │            │  │
     D3  ──────────────────┼──────┘  │            │  │
     ...                   │         │            │  │
     A2  ──────────────────┼─────────┼────────────┘  │
     GND ──────────────────┴─────────┴───────────────┘

     +12 V ──────┬───────────────┬──────────── Nano VIN
                 │               │
           solenoid coil     1N4001 (stripe to +12 V), across the coil
                 │               │
                 ├───────────────┘
                 │ drain
     D11 ─220R── gate   IRLB8721
         100k to GND
                 │ source
     GND ────────┴──────────────────────────── 12 V supply minus

     D13 ──330R──►|── GND  (red, ESCAPED)        A6 ──┬── RESET button ── GND
     A3  ──330R──►|── GND  (green, ARMED)              └── 10k ── 5V
                                                 A7 ──┬── CAL jumper ──── GND
                                                       └── 10k ── 5V
```

## Timing

- Each sensor gives a new reading every 20 ms: short mode, 20 ms timing budget, continuous.
- The loop reads all 13 in under 10 ms. So something crossing 150 mm is seen 0 to 30 ms after it crosses.
- The solenoid fires 100 ms after that first reading, and the pad is behind the face about 10 to 20 ms later.
- That gives 110 to 150 ms from the crossing to the pad being gone.
- In the simulator the check runs at 20 Hz, and the fly leaves at the second check after the first check that finds
  something within 15 cm: 100 to 150 ms after the crossing. The two windows agree.
- `CALIBRATION.md` step 5 measures the real window with a 240 fps video.

## What the detector does not match exactly

The simulator's rule is a 150 mm hemisphere round the fly, checked against points on the swatter and both grippers'
fingers. The 13 sensors' cones cover that hemisphere in 13 directions. `python coverage.py` prints where they fire.

- **The swatter's head is caught from every side, side-on included.** A 100 x 100 mm card (smaller than the 127 x 119 mm
  head) brought straight in, face-on to the fly, fires the detector 135 to 152 mm from the fly from every direction above
  the fly's height or to either side, from straight out to grazing the wall at 85 degrees. From below, where the table top
  is 140 mm under the fly, it fires at 126 to 151 mm.
- **A head held flat and slid along the wall** from either side or from above, 5 to 30 mm off it, fires the detector with
  its nearest point 149 to 150 mm from the fly; 60 mm off the wall, at 133 mm.
- **A thin object can get closer unseen.** A fingertip, or the handle's 4 mm edge, coming in between two cones (at 20 to 60
  degrees from straight out on the diagonals) may not be seen until a few centimetres from the fly. `coverage.py` lists
  these directions. A head held edge-on in such a gap is the same case.
- **What these gaps change.** A real fly can be scared off later than in the simulator, never sooner, so the gaps can only
  make a real trial easier. A swat still needs the head, which is caught 126 mm or more out, to land on the pad within the
  window.
- **The VL53L1X's range.** Its minimum accurate range is 4 cm, and a reading closer than that can be wrong. By then the
  timer is already running for anything that came in through a cone.
- **Not yet built.** We have not built this detector. The sketch has not been compiled or run on hardware here. The
  thresholds and firing distances above come from `coverage.py`'s geometry, and `CALIBRATION.md` is how a build is checked
  against them.

Report real-trial results with these differences stated.
