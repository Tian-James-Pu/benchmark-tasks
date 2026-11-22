# Fly escape detector: calibration and acceptance check

Run this once after building the detector (`WIRING.md`), and repeat steps 2 and 3 at the start of each session of real
trials. A detector that fails any step is not used for scored trials until it is fixed and the step passes.

**Tools:**
- a 300 mm steel rule;
- the angle template: a 300 mm half-disc of stiff card with lines from its centre at 0, 20, 30, 45, 60, 70, 80 and 85
  degrees, and arcs at 120, 135 and 150 mm;
- a 100 x 100 mm test card of white 300 gsm card taped to a 30 cm wooden stick;
- the swatter;
- feeler gauges (or 1 mm and 4 mm spacers);
- a phone that films at 240 fps;
- a laptop with the Arduino IDE's Serial Monitor at 115200 baud.

**Calibration mode:** fit the CAL jumper and press the Nano's reset. The serial line reads "# calibration mode". The
solenoid is then disabled, the red LED is lit whenever any sensor reads below its threshold, and each line prints
`ms S0 ... S12 near`.

## 1. The pad and the face

| Check | How | Pass |
|---|---|---|
| Puck flush | Straightedge across the puck and panel in 4 directions | No light under it; no step over 0.3 mm (feeler gauge) |
| Pad out | Caliper depth rod, or the rule's end and a feeler gauge, from the wall face to the pad's top | 3.0 +- 0.3 mm |
| Pad in | Run mode (jumper off). Bring the card within 10 cm until the LED lights, then measure again | Pad 2.0 to 3.0 mm behind the face |
| Reset | Press RESET with nothing near | Pad back out at 3.0 +- 0.3 mm; red LED off, green on |

## 2. Quiet scene (no false triggers)

Set the full scene as for a trial: the swatter on its peg, the distractors on their marks and the robot at its ready pose.
Stand back. In calibration mode, log 60 s of readings.

**Pass:** `near` is 0 on every line.

If a sensor reads the wall, the table or the robot, lower that sensor's threshold in `detector.ino` until it is quiet for
60 s. Then redo step 3 for it and record the new value. The S4 and S12 thresholds are already set 15 mm short of the table.

## 3. Firing distance in every direction, side-on included

Hold the angle template against the wall, perpendicular to it, centre at the fly's centre, along one azimuth: +v (towards
the robot's left), +w (up), -v, -w (down), and the four diagonals between them.

For each line on the template, hold the test card face-on to the fly, its centre on the line, and move it slowly (about
1 cm/s) from 250 mm in along the line. Stop the moment the red LED lights. Measure with the rule, along the line, from the
fly's centre to the card's nearest point.

The expected values are `coverage.py`'s card column, the number after the slash:

| Elevation (degrees from straight out) | 0 | 20 | 30 | 45 | 60 | 70 | 80 | 85 (side-on, along the wall) |
|---|---|---|---|---|---|---|---|---|
| +v, +w, -v (sides and above) | 149 | 144-151 | 144-147 | 149 | 152 | 151 | 150 | 150 |
| diagonals +v+w, -v+w, -v-w, +v-w | 149 | 143-152 | 135-149 | 138 | 146 | 149 | 151 | 151 |
| -w (below; the table top is 140 mm under the fly) | 149 | 151 | 147 | 138 | 142 | 140 | 134 | 126 |

**Pass:**
- every reading is within 12 mm of its expected value;
- no reading is under 120 mm;
- no reading is over 165 mm (the detector must not fire much earlier than the 150 mm rule either).

## 4. The swatter's head slid along the wall

Hold the swatter's head flat and parallel to the wall, 10 mm off it (a 10 mm spacer under its rim), level with the fly.
Slide it towards the fly along the wall at about 5 cm/s:
- from 250 mm out on the +v side;
- from the -v side;
- from above (+w).

When the red LED lights, stop, and measure from the fly's centre to the head's nearest point.

**Pass:** 140 to 160 mm each time. `coverage.py` gives 150.

## 5. Timing, in run mode

Remove the jumper and press reset. Lay the rule along the wall with its 150 mm mark at the fly. Film from the side at
240 fps, with the pad, the rule and the red LED in view.

| Test | What to do | Pass |
|---|---|---|
| Fast approach | Swing the card in along the 0 degree line from 250 mm, at about 1.5 m/s, stopping 30 mm short of the face | From the frame where the card crosses the 150 mm mark to the frame where the red LED lights: 24 to 36 frames (100 to 150 ms). The pad is behind the face within 5 frames after that |
| Approach and retreat | Push the card in to 120 mm and pull it straight back out within 0.05 s | The LED still lights, and the pad pulls in, 24 to 36 frames after the crossing. The fly escapes, as in the simulator |
| Near miss | Bring the card to 165 mm along the 0 degree line, hold 2 s, take it away | Nothing fires |

Repeat each test 3 times. All must pass.

## 6. Contact, not proximity

In run mode, ink the pad (step 1 of a trial: the README's real-trial preparation).

- **Hover:** hold the swatter's head flat over the fly on 4 mm spacers at its rim, so its face is 1 mm above the pad, for
  2 s. Then take the head away.
  - **Pass:** no ink on the head's mesh.
  - The LED lights after 0.1 s, as it must: the head came within 150 mm.
- **Swat:** press RESET, re-ink, and slap the head flat onto the pad by hand from 250 mm out, the last 150 mm in under
  0.1 s.
  - **Pass:** a pad-coloured fly mark on the mesh face.

## Record

Write the date, every threshold used, the step 3 and 4 table of measured distances, and the step 5 frame counts on one sheet
kept with the trial results. A trial run with a detector whose last calibration failed is not scored.
