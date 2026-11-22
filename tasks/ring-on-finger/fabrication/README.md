# Fabrication: hand form, ring, plate and reset template

All dimensions in mm; every number is in `../params.json`. Drawings: `hand_form.scad` (OpenSCAD). Values marked
"estimate" are not measured. No physical build is claimed.

## Hand form (left adult hand, fingers up, palm toward the robot)

- PLA core printed in one piece: palm 28 x 80 x 95, wrist stub 32 x 55 x 25, index / middle / little fingers and thumb
  as rigid round-section fingers, and a 9.6 mm ring-finger bone running 10 mm into the palm. Painted skin tone.
  Module `core`. Mass without the plate about 260 g (estimate).
- Other fingers (rigid): middle at y -20 (base 3 mm into the palm), 12 deg toward -y, 78 long, d 17; index at y -38,
  22 deg, 70 long, d 16.5; little at y +18 (base 8 mm below the web), 12 deg toward +y, 58 long, d 14.5; thumb from
  the palm side at (y -45, z 70), 45 deg out, 60 long, d 20. All in the palm's mid-plane.
- Ring-finger skin: Smooth-On Dragon Skin 10 Medium (datasheet: Shore 10A, 100% modulus 22 psi, specific gravity
  1.07), cast 2.7-4.2 thick over the bone in a two-part printed mould (module `ring_mould`, split at x = 0). Outer
  profile d(u), u from the palm-side web: 19.0 at u -3 (flare into the palm), 17.0 at 0-10, 17.3 at 18, 18.0 at 28
  (PIP crest), 17.0 at 36, 16.0 at 44, 16.4 at 52 (DIP crest), 15.6 at 60, 15.0 at 64.5, then a 7.5 radius dome to the
  tip at u 72. Fallback mould insert: `ring_mould(crest = 17.9)`. Cast two spare skins.
- Painted lines: three 1 mm bands all round the skin, Dragon Skin 10 Medium from the same kit tinted with Silc Pig
  Brown: base u 7-8, PIP u 26-27, DIP u 50-51. Thresholds: the line edge nearest the web for PIP and DIP, the far
  edge for the base.
- Base plate: 150 x 150 x 10 mild steel (1,770 g, estimate), centred 20 behind the ring-finger axis; the wrist stub
  is bolted to it with 2 x M5 x 20 countersunk screws from below into heat-set inserts at x 0, y -30 and +6 from the
  ring-finger axis. Module `plate`.

## Ring

Plain band, US size 7.5, 316L stainless steel, polished: bore 17.75, width 4.0, wall 1.9, outside 21.55, flat bore with
inner and outer edges radiused 0.5 (as simulated). Mass 3.74 g (estimate: 469 mm3 x 7.98 g/cm3). Turn from 22 mm 316L
round bar: face, drill and bore to 17.75, turn the outside to 21.55, part to 4.0, radius the edges. Accept only a bore
of 17.75 +/- 0.05 by caliper.

## Reset template

Module `template` (2D, print 1:1 on A3 and tape to the table at the rig's fixed marks): plate outline at the nominal
pose with 1 mm ticks to +/-15 along two edges, and the ring circle (d 21.55) at (-100, +160) from the ring-finger
axis with 1 mm ticks to +/-20 in x and y. Draws: `../draws.csv` (seed k = row k).

## Push-test clamp (optional calibration)

Module `lever_clamp`: a 1:1 PLA lever holding two spare gripper pads one ring diameter apart, loaded by a 612 g hung
mass (6.0 N pinch), for measuring the push over the PIP on the real form. Not needed to run the task.
