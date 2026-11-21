# Ordinary finger-launched top: NO-GO for acceptance-priority development

Checked 2026-11-20 UTC. No task claim/build, GPU run, purchase or submission.

A real, obtainable object exists: [ForeverSpin stainless steel brushed top](https://foreverspin.com/products/stainless-steel-brush), 28 g, listed $48 with `available:true` in the manufacturer page. The [manufacturer FAQ](https://foreverspin.com/) gives 35.5 mm height and 28 mm diameter for all tops. Its [official launch video](https://foreverspin.com/cdn/shop/t/64/assets/how.mp4?v=164307495627834251381733356716), saved locally and visually inspected in `manufacturer-how-sheet.jpg`, shows an ordinary thumb/index snap with the tip initially on the surface. This is a valid hand toy, unlike Steel’s motorized chuck example. Manufacturer evidence does not supply stem diameter, precise profile, mass distribution, tip radius, contact friction or measured launch speed.

## Acceptance gate

| Evidence | What it establishes | What it does not establish |
|---|---|---|
| [2722 Twister spinner](https://github.com/robocurve/benchmark-tasks/pull/2722), pending | Top release and free gyroscopic balance differ physically from a riveted arrow flick. | Acceptance or sufficient importance for either task. |
| [2522 Jenga](https://github.com/robocurve/benchmark-tasks/pull/2522), accepted | A full familiar game turn with hidden load-sensitive selection can qualify. | Blanket approval of balancing puzzles or desk toys. |
| [2862 card house](https://github.com/robocurve/benchmark-tasks/pull/2862), final rejection | Reviewer explicitly rejected a balancing puzzle for insufficient real-world impact. | That every recreational task necessarily fails. |

The current top proposal remains ordinary spin-duration play or a gyroscope demonstration. Manufacturer marketing mentions entertainment, focus, collecting and learning about metals; none is evidence that this robot operation satisfies the importance bar. Added durations, target areas or more tops would be extra constraints, not a substantive answer to the rejected card-house precedent. **Importance is the decisive NO-GO; mechanical feasibility remains unresolved, not disproved.**

## Launch estimate, with explicit assumptions

[David Tong’s rigid-body notes, sleeping top](https://www.damtp.cam.ac.uk/user/tong/dynamics/dynhtml/S3.html#S3.SS6.SSS3) give upright fixed-tip stability when `omega² > 4*m*g*l*I1/I3²`. For an illustrative thin disk of the retail radius14 mm and mass28 g, `I3 = mR²/2`, `I1 = mR²/4 + ml²`. Assumed COM heights6,8,10 mm give thresholds **45.65,60.77,78.02 rad/s (436,580,745 rpm)**. These are sensitivity examples, not measured product values or certified bounds; a free sliding/rolling rounded tip also departs from fixed-tip theory. A short precessing spin need not satisfy upright sleeping-top stability.

Installed unmodified YAM constants give shoulder5.5 rad/s and wrist20 rad/s. A top held rigidly to one axial wrist alone cannot reach the illustrative thresholds. This is not a universal bound on combined arm angular motion. Parallel gripper fingers also cannot reproduce independent thumb/index tangential motion.

A plausible native alternative is two opposed pads from separate arms rolling the stem with opposite tangential sweeps and then releasing. With **assumed** stem radius3 mm, each pad needs approximately0.137–0.234 m/s at the illustrative threshold. Total tangential impulse is0.042–0.071 N·s, equivalent to0.42–0.71 N summed tangential force over100 ms. Available actuator effort is therefore not the obvious obstacle; collision-free access to the small stem, tip contact, slip and symmetric release are. No contact coupon has established those conditions. Broad fingertips may hit the rim before a useful stem sweep. The installed50 N gripper effort does not establish usable tangential impulse on this geometry.

A future coupon, only after the importance gate changes, must preserve manufacturer dimensions, use a freely moving top with constant materials and native contacts, and log release angular velocity plus untouched persistence. No hidden hub/chuck drive, post-release velocity injection, friction switching, invisible constraint or toy rescaling is legitimate.

**Recommendation:** park this direction and use the acceptance-priority budget on better supported PDF candidates. Reopen only with a substantive ordinary-use outcome and measured geometry/inertia. A mechanically successful top video alone would not resolve the current acceptance objection.

Raw PR snapshots, manufacturer HTML/video and numerical assumptions are preserved in this folder; `gate.json` includes hashes. Manufacturer media are research evidence only, not submission assets or license-cleared redistribution.
