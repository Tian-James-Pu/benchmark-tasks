# What to borrow from STEEL

Checked 20 November 2026 UTC. All 81 gallery examples were catalogued; 27 relevant source, preset and license files were retained at commit `a8471c460ba7e36b1c4bb6e63eacd28e282b6e5a`. The recommendations below are based on source inspection and task-board PR comparisons, not only gallery videos.

| Priority | Borrow | Why it helps acceptance | What still needs proving |
|---|---|---|---|
| 1 | [Nut/bolt SDF geometry](https://steel.skild.ai/examples/nut_bolt_sdf/) and convergence diagnostics | Replaces captive, angle-triggered fastening with actual contact between free parts. | Novel useful operation, robot grasp and free re-entry; hardware-relevant loads. |
| 2 | [RJ45 latch geometry](https://steel.skild.ai/examples/contacts_rj45_plug/) and elastic release/retention pattern | Supports clips that really deflect, engage and resist pull-out. | Free alignment and real gripper access. The example cancels gravity, locks plug rotation and uses zero-friction contact. |
| 3 | [Load/friction tests](https://steel.skild.ai/examples/load_friction/), torsion and parallel-gripper linkage examples | Quickly exposes unsupported bracing, gripping and slip claims before expensive full-task runs. | Source-supported materials and actual actuator limits; Newton coefficients are not calibrated PhysX forces. |
| 4 | [Cable handling](https://steel.skild.ai/examples/franka_cable_ik_pick_place/) and cloth coupling patterns | Potentially enables deformable tasks beyond rigid assembly. | YAM contact grasp/release and solver integration. Franka performance and phase-dependent friction do not establish our robot's feasibility. |

## Reuse already tested

Original-size Factory M20 thread meshes were ported to the existing PhysX runtime. Three-turn advance and reverse tracked the 2.5 mm pitch closely: 2.4978 and 2.5016 mm/revolution. Free removal and final released retention passed. The first recontact turn reached only 2.1451 mm/revolution and failed the predeclared tolerance. These are compliant-force diagnostic results, not a robot-completed fastening task or a validated tightening torque.

See [the exact R2 evidence](thread-contact-r2-result.json). The shared IsaacLab environment was not replaced. Keep the project's Apache-2.0 notices and the Factory mesh's separate BSD-3-Clause terms with actual reuse.

## Direct task copies are weak candidates

Jenga overlaps accepted PR #2522, toy-train assembly overlaps pending #2849, sweeping has a duplicate precedent #2633, and card-house building received an importance rejection in #2862. Air hockey and beer pong also overlap established robotics benchmarks. A different scene or object color does not resolve those issues.

Use STEEL to strengthen a distinctive task's mechanism and validation. New tasks need their own importance, scoring, reproducible setup and full-score demonstration.

## Current submission priorities

Our live PR check still shows **one accepted, nine pending and seventeen final rejections**. The stronger pending revisions are can-lid recycling #2951, crutch ferrule #2609, dishwasher obstruction #2520 and infant carrier #2879, because their revisions address relatively specific review requests. This is a qualitative assessment, not an acceptance prediction.

The broader queue audit found no immediately eligible completed new submission. Eight known competing-task holds are now checked by the submission script. Lumber and floor-cover builds have substantial demonstration evidence but still have active holds and task-specific remaining issues. Ten surviving tasks have independently checked simulation/source/media evidence; that is not ten acceptances or ten physically validated tasks.

Detailed artifacts: [catalog](example-catalog.json), [adoption plan](adoption-plan.json), [acceptance priorities](../../acceptance-priorities-20261120.json), and [new-build queue review](../resume-20261120/ready-queue-screen/ranking.json).
