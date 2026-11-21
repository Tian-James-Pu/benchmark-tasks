# Air hockey and beer pong: neither clears the new-task gate

Read-only screen, November 20 UTC. No task claim, build, GPU run or publishing. Detailed evidence and pinned source hashes are in `screen.json`.

Both games are recognizable activities with genuine dynamic manipulation. Commercial miniature game hardware is legitimate at its own scale. The decisive problem is **existing robotics benchmark coverage**, not a blanket objection to toys.

| Candidate | Real hardware and contact path | Decision |
|---|---|---|
| Air hockey | Playcraft PSAH4001: 40×20×8in, 39.5×19.5in field, 19lb, supplied pushers/pucks and blower. Franklin also documents a real 20in tabletop product. YAM would need to grip the stock pusher, slide it along the field and strike a freely coasting puck. | NO-GO: the established Robot Air Hockey Challenge already tests dynamic puck strikes, goal entry, reactive play and actuator/table constraints. Smaller hardware and a preceding pickup do not establish a distinct central skill. |
| Beer pong | BPONG specifies an 8ft table, 40mm balls, 16oz cups with 92.1mm mouths, and water. Ordinary bounce shots are allowed. YAM would need a physical pinch, fast arm motion and timed jaw release, followed by native bounce and cup capture. | NO-GO: Fancy Gym BeerPong already tests ballistic release into a cup. Its current scorer permits table contact without requiring exactly one bounce. Making one bounce mandatory narrows the route without yet establishing a new core ability. |

Live board searches found no exact air-hockey or beer-pong task. That is not external novelty clearance. [2631](https://github.com/robocurve/benchmark-tasks/pull/2631) and [2833](https://github.com/robocurve/benchmark-tasks/pull/2833) were rejected for overlap with embodied-olympics-fun Paddle Bounce, showing that external dynamic-game benchmarks matter. [2481](https://github.com/robocurve/benchmark-tasks/pull/2481) is a pending revised can toss with a pedal-bin requirement; it is adjacent, not an asserted exact duplicate.

## What the Steel source actually establishes

Pinned at `a8471c460ba7e36b1c4bb6e63eacd28e282b6e5a`:

- Air hockey has a free, gravity-enabled puck with zero initial velocity; only six UR5 joints receive targets. However, the mallet is attached geometry on the wrist, not a gripped retail pusher. The source field is 880×1300mm; puck82×28mm, mass88.7g. Low friction substitutes for air-cushion behavior and is not calibrated to either retail product. A native YAM grasp and dynamic surface strike remain unproved.
- Beer pong has a free40mm,2.7g ball and drives seven Franka joints plus two fingers. The exported runner does not inject release velocity into the ball. Nominal starting ball/cup separation is about1.03m, shorter than regulation play. Peak commanded finger rate is0.625m/s, versus the installed YAM jaw limit of approximately0.146m/s. Cups are static world geometry; no liquid capture model was identified. Those differences prevent treating the gallery as a YAM feasibility demonstration.

An illustrative0.8m puck slide with assumed friction0.02–0.04 needs0.56–0.79m/s initial puck speed. That is a plausible contact-coupon target, not measured product behavior. A direct equal-height45-degree ball throw over1–2m needs3.13–4.43m/s before drag; a bounce trajectory needs a separate calculation. Joint velocity limits alone establish neither feasible tip speed nor release accuracy. No claim of mechanical impossibility is made.

Fancy Gym's implementation places the ball at the hand until release. Replacing that approximation with a real grasp would improve fidelity, but does not automatically make another cup toss a novel task. Likewise, ordinary mallet pickup does not remove air-hockey benchmark overlap. Further work should require a substantive distinct ordinary operation, not another count, target or starting layout.

Primary sources: [Playcraft hardware](https://www.playcraft.com/playcraft-sport-table-top-40-air-hockey-table.html), [Franklin manual](https://franklinsports.com/media/wysiwyg/Manuals/replacement-parts/88906-20in-air-hockey.pdf), [BPONG equipment and rules](https://bpong.com/wsobp/official-rules-of-the-world-series-of-beer-pong/), [Robot Air Hockey Challenge](https://air-hockey-challenge.robot-learning.net/), [challenge rules](https://air-hockey-challenge.robot-learning.net/game-rules), [Fancy Gym pinned documentation](https://github.com/ALRhub/fancy_gym/blob/da1bced0181ced8267247779c001509a2fc94635/docs/source/envs/fancy/mujoco.md), [pinned BeerPong source](https://github.com/ALRhub/fancy_gym/blob/da1bced0181ced8267247779c001509a2fc94635/fancy_gym/envs/mujoco/beerpong/beerpong.py).
