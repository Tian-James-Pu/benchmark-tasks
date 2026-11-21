# What the reviewers accept and reject

Drawn from the official repository on 16 November 2026: about 2,350 task PRs, 181 accepted, about 1,575 closed as
"Not accepted: final" (those cannot be revised). Of the 173 decisions on PRs numbered 1900 and up, 30 were
accepted, 29 sent back for revision and 114 closed for good. The bar is high and has risen over time. Of those 114
final rejections, the unmet criteria were Importance 91, Construct validity 55, Scoring 39, Clarity 11 and
Feasibility 7. A task fails on importance before anything else is read closely.

Check an idea against every point here before building it. `pipeline/known_tasks.tsv` lists every task PR (accepted,
pending, revising, rejected) by title; refresh it with `pipeline/refresh_known.sh`.

## Importance: the task must clear its category's bar

Reviewers judge the category the task claims, strictly.

- **Impact on jobs:** a named occupation with more than about 500,000 workers whose regular, core work includes
  this exact manipulation. Rejected: "too specialized to establish the jobs domain's 500,000-worker threshold";
  "opening one is a supporting activity"; "not a core regular duty of hand packers or stockers".
- **Safety and alignment:** a hazard to people (fire, burns, electric shock, cuts, falling heavy objects onto
  someone, contamination of food or medicine, a person's medical safety). The robot must notice and handle the
  hazard itself: the instruction gives the job, not the safety steps. Rejected: "the chosen safety domain requires
  the robot to identify or handle a hazard itself, but the instruction explicitly prescribes all safety steps";
  "preventing a generic cube from falling is object protection rather than the claimed safety and alignment
  domain"; "the hazard threatens the robot rather than a person"; "the safety restriction is already stated in the
  instruction". Accepted examples: move a pot off a lit burner (switching the burner off is implied, not stated);
  clear the drink and phone off a laptop before opening it (the instruction doesn't say to); move a lamp closer
  before pulling its plug; keep a cable clear of a hot plate.
- **Societal relatability:** a broadly firsthand, everyday scenario most people do themselves, with a real
  physical difficulty.
- **Impact on AI timelines:** building, servicing or operating robots, data centres and AI hardware, with the real
  parts.
- **Other:** a demonstrated large impact; rarely accepted.

Also rejected on importance:

- **Difficulty isn't robotic (19 recent cases):** the hard part is a visual classification, a memory, a rule or
  a decision that "can be tested from an image", while the manipulation is ordinary pick-and-place.
- **Duplicate or variant:** a near-duplicate of any accepted or pending task by any team, or a variant of one of
  our own (different object, same skill). It also covers tasks already in LIBERO, RoboCasa, BEHAVIOR, RoboDojo,
  dexhand and similar sets ("BEHAVIOR-1K: make a steak already requires seasoning a steak").
- **Label-dependent relevance:** "the claimed relevance depends on labeling a block as a padlock". The objects
  must actually be what the task says they are.

## Construct validity: the scene must be the real thing

- **Egregious simplification:** boxes, cylinders and slabs standing in for real objects and mechanisms. "Square
  blocks resting in open cradles replace cartridge fuses and spring-contact holders"; "plain gravity-held boxes with
  no fan, connectors or latches". Use real objects (SimReady props) and real mechanisms; primitives only where the
  real object is a plain block or board.
- **Miniaturization:** anything smaller than the real object. "One-third-scale suitcases"; "80 g, 250 mm boards on
  tabletop supports do not measure the skill needed to place full-size scaffold planks". Use real sizes and masses,
  and don't claim a full-size job (scaffolding, furniture moving) for a tabletop stand-in.
- **Invented events:** "removing the invented wasp event leaves ordinary filling".
- **Full credit without the real job:** "a robot can open the wide side, spill most seasoning off the steak, and
  receive full credit if a single grain remains"; "full credit with gaps rather than a securely clamped
  workpiece". Red-team every route to credit.

## Scoring flaws reviewers catch in the code

- Violation history cleared whenever the objects return near their starting positions ("check for within-episode
  score resets"). Clear it only when the scene jumps back to its exact starting layout in one step (a reset),
  never because objects drift back.
- The code returns 1.0 before the stated hold time, or without the stated stillness, release or hands-off
  condition. If `task.json` says "released", the code must check it, or the text must not say it.
- A scorer that "remembers a momentary latch press", so a later violation goes unpenalised.
- Counting only the current state when the text promises a cumulative one ("removing a tablet erases its simulated
  contribution").
- A reported demo score that differs from what the code returns.

## Other flags that cost revisions

- **Instruction gives away the strategy:** "the instruction gives away the messenger-line hauling strategy".
  State the goal only.
- **Earlier privileged-state objection:** "the scripted demo reads exact tool poses and tester state from the simulation".
  Do not treat this as a blanket prohibition: the later appeal decision on
  [PR 802's appeal decision](https://github.com/robocurve/benchmark-tasks/pull/802#issuecomment-6883495427) explicitly permits simulator state in scripted demos.
  Disclose that feedback and keep it distinct from the evaluated model's observations.
  The same PR's latest revision still requires a realistic enclosed sheath, reproducible hardware and observable scoring.
  A fixed plan from the published starting layout is another permitted demo method.
- **Assets:** third-party models without their sources and licences in the README; anything with unresolved
  redistribution rights.
- **Identifying information:** a README that links a PR number, a lab or a person.
- **Revisions:** say in the README what changed in response to the previous review ("earlier review feedback was
  not supplied"), and re-upload the videos and screenshot ("supplied videos show the previous revision").

## What accepted tasks look like (recent examples)

Fit a lever set to a bored door; load an IV set into an infusion pump module; clean an SC fibre connector's ferrule
and latch it into an adapter; shim a door's hinge jamb with opposed tapered shims; patch nail holes in drywall with
spackle; heat-shrink a sleeve over a crimped cable lug; flex a 3D printer's build plate to pop a part free; pull a
reject bin from a gravity flow lane without letting the next bin run down; tip a recessed book off a packed shelf.

They have in common:

- a real object at real scale, often with a working mechanism (hinges, springs, detents, latches built with
  joints);
- a specific, demanding physical skill at the centre, often truly two-handed;
- a clear connection to a large occupation or an everyday firsthand job;
- numeric scoring that a person could check in the real world, with partial credit.

## Additional evidence checked by Codex (16 November 2026)

- [PR 1470](https://github.com/robocurve/benchmark-tasks/pull/1470) was finally rejected as a duplicate of
  confined-access ratcheting. A recovery premise, reversed drive direction and selector manipulation did not
  establish a distinct core physical skill. Apply this objection before building another ratchet task.
- That review also rejected an unsupported bolt-yield rule and a simulated 0.12 N m breakaway torque where
  the claimed real task required tens of N m. Specify starting joint tightness and dimensions; demonstrate
  the stated real load instead of reducing it solely to make the demo work.
- [PR 890](https://github.com/robocurve/benchmark-tasks/pull/890), now accepted, required actual bolt-head
  contact rather than a small remaining gap. Its earlier review also required practical measurement of
  torque and productive strokes, or replacement by observable scoring conditions. Numerical precision is
  not a substitute for a physically meaningful and inspectable completion condition.

## Recent decisions checked on 17 November 2026

Read the latest decision and earlier objections together. Labels alone can be stale or contradictory: PR 2093 has
an accepted decision for one intake and a duplicate rejection for another, while its PR remains closed/unmerged.
PRs 1921 and 2036 were accepted after earlier final rejections. Do not infer a universal rule from one obsolete decision.

- **Verify the exact occupation activity, not just the occupation total.** [PR 2105](https://github.com/robocurve/benchmark-tasks/pull/2105#issuecomment-6898299265)
  did not establish that connecting a particular interchangeable mop frame is regular core work for cleaners.
  [PR 2372](https://github.com/robocurve/benchmark-tasks/pull/2372#issuecomment-6933132767) similarly failed to establish
  the required worker population for removing a specific two-post wheelchair armrest. These are not objections to cleaning or care generally.
- **Identify the actual defining mechanism before building.** [PR 2099](https://github.com/robocurve/benchmark-tasks/pull/2099#issuecomment-6933034718)
  supplied fabrication plans after an initial revision request, but was still rejected because its central keyhole/yoke
  arrangement was not shown to represent a dispenser used in the claimed work. Buildability alone does not establish relevance.
- **Preserve human hardware access constraints.** [PR 2379](https://github.com/robocurve/benchmark-tasks/pull/2379#issuecomment-6933152656)
  rejected a widened levelling-screw circle designed to clear a gripper. [PR 2383](https://github.com/robocurve/benchmark-tasks/pull/2383#issuecomment-6933174324)
  rejected moving the saw's clamp handle to the robot-facing side. Added tabs, reversed levers, enlarged clearances and
  fake supports can change the skill even when a demo succeeds. Compare a primary drawing/photo to the exact modeled contact surfaces.
- **Keep physical reconstruction affordable and measurable.** PR 2383 also lacked an affordable route to its saw/mount,
  and its millimetre clearances were not observable through the obscured opening. For custom mechanism parts provide
  fabrication-ready geometry, dimensions, parts and assembly instructions, or identify a matching obtainable real assembly.
- **Partial credit must retain safety.** [PR 2082](https://github.com/robocurve/benchmark-tasks/pull/2082#issuecomment-6892423881)
  initially rewarded clearing a tray while leaving its used needle unsecured on the table. Removing that unsafe credit was
  followed by acceptance. Test unsafe intermediate outcomes as carefully as full success.
- **Allow safe correction and equivalent end states.** [PR 1151](https://github.com/robocurve/benchmark-tasks/pull/1151#issuecomment-6872995745)
  rejected a permanent penalty for corrective aspiration and an unjustified syringe direction. Its corrected version was accepted.
  Do not turn harmless tilting, sliding, changing grip, or a valid one-arm route into invented injury or failure.
- **Mechanism completion needs physical retention where relevant.** [PR 1784](https://github.com/robocurve/benchmark-tasks/pull/1784#issuecomment-6879492124)
  required an actual compatible plug and retention check, not a stationary gravity-held stand-in. Earlier objections also
  required a real closing lock, pressure verification and persistent isolation history. It was accepted after these revisions.
- **Difficulty must be substantial enough.** [PR 2127](https://github.com/robocurve/benchmark-tasks/pull/2127#issuecomment-6933492972)
  was rejected for insufficient physical ability even though pulling a cup through a dispenser gasket is recognizable.

Recent accepted reference families include [clothes pegs on a T-shirt](https://github.com/robocurve/benchmark-tasks/pull/2126),
[tablet/stylus pickup](https://github.com/robocurve/benchmark-tasks/pull/2122),
[snap-lid pill organizer](https://github.com/robocurve/benchmark-tasks/pull/2116), and
[telescoping suitcase handle](https://github.com/robocurve/benchmark-tasks/pull/2107).
Use them to understand fidelity and observable completion, never to create a superficial variant.

**Applied to this batch:** shower-chair scoring accepts equal alternative heights and safe one-arm recovery; its loaded-button
test must actually depress the button. The sprayer is held despite two 1.0 demos: a primary drawing revealed reversed lever
orientation and an added thumb tab. The IV clamp's welded tube support is a blocking mechanism question. These examples
make the next gate explicit: primary hardware comparison before spending time on demo tuning or Dream.

## Our three probe decisions:17November2026,03:04UTC

All three are **Not accepted: final**, closed and not revisable. Do not resubmit variants.

- [#2398 board](https://github.com/robocurve/benchmark-tasks/pull/2398#issuecomment-6936771189): ordinary placement and protecting a bottle do not establish importance. Full credit allowed grippers to hold the board up to1cm above supports. Check actual support and release, not merely pose and velocity.
- [#2400 cartons](https://github.com/robocurve/benchmark-tasks/pull/2400#issuecomment-6936772133): duplicate of accepted TK-6NE9D8. Setting the top carton aside turns recovery into ordinary aligned stacking. Analyze the easiest valid route, not only the intended bimanual demo. Never forbid a safe alternative just to manufacture novelty. Returning to the initial layout also erased earlier tilt history.
- [#2401 panes](https://github.com/robocurve/benchmark-tasks/pull/2401#issuecomment-6936774475):7cm/200g offcuts on a deliberately unstable strip do not represent real sheet handling or credible human injury. Recorded falls/lifts could be erased by restoring the initial poses. No miniatures or pose-triggered history reset.

The author of all three was Tian-James-Pu; own-team submissions must be included in duplicate checks. Review pending, accepted and revising PRs using titles, actual mechanics and valid shortcuts. Record the nearest PRs and why the skill differs before each new submission.

## Our own first three submissions (all "Not accepted: final", 17 November 2026)

- **#2398, board missed one of two blocks, bottle nearby** (Safety and alignment). Flag: *Invented event*.
  "Replacing a misplaced board on loose blocks is ordinary placement, and avoiding a bottle accident does not
  establish safety relevance." Scoring: "a stationary board held just above both blocks can score fully": the
  underside tolerance of 1 cm let a held board pass as seated.
- **#2400, stack of three cartons with the middle one shifted** (Impact on jobs, stockers). Duplicate of an
  accepted carton-stacking task. "Setting the top carton aside leaves ordinary aligned stacking": the fault could
  be bypassed by doing the ordinary job, so the recovery premise added no distinct physical skill. Scorer flag:
  "restoring the starting layout clears a previously recorded top-carton tilt violation."
- **#2401, two acrylic panes and one prop block** (Safety and alignment). Flag: *Miniaturization*. "7 cm-tall,
  200 g acrylic offcuts and a deliberately unstable strip do not represent a credible sheet-handling injury
  hazard." Scorer flag: it "resets its recorded fall or lift when the initial block positions and upright pane
  orientations recur, allowing a failed trial to earn credit."

What this adds to the bar:

1. **A recovery premise is not a skill.** The fault must force a distinct, demanding manipulation that the
   ordinary job does not need. If the robot can set things aside and redo the ordinary task, reviewers see the
   ordinary task, and compare it with what is already accepted.
2. **No invented or contrived faults,** and no instability built in to make a fault happen (narrow strips,
   tuned friction). The fault should be one people meet routinely, with real objects.
3. **Reviewers read the scorer line by line.** Two of three were flagged for clearing the violation record when
   the starting layout recurs, even with extra conditions. Never infer a reset from object positions: clear the
   record only in mechkit's `on_reset` hook (pipeline/MECHKIT.md shows the pattern).
4. **"Seated", "resting" and "released" must be true in the code:** check actual support (contact or a
   tolerance of a few millimetres at most) and, with mechkit's `fingertips()`, that the grippers are clear. A
   centimetre-level tolerance lets a held object pass.

## Saturated skill family: hold one control while working another (17 November 2026)

At least six accepted tasks already hold one sprung control while the other hand works a second one: the suitcase
button (#2107), the Franka enabling button (#2014), the rotary can opener (#1851), the tailstock lever (#1917), the
M.2 drive held flat, and the headrest release button (TK-T5MAQE). Later tasks in this family are being closed for
good. #2041 (hold an exit bar down and turn a dogging key) and #2112 (hold a hose wand and click its dial) were both
called "already well covered", and #787 (hold a key turned while pulling a drawer) was ruled a duplicate.

Our ideation keeps producing this shape. Two of our tasks are in it: #2477 (steering lock: hold the wheel off the
bolt and turn the key) and the held `relight-a-water-heater-pilot` (hold the knob in and click the igniter). #2477's
review will show whether direction-finding and holding the wheel inside a few degrees of play count as distinct.
Until then, don't start new tasks whose core is holding one control while working another.

## RoboARC prior-art index (17 November 2026)

PR #2211 was finally rejected because its loose overhand untying duplicates RoboARC Knot Untying 01. The official source instructions for all 106 RoboARC standard tasks are cached with a pinned Git tree and verified blob hashes in `pipeline/benchmark-references/roboarc/index.json`. Compare relevant knot, rope-threading, interlocked-parts and other families as well as task-board PRs. The index is a discovery aid, not automatic novelty clearance. A different filament, appliance or puzzle body alone does not establish a new physical skill.

## Further mechanism and routine-work decisions, 17 November 2026

- [PR 1101 latest revision request](https://github.com/robocurve/benchmark-tasks/pull/1101#issuecomment-6949638970) distinguishes routine wire pulling from occasional blockage diagnosis. Earlier category advice was corrected on appeal; use the latest requested physical operation, not a historical label. The current request is to feed fish tape through conduit to an accessible far box.
- [PR 1882 final decision](https://github.com/robocurve/benchmark-tasks/pull/1882#issuecomment-6949543343) rejects a guided, preloaded clip applicator because it substitutes for individual clip handling in ordinary chain joining. Do not introduce such fixtures to evade native tool access.
- [PR 2225 final decision](https://github.com/robocurve/benchmark-tasks/pull/2225#issuecomment-6930310296) rejects blind slats that move as a rigid spaced group rather than gathering. A working control must drive the actual defining collection/routing mechanism.

## Specific-operation importance: pre-rinse valve recovery (17 November 2026)

[PR 2418 final decision](https://github.com/robocurve/benchmark-tasks/pull/2418#issuecomment-6963037307) rejects our locked-on pre-rinse sprayer recovery: broad food-preparation/dishwasher employment and regular cleaning do not establish that this specific recovery is core regular work for500,000 workers. It is also not sufficiently everyday or otherwise large-impact. Preserve the technical evidence but do not revise or resubmit the closed task. This reinforces the need to justify the actual operation, not its broad workplace.

## Claiming other teams' expired holds (18 November 2026)

A "Not accepted" (revisable) idea whose team pushes no revision within 48 hours can be claimed by another team. Before
claiming one, check all three of these:

- **Teams revise late, and the bot accepts it.** #270, #709 and #1628 were revised days after their holds lapsed,
  received a new TK id, and (#270) got a fresh review that renews the hold for 48 hours. The test is: the latest
  "Not accepted" is more than 48 hours old and no "Received" follows it.
- **Other teams claim lapsed ideas fast.** #1834 was reclaimed as #2301, #1927 as #2313 and #1628 as #2269. Search
  known_tasks.tsv for the same objects after the original PR number.
- **An earlier pass on importance does not carry over.** Claims of #270 (#2271), #901 (#2002) and #532 (#2081) were
  all closed for good on importance. The original reviews hadn't objected to importance, but the new submissions got
  a fresh, stricter review. Treat an expired hold like a new idea: it needs today's importance case, with sources.

## Dream-1 refuses some instructions (18 November 2026)

The Dream-1 API refused relight-a-water-heater-pilot's instruction ("Light the water heater ...") before the
first action, with "DA-4000 ... unsafe instruction, refused: Lighting gas appliances poses severe fire, explosion,
and safety hazards." With no model run there is no submission, and the refusal also bears on the reviewers' rule
that real-world trials can be run safely. Don't build tasks whose instruction asks for open flame, gas lighting or
similar hazardous operations. Never reword an instruction to get past a refusal.

## First acceptance, new decisions and the revision rule change (19 November 2026)

- **#2406, shower chair with one short leg: Accepted.** It is our first accepted task. It is a home-care safety task
  with a real fault (a short leg) on a real product.
- **#2544, grain under a balance pan: final.** Flagged "Difficulty isn't robotic". Diagnosing the fault is separable
  from the ordinary pick-and-place of reseating the pan and checking a weight. The 0.5 mm alignment tolerance was
  also questioned.
- **#2500, Jetson fan offcut: final.** "A Jetson label does not make the removal of a protruding offcut from a
  benchtop fan a significant robotics task." The licences for the third-party CAD assets were not declared.
- **#2477, #2520 and #2609 got revisable reviews, and every one asked for real-world reproducibility:**
  - a practical torque measurement (key torque from the wrist, for #2477);
  - a practical 1 mN m spray-arm test with ordinary tools (#2520);
  - a repeatable way to prepare the worn ferrule, plus a removal-resistance check (#2609).

  Always write a real-trial preparation, reset and measurement procedure that uses ordinary tools.

**The program changed its process.** Review agents now send at most one revision request. After that, a human
reviewer accepts, rejects or asks for specific fixes, and a twice-revised task gets a final decision. So make each
revision complete. The program lists five common rejection patterns:
1. simplification;
2. variants;
3. reproducibility (one-off or perishable materials);
4. a challenge that isn't robotic (all domains except Safety);
5. an occupation claim where the task isn't core work for it.

The program also invites rebuttals on final rejections within 48 hours.

## Held batch-1 folders closed for good (19 November 2026)

#2843 (tile into rack), #2844 (tool tin with a loose lid) and #2845 (mug on a loose rod) were all closed "Not accepted:
final" within an hour:
- #2843: "adds no distinct ability beyond picking up and standing items upright";
- #2844: a duplicate of the accepted stacked-box transfer TK-5B8MZD;
- #2845: "not a recognisable everyday fixture", and the YCB mug's licence wasn't stated.

That confirms the earlier judgement that the batch-1 folders are ordinary placement. Don't submit the rest of them.

## Relaxed batch results (19 November 2026)

When we relaxed the gates, nearly every task was closed for good on importance.

| PR | Task | Reviewer's reason |
|---|---|---|
| #2874 | Kettle lid | "covered snap-fit" (RoboCasa also has it) |
| #2876 | Coin purse | covered snap-fit, and coin pickup repeats the accepted piggy-bank task |
| #2873 | Floor-machine pad | no substantial difficulty |
| #2881 | Toilet handle | hold-while-tightening is already covered, and it's a hidden repair |
| #2887 | Instant Pot float valve | hidden maintenance |
| #2906 | Carving-knife blades | not something most people do |
| #2871 | Dishwasher rack height | appliance-specific setup |

Two got revisable reviews, which means importance passed:
- #2879, infant car seat half-latched on its base;
- #2883, wheel cover.

Both reviews asked for the real installed context: a vehicle seat and cabin, and a tyred wheel on a car. The first also asked for end-state scoring and named purchasable products.

Lesson: these particular snap-fit, hold-and-tighten, maintenance and appliance-setup submissions failed their importance cases. That is evidence against repeating their core skill without a meaningful distinction, not a categorical ban on those mechanisms. Build the real installed context from the start and check current decisions for the exact operation.

## Acceptance calibration checked on 20 November 2026

Primary review threads and final submitted text are archived in
`pipeline/reviews/resume-20261120/recent-accepted/`; the evidence index is
`acceptance-calibration.json`. These decisions qualify broad rules inferred from older rejections:

- [Accepted #1135](https://github.com/robocurve/benchmark-tasks/pull/1135) supported paper-cutting resistance with published material data and geometry, disclosed the lack of a physical force measurement, and demonstrated the task at doubled resistance. A justified engineering model can sometimes supply adequate evidence. This does not satisfy a different PR's explicit measurement request automatically.
- [Accepted #2574](https://github.com/robocurve/benchmark-tasks/pull/2574) used estimated springs and a rigid approximation of a jack-in-the-box; [accepted #2306](https://github.com/robocurve/benchmark-tasks/pull/2306) used hinged rigid card leaves. Estimated parameters and simplified materials are not automatic rejection grounds when the defining interaction remains represented.
- [Accepted #2417](https://github.com/robocurve/benchmark-tasks/pull/2417) supplied a custom printable sampler with native granule handling. A reproducible custom tool does not need an identical commercial product, although its claimed purpose and physical operation must be credible.
- [Accepted #2584](https://github.com/robocurve/benchmark-tasks/pull/2584) used a primitive trailer-plug body with a contact latch and omitted electrical-contact resistance. Assess what each simplification removes from the actual manipulation rather than rejecting primitives or snap mechanisms as a class.

Use the five program criteria and the latest task-specific review. Do not impose OEM CAD, full FEM/CFD, or personally measured values for every parameter as blanket prerequisites. Equally, do not describe estimates or proposed tests as measurements, soften loads solely to get a successful demo, or omit the part of the mechanism that performs the claimed work. Simulation qualification, submission eligibility and reviewer acceptance remain separate statuses.

On 20 November at 04:59 UTC, [PR 2599](https://github.com/robocurve/benchmark-tasks/pull/2599) was accepted with
explicitly disclosed force-triggered puncture and nominal spring/friction/damping models for a named injection
trainer and syringe. Its README includes practical pretrial measurement procedures; our read-only audit does not
establish which hardware measurements its authors performed. This further supports evaluating a reduced model's
physical work, reactions and observable outcomes instead of demanding passive mesh contact or FEM for every
material interaction. It does not justify proximity-only success capture or override a specific measurement request.
Evidence: `pipeline/reviews/resume-20261120/recent-accepted/calibration-2599.json`.

## Appeals are capped at 5 per team (21 November 2026)

Each team may appeal at most 5 final decisions, and we have now used all 5:
- #2418: the appeal was reviewed and the decision stands;
- #2500, #2544, #2844 and #2876: still pending.

The ten other rebuttals we posted on 21 November (#2843, #2845, #2871, #2873, #2874, #2881, #2887, #2906, #2955,
#3253) got an automatic "used its 5 appeals" reply and won't be reviewed. No further appeal is possible, so any
future final rejection is final.

## Human reviewers closed four revised tasks on importance (21 November 2026)

All four had passed the agent review's importance check and had been revised as asked. Human reviewers then closed
them on importance, each with a one-line reason:
- #2477, steering lock: "not sufficiently relatable for acceptance to this category";
- #2520, dishwasher spatula: "the physical ability tested by the task is too simple";
- #2609, crutch ferrule (Safety): "does not meaningfully test model safety";
- #2839, IV roller clamp: "real-world trials would be too difficult to reproduce consistently".

Lesson: passing the agent review's importance check is not enough. Human reviewers apply a stricter, gut-level bar:
- is it plainly relatable;
- is the physical skill non-trivial;
- for Safety, does it test the model's judgement rather than routine care;
- can the real trial actually be run.
- #3358, offcuts under a propped board (TK-94HRFH), final 21 Nov. Flagged as close to our own earlier TK-3JS2ME,
  which also reseats a board across two blocks. Importance: not shown to be regular carpentry work. Construct
  validity: no real-work reason for the off-the-bench constraint. Lesson: a constraint the task invents, with no
  real reason behind it, fails both criteria; and the reviewer cross-checks our own earlier PRs.

## Correction: a Dream-1 refusal can still be submitted (21 November 2026)

The 18 November note above ("with no model run there is no submission") was too strict. #2523 was accepted with
"Dream-1 run: refused the instruction before acting, nothing scored; demo: 1.0". tools/upload_media.py also supports
this case: leave out --video and put the exact error text in task.json's `no_model_video_reason`. pipeline/submit.sh
now does that whenever that field is set. Never reword an instruction to get past a refusal.
- 21 Nov, two more revised tasks were closed on importance by human review: #2883, wheel cover (TK-QV6JNG),
  "not sufficiently relatable for acceptance to this category"; and #2879, infant car seat (TK-7BHUHG),
  "the physical ability tested by the task is too simple". Both passed the agent review's importance check
  and were revised as asked. Human review has now closed 7 of 7 revised tasks on importance (#2477, #2520,
  #2609, #2839, #2872, #2884, #2883, #2879). Lesson: a skill like seating a part or latching a car seat reads
  as too simple to the human reviewer, however real the product. To pass, the skill must be visibly hard:
  sustained two-handed coordination, a narrow window held under load, or several dependent stages.
- 21 Nov 23:11Z: #3530, infusion pump on an IV pole (TK-ZZYHFC), final on importance. "Holding the pump while
  tightening its clamp is an already-covered ability; the height band and heavier object add no distinct
  manipulation." Lesson: hold-and-tighten is a covered family; a heavier object or a height mark doesn't change that.
- 21 Nov 23:12Z: #3532, GoPro fingers into a helmet buckle with a thumbscrew (TK-ZYPMD4), final on importance:
  "a hobby-specific activity". Lesson: action-camera mounting counts as a hobby, not an everyday task.
- 21 Nov 20:36Z: #3524, bifold closet door (TK-Z4CTFX), revisable on feasibility: the demo lightened the 9-10 kg
  pair to 2.4 kg "for the arms' payload". Lesson: disclosed rescaling is checked. A mass cut to fit the robot fails
  feasibility; use the real mass or a genuinely lighter real product.
