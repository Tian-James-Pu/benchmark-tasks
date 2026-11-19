# Revised-then-accepted study (revised)

Scope: the 70 highest-numbered accepted PRs (2065-2534) were inspected; 25 had a "Not accepted" (revisable) review before acceptance, and one (2306) was accepted after a rebuttal of a final rejection. Also read: reviews of our revisable 2879, 2883, 2520 and 2609, plus REVIEW_BAR.md's 19 Nov section. Raw threads are in prs/<n>.txt next to this file.

## What the first review asked, and what fixed it

| PR | Task | First review asked | Fix that got it accepted |
|---|---|---|---|
| 2396 | desk lamp beam | scorer permanently zeroes transient base/notebook movement; the rule is end-state | check at end only; also a laser-safety flag |
| 2365 | seatbelt twist | open bench lacks seat, pillar, belt path | vehicle-representative install, affordable repeatable setup, new demo |
| 2355 | lining caught in zip | no reproducible way to create the snag; bag contents/weight missing | prep protocol: insertion depth plus a repeatable resistance check |
| 2334 | vacuum bag swap | jobs claim (ancillary upkeep); no commercial vacuum; no clean bag fill | category changed to Societal; commercial vacuum named; clean fill recipe |
| 2320 | cling film watermelon | perishable fruit with no prep, temperature or stand-in | reusable stand-in or prep/storage protocol; pad protection |
| 2306 | birthday card tent | final: duplicate of RoboWits Stand Pages | rebuttal within 48 h: quoted the paper, showed a different skill; accepted |
| 2291 | pressure cooker lid | gasket check impossible on a closed cooker | 1st fix failed (tester handling created the evidence); 2nd: end-state readings before anyone touches it (caliper, photo, steel rule), with manual citations |
| 2282 | drive-thru window | commercial window too expensive | low-cost full-size slider and closer from ordinary parts |
| 2280 | unfold wheelchair | jobs claim not core to CNA transport | category changed to Societal |
| 2277 | zip-top bag | jobs claim; spill penalty vanishes when closed; movement limits penalise success | 2nd review (new): cells closed by position/jaw gap without contact; fixed with contact plus press |
| 2267 | brake pads | open bench corner lacks wheel well and suspension | wheel-well/suspension collision geometry, new demo |
| 2259 | lancet loading | not familiar enough for Societal | category changed to Other (glucose-testing scale) |
| 2253 | strike a match | 2 N strike unreadable with a kitchen scale; flame near pads | video-observable ignition steps; McMaster heat shields on the pads |
| 2245 | control box close | one broad cover touch latches both catches; terminals not visible | per-catch presses with real force; visible terminal row and wiring |
| 2242 | wristband scan | custom sprung elbow fixture has no build plan | drawings and parts for bearing, spring, stop |
| 2194 | napkin roll | full credit without release or table support | resting plus grippers clear for an interval |
| 2187 | bracelet clasp | proximity capture pins points to grippers | contact grasping; 2nd review (new): clasp detent has no source, so source or fabrication files |
| 2186 | tensile specimen | jobs workforce not established; stand and grips too costly | Other category; ~$314 parts list and STL build files; point-by-point table reply |
| 2165 | tablet back support | custom hinge/magnet support not build-ready | fabrication files or an equivalent product |
| 2147 | capsule machine | custom fixture has no build drawings; not recognisable as a coffee machine | drawings, or a commercial machine and a matching sim |
| 2139 | French press | pre-plunge on stand earns credit; orientation rule too strict | credit only for travel while seated; 2nd review (new): coffee/water prep, temperature, spill protection |
| 2133 | tyre tread gauge | named tyre is new (10.5/32), task needs 5/32 everywhere | available tyre, groove depths set at setup, targets updated |
| 2100 | drill chuck key | irreversible zero for lifting/tipping/moving the drill | removed; kept only fall or leave-workspace |
| 2082 | needle tray | 0.25 tier for leaving a used sharp on the table | unsafe partial states score 0; reset the container |
| 2065 | plumb bob | pencil mark appears from proximity, without contact | contact plus light marking force |

Our open revisable reviews follow the same patterns: 2879 (car seat and cabin, commercial carrier and base, end-state scoring), 2883 (tyred wheel on a car, clip seated on the wheel), 2520 (ordinary-tools spray-arm check), 2609 (worn-ferrule prep and removal-resistance check).

## How often each request came up (26 accepted PRs, plus 4 of ours)

1. **Real-trial reproducibility: purchasable product, build files, consumable prep (14).** 2355, 2334, 2320, 2282, 2242, 2165, 2147, 2133, 2186, 2187(2nd), 2139(2nd), 2879, 2609, 2520. Reviewers want a named commercial product with a price and a link under a few hundred dollars. Failing that, they want dimensioned drawings or STL files with springs, magnets and hinges specified. Perishables need a recipe with mass, temperature and timing, or a reusable stand-in. Faults (snag, worn part) need a measured way to create them and a resistance check that confirms they are the same each trial.
2. **Scoring must be end-state and must not over-penalise valid routes (9).** 2396, 2100, 2277 (transient movement zeros removed), 2879 (credit must not latch after a 3 s hold), 2194 (resting plus released), 2139 (exploit: pre-plunging), 2082 (unsafe partial credit), 2291 (evidence must not be created by tester handling). Penalties for real harm (a spill, a drop, an exposed sharp) must persist. Harmless lifting, tilting or regrasping must not zero a trial.
3. **Mechanisms must be physically driven, not triggered by proximity (6).** 2187 (proximity grasp), 2065 (proximity mark), 2277 (closure by jaw gap), 2245 (broad-area latch), 2883 (clip height, not seated), 2291 (a contact force the real trial can't see). Every state change needs contact plus a realistic force at the actual feature.
4. **Real installed context (4).** 2365 (seat and pillar), 2267 (wheel well), 2879 (vehicle seat and cabin), 2883 (mounted tyred wheel and bodywork). If the object normally sits installed, model the surroundings that limit access, from the start.
5. **Category mis-claim, fixable by relabelling (5).** 2334, 2280, 2277 (jobs to Societal), 2259 (Societal to Other), 2186 (jobs to Other). Choosing the category that fits saves a round.
6. **Real-world measurability with ordinary tools (5).** 2253 (force on a hand-held object), 2291 (a check hidden inside a closed appliance), 2520 (1 mN m), 2242 flag (peak angular speed from video), 2065 flag (3 mm steadiness). Use photo, ruler, caliper, feeler gauge, hex key, spring scale or video frame, on a surface that is visible at the end state.
7. **Rig protection and safety flags (7).** 2253 heat, 2277 blade, 2259 lancets, 2282, 2139 and 2320 liquid or produce, 2267 grease, 2396 laser. Each one adds a sentence on protecting the shared rig.
8. **Recognisability (2).** 2245 (terminals and wiring visible), 2147 (the appliance should read as a coffee machine).
9. **Licences (from REVIEW_BAR).** 2500 (third-party CAD) and 2845 (YCB mug) were flagged. Name every asset source and licence in the README.

## Process lessons

- In 4 of 26 cases (2277, 2187, 2139, 2291) the second review raised a new issue. Since 19 Nov, agents send at most one revision request, then a human decides. Pre-empt the second-round items too: contact-driven mechanisms, consumable prep, source of the custom mechanism, and evidence the tester can't create.
- Accepted revisions answered point by point in a PR comment, quoting each request with the change and the file (2186's table; 2139, 2253 and 2291 had numbered lists). They re-uploaded media when the demo changed and said so when it didn't.
- A rebuttal works when it is precise. 2306 overturned a duplicate finding by quoting the prior-art paper's actual goal and mechanism.
- Turnaround was 1-12 h after review, well within the 48 h hold.

## Up-front checklist for our tasks

- [ ] A named, purchasable product (model, price, link), or dimensioned build files covering every spring, hinge, magnet and detent; a real rig for a few hundred dollars at most.
- [ ] Installed context: the surrounding structure that limits access (seat, cabin, wheel well, cabinet, bodywork) in both the sim and the real setup.
- [ ] Fault preparation: a measured procedure to create the starting fault, a consistency check (insertion depth, pull-out force with a spring scale) and a reset.
- [ ] Consumables: a recipe (mass, temperature, time), or a reusable stand-in; rig protection stated.
- [ ] Scoring: read at the end state, before anyone touches the scene; resting on real support with grippers clear; real-harm penalties persist; no zero for harmless lifting, tilting or regrasping; no credit for unsafe partial states.
- [ ] Every real-trial check uses ordinary tools on a surface visible at the end (no hidden interfaces, no forces read on hand-held objects).
- [ ] Sim mechanisms: contact plus a realistic force at the actual feature; no proximity captures or broad-area triggers; the "seated" check must be true in the code.
- [ ] Red team: test the easiest exploit routes (doing the step out of order, pre-loading, holding instead of placing) and check they score 0.
- [ ] Category: pick the one that honestly fits (Societal for familiar chores; Other for niche lab or medical; jobs only with core, regular work for at least 500k workers).
- [ ] Hazard elements visible and the appliance recognisable; asset licences in the README.

## Best examples to copy

- **2291 (pressure cooker), the strongest real-trial scoring writeup:** end-state readings before anyone touches the cooker, a caliper that "can fail a reading but never create one", manual page citations, product price links, and a scoring kit for about $30.
- **2186 (tensile specimen):** a point-by-point revision table, a parts list costed at about $314 against a $2,500 commercial stand, STL files, a geometry check and an assembly guide.
- **2139 (French press):** an exploit fix verified with control runs (pre-pressed scores 0.5; a fast insert scores 0.0), plus a consumable recipe with tolerances.
- **2253 (match):** observable video steps replacing a force threshold; part-numbered heat shields.
- **2365 and 2267:** what an "installed context" revision looks like.
