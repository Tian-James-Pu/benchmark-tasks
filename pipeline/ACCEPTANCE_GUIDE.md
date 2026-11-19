# Acceptance guide: what an accepted task satisfies (19 November 2026)

Sources: the decision threads on about 50 accepted PRs (26 of them revised before acceptance), all 86
"Not accepted: final" PRs from #2755 to #2945, and our own PRs from #2398 to #2955. About 8-12% of decided PRs
are accepted. **Importance was unmet in 86 of 86 final rejections.** Every revisable review we got passed
importance and failed only construct validity, scoring or clarity (#2477, #2520, #2609, #2879, #2883).
Importance is the only gate that closes a PR for good, so settle it before building anything.

## 1. Category: claim one honestly and prove it in `context`

| Category | Bar | Evidence accepted tasks gave | Rejected for |
|---|---|---|---|
| Societal relatability (26 of the last 60 accepts) | Most adults have done it themselves, and it is recognisable in a 10 s clip | "Most adults have done it", plus the maker's own instructions quoted (#2524, #2158, #2493, #2342) | hobbyist (#2807), specialist home care (#2782), appliance setup or hidden maintenance (#2871, #2887, #2919, #2921, #2886), a puzzle (#2862) |
| Impact on jobs (13) | More than ~500k workers whose regular core work is **this exact operation** | O*NET/SOC code, the task statement quoted, worker count, frequency ("several a day": #2492, #2267, #2376) | "counts everyone in the listed occupations" (#2909); ancillary, prerequisite or occasional chores (#2863, #2894, #2767, #2895); our #2418 |
| Safety and alignment (8) | An immediate hazard to a **person**, visible in the scene, **not stated in the instruction**; cite a standard or regulator | FDA CPG 555.425 (#2523), CDC/AAP (#2399), the maker's guidance (#2491, #2511), OSHA (#671) | the hazard stated in the instruction (#2882), a hazard that only arises later (#2874), object protection or an invented event (our #2398) |
| Impact on AI timelines (3) | Building, servicing or running robots, data centres or AI hardware, using the real parts | real parts at real size | "A Jetson label does not make..." (our #2500) |
| Other (6) | A published procedure and a demonstrated large impact | a lab SOP or ASTM standard (#2417, #2186, #2259) | custom fixtures on oscillating carriers (#2756-#2803) |

A wrong category costs a round: #2277, #2280 and #2334 were moved from Jobs to Societal, #2259 to Other.
Default to Societal unless an O*NET task statement names the operation. Safety is the only domain
where the difficulty may be judgement rather than manipulation (#2523 passed with Dream-1 refusing).

## 2. One knack, in one sentence, with numbers

Pattern: *"X springs back, hides its state or won't stay put, so one hand does A while the other does B; done
naively, Y happens."* Give numbers: an 8 N wiper spring (#2507), 4.5° dial tolerance (#2158), ±0.3 mm block
spread (#2522), 6 mm filter play (#2492), 2.8 mm buttons 2.7 mm apart (#2524).
- Two arms in different roles (one holds or steadies, the other acts), or two simultaneous forces (#2524).
  A single arm passes only with a fine knack (#2527 wand, #2354 night light, #2158 dial).
- 2-5 task objects (#2342 and #2354 have two). Very short instructions are fine ("Seal the bag.", #2277).
- **Already covered; don't build these:** snap-fit, detent or latch catch (#2874, #2876, #2873, #2514);
  holding one control while working another (#2041, #2112, #2881, #787: at least six accepted already);
  pick-and-place or standing an item upright (#2843, #2815, #2823); stacked-load transfer (#2844); carton
  flaps (#2893); sweeping (#2944); abrasive strokes (#1374 covers sanding); food turning; crank, valve,
  switch or single-lever operation (#2802, #2918).
- **Not robotic:** the hard part is seeing, remembering or choosing, followed by ordinary pick-and-place
  (#2918, #2896, #2828, #2815, #2818, #2788, our #2544).
- **A recovery premise is not a skill.** If the robot can set the fault aside and do the ordinary job, it is
  the ordinary job (our #2400). The "recover / jammed / stuck" framing has a 9% acceptance rate (our #2871,
  #2881, #2887, #2906 failed on it).

## 3. The real thing: product, scale, mechanism, installed context

- **Name one purchasable product:** brand, model, mm, g, price and link, for a few hundred dollars at most
  (#2524 iPhone 12, #2522 Hasbro A2120, #2158 Master Lock 1500). #2859 failed at £318; #2186 replaced a
  $2,500 stand with a $314 parts list plus STL files.
- **Full scale.** Miniatures are fatal and reviewers write them into importance: "the 17 cm-wide miniature
  changes the reach, forces and manipulation" (#2903; also #2897, #2895, #2908, #2798, our #2401).
- **Model the defining mechanism with real resistance.** These were fatal: a clamp that applies no force (#2905),
  a camlock with no cams (#2898), a vise with no moving jaw (#2826), a printer with no paper end (#2894),
  cylinders standing in for seedlings (#2820).
- **Contact-driven state changes.** No grasps or closures triggered by proximity or jaw gap (#2187, #2277,
  #2065); no broad touch that latches two catches (#2245); "seated" means seated, not "at the right height" (#2883).
- **Installed context from the start.** Reviewers sent tasks back for a wheel arch and suspension (#2267),
  a seat and pillar (#2365), a vehicle seat (#2879), a tyred wheel on a car (#2883). Keep the human access
  geometry: no widened clearances or moved handles (#2379, #2383), and no robot-only aids such as an added
  handle or a 110 mm grip (#2895, #2865).
- **Assets:** SimReady props; every source and licence in the README (flagged on #2500, #2845).

## 4. Scoring

- 3-5 discrete levels (0/0.25/0.5/0.75/1.0), with "Possible scores" listed. Every condition is numeric and
  checkable with a ruler, photo, feeler gauge, kitchen scale or video frame.
- **Read the end state before anyone touches the scene**, so the checker can't create the evidence (#2291
  took 3 rounds: "can fail a reading but never create one").
- **No force, torque or impulse rules** (16 PRs, among them #2782 "50 N", #2865, #2477, #2520 "1 mN m"); no force read
  on a hand-held object (#2253); no sub-millimetre or hidden checks (#2544, #2887).
- **Full credit means:** resting on real support (a few mm at most), grippers at least 3-8 cm clear, everything
  still for 1 s, all checked in the code (#2194, our #2398 with its 1 cm tolerance).
- **Harm persists:** record violations across steps and clear them only in mechkit's `on_reset` (#2400,
  #2401). Credit must not stick after the hold (#2879). Unsafe partial states score 0 (#2082). Harmless
  lifting, tilting or regrasping is not zeroed (#2100, #2396, #1151).
- **Red-team the cheapest route** and confirm it scores 0 or only the intended partial: tipping instead of
  scraping (#2945), sliding the pan instead of brushing (#2944), pre-pressing the plunger (#2139),
  setting the top carton aside (#2400), a layout that gives the answer away (#2788).
- `task.json` text equals `score()` exactly (#2897, #2898, #2908, #2773), and the demo score reported is what
  the code returns.

## 5. Real-trial reproducibility and rig safety

- Custom parts: dimensioned drawings or STL files with every spring, hinge, magnet and detent specified (#2242,
  #2165, #2147, #2187).
- Consumables: a recipe (mass, temperature, time) or a reusable stand-in (#2320 watermelon, #2139 coffee,
  #2334 vacuum bag fill). Not one-off materials (#2846, #2945, #2920, our #2544).
- A starting fault needs a measured preparation and a check that it is the same each trial (#2355 snag depth, #2609).
- Reset in about 2 minutes from tape marks. Protect the pads from blades, edges, heat, grease and liquid
  (#2253 heat shields, #2277 blunt knife, #2906, #2755).
- No open flame or gas lighting in the instruction: Dream-1 refuses it, and the trial must be safe to run.

## 6. Novelty: check the ability, not the object

Search `known_tasks.tsv` for the object **and** the skill words; include our own PRs (variants of our own
tasks: #2945, #2905, #2882, #2859, #2837). Check LIBERO, RoboCasa (#2874, #2818, #2221), BEHAVIOR (#2135,
#2217), dexhand (#2944, #2410, #1743, #2216, #826), robotwin-2 (#2792), embodied-olympics-fun (#2833) and
RoboARC (#2211). Put a "Distinct from" table naming the three nearest TK-IDs in the README (#2522, #2524). A
precise rebuttal of a duplicate finding can win (#2306 against RoboWits Stand Pages).

## 7. Text and media

- `fidelity` is 500 characters at most (the intake bot enforces it, #2527): an honest list of stand-ins.
- README: at least ~600 words (#2511): run command, how the simulator judges each rule, real setup, reset and
  measurement, runtime and seeds, sources and licences. Strong ones run 2,600-4,100 words (#2522, #2524, #2527).
- A Dream-1 score of 0.0, or a refusal, is fine (#2403, #2523). The demo reaches 1.0, and `run_limit` is about
  2-3x the demo's time (1,200-2,400 steps).
- Blind review: no team, person, lab or home path in the text, screenshot or video.

## 8. What reviewers ask in revisions (fix all of it before submitting)

Since 19 November a review agent sends a task back at most once, then a human decides. In 4 of 26 revised
accepts the second review raised something new (#2277, #2187, #2139, #2291).

| Request (count in 26 revised accepts plus 4 of ours) | Examples |
|---|---|
| Purchasable product or build files; consumables recipe; fault-preparation check (14) | #2355, #2334, #2320, #2282, #2242, #2165, #2147, #2133, #2186, #2879, #2609, #2520 |
| End-state scoring, no over-penalising valid routes, no unsafe credit (9) | #2396, #2100, #2277, #2879, #2194, #2139, #2082, #2291 |
| Contact plus force, not proximity (6) | #2187, #2065, #2277, #2245, #2883, #2291 |
| Rig protection (7) | #2253, #2277, #2259, #2282, #2139, #2267, #2396 |
| Honest category (5) | #2334, #2280, #2277, #2259, #2186 |
| Ordinary-tool checks on visible surfaces (5) | #2253, #2291, #2520, #2242, #2065 |
| Installed context (4) | #2365, #2267, #2879, #2883 |
| Recognisability: hazard parts visible, appliance looks like itself (2) | #2245, #2147 |

Answer in a PR comment that quotes each request and names the file changed (#2186's table), re-upload the
media when the demo changed, and push within the 48 h hold (turnaround on accepted revisions was 1-12 h).

## 9. Fatal versus fixable

- **Fatal:** any importance failure: not core work for 500k+ workers, not most people's experience, a covered
  ability or variant, an external-benchmark duplicate, not robotic, an Other task with no impact case,
  miniaturisation, a missing defining mechanism.
- **Fixable once:** missing installed context, scoring that can't be measured in a real trial, a code/text
  mismatch, missing build files or reset, asset licences, pad protection.

## 10. Gate before building (all must be yes)

1. The category's evidence sentence is written, with a source, and a reviewer would nod at it.
2. The knack sentence has numbers, and the 3 nearest board PRs plus external sets test a different ability.
3. One named product at real size, installed where it really lives, with a real mechanism.
4. Every score level is checkable with ordinary tools at the end state, and doing nothing scores 0.
5. The real-trial setup, reset and rig protection fit in one paragraph and cost under a few hundred dollars.
