# What gets rejected (study: "rejected", 19 Nov 2026)

Source: decision comments on all 86 PRs labelled "Not accepted: final" among #2755-#2945 plus our own (#2398-#2955).
Raw comments: study/rej/<n>.txt; condensed: study/rej_condensed.txt. Reviewer: achyutha11 (bot), batches every few hours.

## Headline
- Importance "not met" appears in 86/86 final rejections. Every final closure failed importance. Every revisable
  ("Not accepted", not final) review passed importance and only failed CV/scoring/clarity (#2883, #2879, #2609, #2520, #2477).
  => Importance is the only fatal criterion. CV / scoring / reproducibility are fixable on their own, but they get piled on.
- Co-flags on finals: Construct validity 40, Scoring 28, Clarity/repro 25, Feasibility 4.

## Importance sub-reasons (approximate counts, a PR can have several)
| Sub-reason | n | Examples |
|---|---|---|
| Not core/regular work for >=500k (count of the occupation != count doing THIS operation; incidental/prerequisite/side chore) | ~22 | 2909 pulley ("counts everyone in the listed occupations rather than those who regularly perform this specific operation"), 2912, 2902, 2890, 2880, 2865, 2855, 2846, 2841, 2820, 2755, 2418(ours); incidental: 2863 ceiling tile "ancillary step", 2894 receipt roll "occasional support chore", 2773 stow level, 2767 open toolbox "prerequisite", 2895 close hood "incidental"; 2942 empty cart "omits that core work" |
| Covered ability / variant / duplicate of board task | ~20 | snap-fit: 2874, 2876, 2873; hold-one-turn-other: 2881, 2772, 2908; stacked load: 2844; stand upright: 2843; carton flaps 2893; panel+retainer 2868; own-team variants 2945, 2905, 2882, 2859, 2837 |
| Not something most people do / not recognisable in a short clip (Societal) | ~18 | 2922 fat separator, 2921 blind chain, 2919 shaver foil, 2886 lamp timer, 2878 V-brake, 2827 boat winch, 2814 TT net, 2807 French cleat "hobbyist", 2782 raised toilet seat "specialist home-care", 2765 euro cylinder, 2764 mahjong, 2862 card house "puzzle", 2845/2906/2871/2887 (ours) |
| "Other"/custom-fixture tasks with no real procedure (moving carriage, preset detent, custom rig) | ~15 | the hars-singh batch 2756-2803: "continuously oscillating carrier adds difficulty without a credible workplace purpose"; 2802 "only robot action is moving a lever to a fixed detent"; 2803 robot holds guide while motor works; 2500 (ours) "A Jetson label does not make..." |
| Difficulty isn't robotic (perception / rule / sorting, then ordinary pick-place) | 9 | 2918 breaker, 2896 valves in given order, 2828 read B2, 2823 match lengths, 2815 red carton, 2818 bruised apple, 2788 bounce test, 2772 "wind to 55 first", 2544 (ours) |
| Miniaturization (folds into importance; effectively fatal) | 8 | 2903 17 cm wheelchair, 2897 16x18 cm bed, 2895 hood, 2908 21 cm door, 2798, 2797, 2795, 2401(ours) |
| Hidden maintenance | 4-6 | 2919, 2887, 2881, 2765 "concealed" |
| External benchmark duplicate | 5 | dexhand Sweep Dustpan (2944), RoboCasa start electric kettle (2874), RoboCasa portion fruit bowl (2818), embodied-olympics-fun Paddle Bounce (2833), robotwin-2 Stamp seal (2792). Also earlier RoboARC knot (2211), BEHAVIOR steak. Reviewers check dexhand, robotwin-2, embodied-olympics-fun, RoboCasa, RoboARC, BEHAVIOR. |
| Safety claimed but hazard stated in instruction / not immediate | 3 | 2882 "instruction explicitly states the hazard and precaution", 2874 "scald hazard depends on later heating", 2398 bottle |

## Simplification / construct validity patterns (fatal when they remove the defining mechanism)
- Defining mechanism missing: 2905 clamp that applies no force; 2898 camlock with no hose/cams; 2826 vise with no moving jaw; 2894 printer roll with no paper end; 2903 brakes that don't lock wheels; 2895 no latch; 2820 rigid cylinders for seedlings; 2823 square bars for shafts.
- Missing installed context (fixable once, then required): 2880 bench battery w/o robot, 2883 wheel cover w/o tyre/car, 2879 car seat base w/o vehicle seat, 2873 pad driver detached.
- Valid shortcut to credit: 2945 tip the board instead of scraping; 2944 slide pan instead of brushing; 2788 layout reveals the answer; 2400 set top carton aside.
- Demo method differs from real method: 2912 blade press vs screwdriver strike; 2906 latch lacks resistance.
- Added robot-specific aids: 2865 110 mm grip on weight; 2895 added handle; 2778 dedicated tabs.

## Scoring flags (fixable, but common)
- Force/torque/impulse a person can't measure in a real trial: 16 PRs (2797, 2795, 2789, 2782 50 N, 2779, 2775, 2774, 2771, 2762, 2757, 2756, 2802, 2804, 2865 impulse, 2477 torque, 2520 1 mN m). Use observable end states (tape measure, photo, spin test).
- Sub-mm or concealed measurements: 2887, 2544, 2919.
- Code vs text mismatch: 2897 (5.9-7.2 vs 7.7-9.0 cm), 2898 (10 mm vs 4 mm), 2908, 2896, 2918, 2773 (1.0 before hold).
- Release check from a midpoint (2897); centre-only nest check allowing perched part (2905); credit persists after success hold (2879).
## Reproducibility flags
- Custom fixture with no drawings/BOM/commercial equivalent (2890, 2902, 2905, 2841, 2808, 2804, 2873, 2477).
- Perishable/one-off materials: fresh weld coupons 2846, carrot dice temp 2945, lint deposits 2920, salt cubes 2544, worn ferrule 2609.
- Cost: 2859 beer engine £318 exceeded the equipment guideline. Asset licences (YCB, CAD, YAM meshes): 2845, 2500, 2867, 2779, 2757.
- Gripper safety: sharp blades 2906, sheet-steel edges 2755, mineral fibre 2863.

## Fatal vs fixable
- Fatal (closed for good): any importance failure, which covers covered ability/variant, external duplicates, niche/not-most-people,
  hidden maintenance, not core/500k-specific, not robotic, Other without an impact case. Miniaturization and egregious simplification
  are fatal in practice because reviewers write them into Importance ("the miniature changes the reach and forces").
- Fixable (one revision): construct validity missing installed context, unmeasurable scoring, a code/text mismatch, a missing build spec
  or reset procedure, asset licences, gripper protection. Only when importance passes.

## What passed importance recently (contrast)
Revisable: 2883 wheel cover, 2879 infant seat half-latched (safety), 2609 crutch ferrule, 2520 spatula on spray arm, 2477 steering lock.
Merged: 2511 hair dryer out of a filled sink, 2523 broken glass in bread basket, 2493 child-proof drawer, 2492 air filter with cover held,
2412 sheet pan on toaster rails, 2527 blind tilt wand, 2534 pinch-zoom tablet, 2406 (ours) shower chair short leg.
