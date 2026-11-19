# gaps: coverage clusters and under-covered activity areas (snapshot 2026-11-19, known_tasks.tsv fresh, 2,948 PRs)

Overall: 301 accepted, 319 pending, 161 revising, 2003 rejected-final, 165 closed.
PRs #2000+: 79 accepted, 478 final, 98 revising, 26 closed, 270 pending. That is about 12% of decided PRs.

## Cluster counts (keyword regex over titles; clusters overlap, so treat as approximate)
| cluster | acc | pend | rev | rej | closed | acc% |
|---|---|---|---|---|---|---|
| cables/plugs/connectors | 34 | 25 | 11 | 142 | 7 | 15.5 |
| lids/caps/snap-fit/containers | 31 | 35 | 10 | 126 | 8 | 14.8 |
| drawers/doors/hinges/cabinets | 25 | 26 | 8 | 143 | 16 | 11.5 |
| screws/bolts/fasteners | 20 | 15 | 17 | 83 | 9 | 13.9 |
| medical/clinical | 22 | 26 | 11 | 79 | 3 | 15.6 |
| kitchen/cooking | 39 | 35 | 14 | 180 | 17 | 13.7 |
| laundry/textiles/clothing | 34 | 29 | 13 | 164 | 12 | 13.5 |
| hand tools/workshop | 40 | 32 | 27 | 165 | 6 | 14.8 |
| electronics/computing/AI-hw | 44 | 49 | 25 | 241 | 28 | 11.4 |
| packaging/warehouse/retail | 53 | 46 | 26 | 226 | 26 | 14.1 |
| vehicles/bikes | 20 | 20 | 11 | 106 | 11 | 11.9 |
| childcare/baby | 8 | 10 | 2 | 22 | 2 | 18.2 |
| garden/outdoor | 15 | 8 | 9 | 78 | 8 | 12.7 |
| plumbing/pipes/HVAC | 20 | 19 | 8 | 129 | 5 | 11.0 |
| lab/science | 11 | 16 | 5 | 83 | 4 | 9.2 |
| furniture/home-assembly | 45 | 45 | 31 | 254 | 26 | 11.2 |
| cleaning/bathroom | 13 | 16 | 5 | 59 | 4 | 13.4 |
| locks/keys/security | 18 | 22 | 15 | 194 | 13 | 6.9 |
| food/drink serving | 22 | 31 | 12 | 128 | 8 | 10.9 |
| office/stationery | 36 | 40 | 21 | 151 | 8 | 14.1 |
| safety/hazard | 17 | 28 | 10 | 123 | 17 | 8.7 |
| music/leisure/games | 10 | 18 | 4 | 86 | 5 | 8.1 |
| personal care/accessories | 12 | 12 | 8 | 39 | 3 | 16.2 |
| recovery/jam/stuck framing | 14 | 17 | 7 | 104 | 13 | 9.0 |

No object family is empty. Every one has 8 or more accepted tasks plus a pending queue. Reviewers find duplicates by
ABILITY, not by object. So a gap here means a physical ability, inside a recognisable activity, that no accepted or
pending task tests.

## Abilities reviewers now call "already covered" (final rejections, quoted)
- snap-fit / detent / latch catch: 2874, 2876, 2873, 2514, 2178, 2266, 2897
- hold one part while turning a fastener, dial or ring: 2540, 2881, 2112, 798
- ordinary pick-and-place / standing an item upright: 2843, 2815, 2826, 2823
- external sets: dexhand (battery swap 2410, pump stroke 1743, grinder crank 2216, binder page load 826, sweep dustpan 2944),
  RoboCasa (dishwasher 2221, kettle 2874), BEHAVIOR (bulb 2135, grains jar 2217), humanoidbench window wipe 1978,
  robotwin stamp 2792
- abrasive strokes (filing 1374 covers sanding 772), food turning (steak flip covers pancake 1435)
- dispensing beer by lever (2859), flat-coin pickup (2876)

## Relatability and jobs bar (the most common killer of our own submissions)
- "specialist/hidden maintenance most people have not done or watched": 2765 euro cylinder, 2782 raised toilet seat,
  2871 dishwasher rack (ours), 2887 float valve (ours), 2906 carving knife (ours), 2919 shaver foil, 2921 blind chain,
  2878 V-brake, 2886 plug timer, 2922 gravy separator, 2462 e-bike battery, 1493 dipstick (lawn tractor)
- jobs: the exact operation must be regular core work for 500k or more workers (2909, 2912, 2865, 2846, 2855, 2863, 2252 hood filter,
  2445, 2659, 1468)
- miniaturization: 2487 curtain swatch, 1700 ladder, 2903 wheelchair, 2795/2797 carts, 1659 bike wheel, 166 door, 1437 mast

## 20 under-covered activity areas (verified against all 2,948 titles; nearest PRs listed)
Ranking: A = no title match and a clear domain; B = some neighbouring work or a borderline domain; C = open, but with a stated risk.

1. (A) **Install a cable tie round a cable bundle, pull it tight and trim the tail flush.** Electricians and data-centre
   technicians (AI timelines). No title mentions installing one. Nearest: 1441 cut a tie off (accepted, the reverse
   operation), 2500 tie offcut in a fan (ours, rejected). Build: the strap as rigid links with a pawl joint in the head.
   Score: bundle slip under a hand pull and tail length. Check dexhand first.
2. (A) **Make a robot's or rack's cable bundle with ties at set pitch along a tray or arm link.** Same family as 1, aimed
   at AI timelines (1916 cable carrier accepted, 2150 clip rejected). Submit 1 or 2, not both.
3. (B) **Strip a solid conductor with a gauge-notched hand stripper, keeping the copper unnicked.** Electricians core.
   Pending 757 uses a stripping *station*, so it may count as a duplicate. Only go ahead if a hand-held, two-hand
   technique is clearly a different skill.
4. (A) **Lift a food processor's S-blade out by its hub before emptying the bowl** (Safety: the hidden blade). No
   food-processor title anywhere. Nearest: 2724 blender spoon and 2769 knife edge guard (both pending), 672 utility knife
   (accepted). Real bowl, real blade on a spindle. The instruction must say only "empty the bowl".
5. (B) **Start and seat a spark plug with a plug socket on an extension, in a deep well, by hand first.** No spark-plug
   title. Automotive techs (about 800k). Risk: 1471 (starting a nut by hand) and the "regular core" wording. Scale is fine
   (plug and head section).
6. (B) **Plug an EV charging connector into a car inlet until its latch catches, then release and holster it.** Only 162
   (holster, rejected) and 2479 capless fuel nozzle (revising, the nearest). Relatability is growing, but the snap-detent
   objection is likely unless the heavy connector and cable handling are central.
7. (B) **Assemble a shock-corded tent pole at real scale (40–50 cm, 8.5 mm sections) and seat it in a grommet.**
   1437 was rejected only for miniaturization and a "mast" framing. A real-scale pole fits on a tabletop. Societal relatability.
8. (B) **Fold an umbrella stroller: both handle release slides at once, collapse, frame clip engaged.** No stroller title.
   2280 opening a wheelchair (accepted) proves floor-scale objects can pass. Variant risk with 2280 and 1870 (two release levers).
9. (B) **Assemble a baby bottle (teat through the collar, collar threaded square, cap snapped) without touching the teat tip.**
   No baby-bottle title. Childcare and parents. Deformable-teat and "snap/thread covered" risk; it needs the hygiene
   constraint to be central.
10. (C) **Car engine oil check with a real curved-tube dipstick: pull, wipe, reinsert fully, read.** 1493 (lawn tractor)
    was rejected only on relatability. A car is far more familiar. The flexible blade must actually bend.
11. (C) **Gym weight-stack selector pin: pull, move to a plate, push fully through the stack hole.** No gym-machine title.
    Relatable, but the skill is close to "insert a pin" (30 accepted).
12. (B) **Close a door chain or swing-bar guard from inside.** No door-chain title. Everyday, but probably "slide into slot". Low priority.
13. (B) **Bus a stack of plates one-handed while scraping into a bin** (waiters/bussers, more than 500k). 286 and 503 were
    rejected as ordinary stacking. Only viable if a real carried stack (3 or more plates in one hand) is the centre.
14. (C) **Load a commercial peg dish rack (plates on edge between pegs) for a conveyor dishwasher.** Dishwashers are about
    500k. Risk: RoboCasa dishwasher loading (2221).
15. (B) **Change a hand-held tape-gun or price-gun roll.** 2043 tape gun was rejected only for simplified tape (construct), not importance;
    2428 price gun is revising. Needs a real flexible web. Out of scope for rigid-only builds.
16. (B) **Reset a mousetrap / spring snap trap safely** (Safety, relatable). Check the board for "trap": 17 rejected
    titles hit "trap" (mostly P-trap or hair trap). Worth a manual check.
17. (B) **Hang a framed mirror or picture on two hooks at once by a D-ring pair** (2694 wire-on-hook pending, 694 keyhole rejected).
    It is a variant risk, so skip unless a distinct two-point blind engagement is clear.
18. (C) **Prune a rose stem above a bud with secateurs.** Pending 1298 already holds this. **Taken, do not build.**
19. (C) **Boston shaker shake and open.** Pending 520 holds this, and 1956 was rejected as its duplicate. **Taken, do not build.**
20. (C) **Range-hood baffle filter swap overhead.** 2252 was rejected on jobs, and also because the counter-height panel let gravity
    hold the filter. Only an Other or societal reframe with a real overhead hood could revive it; low odds.

Takeaway: realistically only 1/2, 4, 5, 7 and 8 are strong and open. Items 3, 6 and 9–17 need a decisive distinct-skill argument.
18–20 are listed so we do NOT spend time on them.

## Do
- Pick an activity a person would recognise in a 10 s clip, or a named large job's core operation, and state the one
  physical ability no accepted or pending task tests (write the 3 nearest PRs and why they differ).
- Real scale, real mechanism (joints for springs and latches), safety credit kept in partial credit (2082), no pose-based
  history reset (2400, 2401).
- Recent accepted models: 2493 child-proof drawer, 2412 sheet pan rails, 2406 shower chair (ours), 2365 seatbelt
  twist, 2355 zip lining, 2342 CD hub, 2280 wheelchair open, 2268 coins in piggy bank, 2263 fry basket, 2253 match + candle,
  2182 SIM eject pin, 2153 fridge door bin, 2107 suitcase handle, 2100 keyed chuck, 1972 heat-shrink, 1969 shims, 1902
  shelf through face-frame.

## Avoid
- "Recover/jammed/stuck" hidden-repair framing: 9% acceptance; our 2871, 2887, 2906, 2881, 2874, 2876, 2873 all failed on relatability or covered snap-fit.
- Snap/press-to-detent cores, hold-and-turn cores, single-lever press, valve turning, switch toggling, crank turning.
- Saturated families with long pending queues: data-centre DIMM/PSU/rails (39 DIMM rejections), lab bench (pipette, gel, centrifuge pending),
  manikin care (more than 20 pending), toys/games (more than 10 pending), locks/keys (6.9% acceptance).
