# Put a lift-off oven door back on its range

**Instruction:** *Put the oven door back on the range and close it.*

The oven door of a 30-inch free-standing electric range has been lifted off for cleaning. It stands on a folded towel on the
floor in front of the range, its top leaning on the range's front frame. The range's two hinge arms stick up out of the front frame at
the bottom corners of the oven opening, held at the hinges' *special stop*, about 7.5 degrees open. The robot has to take the door by
its sides, stand it up, lift it, turn it to the hinges' stop angle, carry it over the hinge arms with both of the slots in its bottom
edge squarely over both arms, lower it straight down until it sits on them, let go, and push it shut.

## Importance and sources

Category: **Societal relatability**. Taking the oven door off and putting it back is the cleaning procedure that range makers print
in their owner's manuals; the refit is the step people get wrong (the door goes back crooked, on one hinge, or binds half way).

- The product, the Hotpoint RB525DDWW 30" free-standing electric range (GE Appliances), "White Porcelain-Enamel Frameless
  Lift-off Oven Door", standard clean. Spec page (overall width 29 7/8 in, depth 26 3/8 in, depth with door open 46 3/8 in,
  oven interior 23-3/4 x 19 x 18-1/2 in, net weight 135 lb):
  https://products.geappliances.com/appliance/hotpoint-specs/RB525DDWW
- Its owner's manual (linked from that page), "Lift-Off Oven Door": "Do not lift the door by the handle." "To remove the door,
  open it to the special stop position. Grasp firmly on each side and lift the door straight up and off the hinges. Do not place
  hands between the hinge and the oven door frame. To replace the door, make sure the hinges are in the special stop position.
  Position the slots in the bottom of the door squarely over the hinges and slowly lower it over both hinges."
  https://images.salsify.com/image/upload/s--DRpbFRdk--/1af6f53ab3b66950acd2a0b360a7bd48c86eaad8.pdf
- Its quick-specs sheet: "Visor handle"; "Dimension from wall to the front of closed oven door handle is 28-1/2"" (the handle
  stands 2 1/8 in proud of the door's front).
  https://images.salsify.com/image/upload/s--hLmPSAx6--/d1c51f60831910b50376ac893166705e6ef5f714.pdf
- GE Appliances' support article on lift-off oven doors puts the stop position at "about 2 to 3" open" ("where you feel the
  hinge catch slightly"), and says to grasp the door at the sides, line it up with the hinges and push it into place.
  https://products.geappliances.com/appliance/gea-support-search-content?contentId=19121
- KitchenAid's help page on removing and replacing an oven door describes the same design (doors without hinge latches): open the
  door to its broil/removal stop, lift it by its side edges straight up off the two exposed hinge arms, and to refit, position the
  bottom openings over the two hinge arms and slide the door straight down until it is seated; it also warns that oven doors are
  heavy and must not be lifted by the handle.
  https://producthelp.kitchenaid.com/Cooking/Wall_Ovens_and_Ranges/Oven_Product_Assistance/Removing_and_Replacing_the_Oven_Door

Limits of this evidence: the manuals establish the real operation and its geometry. They do not give a count of how many people do
it, and we make no claim beyond "the procedure every owner of such a range is given for cleaning the oven".

**The knack.** The door is a wide, heavy panel held only at its two sides. Its two slots are 10 mm wide, 26 mm deep through the door's
thickness and 64 mm deep, 556 mm apart, and must start over the two 4.3 x 18 mm hinge blades at the same time, at the hinges' 7.5 degree
stop angle: a door held 3.6 degrees off that angle, or 3 mm sideways, catches a blade's tip on the slot's wall instead of sliding down.
The door stands leaning the other way (8 degrees towards the range), so it has to be stood up, lifted 8 cm and turned about 15 degrees
while both hands carry it together, then lowered straight along the blades. The two arms share the load and must move as one: if one
hand leads, the door twists on the other grasp. Only once it sits fully on both blades does a push close it.

**Distinct from** the nearest tasks we know of:

| Nearest | What it tests | Difference here |
|---|---|---|
| TK-NVTZYW (#3094, pending), lift a cabinet door off its loose-pin hinges | Lifting a small 0.55 kg cabinet door up off one remaining hinge pin | Refit, not removal; an appliance door carried by both arms at its sides and lowered onto two fixed hinge arms at once, at a set angle, then closed |
| TK-RP7Y3X (#2591, accepted), move a toaster oven's rack down one rung | Sliding a rack past its anti-tip stop in a toaster oven | Different object, mechanism and skill (no door, no hinge) |
| TK-QW5SPV (#3359, pending), French-door oven, swing the linked doors onto a magnetic holder | Opening linked doors and taking out a pan | Nothing is removed or refitted there; here the door comes off its hinges and must be put back on them |

Searches of the program's task list for "oven", "door" and "hinge" (3,510 pull requests, 21 November 2026) found no task that puts an
oven or range door back on its hinges. LIBERO, RoboCasa and BEHAVIOR open and close oven doors on fixed hinges; none removes or refits one.

## Scale and mass

The arms cannot reach the top of a full-height, full-size range door, or carry its weight, so the task is modelled at a stated
scale. Nothing else about the mechanism changes.

| | Real (Hotpoint RB525DDWW) | Modelled |
|---|---|---|
| Range width | 29 7/8 in (759 mm), spec page | 645 mm (85 %) |
| Door width | about 29 1/2 in (749 mm): just inside the range's width (estimated from the spec) | 637 mm (85 %) |
| Door height | about 20 in (508 mm): depth with the door open (46 3/8 in) less the closed depth (26 3/8 in) | 432 mm (85 %) |
| Door thickness | about 2 in (51 mm), estimated | 43 mm (85 %) |
| Visor handle | stands 2 1/8 in (54 mm) proud, quick specs | 46 mm (85 %) |
| Oven opening | 23 3/4 x 19 in (603 x 483 mm), spec page | 513 x 410 mm (85 %) |
| Hinge arms, slots | about 3/4 x 3/16 in blades, 3 in of arm (estimated) | 18 x 4.3 mm blades standing 64 mm, slots 26 x 10 x 64 mm (85 %) |
| Special stop | door top about 2 1/2 in out (GE), 7.5 degrees | 7.5 degrees (an angle does not scale) |
| Door mass | about 11 kg (24 lb), estimated: GE does not publish it | **2.5 kg**: what the two arms can carry and hold steady |
| Hinge line height | about 6 in above the floor (top of the storage drawer) | 35 mm above the station table: the table stands in for the drawer's top, so the oven is lowered 12 cm onto it; the drawer's front shows only as a 35 mm strip |

Everything is scaled uniformly by 0.85, so the proportions, the slot clearances relative to the door, the stop angle and the human access
(hands on the door's sides, the visor handle across its top) are the real ones. The range's body is modelled only 30 cm deep (its front
part); its back would stand beyond the station table. The lighter door changes how much the arms must lift (the real one is about 4.4
times heavier) but not the geometry of the refit: the slots still have to be squared over both arms at the stop angle and lowered
together. A real trial uses the real, full-size range and door (see below).

## Run and score

Use the Python environment that has `yam_arena` and `inspect-robots` (on the program's machines: `source ~/IsaacLab-Arena/yam.env`).
A model trial:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/refit-oven-door-at-hinge-angle/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the oven door back on the range and close it." \
  --scorer task_score --max-steps 2400 --store-frames \
  --log-dir ~/yam-eval/refit-oven-door-at-hinge-angle/model
```

The scripted demonstration is closed loop (it reads the door's pose as it carries it), so it runs through `demokit.py`, which
drives the same `yam_arena` embodiment, cameras and `task_score` scorer as `inspect-robots`:

```bash
cd tasks/refit-oven-door-at-hinge-angle
python demokit.py task.py ~/yam-eval/refit-oven-door-at-hinge-angle/demo --mode full
```

It writes `full/demo.mp4`, `full/setup.png` and `full/result.json` (with `task_score`). Other modes (`--mode none,onearm,careless,noclose,hold`)
are the failure checks listed under *Validation*. The layout is fixed (`object_jitter=0`); nothing is random, so there are no seeds.
RUNTIME_PLACEHOLDER

## How the simulator judges it

The range and its hinges are one articulation welded to the world. Each hinge arm is a steel blade on a hinge about the door's bottom
back edge. Joint zero is the special stop, and the hinge's stop is a joint limit there; the travel past it to fully open is not
modelled. The hinge has a light catch at the stop (a 20 N m/rad spring), and pushed more than 3 degrees towards shut, its closing spring
(40 N m/rad) takes the door shut against the frame (the other joint limit, 7.5 degrees from the stop). The door is a free 2.5 kg body;
its slots are empty channels between its own box colliders, so it is held on the range only by contact between the slots and the blades.

Each control step the scorer reads, from poses alone:

- **seat gap**, for each hinge: how far the top of the door's slot stands past the tip of its blade, along the blade (0 when the door
  sits on the blade), and how far the blade's tip is from the slot's axis (under 6 mm when the blade is in the slot);
- **shut**: both hinges within 0.5 degrees of shut;
- **hands clear**: no point of either gripper's fingers within 3 cm of the door or its handle;
- **still**: the door moving under 5 mm/s and turning under 0.05 rad/s;
- **harm**, recorded and kept until the scene is reset: the door dropped (falling faster than 0.6 m/s), fallen over (more than 60
  degrees from upright), or lifted by its visor handle (both fingers of one gripper within 2 cm of the handle while the door is off the
  towel and not on the hinges; the manual says "Do not lift the door by the handle").

Scores:

- **1.0**: both slots fully down on their blades (each seat gap within 3 mm), the door shut, hands clear and the door still, all held
  for 2 seconds (40 control steps), and no harm recorded.
- **0.5**: both slots fully down on their blades, hands clear, door still, but the door not shut (it hangs at the stop).
- **0.25**: both blades at least 15 mm into their slots but the door not fully down on both (it has bound or hangs crooked), hands
  clear, door still.
- **0**: anything else, including the untouched start, a door on one hinge or held by the robot, and any recorded harm, whatever
  happens afterwards.

## Real trial

Setup: a Hotpoint RB525DDWW (or any range whose manual gives the same lift-off door procedure) installed against a wall on its anti-tip
bracket, cold and unplugged. Open the door to the special stop, lift it off as the manual says, and stand it on a folded bath towel on the
floor in front of the range, its bottom edge 8 cm out from the front frame (tape marks on the towel), its top leaning on the frame above
the oven opening. Check that both hinge arms are at the special stop (top of a held-on door about 2 1/2 in out). Two robot arms able to
carry the real 11 kg door are needed; with the YAM arms used here, the door would have to be lightened (for example the glass packs
removed and the weight noted), which must be reported.

Reset (about 2 minutes): lift the door off at the special stop, stand it back on the towel marks.

Measurement, after the robot has let go and before anyone touches the range:

1. Seated: with a ruler, the gap between the door's bottom edge and the hinge arm's shoulder at each corner. Seated is when the door
   has come down onto both arms: both gaps equal to those of a door seated by hand (measured once at setup) within 3 mm.
2. Shut: the door closed against the frame, its top within 1/16 in (1.5 mm, a feeler gauge) of the seated-and-closed reference.
3. Hands clear for 2 seconds, from the video.
4. Harm from the video: the door dropped or toppled, or carried by its handle while off the towel and off the hinges.

Pad protection: the door's edges are porcelain-enamelled steel with no sharp edges; the hinge arms are blunt steel. Keep the
gripper pads off the hinge arms' path (the manual's own warning, "Do not place hands between the hinge and the oven door frame").

## Assets and licences

The range body, hinge arms and door are custom parts built from boxes in `task.py` (no SimReady range or oven door exists).
The towel and three distractors are NVIDIA SimReady props, downloaded at first run from the pinned dataset revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, © NVIDIA, CC-BY 4.0:

- [`consumable_microfiber_towel_folded_large_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/consumable_microfiber_towel_folded_large_a01): the towel the door stands on; mass 0.21 kg (as authored).
- [`cleaning_windowSpray_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/cleaning_windowSpray_a01): distractor (glass cleaner); mass set to 0.5 kg, its joints welded into one body.
- [`cleaning_sponge_b01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/cleaning_sponge_b01): distractor; mass set to 0.03 kg.
- [`consumable_paperTowel_roll_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/consumable_paperTowel_roll_a01): distractor; mass set to 0.15 kg.

`simready.py` is the shared SimReady loader (adapted from the open-source `cutters-off-pegboard` task). `mechkit.py` is copied unchanged
from `tasks/heat-shrink-lug/mechkit.py` (another team's open-source mechanism kit) and `demokit.py` from `tasks/heat-shrink-lug/demokit.py`.

## Validation

VALIDATION_PLACEHOLDER
