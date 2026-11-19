# Broken-key extraction prototype

This folder currently contains contact-feasibility fixtures, not a finished
benchmark or a successful robot demonstration. Their score is always zero.
The proposed instruction is: “Clear this lock so the spare key can be used.”
The intended initial fault is a preexisting blunt key fragment broken flush
with the face of a turned, enclosed pin-tumbler cylinder.

The first gate checks whether physical pin/key contacts permit withdrawal at
the normal angle and retain the fragment when turned. The coupon uses a fixed
plug angle, an axial key guide and a stated applied withdrawal force. These
are test-fixture constraints, not a proposed robot policy or task mechanism.
No angle-dependent collider switch, simulated fracture, key removal, unlock
event or pose-triggered coupling is implemented.

`probe_pin_contact.py` applies 1 N with a zero-degree plug.
`probe_pin_contact_straight_3n.py` and
`probe_pin_contact_turned_3n.py` compare 3 N at zero and 30 degrees. Each uses
1,920 Hz physics and 20 Hz control. The two 3 N probes run for 20 control steps;
the 1 N probe runs for 40. A trial takes several minutes on a shared GPU.
There is no random layout or jitter.

From the repository root, with the simulator environment active:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/extract-broken-key-from-door-cylinder/probe_pin_contact.py \
  -E object_jitter=0 -E control_hz=20 -E device=cuda:0 \
  --instruction "Inspect key fragment contact fixture." \
  --scorer task_score --max-steps 40 --store-frames --log-dir ~/key-contact-run
```

Change the task filename and use `--max-steps 20` for either 3 N comparison.
`LOCK_CONTACT` records the key and pin displacements. A finite simulation is
not enough to establish the intended retention behavior or hardware fidelity.

The dimensioned geometry comes from the [Schlage full-size cylinder service
manual, page 14](https://commercial.schlage.com/content/dam/allegion-us-2/web-files/schlage/technical-documents/Schlage_Everest_Full_Size_Cylinder_Service_Manual_108484.pdf):
six chamber centers, .343-inch blank height, .031-inch cut flats, 100-degree
cut angle, .015-inch cut increments and the corresponding bottom-pin lengths.
For codes 2-5-3-6-2-4, each root depth plus its pin length is exactly .500 inch.
The .115-inch pin diameter is supported by [LAB's pin specification](https://www.labpins.com/product/universal-115-diameter-serrated-top-pins/).

The C warding shape follows the vector illustration on page 55 of the
[Schlage key systems catalog](https://commercial.schlage.com/content/dam/allegion-us-2/web-files/schlage/information-documents/Schlage_Key_Systems_Cylinders_and_Keys_Catalog_109487.pdf).
That illustration is not a machining drawing. Its height was scaled uniformly
to the specified blank height and the blank was inset 40 micrometres. The
[Ilco actual-size blank directory](https://a-us.storyblok.com/f/1015476/x/a6fcf302db/dwn-key-blank-directory-full.pdf),
page 2-271, supplies an independent SC4 profile; it has not yet replaced the
initial coupon geometry. The source dimensions must be reconciled before a
tool-access claim. In particular, a rigid extractor must really fit beside a
flush fragment; no keyway enlargement to accommodate a gripper is permitted.

The repair sequence is supported by [SouthOrd's broken-key removal guide](https://www.southord.com/blogs/news/removing-a-broken-key-from-a-lock-a-complete-guide):
return a turned cylinder to the withdrawal position, engage the fragment with
an extractor and remove it. No claim of demonstrated extraction is made here.

Current reconstruction assumptions are explicit: 40-micrometre radial plug
and pin-bore clearances; generic 140 N/m driver springs preloaded 1.4 mm;
0.20/0.15 static/dynamic friction; approximate chamfered pin noses; and a custom
fixed outer housing. Sixteen-sided pin bores are circumscribed around the
specified clearance radius, with at most 29.4 micrometres radial excess.
The fragment has brass-density mass calculated from its convex pieces.
Tiny convex meshes are cooked with inverse coordinate scaling while preserving
their physical dimensions. The fixed coupon shell uses a triangle-mesh collider.

Before this can be a benchmark, it needs a supported commercial-cylinder
clearance and extractor fit, a physically rotating plug, free fragment and
spare key, ordinary tool geometry, robot grasp and reach checks, two complete
repair demonstrations, meaningful negative controls and observable scoring.
Importance and novelty also remain to be established; an extraction fixture
alone does not demonstrate an everyday household repair task.

`mechkit.py` derives from the shared open-source mechanism kit used by
`heat-shrink-lug`. Local helpers preserve small convex meshes during cooking
and apply the stated pin-spring targets during physics steps, including settle.

### Access disposition: parked

The later GMS K001-SX investigation uses the manufacturer's real C–K
composite opening, which leaves a side channel beside the independently
traced Ilco C blank. GMS specifies its ordinary K001 as five pins in a
six-chamber cylinder. This differs from the historical six-pin C coupon above.
The published Multipick ELITE 05 saw extractor is 0.5 mm thick and 121 mm long.
A photograph-derived front-blade envelope is 1.827 mm high; its best straight
insertion projection still overlaps the reconstructed channel by 0.00157 mm².
The opening scale and photo silhouette are uncertain at this level, so this
is neither a clearance proof nor proof that the real repair is impossible.
Neither opening nor extractor was resized to force compatibility.

The build is parked until compatible geometry and real extractor engagement
can be established. The finite aligned/turned force controls remain useful
contact diagnostics, but are not a repair demonstration or a submission gate.
Detailed source profiles and limitations are in
`pipeline/pdf-screening/sr068-key-extraction/access-disposition.json`.
