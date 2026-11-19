# Guard-service prototype: build and approximation record

This is not a completed benchmark task. No task score or acceptance claim is made.

The scene uses the pinned original-size USDCraft 190-mm cordless circular-saw asset. Blade and battery are removed before the episode. The manufacturer's lower-guard servicing procedure supports a pre-disassembled setup; the source asset is a generic generated saw, not manufacturer CAD.

## Reference mechanisms

- Makita 5007MG/5007MGA parts breakdown: lower guard, retaining ring, bearing boss and tension spring 231839-2. https://www.makita.ca/data/upload/tools/5007MG%20-%20NOV2009_parts_English.pdf
- Hitachi C7SB3 manufacturer service manual, mirrored copy: blade removal and bearing-cover access precede spring/guard removal. https://manuzoid.com/manuals/qO6X9-Hitachi%20C%207SB3%20Service%20Manual
- A ruler photograph of Makita 231839-2 supports approximately 79-mm overall free length, 58-mm close-wound coil body, 4-mm outside diameter, 0.45-mm wire and open end hooks. These are image estimates, not a manufacturer dimensional drawing. https://static.reposicaoonline.com.br/public/reposicaoonline/imagens/produtos/mola-da-protecao-do-disco-da-serra-circular-5007mg-makita-231839-2-67363ae96f958.jpg
- Bahco 2633/S4 supplies a real 130-mm miniature hook/pick envelope, plastic handle and stainless shaft, with 80 g for four tools. The scene's 20-g right-angle pick is a dimensioned equivalent; its 60-mm handle, 15-mm diameter and 1.5-mm shaft are disclosed reconstruction estimates. https://www.bahco.com/int_en/miniature-awls-hooks-picks-tool-set---4-pcs-pb_2633-_.html

## Reconstructed service interface

The source has no spring or spring anchors. The added interface is a buildable equivalent, not claimed as an OEM part: a 2.0-mm diameter steel shaft, 5.0-mm exposed length, and 3.6-mm diameter by 1.0-mm retaining head, at radius 24.0 mm on the existing lower guard's inner face. The bearing boss radius is 19.5 mm; the spring's centerline guide radius is 21.3 mm. The fixed anchor is at 65 degrees in the saw-face plane. These features remain inside the original guard envelope. No enlarged handle or external robot grip tab is added.

The rigid free hook follows the reference J profile: 0.45-mm wire, 3.6-mm bend diameter, 6-mm lower stem and 3-mm return tail. Individual analytic capsules preserve the open mouth; no convex hull fills the opening. Its only connection to the moving guard is collision with the headed anchor. The guard's original return drive is disabled.

## Reduced spring model

A continuously evaluated extension spring follows a frictionless bearing route. Its active free length is 65 mm, with stiffness 60 N/m. The rate is estimated from steel shear modulus 79 GPa, 0.45-mm wire, approximately 3.8-mm mean coil diameter and 125 active turns; it is not a measured OEM force curve. The reconstructed fixed-anchor location creates installed preload. The spring endpoint includes a lumped half-coil mass of 1 g and estimated distributed rotational inertia; these are not the mass/inertia of the short wire hook alone.

A weak transverse stiffness of 5 N/m and rotational damping of 1e-6 N m s/rad represent omitted distributed coil bending/damping. Coil turns are rendered but do not have individual collisions. The working hook and the anchor do. The winding angle is continuous: crossing the fixed anchor cannot add a fictitious full turn of stored energy. Endpoint velocity uses link-frame velocity rather than adding an offset to COM velocity twice.

All spring forces are continuous functions of geometry and velocity. There is no attachment flag, proximity latch, dynamic joint creation, force activation upon reaching the anchor, pose snapping or scripted object motion.

## Verified so far

Two V6 passive controls, each 35 control steps at 20 Hz after settling, with 960-Hz physics and inspected images:

- Relaxed, disengaged spring: guard remains at 34.999 degrees; final tensile force 0 N.
- Retained hook: guard returns from a runtime-confirmed 35 degrees to effectively 0 degrees; final tensile force 0.199 N.

These are component controls, not full robot repair demonstrations. The tool-recovery script and scoring are still being developed. Earlier V1–V5 controls exposed reset, contact margin, winding and numerical-model defects; they are not evidence of a working task.
