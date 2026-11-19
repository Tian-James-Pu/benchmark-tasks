# Robot vacuum brush recovery — contact prototype

This is an unvalidated prototype, not a submission. Scoring always returns zero.

The starting robot is powered off and inverted on an ordinary folded towel. Its main rubber brush was left canted after cleaning, with the drive end outside its socket. Restore the brush and its retaining cover.

Hardware reference: [Roborock S7 user manual](https://support.roborock.com/hc/en-us/article_attachments/900008254623), printed pages 27–29, and the manufacturer's [roller product photographs](https://us.roborock.com/products/s7-main-brush) and [cover product photographs](https://us.roborock.com/products/roborock-s7-s7-maxv-s7-max-ultra-q5-q7-q7-max-series-main-brush-cover).

The manufacturer diagrams show a lift-off cover, not a permanent hinge. Photographs show four fixed rear hooks and two inward-sliding spring catches, as well as the roller's red rubber vanes, two flanged caps and black end features. The model reconstructs these task-critical surfaces. It is not manufacturer CAD. Small part dimensions and fits are estimates from photographs and must be reviewed before validation. The full chassis is 353 mm across; rigid rubber vanes and fixed-at-rest internal floating head are approximations.

No scripted coupling, pose-triggered lock, hand-count requirement or added grasp tabs are used. The chassis, roller and cover are free physical bodies. The actual cover catches are spring sliders; their hook contacts must hold against the chassis lips. The service towel is an ordinary support to protect the inverted robot's top, not a fixed fixture.

Run and score a trial with the Arena task runner using `task.py`. No successful demo exists yet. Initial gravity/contact test: 62 control steps, approximately 2–3 minutes including startup and media rendering.

`mechkit.py` is the mechanism kit from the accepted heat-shrink-lug task. `simready.py` loads NVIDIA SimReady assets. The folded towel, mug and tape are NVIDIA SimReady props (© NVIDIA, CC-BY 4.0). Masses and nominal dimensions are specified in `task.py`; asset joints, if present, are welded. The robot mechanism uses source-reconstructed meshes rather than a SimReady robot-vacuum asset, which the pinned catalogue lacks.

Prototype revision: the motor output socket now has a free revolute joint on the roller axis; its gears and electrical actuation are omitted. The roller can drive that socket only through square-key contact. Cover catches have an estimated 45-degree lower lead-in consistent with the manual's press-to-lock operation and a horizontal retaining shoulder; exact tooth dimensions and spring rate remain reconstruction estimates. The 0.5 mm retraction clearance was specified before controller tuning. Total body, output socket, roller and cover mass is 4.7 kg; allocation among these parts is approximate. No new successful assembly or retention demonstration has been established.

Geometry revision: the reconstructed assembly is 97 mm tall, within 0.5 mm of the manual’s 96.5 mm overall height. The 197 mm roller includes a 170 mm rubber section, a 5 mm square drive projection and a separate freely rotating outer bearing. The cover has a 186 × 36 mm opening and estimated 22 mm rails consistent with the manufacturer photographs. Its four rear-hook pockets and two catch pockets are physical openings in the underside deck. The output cup has a 22 mm round exterior surrounding the unchanged 8.6 mm square bore. These local dimensions remain disclosed estimates; installed-pose collision clearance alone does not demonstrate insertion or retention.

## Importance and sources

This task concerns restoring the working brush assembly of a deployed autonomous cleaning robot after maintenance. The [Roborock S7 user manual](https://support.roborock.com/hc/en-us/article_attachments/900008254623), printed pages 27–29, specifies main-brush maintenance every two weeks and describes removing and reinstalling the brush, bearing and retaining cover. This establishes a recurring manufacturer-supported service operation. The proposed relevance to robot servicing is an inference from that operation; the manual does not quantify technician jobs, installed fleet size or time saved by automation. The misseated starting condition is a reconstructed maintenance error, not a manufacturer-reported failure-rate claim.
