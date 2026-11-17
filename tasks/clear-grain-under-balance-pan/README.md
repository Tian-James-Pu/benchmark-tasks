Restore accurate weighing with a trapped grain under the balance pan
=================================================================

Two fresh scripted trials scored 1.0 with the final contact-aware scorer. Both recorded readable initial faults and released stable zero/100 g/zero witnesses, with unchanged power and zero offset. The four canonical negative controls scored 0.0. Dream-1 completed all 1,600 actions and scored 0.0. The model run took 25.9 minutes of wall time, including native retries after transient provider-session failures; the same evaluation resumed and completed without task changes.

The goal is to restore accurate weighing and leave the check weight beside the balance. Removing the grain is one possible route. Moving or reseating the cover so that it no longer bridges to the fixed case is also valid if the released empty, loaded and empty readings pass. The score does not require a particular repair sequence, arm assignment, tool or debris destination.

The reconstruction uses the current [A&D HT-120 specification](https://www.aandd.jp/products/weighing/scale/value_weighing/ht-120.html) and [manufacturer manual](https://www.aandd.jp/products/manual/balances/ht-120.pdf), especially the removable cover and fixed support shown on page 3. The external envelope is 136×150×57.5 mm, pan diameter 70 mm, readability 0.01 g and total balance mass 570 g including batteries. The source specifies a 15 mm LCD and 10 Hz updates. The modeled shell profile, support diameter 66 mm, sheet 0.5 mm, clearance 2.5 mm, internal masses and 20 kN/m load spring are estimates. This is a functional reconstruction, not manufacturer CAD or an exact firmware model.

A rigid 3 mm NaCl cube initially bridges the cover rim to the fixed case. The cube could not freely enter the 2.5 mm modeled gap: the starting fault represents a cover reseated over debris. The steel cover is a free body with an open underside and 64 separate rim collision pieces. The 25 g sensor support moves on a gravity-loaded spring; the 30 g cover rests on it through contact. The display converts the measured spring displacement into force-equivalent grams, minus the zero offset. No object identity, distance-to-grain flag or expected answer controls the displayed number.

The [KERN 347-07M 1 reference](https://www.kern-sohn.com/cosmoshop/default/pix/a/media/347-07/TD_347-07_en.pdf) specifies 100 g,22 mm diameter,39 mm height and±5 mg tolerance. The knob/neck subdivisions are inferred; the internal adjustment cavity is omitted. This weight is for a functional verification check. It does not meet the HT-120 manual's±2 mg default 100 g calibration-weight requirement, and the task does not perform factory calibration.

The source's selectable response/stability modes do not publish their filtering algorithm. The approximation uses a 1 s mean,10 Hz digit updates,0.01 g rounding and a stable indicator when the last 0.5 s of filtered readings span no more than 0.03 g. Normal-mode keys are real spring-loaded contacts with estimated 0.3 mm travel and 1.2 N full-stroke force; pressing beyond 0.15 mm registers. RE-ZERO can be retried after an error; a negative offset beyond 2% of capacity is rejected. Power cycling clears prior verification and zeros a stable load after a 2 s startup. The manufacturer's exact power-on acceptance range is unpublished; that behavior is approximate. Only grams is enabled, a configuration the manual permits, so UNITS cycles the sole enabled unit. Configuration/calibration firmware is not implemented.

The shell, pan and grain are rigid. Salt crushing, pan deformation, electronic overload damage and fracture are not simulated. A real trial must stop and score zero on observed damage; the simulator cannot certify the absence of these damage modes. Its explicit integrity proxy records a fall of the balance, pan or check weight more than 20 mm below the tabletop. Ordinary safe regrips, repositioning, corrective zeroing and alternative recovery routes are permitted.

The score is 0.5 after a released stable empty reading within±0.05 g for 2 s, followed by a released stable 100 g reading within±0.05 g for 2 s. It is 1.0 after the check weight returns upright to the bench and the empty reading again stays within±0.05 g for 2 s. The zero offset must remain unchanged during verification; changing it clears the measurement witnesses and allows a new attempt. The tolerance is a task acceptance window, not a claim that a display's resolution guarantees its accuracy. The numeric support, pose, motion and release conditions are listed in task.json and implemented in task.py. Both full demonstrations and all four controls use those same conditions.

This task uses the category Other for laboratory automation. [Dual-arm solid-dispensing research](https://doi.org/10.1039/D3DD00075C) uses balance feedback to regulate dosing and repeat overdoses; it reports manipulation failures when continued dispensing does not produce a changed reading. [Adaptive powder-weighing research](https://doi.org/10.1039/D5DD00569H) likewise closes its control loop through measured mass. The relevance here is an inference from that dependence: a mechanically shunted pan can invalidate the measurement that governs the next experimental action, and pressing zero cannot repair a changed load-to-reading response. This task isolates physical restoration and released zero/load/zero verification. It does not claim measured productivity gains, a particular fault frequency, or that this affordable 0.01 g balance reproduces the milligram performance of those research instruments.

[General manufacturer maintenance guidance](https://www.mt.com/dam/P5/labtec/17_Miscellaneous/UM_ME-T_en.pdf), printed page 20, calls for cleaning after use or substance changes, removing and reassembling the pan, and testing afterwards. This supports the maintenance setting. It does not establish the estimated HT-120 internal gap or imply that this short repair performs the manual's complete cleaning procedure.

Run a trial after installing the repository's YAM Arena and Inspect Robots environment:

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/clear-grain-under-balance-pan/task.py \
  -E object_jitter=0 -E control_hz=20 \
  --instruction 'Restore accurate weighing, then leave the 100 g check weight beside the balance.' \
  --scorer task_score --max-steps 1600 --store-frames --log-dir balance-demo
```

Use `--policy dream` for the evaluated action policy. Control is 20 Hz and physics 960 Hz. The run limit is 80 s of simulated time. Both final demos succeeded after 1,402 control actions (70.1 s). Measured wall times on shared GPUs were 39.4, 18.0 minutes, including startup and rendering; GPU contention affects this runtime. No object jitter or randomized layout is used. Reset through the environment reset hook; pose recurrence never clears recorded falls. The scripted demonstration is a fixed waypoint plan based on the published starting layout.

Assets
------

The balance and verification-weight reconstructions are procedural meshes. Their dimensions and estimated details are described above. The mechanism helper is adapted from the shared open-source `mechkit.py` used by the heat-shrink-lug task; `scaled_convex.py` preserves small collision pieces during cooking. The camera retains its physical mount and uses a narrower lens to show the small LCD and service area. The display is in-scene geometry, not an added video overlay.

The following props use NVIDIA SimReady at revision`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, © NVIDIA, CC-BY 4.0:

- [instrument_spatula_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/instrument_spatula_a01): unscaled 243×26×13 mm; mass set to 40 g.
- [dishware_cup_paper_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_cup_paper_a01): unscaled 80×80×110 mm; mass set to 6 g.

The loader downloads the pinned assets into a local cache and welds their parts into rigid bodies. No asset geometry is resized. The spatula and paper cup are optional bench objects; safe useful tool use is allowed. Their contact with the weighing assembly cannot substitute for a released measurement.

The balance articulation has solver sleep disabled so the force-derived readout keeps updating instead of freezing a tiny equilibrium displacement residual. Geometry, spring stiffness and measurement tolerances are unchanged. Release requires conservative finger bounds to remain more than 20 mm from the case, pan and weight. A read-only PhysX contact view covers all 18 robot rigid bodies and every rigid body of the balance, pan, weight, spatula, cup and grain. It samples all 48 physics substeps plus the terminal boundary of each 20 Hz control interval. Any robot contact with the weighing assembly blocks that interval. A prop or grain in robot contact, or within 2 mm of a finger bound, must also be more than 20 mm from the case, pan and weight. The held-object set propagates through supplied-object bounds within 40 mm, covering two maximum 20 mm contact offsets, so a held cup supporting a spatula against the pan also blocks verification. Runtime setup reads the effective native collider offsets and rejects larger values; auto-computed USD sentinels are never treated as physical distances. Ordinary unheld bench props are ignored. Contact counts prevent opposing forces from cancelling into false release; incomplete coverage, sample counts or exhausted/invalid contact buffers fail closed. The contact callback runs before scoring and the interval counters must match.

The paper cup stands at (42, −28) cm, clear of the weight pickup. The fixed camera mount uses a 900 px focal length at 640 × 360 resolution. At reset and after electronics updates, camera sensor caches are refreshed after the measured LCD is painted; this does not advance physics. Temporal rendering can retain faint segments while values change rapidly. Held stable values are readable; the initial fault and every scored stable witness are checked in the recorded media. The stability indicator distinguishes settled measurements from changing readings. The optional diagnostic controller `closedloop.py` is the shared 6-DoF YAM helper and commands ordinary robot joints only.


Reproducible scorer controls
---------------------------

Use the same command above with the named task file substituted. Every control
imports the canonical scene, electronics and scorer; none overrides its score.

- `control_idle.py`, 180 steps: leave the original fault untouched.
- `control_false_zero.py`, 900 steps: physically operate the power key, then place
  and remove the actual 100 g weight without repairing the load path. Explicit
  power/startup logging distinguishes a false zero from a switched-off display.
- `control_palm_support.py`, 300 steps: a simulation-only horizontal wrist approach
  stops at positive pan contact while the fingertips remain clear. This audits
  nonfinger support detection; it is not a safe real-balance procedure.
- `control_held_weight.py`, 1,600 steps: perform the physical repair, then keep
  holding the weight while it rests on the pan. A held load cannot earn a witness.

All four controls scored 0.0. Idle retained the initial −12.08 g fault. The false-zero route achieved a misleading released zero, but the real released 100 g weight read about 59.43 g with power on and startup complete. The palm route produced a measured 7.22 N vertical pan reaction while fingertips were clear; the all-body gate rejected that interval. Holding the weight after repair kept robot contact and produced inaccurate, sometimes overloaded readings, so that control is a combined careless-route negative rather than an isolated release test. These routes are simulation diagnostics, not alternative successful demos or safe real-balance procedures.


Reconstruction cost
-------------------

On 17 November 2026, [Scale Warehouse and More listed the HT-120 at $176.70](https://scalewarehouseandmore.com/a-d-ht-120-ht-series-compact-scale-120g-x-0-01g/), and [KERN listed the 100 g M1 347-07 weight at €21](https://www.kern-sohn.com/shop/en/products/test-weights/oiml-m1/347-07/) excluding VAT and shipping. These dated listings identify obtainable low-cost hardware; prices may change. The modeled shell/support details remain the functional reconstruction described above, not manufacturer CAD.
