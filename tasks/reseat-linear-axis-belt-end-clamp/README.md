# Linear-axis belt end-clamp development gate

This folder is **not a finished task or submission**. `belt_span_gate.py` tests a native elastic belt representation at a manufacturer-supported span and preload. Its score is always zero. Diagnostic supports are material-test fixtures and are not the final axis or an authorised shortcut for the repair.

The proposed final operation corrects an end plate short of its real stop during **new-belt installation**, then completes the full axis repair. No restored axis, physical driver engagement, full-scale slot access, toothed retention, final demo or novelty clearance exists yet.

## Importance and sources

[Festo's DGE repair manual, pp17–22](https://www.festo.com/spc/en_us/data/SPC/1/PDF_rep/7DGE_ZR_RF_T01_EN.PDF) documents centring the plate over four belt teeth, seating against the clamping-component stop, tightening the M5 threaded pin to0.5Nm and restoring the **new** belt's267–282N pretension. A plate short of the stop loses pretension during operation. The manual prohibits retensioning an aged belt merely because its tension has diminished.

[The matching2019 DGE catalog, pp4,48–53](https://ftp.festo.com/Public/PNEUMATIC/SOFTWARE_SERVICE/Documentation/2019/EN/DGE-ZR_EN.PDF) places these axes in handling and assembly systems and gives actual dimensions, materials and operating loads. This establishes a real maintenance operation, not the frequency of that operation or a quantified AI deployment benefit. The complete axis is714mm long for the300mm-stroke standard-carriage variant.

## Diagnostic and limits

The test uses82 native3mm rigid belt elements joined by passive native D6 constraints and elastic axial/bending drives. It has no commanded poses or programmed success flag. Native substep transforms record transverse motion; any measured frequency must come from that motion, not tension metadata. The catalog's0.16% elongation at260N gives an estimated axial rigidity162500N. Width15mm, effective thickness2.4mm, rubber bending modulus5MPa and damping remain explicit reconstruction estimates. This diagnostic has no belt teeth or pulley contacts and cannot prove retention or a complete repair.

The standard `mechkit.py` is copied from `pipeline/lib/mechkit.py`, originating in the open-source `heat-shrink-lug` task.

Run only through the repository's restricted physical-GPU2/3 launcher:

```
INSTRUCTION='Observe the belt representation diagnostic.' TASK_FILE=belt_span_gate.py pipeline/run_trial.sh reseat-linear-axis-belt-end-clamp demo 20
```

Source/access/nearest-task evidence is under `pipeline/pdf-screening/sr076-belt-clamp/`. The final task's weakest-route novelty comparison againstPR2203 remains required; a tension-only task is not distinct.
