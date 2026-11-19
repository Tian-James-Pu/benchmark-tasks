This is an incomplete SR-095 reach diagnostic, not a submission-ready task.
The score is disabled. No successful repair is claimed.

The full-size USDCraft cordless 190 mm saw is placed on its motor-housing side.
Source meshes, collision decomposition, masses and joints are retained. An
explicit articulation root makes its initial pose reproducible. The probe
checks access to the existing guard rim without added handles or resizing.

Source: [NVIDIA USDCraft-10K](https://huggingface.co/datasets/nvidia/USDCraft-10K),
Apache-2.0. `source_asset.json` pins the revision, path and model SHA-256.
The model is generated geometry, not a manufacturer replica. `mechkit.py`
is adapted from the shared open-source mechanism kit used by heat-shrink-lug.

The [Makita 5007MGA parts diagram](https://cdn.makitatools.com/apps/cms/doc/prod/500/3710b366-f401-4f70-8d3d-9a4f75c468aa_5007MGA_PB.pdf)
establishes a real tension spring between lower guard and blade case. That
corded 184 mm product is a mechanism reference, not this generic cordless
190 mm asset's identity. The manufacturer's manual requires a freely closing
guard and service of a malfunctioning guard spring before use.

Unfinished: physical spring and retaining hooks, removable power interface,
actual repair sequence, observable scoring, material/force calibration,
distractors, negative controls, and two complete repair demos. The source
asset's fixed battery and spring represented only by joint stiffness do not
implement the proposed repair. The source guard extends below the shoe, so
the original upright pose cannot rest on the shoe as its prompt describes.

First reach diagnostic: source guard moved 6.49 degrees but the saw shifted
before the attempted rim grip. This is not a successful repair. A second
diagnostic rotates the entire original saw in-plane and uses the other arm
on its existing main handle. Both arm plans pass offline IK within 0.084 mm
and 0.017 degrees; physical support/grasp remains under test.

Supported reach comparison completed: guard peak 16.12 degrees; final 7.19 degrees; maximum housing displacement 5.78 mm. Contact sheet inspected. Both diagnostic scenes score 0 and are not full-task validation.
