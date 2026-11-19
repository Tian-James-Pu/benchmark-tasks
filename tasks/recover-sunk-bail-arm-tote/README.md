# Parked prototype — do not submit

This prototype is parked on importance: the safe alternative of removing the upper tote, folding the unloaded bar and restacking does not establish enough distinctive physical difficulty. It does not count toward validated submissions.

The scene represents a full-size400x300x135mm commercial swing-bar produce tote, not a reduced600mm tote. Its near lower bar starts folded out, letting the upper tote sink into the lower tote at one end. No task score is currently enabled.

Hardware reference: Schoeller Allibert Maxinest9633.002, nominal400x300x135mm,0.82kg tare and127mm stacked height. [Manufacturer catalogue, page8](https://assets.ctfassets.net/kyg7xeoqgrqu/5wb5llRu8wW7DcaZFZeO3Q/034d7e6aef3d9fab4ea8701365f86891/SCHOELLER-Product_Groups-Stack-_and_nestable_containers.pdf). The equivalent [IFCOCHBA3413](https://www.ifco.com/product/bale-arm/) is401x302x135mm. Fine shell taper, bar pivots and support geometry remain approximate and under validation; they are not claimed as manufacturer CAD.

The visible perforated crate shell is derived from NVIDIA SimReady `crate_blue_a08` by resizing and tapering it to the dimensions of the actual smaller commercial model. It is decorative geometry, not an exact manufacturer model. The contact shell, rim and free hinge bars are modeled separately. No gripper tabs or supports are added.

NVIDIA SimReady assets, revision9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd: `crate_blue_a08`, `food_produce_apple_b01`, `tape_clear_a01`, `dishware_mug_a01`, from [nvidia/simready-assets](https://huggingface.co/datasets/nvidia/simready-assets), ©NVIDIA, [CC-BY4.0](https://creativecommons.org/licenses/by/4.0/). `crate_visual.npz` is an attributed mesh derivative. The other assets download through `simready.py`.

`mechkit.py` derives from the repository's heat-shrink-lug task; `simready.py` derives from cutters-off-pegboard. Both retain their original source documentation.

Run a prototype inspection with:

```sh
source ~/IsaacLab-Arena/yam.env
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/recover-sunk-bail-arm-tote/task.py -E object_jitter=0 -E control_hz=20 --instruction 'Make the two produce totes a stable stack, keeping their contents in the totes.' --scorer task_score --max-steps80 --store-frames
```

No full-score demonstration or red-team validation has been completed. Reproducible setup, final scoring and runtime will be documented after feasibility validation.
