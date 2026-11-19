# Separate two tangled wire clothes hangers

Prototype under validation. The demo has not yet reached full score; this folder is not ready for submission.

The blue hanger's open hook surrounds the hanging white hanger's lower crossbar. Both hangers are full-size bent steel wire; the task is to unthread the open hook and hang the blue hanger independently on the other peg. There is no programmed attachment or unthreading event: each wire segment has a rigid collider.

Run a scripted trial:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/untangle-two-wire-hangers/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=640 -E cam_height=360 \
  --instruction "Separate the tangled hangers and hang the blue one on the empty peg." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/hangers-demo
```

Use `--policy dream` for a model trial after final validation. Layout is fixed, `object_jitter=0`, and the demo is open loop. Runtime and validated trial counts remain pending.

`mechkit.py` is the shared mechanism kit from the repository's `heat-shrink-lug` task. `simready.py` is the shared SimReady loader used by `cutters-off-pegboard`. The hangers are custom full-size bent-wire structural geometry because no catalogued hanger is available; their collision segments match their visible wire. They are rigid, so elastic bending and deliberate permanent deformation are outside this simulation.

The distractors use these NVIDIA SimReady packages, pinned to revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`:

- [dishware_mug_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01)
- [tape_clear_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/tape_clear_a01)

© NVIDIA, CC-BY 4.0. The loader sets the listed masses and welds any authored joints into a rigid prop. Files download on first run and are not vendored.
