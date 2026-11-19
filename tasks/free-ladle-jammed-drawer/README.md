# Utensil drawer recovery — feasibility prototype

This folder is under development and must not be submitted. The current scorer deliberately returns zero.

The intended task is to open a full-size drawer obstructed by a free ladle resting over a loose spoon, using an ordinary plastic turner if needed. The two utensils are free rigid SimReady objects; no spring, attachment, pose-triggered stop or active force creates the jam. Cabinet panels and a single prismatic drawer joint are custom structural geometry. The drawer travels 36 cm. The fixed cabinet extends behind the station table; its mounting area is on the table and would be bolted down in a reproduction.

Contact diagnostics established that a 5 N pull opens an empty drawer 36 cm, while ladle and spoon alone block it after 4.23 cm. An interfering fork was removed after a separate diagnostic showed it jammed the side wall. Those force-driven probes are physics diagnostics, not success demonstrations. The actual recovery plan contains only robot waypoints.

The nominal ladle is 31.2 cm long; the available turner is 23.7 cm long. Assets are NVIDIA SimReady `kitchenware_ladle_d01`, `kitchenware_spoon_k01`, and `kitchenware_spatula_f01`, pinned by the included loader. The turner's authored physics material is labelled plastic. The helper is adapted from the open-source `cutters-off-pegboard` loader; mechanism helper from `heat-shrink-lug`. Assets are copyright NVIDIA, CC BY 4.0.

The full recovery, scoring, final camera, redteam checks, reset checks and two fresh full-score demonstrations remain unfinished. No Dream-1 run or submission is authorized from this prototype state.
