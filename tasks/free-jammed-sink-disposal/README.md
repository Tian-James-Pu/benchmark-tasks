# Sink-disposal contact gate — development only

This folder contains an unvalidated mechanism diagnostic, not a completed task.
The full-size Moen GXP50C disposal is mounted under a GS181210B sink beside the
unaltered robot station. The apparatus starts disconnected with its removable
splash guard already removed for inspection. This does not demonstrate that a
robot identified or handled an electrical hazard.

`probe_jam.py` applies a declared ±0.10 Nm diagnostic shaft-torque ramp to the
turntable with a full-size metal spoon between a swivel impeller and grind ring.
The load is a mechanism test, not measured OEM jam torque. It always scores0.
The ordinary spoon is an unscaled NVIDIA SimReady asset; the sink, disposal and
moving grinding mechanism are source-informed reconstructions. Internal sizes,+friction and bearing damping remain estimates requiring native contact review.

## Importance and sources

[Moen's troubleshooting procedure](https://solutions.moen.com/Article_Library/How_to_Unjam_a_Garbage_Disposal)
identifies metal spoons as objects that can jam the grind ring. It prescribes
power disconnection, removal of a removable splash guard, retrieval with tongs
or needle-nose pliers, and the1037 top-entry wrench if required. Direct removal
is a valid route; a wrench or two arms is not compulsory. This establishes a
real household repair procedure, not its prevalence or a completed importance
assessment for this particular pre-isolated diagnostic stage.

[GXP50C specifications](https://assets.moen.com/shared/docs/product-specifications/gxp50csp.pdf)
provide the340mm height and136mm width.
[GS181210B specifications](https://assets.moen.com/shared/docs/product-specifications/gs181210bsp.pdf)
provide the787×457mm sink envelope,737×406×152.4mm bowl and92mm drain.
The914.4mm counter height and25mm top are disclosed ordinary-service-layout
assumptions. The150mm turntable depth and internal lug/ring dimensions are photo
estimates; no unpublished OEM precision is claimed. The published108mm
dishwasher inlet and175mm discharge levels constrain the turntable estimate.
The earlier75mm estimate was rejected before any native run; it placed the
turntable implausibly above the dishwasher inlet.

`mechkit.py` is copied from the heat-shrink-lug mechanism helper; `simready.py`
is the shared pinned NVIDIA asset loader. No validation, Dream evaluation or
submission has been performed for this folder.
