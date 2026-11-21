# Fit a replacement lamp harp

> WORK IN PROGRESS, NOT READY TO SUBMIT: the scripted demo does not yet reach full score (the harp is not yet carried
> upright to the saddle reliably). Validation below is pending.

A small table lamp stands with its bulb fitted and no harp. Its new 9-inch harp lies flat on the table beside it. The
instruction is only: **Fit the new harp to the lamp.**

A detachable harp has two spring-steel legs whose ends sit in two C-shaped channels on the saddle under the socket,
and a small sliding ferrule on each leg that goes down over the joint to lock it. The legs rest 70 mm apart and the
channels are 64 mm apart, so the legs must be squeezed 6 mm closer and **held** there while each ferrule is pushed down.
Let go before a ferrule is down and that leg springs back out of its channel. One hand cannot both hold the squeeze and
slide a ferrule 12 mm down a leg next to the bulb, so it is a two-handed job: one hand holds the legs squeezed and
seated, the other pushes the ferrules down one at a time.

## Run and score

Use the Python environment that has `yam_arena` and `inspect-robots` (on the station: `source ~/IsaacLab-Arena/yam.env`).
From the repository root:

```bash
# scripted demonstration (fixed plan from the fixed starting layout)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/swap-table-lamp-harp-into-saddle/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "Fit the new harp to the lamp." \
  --scorer task_score --max-steps 900 --store-frames \
  --log-dir ~/yam-eval/lamp-harp-demo

# a model trial
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/swap-table-lamp-harp-into-saddle/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the new harp to the lamp." \
  --scorer task_score --max-steps 1800 --store-frames \
  --log-dir ~/yam-eval/lamp-harp-model
```

The run prints `task_score`. The layout is fixed (`object_jitter=0`) and nothing in the scene depends on the seed. The
demo's plan is about 40 simulated seconds (the trial ends on success at about 38 s, roughly 770 control steps); one
trial takes about 1.5 to 3 minutes of wall time on an RTX-class GPU once shaders are compiled. The run limit, 1,800
steps (90 s), is a little over twice the demo's time.

## The objects and the mechanism

All task objects are full size. The lamp, its saddle, socket and bulb, and the harp are authored in `task.py` from
meshes, because the SimReady library has no table lamp or harp; the candle, mug and glasses are SimReady props.

**The harp** is a standard two-piece detachable harp with a 1/8 IP saddle and a 1/4-27 finial thread, 9 inches from
the saddle to the top (Satco lists this harp in 6 to 15 inch heights; the 9 inch is item 90-921 in brass, 90-2002 in
nickel). The model: 3.2 mm wire, a straight lower section on each leg that rests 70 mm apart (centres), flaring to
about 100 mm across around the bulb, an 18 mm boss and a 12.7 mm 1/4-27 stud on top, 46 g in all. Each leg carries a
ferrule 9 mm across and 8 mm long; slid up, its lower end is 20.5 mm above the leg end; slid down 12 mm, it covers the
top of the saddle's channel.

**The springs.** A harp's legs flex from its shoulders. Each leg is a rigid wire on a hinge at the shoulder (205 mm above
the leg end), with a spring of 44 N m/rad toward its rest: holding a leg end 3 mm in takes about 5 N at the leg end
(about 5.6 N where the demo holds it, 90 mm up). This stiffness is an engineering estimate for 3.2 mm harp wire,
not a measured value; calibrate the real harp with a luggage scale before relying on the number.

**The saddle** is a 1.6 mm steel strap under the socket, its ends turned up into C-shaped channels 64 mm apart, open on
their outer side (the side the leg springs toward). Collision: an inner wall and two side walls 6 mm tall, 5.2 mm apart
inside, with a 2 mm flared lip on each side wall. A squeezed leg end can sit in a channel; let go, it springs out
through the open side and off the end of the strap.

**The lamp** is a small table lamp: a 13 cm base, a glazed ceramic body, a brass neck, the saddle 15 cm above the table,
a 34 mm by 63 mm brass E26 socket, and a clear A19 bulb (60 mm across, 113 mm long, its base 25 mm into the socket).
The lamp is held fixed in place: in a real trial it is taped down (see below).

**The ferrules' lock.** The contact of a ferrule over the channel, which is what really holds the leg in, is not
simulated. Instead, `Trial.step()` reads each leg end's position from the harp's pose; once a leg end is in its channel
with its ferrule clear above the channel (slid down no more than 9 mm, its lower end at least 11.5 mm above the leg end), and the ferrule is then pushed down at least 10.5 mm of its 12 mm
while the leg end stays in, that leg's hinge is locked where it is (a joint lock, as other accepted tasks use for
latches). With both legs held the harp is fixed to the saddle where it stands. Raising a ferrule, or a leg end leaving
its channel, frees that leg again. A ferrule pushed down before its leg is in the channel does not lock it: on the real
harp it would cover the leg end and stop it going in.

The harp is a free body: in `task.py` it is an articulation hung from the world by six free joints (three slides, three
hinges, with no stiffness, damping or friction), so that the ferrule lock can fix it to the saddle by locking those six
joints. Until then it falls, slides and is carried like any loose part.

## Scoring

Scored on the final state, every control step, from the harp's pose and joints:

- A leg end is **in its channel** when its centre is within 1.5 mm of the channel's centre radially (inward or out
  toward the open side), within 1.3 mm sideways, and from 0.8 mm below to 2.0 mm above the saddle's top.
- A leg is **held** when its ferrule was pushed down at least 10.5 mm of its 12 mm while that leg end stayed in its
  channel, as described above.
- **1.0:** both legs held; the harp's axis within 3 degrees of vertical; every fingertip point at least 3 cm from the
  harp (sampled along the wire, with 9 mm added for the boss and ferrules); the harp moving slower than 1 cm/s; all
  continuously for 3 s.
- **0.5:** both leg ends in their channels without full credit: for example the ferrules still up, or a hand still on
  the harp.
- **0.25:** exactly one leg end in its channel.
- **0:** anything else, including the starting layout.

Possible scores: 0, 0.25, 0.5, 1.0. `score()` and `success()` only read what `Trial.step()` computed; nothing is
cleared except in mechkit's `on_reset` hook. `SUCCESS_HOLD_S` is 0 because the trial counts its own 3 s hold; the episode
then ends with success.

## Real trial: preparation, reset and measurement

- **Parts:** any small table lamp with a 1/8 IP saddle under an E26 socket (saddle about 15 cm above the table), an A19
  bulb, and a 9-inch detachable harp with sleeves (ferrules) that fits that saddle; candle, mug and glasses as
  distractors. Tape the lamp's base to the table and mark the harp's outline where it lies.
- **Preparation check:** slide both ferrules up to 20.5 mm above the leg ends (a pencil mark on each leg), and check with
  a ruler that the leg ends rest about 70 mm apart and the channels are 64 mm apart. The bulb is cold and the lamp
  unplugged.
- **Measurement:** after the robot lets go, look and measure: each leg end inside its channel; each ferrule's lower end
  within 1.5 mm of covering the channel top (a ruler against the leg); the harp's top within 12 mm of plumb over the
  socket (3 degrees over its 229 mm height: a phone level or a plumb line); hands 3 cm clear; nothing moving for 3 s.
- **Reset (about 1 minute):** slide both ferrules up, lift the harp off and lay it flat at its marks.

## Distinct from

| Nearest task | What it tests | Why this one differs |
|---|---|---|
| Put a clip-on shade on a candlestick lamp (rejected) | presses a clip fitter onto a bulb | no sprung legs, no hold-while-locking, no ferrules |
| Swap a dead E27 bulb in a table lamp (rejected) | unscrews and screws a bulb | a screw thread, not a sprung part held while locked |
| Fit a split wall plate round a pipe (rejected) | spreads a plate past a pipe and lets it spring shut | the spring closes the part; here the spring opens it, and it must be held while two separate locks go on |
| Straighten a training doll's two sprung legs and close the footboard (accepted) | holds two sprung legs flat while the other arm closes a board | the held part must also be aligned into two 5 mm channels and then locked by two sliding ferrules on the held legs themselves |

No task on the board, and none we found in LIBERO, RoboCasa, BEHAVIOR or similar sets, fits a lamp harp.

## Importance and sources

Changing a lamp's harp (for a shade of another height) is a common household job, and the harp is a part of most table
lamps. Lamp makers' own instructions describe exactly this skill:

- Living Spaces' table-lamp assembly instructions: "Attach the harp (C) to the saddle (D) by pressing the arms of the
  harp together and inserting them into the saddle."
  https://www.livingspaces.com/globalassets/productassets/assemblyinstructions/30/3080/308043-28-inch-navy-ceramic-table-lamp-assembly-instructions.pdf
- eHow, "How to Connect the Harp on a Lamp": raise "the harp's two sliding cylinders, called ferrules", hold them
  raised "while squeezing the base of the harp together", insert "the notched ends of the harp into the corresponding
  C-shaped channels of the saddle", then "slide the ferrules down over the ends of the saddle to lock the harp".
  https://ehow.com/how_10035539_connect-harp-lamp.html
- Satco hardware catalogue, harps and accessories (page 154): heavy-duty harps with a 1/8 IP saddle and 1/4-27 thread in
  heights from 6 to 15 inches, including 9 inch (90-921, 90-2002, 90-2267, 80-2177).
  https://bralcoelectrique.fidelio.ca/Public/img/catalog/Satco-NuvoLighting_12567_154_CAT.pdf
- Wikipedia, "Lamp harp": a harp usually comes in two parts, the saddle fastened under the socket; 1/4-27 is the
  common finial thread. https://en.wikipedia.org/wiki/Lamp_harp
- Wikipedia, "A-series light bulb": an A19 is 60 mm across and about 110 mm long, usually on an E26 base in the US.
  https://en.wikipedia.org/wiki/A-series_light_bulb

The 64 mm channel spacing, the 6 mm of squeeze, the ferrule size and the spring stiffness are this task's stated build
values for a typical harp and saddle, not figures from one manufacturer's drawing; measure the real parts as above.

## Fidelity

Full-size custom geometry for the lamp and harp; the harp's spring is concentrated in two shoulder hinges; the ferrule's
grip is a joint lock rather than simulated contact (described above); the lamp is fixed; all contacts use uniform
rubber-like friction; there is no force sensing or wire bending beyond the hinges.

## The scripted demonstration

`demo()` is an open-loop plan from the fixed starting layout, played by `yam_scripted`; it reads no simulator state.
The right hand takes the flat harp by its finial stud, turns it upright and holds it hanging beside the lamp. The left
hand takes both legs 90 mm above their ends from the left side, squeezes them until the leg ends are 64 mm apart, and
the right lets go of the stud. The left carries the squeezed harp round the bulb, sets the leg ends in the channels and
keeps squeezing. The right comes back level from the right side with its fingers half open (6 mm between the pads)
astride each leg's wire just above its ferrule and pushes each ferrule 12 mm down. The left lets go and both arms
leave. The waypoints were set from a debugging run's measured settled poses.

## Validation

Not yet validated. Two debugging runs scored 0: the first lost the harp when the legs were pinched on the table (the
pads reached only the top of the 3.2 mm wire); the second picked it by the finial stud, but the stud pivoted in the grip
and the harp hung 13 degrees off vertical and yawed about 20 degrees, so the other hand could not take the legs.

## Assets and helpers

The lamp, saddle, socket, bulb and harp are authored in `task.py`. The distractors are NVIDIA SimReady props, downloaded
at first run from the pinned revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd` (© NVIDIA, CC-BY 4.0):

- [decor_candle_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/decor_candle_a01): mass set to 0.30 kg.
- [dishware_mug_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01): mass set to 0.35 kg.
- [accessory_eyeglasses_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/accessory_eyeglasses_a01): mass set to 0.03 kg; its hinged temples welded into one rigid body at load.

Nothing large is committed; the loader fetches the pinned packages. `simready.py` is the loader from the open-source
`cutters-off-pegboard` task in this repository, unchanged. `mechkit.py` is the shared mechanism kit from the
open-source `heat-shrink-lug` task in this repository, unchanged.
