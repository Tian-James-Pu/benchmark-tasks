# Reassemble an electric carving knife after washing

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena),
Isaac Lab-Arena 0.3.1) on the station scene: [task.py](task.py).

An electric carving knife's two serrated blades lie washed and apart on the counter beside the unplugged power handle.
The robot has to join them the way the maker's manual describes: put the keyhole in one blade over the rivet on the
other, then slide the blades so the rivet runs into the narrow part of the slot and locks them together. Then it
slides the joined tails into the socket in the handle until both sprung latches click into the windows in the tails.
The handle is free on the counter; the demo braces it with the second arm, though in the simulator the insertion needs so little force that one arm can manage (see the red-team results).

## Run a trial

From the repository root, with the Arena environment active (`source ~/IsaacLab-Arena/yam.env`):

```bash
# Dream-1 (needs DREAM_API_KEY), 2,400 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/reassemble-carving-knife-blades/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Put the carving knife's two blades back together and click them into the handle." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/reassemble-carving-knife-blades-dream

# Scripted demo (both arms, open loop), 900 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/reassemble-carving-knife-blades/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=640 -E cam_height=360 \
  --instruction "Put the carving knife's two blades back together and click them into the handle." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/reassemble-carving-knife-blades-demo
```

- The run prints `task_score` at the end. Videos: `inspect-robots video <log>.json`.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The demo is open loop and deterministic.
- Runtime: the demo plan is 36.5 s (730 steps); a trial takes about 4 minutes on an L40S including
  start-up and rendering. A Dream-1 trial (2,400 steps) takes about 8 minutes.

## The demo

The left arm grips blade B by its guard, carries it over blade A with the keyhole's round end over A's rivet, lowers
it so the rivet head passes through, lets go, then grips B's guard again and slides it 7.5 mm back so the rivet runs to
the end of the narrow slot (the guards now line up). It then grips both guards together, lifts the pair, lines the
tails up with the socket and slides them in until both latches drop into their windows. Meanwhile the right arm
holds the back of the handle against the push. Both arms then leave.

## Scoring

All read from poses each control step (`Trial` in `task.py`); on a real knife a feeler gauge at the mouth and a pull test on each blade would do (see Real trials).

- **Joined:** A's rivet in B's narrow slot (3.5 to 6.5 mm past the keyhole's centre toward the tip, within 2 mm of the
  slot's centre line), the blades face to face within 0.6 mm and parallel within 4 deg. With the rivet there its head
  overlaps the slot's sides and holds the blades together.
- **A blade latched:** its latch lip within 0.35 mm of its rest stop, so down in the tail's window; the blade's guard
  within 1.5 mm of the socket's mouth; its tail's end inside the socket (within 2 mm of the centre line sideways and
  within the socket's 4 mm height), so a blade laid over the nose with its guard at the mouth does not count.
- **1.0:** joined, both blades latched, every part at rest (under 1 cm/s and 0.2 rad/s) and every fingertip more than
  4 cm from both guards and from the mouth, for 2 s without a break. Once reached it stays.
- Otherwise the end score is 0.25 for joined plus 0.25 for each latched blade (at most 0.75).
- **0** if any part ever falls more than 5 cm below the table top (off the counter), whatever happens before or after,
  a full score included. That record is cleared only by a reset (mechkit's `on_reset` hook).

Red-team checks, run on the scripted policy as variants of the demo (copies of `task.py` with a different `demo()`):

- **Do nothing** (both arms wait): 0.0.
- **Careless** (B dropped over the rivet but never slid along the slot, then the pair inserted): 0.25. The rivet stays in the round part of the keyhole, so the blades never count as joined and the tails sit 7.5 mm out of step; only one of the three 0.25 conditions is met at the end.
- **One arm** (the left arm does everything, nobody braces the handle): 1.0. This is a legitimate completion, not an exploit: the latch cam's insertion force is not modelled (see below), so the light push does not shove the free handle away. On a real knife the click takes more force and the handle would need holding.

The demo scored 1.0 in five consecutive runs, the last after the latch check was tightened to require the tail inside the socket.

Dream-1 (one run, 2,400 steps) scored 0.0: it brought both grippers to the loose blades and nudged the handle, but never picked up a blade, so the blades were neither joined nor inserted.

## Real trials

- **Equipment:** a Hamilton Beach 74250 electric knife (or any electric carving knife with a rivet-and-keyhole blade
  pair and a two-click handle; its blades match the setup to within a few millimetres), unplugged for the whole trial,
  plus the sponge and mug. Mark the counter with tape: an outline for each blade and for the handle at the positions
  in the setup.
- **Preparation and reset:** press the release button and pull the blades out, slide the rivet back to the round end
  of the keyhole, separate the blades and lay each on its outline (A guard down, B guard up, tips toward +y), and lay
  the handle on its side on its outline, socket facing +y. Nothing is consumed.
- **Measuring at the end (after the robot has let go and 2 s with nothing moving):**
  - *Joined:* the guards line up (the rivet is at the end of the narrow slot); holding blade A by its guard, blade B
    does not come away when lifted straight off, and the rivet is visible in the narrow part of the slot.
  - *Each blade latched:* the guard sits within 1.5 mm of the handle's mouth (a feeler gauge or the edge of a 1.5 mm
    card), and a light pull on that blade alone, without pressing the release button, does not draw it out.
  - *Fingertips clear:* from the video, no gripper within 4 cm of a guard or the mouth at the end.
  - *Off the counter:* any part on the floor or more than 5 cm below the counter top at any time scores 0.
- **Safety:** the knife stays unplugged; the serrated blades are handled by their guards when resetting.

## Importance and sources

Electric carving knives are a common household appliance, used for holiday roasts and bread, and every one of them
is taken apart for washing and put back together before use. The maker's instructions for the Hamilton Beach electric
knife ([Use & Care, model 74250](https://dijf55il5e0d1.cloudfront.net/pdf/HamiltonBeachElectricKnifeOperatingManual.pdf),
page 4, "How to Assemble and Use") say: "If blades are separated, pick up by the plastic guard. Slide the rivet on one
blade into the keyhole slot in the other blade until they are locked together." and "Holding blades together at the
base of the blade with cutting edge down, slide onto motor housing until both blades click into place." The
[Hamilton Beach 74378R Use & Care manual](https://manualmachine.com/hamiltonbeach/74378r/21189998-use--care-manual)
gives the same rivet-and-keyhole step, and the
[74250 manual on ManualsOnline](https://powertool.manualsonline.com/manuals/mfg/hamilton_beach/74250.html?p=3)
describes the blade release button that frees the latches.

For a robot the task is a precise two-part assembly of thin, sharp, slippery parts: a 2 mm rivet through a 7 mm
keyhole and a slide of a few millimetres that the eye can barely see, then a blind insertion of two 1.2 mm tails into
a 4 mm socket against two spring latches, in a handle that lies free on the counter (on a real knife the second hand
braces it; in the simulator one arm can manage). Partial credit separates the
joint from each latch; whether a latch has caught shows only in the guard reaching the mouth.

## Assets and helpers

The distractors are NVIDIA SimReady props, downloaded at first run from
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets), pinned revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`. © NVIDIA, CC-BY 4.0. Nothing large is committed.

- [`cleaning_sponge_b01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/cleaning_sponge_b01) (distractor): mass set to 0.02 kg, convex collider.
- [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01) (distractor): mass set to 0.30 kg, convex collider.

SimReady has no electric knife, so the blades and the handle are custom meshes built in `task.py` with `mechkit.py`
(every piece a convex hull; the handle is a free articulation whose two latch lips are sprung sliders).

**The latch cam is a rule.** In the simulator a sprung lip would not ride up a ramp on the end of a 1.2 mm steel
tail (the tail stalled against it whatever the spring and ramp), so `lip_wanted()` in `task.py` decides each lip's
position from the tail's: the lip is lifted clear while its blade's tail passes under it and drops back (a
sprung slider, so it really moves) when the tail's window lies under it. The cam's insertion force is not modelled
(the arm in the simulator stalls against even 1 N here), so the push needs little force. Once a lip is down in its window, its deep face physically stops the blade being pulled out. `simready.py`
is the open-source loader from `tasks/cutters-off-pegboard`; `mechkit.py` is the shared mechanism kit from
`tasks/heat-shrink-lug`, unchanged.
