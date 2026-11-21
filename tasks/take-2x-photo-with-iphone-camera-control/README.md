# Take a 2x photo with an iPhone 16's Camera Control

An Apple iPhone 16 stands in landscape in a desk phone stand. Its screen faces the arms and its rear cameras face an
artificial foam apple. The Camera app is open at 1x, and Zoom is the setting the Camera Control last adjusted. The
instruction is only: **Use the phone's Camera Control to take a photo at 2x zoom.**

On iPhone 16 and later, zoom and shutter are one 17 mm sapphire button on the phone's edge. A light press shows
the zoom dial, sliding a finger along the control zooms, and a click (a full press) takes the photo. For a robot,
the hard part is force on one small button. It must press just hard enough to open the dial (about 0.5 N here),
slide about 6 mm along a 17 mm strip with no click (under 2 N), and then click. If it presses straight down, the
photo comes out at 1x. If it presses too hard while it slides, the click comes first and the photo is again at 1x.

## Response to review

| Review request | Change |
|---|---|
| "A robot with touch-capable pads could select 2x and take a photo on the touchscreen without performing the specified Camera Control gesture and still receive full credit." | `task.py`: the screen has no touch input, and any fingertip point within 1 cm in front of the screen glass at any time records `screen_touched`, which scores 0 for the rest of the trial (cleared only in `on_reset`). Photos come only from a Camera Control click and zoom only from a slide along it. Stated in `task.json` `scoring` and in Scoring below. Red team: touching the screen, then doing the full Camera Control demo, scores 0; the same route with the finger kept 6 cm off the screen scores 1.0. |
| "Require observable use of the Camera Control for photo credit, and require its slide for zoom credit, so touchscreen shortcuts cannot earn the corresponding scores." | README, Real trial, Measurement steps 2-4: photo credit needs camera C1 to show a fingertip pressing the Camera Control at the frame where C2 shows the shutter flash; zoom credit needs C1 to show the slide along the control while C2 shows the zoom dial; any screen contact seen on C2 scores 0. |
| "Replace the 0.3 mm frame-to-frame motion measurement with a practical photo or video outcome check." | The motion-blur rule (2 cm/s at the click, read from 60 fps video) is removed from `task.py`, `task.json` and the README. It is replaced by end-state checks: the phone seated (lower corners within 5 mm of tape marks, back against the rest; in sim, all four screen-face corners within 5 mm), never more than 2 cm off its marks before full credit, and the photo showing the apple with no gripper over the lens. |
| "Specify camera views that show the relevant control and shutter actions." | README, Real trial, Cameras: C1, a close-up 15 cm above and 10 cm behind the Camera Control, framed on the 17 mm control and the fingertip; C2, a screen view 35 cm in front of the phone showing the zoom dial, zoom chip, shutter flash, thumbnail and any screen contact. |
| "Provide a compatible iPhone 16 sourcing route costing no more than a few hundred dollars, such as an available used or refurbished unit." | README, Real trial, Phone: refurbished unlocked 128 GB iPhone 16 from US$360 (UpTrade), US$478.99 (Reebelo) or US$659 (Apple Certified Refurbished), checked 21 November 2026 with links. Any iPhone 16-series phone has the Camera Control; the iPhone 16e does not and is excluded. |
| Flag: "specify a conductive, compliant pad if needed for the phone's touch-sensitive Camera Control." | README, Real trial, Fingertip covers: a 3 mm silicone-foam pad wrapped in silver-plated knit conductive fabric (Adafruit 1167, or a Razer silver-fibre finger sleeve), grounded to the arm by a thin wire, with a pre-trial check that a light press opens the zoom dial. Also noted in `task.json` `setup`. |
| Flag: "use an artificial cake or another repeatable dry photo subject." | `task.py`, `task.json`, README: the cake slice is replaced by a foam artificial Red Delicious apple (SimReady `food_produce_apple_redDelicous_a01` in sim; Hobby Lobby item 1591, foam, 3.5 x 3 x 3 in, for the real trial). The mug and eyeglasses distractors are unchanged. The scene changed, so the demo and the Dream-1 run were redone and the videos and screenshot re-uploaded. |

## Importance and sources

- **Everyday operation:** 91% of US adults own a smartphone ([Pew Research Center, Mobile Fact Sheet, 2024](https://www.pewresearch.org/internet/fact-sheet/mobile/)).
  That most of them take and zoom photos with it is our inference, not a measured figure.
- **The real control:** Apple's iPhone User Guide documents the light press, the slide that adjusts the setting,
  and the click that takes the shot ([Use the Camera Control](https://support.apple.com/guide/iphone/use-the-camera-control-iph0c397b154/ios)).
  It was introduced with the iPhone 16 and iPhone 16 Plus ([Apple Newsroom](https://www.apple.com/newsroom/2024/09/apple-introduces-iphone-16-and-iphone-16-plus/)).
- **The physical knack:** force held inside a narrow band on one small button while sliding along it. The press
  forces are our estimates; Apple does not publish them.

## Run and score

From the repository root, in the Python environment that has `yam_arena` and `inspect-robots`
(`source ~/IsaacLab-Arena/yam.env`):

```bash
# scripted demonstration
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/take-2x-photo-with-iphone-camera-control/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "Use the phone's Camera Control to take a photo at 2x zoom." \
  --scorer task_score --max-steps 900 --store-frames \
  --log-dir ~/yam-eval/take-2x-photo-demo

# model trial
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/take-2x-photo-with-iphone-camera-control/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Use the phone's Camera Control to take a photo at 2x zoom." \
  --scorer task_score --max-steps 1200 --store-frames \
  --log-dir ~/yam-eval/take-2x-photo-dream
```

The run prints `task_score`. Nothing is random: `object_jitter=0`, and no seed is read. The scripted demo is open
loop and one-armed. It reaches full credit at step 265 (13.3 simulated seconds), and the episode ends there. A demo
trial took 2.5 to 3 minutes of wall time on one GPU, start-up included. The run limit is 1,200 steps (60 s), about
four times the demo. In its 1,200-step run on this revision's scene, Dream-1 reached its right gripper towards the phone and then drew
both arms back; it took no photo that scored, the phone stayed in its stand, and the score was 0.

## The phone and its Camera Control

- **Product:** Apple iPhone 16, US$799 at launch ([Apple Newsroom](https://www.apple.com/newsroom/2024/09/apple-introduces-iphone-16-and-iphone-16-plus/));
  refurbished from US$360 (sources in the real-trial section).
  Size 147.6 x 71.6 x 7.80 mm and mass 170 g ([Apple tech specs](https://support.apple.com/en-us/121029)). The
  model is a rigid box at that size and mass. Its inertia is that of a uniform box.
- **Camera Control position:** from Apple's accessory dimensional drawing
  ([iPhone 16 PDF](https://developer.apple.com/download/files/accessories/dimensional-drawings/iphone-16.pdf)):
  centre 96.23 mm from the phone's top end, 2 x 8.55 = 17.1 mm long, on the side edge, about 3 mm across. In
  landscape with the cameras to the left, this edge is on top and the control is 51.4 mm from the right-hand
  end. The rear camera plateau and the two 16 mm lenses come from the same drawing (detail D).
- **Behaviour:** from Apple's iPhone User Guide
  ([Use the Camera Control](https://support.apple.com/guide/iphone/use-the-camera-control-iph0c397b154/ios);
  [Camera basics](https://support.apple.com/guide/iphone/camera-basics-iph263472f78/ios)). Apple says to "Slide
  your finger on the Camera Control to adjust the setting. The setting is saved and appears the next time you
  lightly press the Camera Control", and to "Click the Camera Control to take your shot". The tech specs list
  Zoom among the control's settings, and a 2x telephoto at 52 mm.
- **Sensor model (in code: `Phone.step`):** the cap is a slider on a 2,500 N/m spring, so its travel stands for
  the force on the control's sensor. A light press is 0.2 mm or more (0.5 N) but short of the click, held 2 control
  steps. The click is 0.8 mm (2.0 N), and it re-arms above 0.4 mm. Apple does not publish these forces; they are
  estimates. The touch is contact: the cap is pressed at least 0.04 mm and a fingertip is over it. The touch
  position is the mean position of that fingertip along the phone. A contact lost for under 0.15 s counts as the
  same touch.
- **Zoom (estimates):** while the dial is open, the zoom doubles for each 6 mm the finger slides towards the
  cameras' end, from 0.5x to 10x. The dial has detents at 1x and 2x: from 1.80x to 2.22x it reads exactly 2x. The
  dial closes 2 s after the finger leaves the control, and the zoom stays where it was.
- **Screen:** the viewfinder (a green-grey patch), the zoom dial with its marker (yellow at 2x), the zoom chip
  (yellow at 2x), a white flash at each photo, and the last photo's thumbnail. A policy can see all of these from
  the top camera. The screen has no touch input: touching it changes nothing and voids photo credit (Scoring).

## Scoring

The score reads the phone's latest photo and the phone's pose. In the simulation **only the Camera Control can take a
photo or zoom**: a click takes a photo at the zoom its dial shows, and only a slide along the control changes the
zoom. The screen has no touch input at all (no shutter button, no zoom chip, no pinch). A screen touch also **voids
photo credit**: if a fingertip point is ever within 1 cm in front of the screen's glass, inside its outline, the score
is 0 for the rest of the trial. This mirrors the real trial, where a touchscreen does respond to conductive pads, so
any screen contact seen in the screen-side video gives no credit (below). The dial reads exactly 2x (52 mm) from 1.80x
to 2.22x. A photo is **blocked** if, at the click, a fingertip point was within 1.5 cm of a rear lens's centre. The
phone is **seated** when each of the four corners of its screen face is within 5 mm of its settled start.

- **1.0:** the latest photo is unblocked and at 2x. The phone is seated and still (under 1 cm/s and 3 degrees/s),
  and every fingertip point of both grippers (each finger's tip and a point 3 cm up it) is at least 3 cm from the
  phone. All of this holds for 1 s, and the score is then 1.0 for the rest of the trial, unless the screen is
  touched.
- **0.75:** the latest photo is unblocked and at 2x, but the full conditions are not met (for example, a finger
  is still on the control).
- **0.5:** the latest photo is unblocked at 1.5x to 3.0x (39 to 78 mm), other than 2x.
- **0.25:** the latest photo is unblocked at any other zoom (for example, 1x after a straight press).
- **0:** no photo; or the latest photo is blocked; or the screen was touched at any time; or, before full credit, a
  screen-face corner was ever more than 2 cm from its settled start (the phone knocked out of its stand). The
  screen-touch and knocked-out records are kept across steps and cleared only in mechkit's `on_reset` hook.

Possible scores: 0, 0.25, 0.5, 0.75, 1.0. Retakes are allowed: only the latest photo counts.

## Real trial: rig, cameras, preparation, reset and measurement

- **Phone:** one dedicated Apple iPhone 16 (any colour, any storage), no case, on a current iOS. Every iPhone
  16, 16 Plus, 16 Pro and 16 Pro Max has the Camera Control; the iPhone 16e does not and cannot be used. Used and
  refurbished units cost well under the US$799 launch price. Prices we checked on 21 November 2026 for an unlocked
  128 GB iPhone 16:
  - [UpTrade](https://uptradeit.com/buy-used-refurbished-iphone-16) (refurbished, 30-day returns): from **US$360**
    (Excellent, black), US$400 (Pristine).
  - [Reebelo](https://reebelo.com/collections/iphone-16) (refurbished): from **US$478.99** (Good).
  - [Apple Certified Refurbished](https://www.apple.com/shop/refurbished/iphone/iphone-16) (new battery and outer
    shell, one-year warranty): **US$659**.
  - [Swappa](https://swappa.com/listings/apple-iphone-16?carrier=unlocked) and
    [Back Market](https://www.backmarket.com/en-us/p/iphone-16) list used units too, but their pages refused our
    automated check, so we quote no price from them.

  The phone is not consumed or worn: the only contact is a fingertip pressing under 3 N on its sapphire control.
- **Stand and marks:** a reclining aluminium desk phone stand (about 15 degrees, a silicone ledge with a 3 mm lip,
  for example any adjustable aluminium desk stand of that shape, about US$10-15), taped to the table so the phone's
  centre is at (30, 0) cm. Put tape marks on the ledge at the phone's two lower corners. Check before each trial
  that seating the phone takes no photo (its weight on the ledge must not press a side button).
- **Fingertip covers (the Camera Control is capacitive as well as force sensing):** on each gripper finger, a 3 mm
  self-adhesive silicone-foam pad, wrapped in a 25 x 40 mm piece of silver-plated stretch knit
  ([Adafruit 1167 Knit Conductive Fabric](https://www.adafruit.com/product/1167), US$9.95 per 20 cm square, under
  1 ohm per foot; a [Razer Gaming Finger Sleeve](https://www.razer.com/gear-accessories/razer-gaming-finger-sleeve),
  silver-fibre knit 0.8 mm thick, slit open, works too). The fabric is held by a cable tie behind the pad and
  connected by a thin stranded wire to the arm's grounded frame, so the control senses it as it senses a finger.
  Before the first trial, hand-guide a covered finger onto the control: a light press must show the zoom dial.
- **Photo subject:** an artificial foam Red Delicious apple about 8 cm across and 9 cm tall (for example
  [Hobby Lobby Red Delicious Apple, item 1591](https://www.hobbylobby.com/Floral-Wedding/Floral-Supplies/Faux-Food/Red-Delicious-Apple/p/1591),
  3.5 x 3 x 3 in, foam, US$2.99). It is dry, reusable and identical every trial; put it on a tape outline at (56, 6) cm.
- **Cameras (both 60 fps, fixed on tripods, running for the whole trial):**
  - **C1, Camera Control close-up:** 15 cm above and 10 cm behind the phone's top edge, at the control (31, -2) cm,
    looking down at the top edge so the whole 17 mm control and the fingertip on it fill a third of the frame. It
    shows the light press, the slide along the control and the click.
  - **C2, screen view:** 35 cm in front of the screen at the screen's height, over the arms' bases, square to the
    screen and showing the whole phone face, both lower corners and the ledge. It shows the zoom dial, the zoom
    chip, the shutter flash and thumbnail at each photo, and any contact with the screen.
- **Preparation:** in Settings > Accessibility > Camera Control, keep the light-press gesture on at the default pressure.
  Open Camera with a click on the Camera Control. Lightly double-press, slide to Zoom, and lightly press it, so that
  Zoom becomes the saved setting. Slide back until the dial reads 1x, and let the dial close. Wipe the screen.
- **Measurement (at the end, before anyone touches the phone):**
  1. *Seated:* both lower corners on their tape marks (within 5 mm, ruler) and the phone's back against the back
     rest (no visible gap). A phone off its marks by more than 2 cm, or out of the stand, at any time before full
     credit scores 0.
  2. *Photo by the Camera Control:* on C2, find the frame where the latest photo's shutter flash starts. On C1, at
     that frame, a fingertip must be on the Camera Control and pressing it. If not, the photo gets no credit.
  3. *No screen contact:* on C2, no gripper touches the screen glass at any time in the trial. Any contact scores 0.
  4. *Zoom by the slide:* open Photos, the latest photo, swipe up for Info. 52 mm is 2x; 39 to 78 mm other than
     52 mm is the 0.5 level; any other focal length is 0.25. A zoom other than 1x (26 mm) counts only if C1 shows
     the fingertip sliding along the control while C2 shows the zoom dial open; with no screen contact (step 3),
     nothing else can change the zoom. Check that the photo's time is after the trial's start.
  5. *Unblocked:* the photo shows the apple, with no gripper in front of the lens.
  6. *Full credit:* C2 shows the phone still and both grippers at least 3 cm from it for 1 s after the photo (judge
     3 cm against the phone's 7.2 cm width).
- **Reset (about 1 minute):** note the latest photo's time, or delete the trial's photos. Lightly press the
  Camera Control and slide back to 1x. Set the phone back on its tape marks, wipe the screen, and put the apple
  back on its outline. Nothing is consumed.
- **Rig safety:** forces are under 3 N, nothing is hot, sharp or wet, and the covers protect the glass from the pads.

## Distinct from

| Nearby task | What it tests | Here |
|---|---|---|
| PR #2524, phone screenshot buttons (accepted) | two side buttons pressed at once | one button, three force levels, and a slide along it |
| PR #2534, pinch-zoom a map on a tablet (accepted) | two-finger zoom on a touchscreen | zoom by sliding along a pressure-sensing side button, then a click on the same button |
| PR #3362, phone panorama (pending) | turning a held phone about its camera | the phone stays in a stand; the skill is graded press and slide |

We know of no task in LIBERO, RoboCasa, BEHAVIOR or SimplerEnv that operates a phone's camera button. This was
not checked exhaustively.

## Simulation notes and honest limits

- **One arm is enough here.** The phone rests on the stand's ledge and back rest, and a press of under 3 N on
  its top edge does not move it (under 0.001 mm in the demo). Steadying it with the other hand is allowed but not
  needed. A careless push that knocks it out of the stand scores 0.
- The cap's collider stands 1.1 to 2.1 mm proud of the frame so that a rigid gripper pad presses it and not the
  frame. The real control is flush and is pressed by a soft fingertip.
- The phone's surfaces use friction 0.35, roughly that of glass and sapphire against a fingertip. This is lower
  than the station's rubber-like 1.2, which made the finger stick and jump. The stand uses 1.2.
- The scripted demo commands the finger 2.5 mm into the cap for the light press, 1.2 mm while it slides 7.5 mm,
  and 8 mm for the click. The arm's own compliance turns these commands into roughly 0.7 N, 0.4 N and the full
  2.5 N stop. The finger lags the command by about 1.5 mm, so it really slides about 6 mm.

## Assets and helpers

Phone, cap and stand are authored parts (mechkit meshes and colliders). The apple (photo subject) and the two
distractors are NVIDIA SimReady props. They are downloaded at first run from
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets) at pinned revision
`9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd`, © NVIDIA, CC-BY 4.0:

- [`food_produce_apple_redDelicous_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/food_produce_apple_redDelicous_a01):
  the photo's subject (stands for a foam artificial apple), loose, mass set to 0.04 kg (convex-hull collision at load; the authored round collider rolled off its spot as the scene settled).
- [`dishware_mug_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/dishware_mug_a01):
  distractor, loose, mass set to 0.30 kg (convex-hull collision at load).
- [`accessory_eyeglasses_a01`](https://huggingface.co/datasets/nvidia/simready-assets/tree/9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd/accessory_eyeglasses_a01):
  distractor, loose, mass set to 0.03 kg (any joints are welded into one body by the loader).

`score()` ignores all three. No meshes or textures are bundled. `simready.py` is adapted from the open-source
`cutters-off-pegboard` loader. `mechkit.py` is the shared mechanism kit, copied unchanged from `heat-shrink-lug`
(another team's open-source task).
