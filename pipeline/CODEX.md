# Handoff for a Codex agent: keep building Safe Recovery tasks

## Mandatory GPU restriction (user instruction, 17 November 2026)

Use only physical GPUs **2 and 3** for all trials (temporary exception from 19 Nov: the user allowed 0 and 1 while free; `pipeline/gpu-allow` holds the current list, and is reset to "2 3" when the labmate needs them) and GPU diagnostics. Leave GPUs **0 and 1** unused by our work. This persists across sessions. `pipeline/run_trial.sh` filters its allocator to 2/3 and masks the selected physical GPU by UUID; inside the child process it is logical `cuda:0`. Do not bypass that mask or mistake logical `cuda:0` for physical GPU 0. Verify actual process placement with `nvidia-smi`, including incidental CUDA contexts. Queue when 2/3 are occupied; never spill onto 0/1. Do not stop unrelated users' processes.

Start Codex in the repository root (`~/code/benchmark-tasks`) and tell it: "Read pipeline/CODEX.md and follow it."
`AGENTS.md` in the root, which Codex reads on its own, has the program rules and the five review criteria. This file
has everything else. Claude Code agents work in the same checkout at the same time; coordinate through the files
named here.

## The goal

We submit simulated robot tasks to the Robocurve Benchmarking Program, one pull request per task, from the GitHub
account `Tian-James-Pu`. Accepted tasks count; the five teams with the most accepted tasks win $20,000 each. The final
deadline is **Sunday 22 November 2026, 23:59 AoE** (Monday 23 November, 06:59 New York).

Our theme is recovery: a manipulation job has gone wrong before the episode starts (the fault is in the starting
layout), and the robot must put it right. The theme is ours; the reviewers judge each task on its own merits.

**The bar is high.** Of about 2,350 task PRs so far, 181 were accepted and about 1,575 closed for good. Most fail on
importance before anything else is read. `pipeline/REVIEW_BAR.md` sums up what the reviewers accept and reject, with
their own words. Read it before anything else, and test every idea against it.

## Where things stand (16 November 2026, evening)

- **Submitted:** `reseat-board-missed-support` (PR #2398, TK-3JS2ME), sent as a probe to see how reviewers treat a
  recovery task. Two more probes are on their way: `restack-shifted-middle-carton` (re-pitched as Impact on jobs) and
  `rechock-two-panes-one-prop`. `pipeline/submitted.tsv` lists every PR we open.
- **Held, do not submit:** the other 10 folders built on 16 November: `reseat-shelf-prop-under-load`,
  `chock-leaning-panel-in-floor-channel`, `free-riser-foot-from-pencil-under-open-jar`,
  `shelve-tool-tin-with-unlatched-lid`, `level-from-under-loose-shelf`, `slide-mug-off-retaining-rod`,
  `cue-ball-held-by-chalk-on-return-track`, `lean-a-standing-tile-into-its-rack`, `clear-offcuts-propped-board`,
  `remove-prop-from-under-shelf`. They run, but they protect objects rather than people, spell out the safety step,
  or are variants of each other: the pattern reviewers close for good. They are useful only as code examples.
- **Being built by Claude agents now:** the concepts in `pipeline/concepts/` that have a matching folder in
  `pipeline/claims/` (pedal bin, cabinet door, kitchen drawer, shower chair, pre-rinse sprayer, IV roller clamp),
  plus concepts from data-centre servicing and the skilled trades, and from logistics, household jobs and hazards
  to people. Check `pipeline/claims/` before choosing anything.
- **Unclaimed concepts** in `pipeline/concepts/`: an infusion pump that slid down its pole (rated weak by its own
  author) and a patient's arm through a bed rail (close to another team's pending task). Build them only if you can
  answer those objections.

## 1. Find an idea worth building

An idea must pass all of these, or it is not worth a day of work:

- **A category bar, met plainly.** Impact on jobs: a named occupation with more than about 500,000 US workers whose
  core, regular work is this manipulation. Safety and alignment: a hazard to a person (burn, shock, cut, a falling
  load, contamination) that the robot must notice although the instruction never mentions it. Societal
  relatability: an everyday job nearly everyone does. Impact on AI timelines: servicing robots, data centres or AI
  hardware with the real parts.
- **A physical difficulty,** ideally two-handed, not a visual, memory or rule puzzle.
- **Real objects at real scale,** with real mechanisms (hinges, slides, springs, latches), not labelled boxes.
- **No duplicate** of any accepted or pending task: `pipeline/refresh_known.sh`, then grep
  `pipeline/known_tasks.tsv` for the objects and the skill. Also not a variant of one of our own folders.

Claude's food-service and factory-assembly round (17 November) judged all 12 concepts not worth building (blender jar,
beer faucet, soda gun, steam wand, grinder hopper, hot cup under spouts, hydraulic hose, engine air cleaner, service
cover, weld gusset, spring hose clamp, carriage bolt); don't redo them. The data-centre and trades round did the same
(1U patch panel, robot Han connector, MIG nozzle, pneumatic gripper release, fibre drawer, OSFP cable, furnace door,
gas pilot relight, remodel box, vent pipe, reversing starter); its one pick, a toilet flapper re-hang, failed an IK
reach check inside a real tank (scratchpad gate scripts; claim marked dropped). Claude is now generating concepts for
healthcare support (nursing assistants, home health aides), cleaning and housekeeping, warehouse and retail backroom,
home everyday life, data-centre servicing and the skilled trades. Angles left for Codex: the PDF candidates first,
then automotive service technicians, grounds maintenance and landscaping, childcare (hazards to a child), painters and
installers, and retail checkout.
Calibrating a strict reviewer (17 November): blind, from task.json text alone, a strict LLM reviewer gave three tasks
that were in fact accepted (#2049, #2110, #2100) 0.22, 0.25 and 0.10, and seven final rejections 0.04 to 0.15. So a
judged acceptance chance of about 0.2 is what accepted tasks look like on paper; build at 0.2 or more rather than
waiting for an "accept" verdict (Claude's earlier rounds built nothing because they waited).
Claude's wide-funnel round (17 November) brainstormed 99 one-line ideas, ranked them, and checked the best 8 at real
scale (product source, duplicates, IK reach). All 8 were killed, and their concept files record why:
- medicine-cabinet sliding mirror door: same skill as #1911's insect screen, which another team holds;
- wire-shelf split sleeve: two-piece sleeve can't be held by a parallel gripper;
- car floor liner under the pedal: out of reach;
- casement crank shoe: the real product makes it a one-arm job;
- rear-view mirror re-hang: weak relatability;
- pinched pull saw: wrong physics;
- microwave tray coupler: gear-phase duplicate;
- desk-lamp spring: not forced.

Lesson: brainstormed faults mostly die on "the real product doesn't do that". Claude's next round starts from faults
that manufacturers' manuals document (troubleshooting tables), with the manual's quoted fix as the forced skill.

That manual round (17 November) found 32 documented faults. All four that it checked in depth were killed:
- hole-saw core: occasional work, and next to #2182;
- coil-element drip bowl: the ordinary refit again;
- dishwasher top rack: a one-hand job on the real product;
- stand-mixer bowl: #2475 holds it.

18 November: Claude took over from the Codex session. Codex's four new PRs (#2500, #2520, #2544, #2609) are now on
the tracker, and #2418 is marked final with Codex's human-review request. Claude is now mining vendor service guides
for Impact on AI timelines recovery faults: servers, GPUs, robots and lab automation.

Write each idea as a short concept file in `pipeline/concepts/<slug>.json` (fields: title, category, justification,
fault, instruction, objects, skill, scoring, nearest, risks), and argue against it as a strict reviewer before you
build it.

Before demo tuning, compare the defining mechanism against a primary manufacturer drawing/manual or an ordinary
rebuildable real object. Record the matching contact surfaces, lever direction, clearances, scale and load, and cite
the source in the concept. Do not add gripper tabs, reverse a lever or invent a fixed support just to make the robot
succeed. A custom build plan alone is insufficient if the mechanism does not represent the claimed real work.
See the 17 November decision examples at the end of `pipeline/REVIEW_BAR.md`.

## 2. Claim it

    mkdir pipeline/claims/<slug> && echo codex > pipeline/claims/<slug>/owner

`mkdir` fails if someone already holds it: pick another. Keep a one-line state in `pipeline/claims/<slug>/status`
(`building <folder>`, `ready <folder>`, `submitted <url>`, `abandoned: <reason>`). Claude reads these to update the
team's tracker page.

## 3. Build it

Follow `pipeline/BRIEF.md` exactly: simulator facts, the SimReady rule (every recognisable object, distractors
included, is an NVIDIA SimReady prop; primitives only for custom structural parts), two to four distractors, the
pitfalls already hit, the trial runner and the definition of done with red-team runs. For mechanisms read
`pipeline/MECHKIT.md` and copy `pipeline/lib/mechkit.py` (the standard kit; credit it in the README). Good accepted
examples of that kit: `tasks/heat-shrink-lug/` and `tasks/bookcase-shelf-pins/`.

The simulator is installed on this machine: `source ~/IsaacLab-Arena/yam.env`. Run trials only through
`pipeline/run_trial.sh`; it shares only authorized physical GPUs 2 and 3 between all agents (three trials per GPU) and writes the
videos, the screenshot and a contact sheet under `~/yam-eval/<task>/media/`. Look at the contact sheet after every
run. Demo mode stops at 900 steps unless you pass a step count (`pipeline/run_trial.sh <task> demo 1400`).

Validate before calling it done: the scripted demo scores 1.0 in two fresh runs; every red-team variant (do nothing,
one arm, the careless route, holding instead of releasing) scores what the scoring text says; the scoring text
matches the code; `fidelity` is at most 500 characters; nothing in the folder names a person, team, institution or
home path, or refers to `pipeline/` (the PR carries only the task folder).

## 4. Dream-1 run (once per task)

When the task is final (scene, instruction and scoring will not change again):

Run `pipeline/submit.sh <task> --preflight` first. It checks the source package and field limits while allowing
the Dream score to remain pending; it does not replace the two fresh demos, red-team trials or visual review.

    pipeline/run_trial.sh <task> dream [steps]     # steps = the task's run_limit, default 2,400

It spends the team's Dream-1 credits ($20 for the whole program, no top-ups; the balance is on the Billing page of
platform.dreamlabs.ai). One run per finished task, never while iterating. Runs are logged in
`pipeline/dream_runs.tsv`. Put the score into `task.json` as `"scores": "Dream-1 run: <score>; demo: 1.0"`. A low
Dream-1 score is normal.

## 5. Submit

    pipeline/submit.sh <task> --check    # all the checks, no side effects
    pipeline/submit.sh <task>            # uploads media, opens the PR

It refuses missing or over-long fields, pending scores, leftovers, identifying text, `pipeline/` references, missing
media and an existing PR. It commits as `robo-alex <mz8690@princeton.edu>` from a temporary worktree of upstream
`main`, so the shared checkout is never switched, and adds no AI attribution. The Robocurve token is in
`~/.robocurve/credentials`. Within a few minutes the program's bot comments on the PR: either "Received as TK-…" or
what to fix before intake (for example a field over its limit); fix and push with `pipeline/revise.sh`.

Submit as soon as a task is ready. At most 20 new tasks in any 24 hours, ours and Claude's together
(`pipeline/submitted.tsv`).
The submission script now holds a shared lock through upload, PR creation and ledger append, and refuses a
21st new task in the rolling 24-hour window. Revisions to existing PRs are exempt. Do not bypass it with a separate submission path.

## 6. Follow up on reviews

    pipeline/check_prs.sh --open

shows each open PR's labels and latest comment. "Not accepted" (revisable) must be revised within 48 hours or the
team loses its hold on the idea: fix what the review says, say in the README what changed in response, rerun the
demo (and Dream-1 if the scene or instruction changed), re-upload media, then `pipeline/revise.sh <task> "what
changed"`. "Not accepted: final" cannot be revised. Add anything new the reviewers teach us to
`pipeline/REVIEW_BAR.md`. Never push to a PR that is still waiting for its first review; those pushes are ignored.

## Never

- commit or push anything except a task folder through `submit.sh` or `revise.sh`. `pipeline/`, `task_list.md`,
  `CLAUDE.local.md` and the brief PDF stay out of git (they are excluded locally);
- edit `AGENTS.md`, `README.md`, `tools/`, or a task folder or claim you do not own;
- write the Dream key, the Robocurve token or any credential into a file in the repository;
- submit one of the held folders, or a task that does not meet `pipeline/REVIEW_BAR.md`: repeated low-quality
  submissions can get the team disqualified;
- run Dream-1 on a task that is still changing.

## Active review/novelty follow-up,17November2026

PRs2398,2400 and2401 were finally rejected at03:04UTC;2406 is awaiting first review. Read the latest local `pipeline/pr-watch-latest.json` at each resumed work turn, including CLOSED PRs: `--open` alone misses new final rejections. `pipeline/pr_watch.py` polls all registered-author/ledger PRs every two hours and saves changes locally; it never posts or modifies PRs. Consume requested revisions promptly; do not revise final rejections.

Before preflight/Dream and submission, refresh `pipeline/known_tasks.tsv`, read nearest live PRs, and write `pipeline/novelty-reviews/<folder>.json`: `checked_at` UTC, `decision`=`clear`, nonempty `queries`, `nearest_prs`, `physical_skill`, `valid_shortcuts`, `distinct_from_nearest`, plus `task_py_sha256` and `task_metadata_sha256` (SHA256 of `json.dumps(task_json_without_scores_or_media, sort_keys=True).encode()`). Review is valid two hours and only for the reviewed source/description. `submit.sh` enforces this evidence gate; it is not an automatic semantic duplicate detector. Do not write `clear` without reading and reasoning through the comparisons.


## Submission safeguards, 17 November 2026

Current user target is **50 validated tasks**, with the unchanged team-wide **20 new tasks per rolling 24 hours** ceiling. Validation may continue while quota is full. Both `submit.sh` and `revise.sh` hold the shared submission lock through publishing and ledger updates. Per the board update supplied by the user at 23:18, revisions are exempt from the new-task quota. `check_submission_quota.py` refuses malformed/future ledger timestamps. Reconcile external team submissions before publishing; this local counter cannot discover submissions made outside the shared process.

`revise.sh` checks the latest official PR decision, refuses pending/final/closed PRs, runs `submit.sh --revision-check` without swallowing errors, and uploads corrected media before committing. Revision attempts are audited separately in `pipeline/revision_attempts.tsv` and never consume slots. The quota checker deduplicates historical revision rows by PR, using its earliest recorded submission. Five regression tests in `pipeline/test_submission_guards.py` cover quota boundaries, review eligibility and failed-preflight publication prevention.

## Candidate source priority, 17 November 2026

The user explicitly replaced `task_list.md` as a planning source with `task_candidates.pdf`. Ignore `task_list.md`; leave it untouched as historical material. Use `pipeline/pdf-candidates-2026-11-16.json` for the 100 extracted PDF candidates and `pipeline/pdf-screening/` for fresh evidence. PDF priority labels and proposed scores are hypotheses, not validated results or novelty clearance. Prioritize the PDF for new work while completing already-running checks and monitoring existing PRs.

## Dream-1 exhaustion exception, 17 November 2026

The user authorizes skipping Dream-1 if the API credits are exhausted. Continue hardware, scripted-demo, adversarial, visual and scorer validation without waiting for more credits. Record model evaluation as not run due to exhausted credits; never report a score or model video from a scripted demo. An active run that is still progressing is not proof of exhausted credits. Preserve the distinction between physical validation and submission eligibility when a model video is unavailable.

## Importance citations and immediate submission priority, 18 November 2026

The user requests ten additional submissions (nineteen total from the current nine), while retaining the fifty-validated-task goal. Submit each task as soon as it passes; do not wait to finish the batch.

For future PRs, always cite supporting sources for importance where available. Put a concise `## Importance and sources` section in the task README with direct primary-source links beside the claims they support. Explain the specific real operation, its occupational or everyday relevance, and any evidence limits. `submit.sh` includes this section and the task's context in the public PR description via `render_pr_body.py`; preview that output before publishing. Keep all submission material blind.

Distinguish importance evidence from hardware fidelity evidence. A manufacturer manual can establish a real operation; it does not by itself prove widespread use. An occupation's total workforce does not establish that every worker performs the specific operation. Cite dates for statistics, label inferences, and never invent a source or overstate what it proves. Preserve citations in any appeal or revision as well. If no adequate source is available, state the limitation in the local review rather than fabricating support.

## Expired holds, 18 November 2026

The user approved building other teams' expired-hold ideas, including ones that aren't recovery tasks. Read the new
REVIEW_BAR.md section "Claiming other teams' expired holds" first:
- original teams revise late;
- other teams reclaim lapsed ideas within hours;
- claimed ideas get a fresh importance review.

Of the first eight that Claude picked, seven were already re-held or reclaimed. The builders stopped themselves; only
the keyed pulley (#2035) went on to be built. A second batch, taken from holds that lapsed today, is building:
- Sankey keg coupler (#1831);
- tyre pencil gauge (#1896);
- rack-and-pinion gripper (#1898);
- add-on board on headers (#1891);
- servo horn on a spline (#1864);
- glued dowels (#2003).

Each build re-checks the lapse before submitting.


## Scope is open, 18 November 2026

The user said tasks don't have to be related to safety or recovery. Choose any task that clears REVIEW_BAR.md on
importance and novelty. The user pointed to https://generalistai.com/ demos as good models:
- packing and box folding;
- kitting;
- copy-building;
- robot-vacuum servicing;
- threading bolts.

The tasks accepted most recently on the board are small everyday jobs with one specific physical knack, for
example the SIM eject pin, the CD jewel case, the straw through a lid's X-slot, and coins into a piggy bank.
Claude has open-scope rounds running for everyday put-back tasks and for Generalist-style packing, assembly and food
service tasks. A salvage pass over Codex's 20 half-built folders is also running.

## Held batch-1 folders released, 19 November 2026

The user asked to submit the 10 held batch-1 folders too. A Claude workflow hardens each one first, against the
#2398, #2400 and #2401 lessons:
- the violation history is cleared only in on_reset;
- "seated" and "released" checks are tight and use fingertips;
- the instruction is goal-only;
- the README has an importance section with sources.

Each folder then gets its Dream-1 run and novelty review, and is submitted. A folder stops instead if the board now
holds a duplicate of it.

