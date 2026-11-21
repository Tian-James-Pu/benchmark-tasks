# Starting a second Claude Code session on another machine

The state of the work at 22 Nov 2026, about 00:00 UTC. The deadline is 23 Nov at 11:59 UTC (Sunday 23:59 AoE).
The goal is as many accepted tasks as possible.

## 1. Clone and set up

    git clone -b team-workspace https://github.com/Tian-James-Pu/benchmark-tasks.git benchmark-tasks
    cd benchmark-tasks
    git remote add upstream https://github.com/robocurve/benchmark-tasks.git
    git config user.name robo-alex
    git config user.email mz8690@princeton.edu
    gh auth login                     # as the team's registered account, Tian-James-Pu

Copy the credentials by hand from the first machine. Never commit them:

    ~/.robocurve/credentials          # ROBOCURVE_TEAM=T-18, ROBOCURVE_TOKEN=...
    ~/.dreamlabs/credentials          # Dream-1 API key

The simulator is Isaac Lab-Arena with the `yam_arena` embodiment and the `inspect-robots` CLI, from the Robocurve
Brev launchable. `pipeline/run_trial.sh` expects the venv at `~/IsaacLab-Arena/.venv` and writes output under
`~/yam-eval/<task>/`. Check the install with the scripted example in CLAUDE.local.md (section 2) before building.

On this machine:
- Set `pipeline/gpu-allow` to the GPU indices you may use, separated by spaces, e.g. `0 1 2 3 4 5 6 7`.
- `GPU_SLOTS` sets trials per card (default 3; an L40S can take 3-4).
- `run_trial.sh` schedules only on the local node. With several nodes, run one Claude session per node, or extend
  the script.

Copy `CLAUDE.local.md` from the first machine into the repo root. It is not committed. Among other things it holds
the asset rule: every recognisable real object is a SimReady prop, plus 2-4 SimReady distractors.

## 2. What to read first (in this order)

1. `pipeline/ACCEPTANCE_GUIDE.md`: what accepted tasks have in common, and the gate to pass before building.
2. `pipeline/REVIEW_BAR.md`: every decision we got. Read the newest sections carefully (below).
3. `pipeline/CODEX.md`: workflow, submission safeguards and the novelty-review format. Also read `pipeline/BRIEF.md`
   and `pipeline/MECHKIT.md`.
4. Working task folders to copy from:
   - `tasks/reset-shower-chair-short-leg/`: our one accepted task, #2406.
   - `tasks/take-2x-photo-with-iphone-camera-control/` and `tasks/close-mason-jar-two-piece-lid/`.
   - `tasks/reassemble-nesting-dolls/`.
   - `tasks/start-car-with-jammed-steering-lock/`: a demokit closed-loop demo.

## 3. Hard-won lessons (21 Nov)

- **Human reviewers close almost everything on importance.**
  - All 8 of our revised tasks that a human reviewed were closed with one-line reasons: "too simple",
    "not relatable to this category", "hobby-specific", "already-covered ability", "does not test model safety".
  - Passing the agent review's importance check does not predict acceptance. The skill must be visibly hard: real
    two-handed coordination, a narrow window held under load, or several dependent stages, on something most adults
    have done.
- **Robot fit.**
  - Builds of heavy, stiff, out-of-reach or ballistic ideas fail.
  - Disclosed rescaling of big objects (at least about 80% of real size) is allowed.
  - But a mass cut so the arms can lift it fails feasibility (#3524, a bifold door lightened from 9-10 kg to 2.4 kg).
- **Demos.** Use a closed-loop demokit demo when open-loop waypoints aren't reliable.
- **Dream-1 refusals.**
  - If Dream-1 refuses an instruction, never reword it.
  - Instead, set scores to "Dream-1 run: refused the instruction before acting, nothing scored; demo: 1.0" and add
    `no_model_video_reason` with the exact error text. `submit.sh` then uploads without a model video, as accepted
    #2523 did.
- **Other teams' holds.**
  - Other teams usually revise minutes to hours before their hold lapses (four times on 21 Nov).
  - Don't plan around claiming lapsed holds.
  - Never submit another team's idea before its lapse.
- **Appeals.** All 5 of our appeals are used.
- **Disk.** It fills fast. Delete `~/yam-eval/<task>/frames/` and `*.npy` after each run; keep `media/`.

## 4. Splitting work between the two machines

`pipeline/claims/` is a local folder, so the two machines cannot see each other's claims. To avoid building the
same idea twice:
- **First machine:** review watch, revisions of our open PRs, and the builds already running:
  - laminate end joint;
  - planter, oven door, TV plate, SIM punch, balsa glider, snap-off cutter, pull-up bar;
  - the nesting dolls and cabinet hinge hold claims (lapsing 22 Nov 07:41 and 07:44 UTC).
- **This machine:** new original ideas only.
  - Prefix every new slug with `m2-` in the claim, for example `pipeline/claims/m2-<slug>`, while keeping the task
    folder name plain.
  - Before building, run `pipeline/refresh_known.sh` and `gh pr list --repo robocurve/benchmark-tasks --author
    Tian-James-Pu --state all`, and skip anything already there.
  - Don't revise our existing PRs from here.

## 5. Suggested first instruction to the new Claude session

> Read NEW_MACHINE.md, then pipeline/ACCEPTANCE_GUIDE.md, pipeline/REVIEW_BAR.md (newest sections) and
> pipeline/CODEX.md. Check that the simulator works with the scripted example in CLAUDE.local.md. Set
> pipeline/gpu-allow to this machine's GPUs. Then run a workflow that proposes original task ideas checked against the
> live board and our own PRs, and builds them in parallel with closed-loop demos. Submit each with pipeline/submit.sh
> as soon as it validates (demo 1.0 twice, red team passes, Dream-1 once, novelty review). Follow section 4 of
> NEW_MACHINE.md to avoid overlapping with the other machine. Commits and PRs are authored as robo-alex
> <mz8690@princeton.edu> with no AI attribution. Never commit credentials, pipeline/ or CLAUDE.local.md to upstream
> PRs.
