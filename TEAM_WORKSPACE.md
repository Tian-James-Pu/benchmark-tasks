# Team workspace branch

A snapshot of our local work so teammates can build and submit tasks in parallel. Task PRs still go to
`robocurve/benchmark-tasks` from a branch off upstream `main` that holds only one `tasks/<folder>/`. Never open a PR
from this branch.

## What is here
- `tasks/`: every task folder we have, both submitted and in progress. `pipeline/submitted.tsv` lists the PRs, and
  `pipeline/claims/<slug>/status` gives each task's state.
- `pipeline/`: the build and submit tooling.
  - `run_trial.sh` runs a trial on a free GPU. The GPUs it may use are listed in `pipeline/gpu-allow`.
  - `submit.sh` and `revise.sh` open and update PRs.
  - `check_prs.sh` shows review decisions.
  - `refresh_known.sh` updates `known_tasks.tsv`.
  - The docs are `BRIEF.md` (how to build a task), `MECHKIT.md`, `REVIEW_BAR.md` (what reviewers accept and reject;
    read it first) and `CODEX.md` (workflow and rules).
- Left out: downloaded third-party manuals and CAD, videos, raw frames, logs, PDFs and credentials.

## Working together
1. Claim before building: `mkdir pipeline/claims/<slug>` and put your name in `owner`. Keep a one-line `status`.
2. Check `REVIEW_BAR.md` and `known_tasks.tsv` for duplicates first.
3. Credentials live outside the repo, in `~/.robocurve/credentials` and `~/.dreamlabs/credentials`. Never commit them.
4. Task text, screenshots and videos must not identify the team, because review is blind.
