#!/usr/bin/env bash
# Submit one finished task as its own pull request to robocurve/benchmark-tasks.
#
#   pipeline/submit.sh <task-folder>            check everything, upload media, open the PR
#   pipeline/submit.sh <task-folder> --check    only run the checks
#   pipeline/submit.sh <task-folder> --preflight  source checks before spending Dream credits
#
# Needs: ROBOCURVE_TEAM and ROBOCURVE_TOKEN in the environment or in ~/.robocurve/credentials (KEY=value lines,
# chmod 600); gh logged in as the team's registered account; media from pipeline/run_trial.sh in
# ~/yam-eval/<task>/media/ (dream.mp4, demo.mp4, setup.png). Optional: COMMIT_TRAILER and PR_FOOTER are appended
# to the commit message and the PR description (none by default). Commits are authored as robo-alex
# <mz8690@princeton.edu>; AUTHOR_NAME and AUTHOR_EMAIL override that.
#
# The PR is built in a temporary git worktree from upstream main, so the working checkout (where other tasks are
# being built) is never switched or touched. It refuses to run if a PR already exists for the branch.
set -euo pipefail

REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
TASK=${1:?task folder name under tasks/}
CHECK_ONLY=false; [[ ${2:-} == --check ]] && CHECK_ONLY=true
REVISION_CHECK=false
if [[ ${2:-} == --revision-check ]]; then REVISION_CHECK=true; CHECK_ONLY=true; fi
PREFLIGHT=false
if [[ ${2:-} == --preflight ]]; then PREFLIGHT=true; CHECK_ONLY=true; fi
DIR=$REPO/tasks/$TASK
MEDIA=$HOME/yam-eval/$TASK/media
ACCOUNT=Tian-James-Pu
UPSTREAM=robocurve/benchmark-tasks
LEDGER=$REPO/pipeline/submitted.tsv
fail() { echo "submit: $*" >&2; exit 1; }

# Serialize uploads and PR creation across all builders sharing this checkout.
# Hold the lock until the successful submission is recorded in the shared ledger.
if ! $CHECK_ONLY; then
  exec 9>"$REPO/pipeline/submission.lock"
  flock 9
  python3 "$REPO/pipeline/check_submission_quota.py" "$LEDGER"
fi

[[ -f $DIR/task.json && -f $DIR/README.md && -f $DIR/task.py ]] || fail "$DIR needs task.json, README.md and task.py"
python3 - "$DIR/task.json" "$PREFLIGHT" <<'PY' || exit 1
import json, sys
d = json.load(open(sys.argv[1]))
need = ["title", "category", "instruction", "setup", "scoring", "run_limit", "fidelity", "scores"]
missing = [k for k in need if not str(d.get(k, "")).strip()]
cats = {"Impact on jobs", "Impact on AI timelines", "Safety and alignment", "Societal relatability", "Other"}
errs = []
if missing: errs.append(f"missing fields: {missing}")
if d.get("category") not in cats: errs.append(f"category {d.get('category')!r} is not one of {sorted(cats)}")
if sys.argv[2] != "true" and "pending" in d.get("scores", "").lower(): errs.append("scores still says pending: run the Dream-1 trial first")
# Field limits: fidelity's 500 is enforced by the intake bot; the others are the longest values in accepted tasks.
limits = {"title": 120, "context": 1000, "fidelity": 500, "instruction": 600, "setup": 4000, "scoring": 4000,
          "reset_notes": 1000, "run_limit": 200, "scores": 300}
long = [f"{k} is {len(d[k])} characters (max {n})" for k, n in limits.items() if len(str(d.get(k, ""))) > n]
if long: errs.append("; ".join(long))
if errs: sys.exit("submit: " + "; ".join(errs))
PY
leftovers=$(cd "$DIR" && ls -d redteam_* probe_* __pycache__ *.jsonl *.log 2>/dev/null || true)
[[ -z $leftovers ]] || fail "remove leftovers first: $leftovers"
# The PR carries only the task folder: no references to our private pipeline/ tooling.
if grep -rIn "pipeline/" "$DIR" --include='*.md' --include='*.json' --include='*.py'; then
  fail "the task refers to pipeline/ files, which are not part of the PR; remove those references"
fi
# Blind review: nothing that names the team, its members, institution, accounts or home paths.
if grep -rIn -i -E "mzhang|mingtong|princeton|tian-james|james pu|safe recovery|t-18|/home/|neuronic" "$DIR"; then
  fail "identifying text found above; remove it"
fi
# `prism` is also the standard geometry kit's function name, and
# PhysicsPrismaticJoint is a USD type. Scan prose for the team name without
# rejecting these Python identifiers or changing the shared kit's API.
python3 "$REPO/pipeline/check_prism_identity.py" "$DIR" || exit 1
python3 "$REPO/pipeline/check_novelty_review.py" "$DIR" || exit 1
if $PREFLIGHT; then
  echo "submit: $TASK passes source preflight (simulation evidence, final scores and media still required)"
  exit 0
fi
for f in dream.mp4 demo.mp4 setup.png; do [[ -s $MEDIA/$f ]] || fail "missing $MEDIA/$f (run pipeline/run_trial.sh)"; done
for f in dream.mp4 demo.mp4; do
  [[ $(stat -c %s "$MEDIA/$f") -le 52428800 ]] || fail "$f is over 50 MB"
  ffprobe -v error -select_streams v:0 -show_entries stream=codec_name -of csv=p=0 "$MEDIA/$f" 2>/dev/null | grep -q h264 \
    || [[ ! -x $(command -v ffprobe) ]] || fail "$f is not H.264"
done
gh auth status 2>&1 | grep -q "account $ACCOUNT" || fail "gh is not logged in as $ACCOUNT"
existing=$(gh pr list --repo "$UPSTREAM" --author "$ACCOUNT" --head "$TASK" --state all --json url --jq '.[].url')
if $REVISION_CHECK; then
  [[ -n $existing ]] || fail "no existing PR for revision: $TASK"
else
  [[ -z $existing ]] || fail "a PR already exists for $TASK: $existing (push revisions to it instead)"
fi
echo "submit: $TASK passes the checks"
$CHECK_ONLY && exit 0

if [[ -z ${ROBOCURVE_TOKEN:-} && -f $HOME/.robocurve/credentials ]]; then
  set -a; . "$HOME/.robocurve/credentials"; set +a
fi
[[ -n ${ROBOCURVE_TEAM:-} && -n ${ROBOCURVE_TOKEN:-} ]] || fail "set ROBOCURVE_TEAM and ROBOCURVE_TOKEN (or ~/.robocurve/credentials)"
if ! python3 -c 'import json,sys; sys.exit(0 if json.load(open(sys.argv[1])).get("media") else 1)' "$DIR/task.json"; then
  python3 "$REPO/tools/upload_media.py" "$DIR" --video "$MEDIA/dream.mp4" --demo "$MEDIA/demo.mp4" --screenshot "$MEDIA/setup.png"
fi

TITLE=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["title"])' "$DIR/task.json")
WT=$(mktemp -d "${TMPDIR:-/tmp}/submit-$TASK-XXXX")
git -C "$REPO" fetch -q upstream main
git -C "$REPO" worktree add -q -B "$TASK" "$WT" upstream/main
trap 'git -C "$REPO" worktree remove --force "$WT" >/dev/null 2>&1 || true' EXIT
BODY_FILE="$WT/.task-pr-body.txt"
python3 "$REPO/pipeline/render_pr_body.py" "$DIR" "$BODY_FILE"
mkdir -p "$WT/tasks"
rsync -a --exclude __pycache__ "$DIR/" "$WT/tasks/$TASK/"
git -C "$WT" add "tasks/$TASK"
MSG="Add task: $TITLE"
[[ -n ${COMMIT_TRAILER:-} ]] && MSG=$(printf '%s\n\n%s' "$MSG" "$COMMIT_TRAILER")
git -C "$WT" -c user.name="${AUTHOR_NAME:-robo-alex}" -c user.email="${AUTHOR_EMAIL:-mz8690@princeton.edu}" commit -q -m "$MSG"
git -C "$WT" push -q -f origin "$TASK"
URL=$(gh pr create --repo "$UPSTREAM" --base main --head "$ACCOUNT:$TASK" --title "$TITLE" --body-file "$BODY_FILE")
printf '%s\t%s\t%s\n' "$(date -u +%FT%TZ)" "$TASK" "$URL" >> "$LEDGER"
echo "submit: opened $URL"
