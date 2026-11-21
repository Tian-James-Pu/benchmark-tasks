#!/usr/bin/env bash
# Push a revision of an already submitted task to its existing PR branch, after a "Not accepted" review.
#
#   pipeline/revise.sh <task-folder> "What changed, in one line"
#
# Pushes made while a task is still waiting for its first review are not picked up, so only run this after the
# review comment arrives. Works in a temporary worktree; the working checkout is not touched.
set -euo pipefail
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
TASK=${1:?task folder}; WHAT=${2:?one-line summary of the revision}
DIR=$REPO/tasks/$TASK
exec 9>"$REPO/pipeline/submission.lock"
flock 9
# Keep the publication lock and review/preflight checks for revisions.
PR=$(gh pr list --repo robocurve/benchmark-tasks --author Tian-James-Pu --head "$TASK" --state all --json number --jq '.[].number')
[[ $PR =~ ^[0-9]+$ ]] || { echo "revise: expected exactly one existing PR" >&2; exit 1; }
REVIEW=$(mktemp "${TMPDIR:-/tmp}/revision-review-XXXXXX.json")
trap 'rm -f "$REVIEW"' EXIT
gh pr view "$PR" --repo robocurve/benchmark-tasks --json state,labels,comments > "$REVIEW"
python3 "$REPO/pipeline/check_revision_state.py" "$REVIEW"
"$REPO/pipeline/submit.sh" "$TASK" --revision-check
# A reviewed revision must reference media from its corrected scene, not the
# previous intake. Upload only after all source, score and media checks pass.
if [[ -z ${ROBOCURVE_TOKEN:-} && -f $HOME/.robocurve/credentials ]]; then
  set -a; . "$HOME/.robocurve/credentials"; set +a
fi
[[ -n ${ROBOCURVE_TEAM:-} && -n ${ROBOCURVE_TOKEN:-} ]] || { echo "revise: missing team upload credentials" >&2; exit 1; }
MEDIA=$HOME/yam-eval/$TASK/media
python3 "$REPO/tools/upload_media.py" "$DIR" --video "$MEDIA/dream.mp4" --demo "$MEDIA/demo.mp4" --screenshot "$MEDIA/setup.png"
git -C "$REPO" fetch -q origin "$TASK"
WT=$(mktemp -d "${TMPDIR:-/tmp}/revise-$TASK-XXXX")
git -C "$REPO" worktree add -q -B "$TASK" "$WT" "origin/$TASK"
trap 'git -C "$REPO" worktree remove --force "$WT" >/dev/null 2>&1 || true; rm -f "$REVIEW"' EXIT
rsync -a --delete --exclude __pycache__ "$DIR/" "$WT/tasks/$TASK/"
git -C "$WT" add -A "tasks/$TASK"
MSG="Revise task: $WHAT"
[[ -n ${COMMIT_TRAILER:-} ]] && MSG=$(printf '%s\n\n%s' "$MSG" "$COMMIT_TRAILER")
git -C "$WT" -c user.name="${AUTHOR_NAME:-robo-alex}" -c user.email="${AUTHOR_EMAIL:-mz8690@princeton.edu}" commit -q -m "$MSG"
# Audit attempts separately, including uncertain/interrupted pushes. Revisions
# never reserve or consume a new-task submission slot.
printf '%s\t%s\t%s\n' "$(date -u +%FT%TZ)" "$TASK" "https://github.com/robocurve/benchmark-tasks/pull/$PR" >> "$REPO/pipeline/revision_attempts.tsv"
git -C "$WT" push -q origin "$TASK"
echo "revise: pushed $TASK"
