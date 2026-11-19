#!/usr/bin/env bash
# Write pipeline/known_tasks.tsv: every task PR in the official repo with its state and title, so a new idea can be
# checked against accepted AND pending tasks (a near-duplicate of either, by any team, is rejected).
#   pipeline/refresh_known.sh        then: grep -i '<object or skill>' pipeline/known_tasks.tsv
set -euo pipefail
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
TMP=$(mktemp "$REPO/pipeline/known_tasks.XXXXXX")
trap 'rm -f "$TMP"' EXIT
gh pr list --repo robocurve/benchmark-tasks --state all --limit 5000 --json number,title,state,labels \
  --jq '.[] | [.number, (if .state=="MERGED" then "accepted" elif ([.labels[].name]|index("Not accepted: final")) then "rejected" elif ([.labels[].name]|index("Not accepted")) then "revising" elif .state=="OPEN" then "pending" else "closed" end), .title] | @tsv' \
  | sort -n > "$TMP"
[[ -s $TMP ]] || { echo "refresh_known: empty response; preserving previous snapshot" >&2; exit 1; }
mv "$TMP" "$REPO/pipeline/known_tasks.tsv"
cut -f2 "$REPO/pipeline/known_tasks.tsv" | sort | uniq -c
