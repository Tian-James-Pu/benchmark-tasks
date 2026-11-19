#!/usr/bin/env bash
# List the team's pull requests with their state, labels and the latest comment.
#   pipeline/check_prs.sh            all of them
#   pipeline/check_prs.sh --open     only open ones
set -euo pipefail
STATE=all; [[ ${1:-} == --open ]] && STATE=open
gh pr list --repo robocurve/benchmark-tasks --author Tian-James-Pu --state "$STATE" --limit 200 \
  --json number,headRefName,state,labels,url --jq '.[] | [.number, .headRefName, .state, ([.labels[].name] | join(","))] | @tsv' |
while IFS=$'\t' read -r num branch state labels; do
  last=$(gh pr view "$num" --repo robocurve/benchmark-tasks --json comments --jq '.comments[-1] | "\(.createdAt) \(.body)"' 2>/dev/null | tr '\n' ' ' | cut -c1-300)
  printf '#%s\t%s\t%s\t%s\n\t%s\n' "$num" "$branch" "$state" "${labels:--}" "${last:-no comments yet}"
done
