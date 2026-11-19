#!/usr/bin/env python3
"""Refuse final rejections and PRs already received for review."""
import json
import sys


def revision_allowed(pr):
    if pr["state"] != "OPEN":
        return False
    if any(x["name"] == "Not accepted: final" for x in pr["labels"]):
        return False
    for comment in reversed(pr["comments"]):
        if comment.get("authorAssociation") not in {"COLLABORATOR", "MEMBER", "OWNER"}:
            continue
        body = comment["body"]
        if body.startswith("**Not accepted: final**") or body.startswith("**Accepted**"):
            return False
        if body.startswith("Received as ") or "already waiting for review" in body:
            return False
        if body.startswith("**Not accepted**") or body.startswith("This pull request couldn’t be submitted yet:"):
            return True
    return False


if __name__ == "__main__":
    if not revision_allowed(json.load(open(sys.argv[1]))):
        raise SystemExit("revise: no actionable revision/intake-fix decision; closed, final, pending or unverified PR")
