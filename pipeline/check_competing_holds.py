#!/usr/bin/env python3
"""Veto known competing ideas using live official decisions, never grant novelty.

The registry is deliberately narrow. An expired/final/withdrawn competitor still
needs the existing written novelty review; this helper does not change it.
"""
import datetime as dt
import json
from pathlib import Path
import re
import subprocess
import sys

OFFICIAL = {"COLLABORATOR", "MEMBER", "OWNER"}


def timestamp(value):
    parsed = dt.datetime.fromisoformat(value.replace("Z", "+00:00"))
    if parsed.tzinfo is None:
        raise ValueError("decision timestamp has no timezone")
    return parsed


def hold_status(pr, comments, now):
    """Return (held, explanation); raise on unverified or malformed live state."""
    if pr.get("merged_at"):
        return True, "accepted/merged; the idea is already taken"
    state = pr.get("state")
    if state not in {"open", "closed"}:
        raise ValueError("unknown pull-request state")
    decisions = []
    for comment in comments:
        if comment.get("author_association") not in OFFICIAL:
            continue
        body = comment["body"].strip()
        if body.startswith("**Not accepted: final**"):
            kind = "final"
        elif body.startswith("**Accepted**"):
            kind = "accepted"
        elif body.startswith("Received as ") or "already waiting for review" in body:
            kind = "pending"
        elif body.startswith("**Not accepted**"):
            kind = "revising"
        else:
            continue
        when = timestamp(comment["created_at"])
        if when > now:
            raise ValueError("official decision is in the future")
        decisions.append((when, kind))
    if not decisions:
        # Even a closed PR without an official decision needs a manual check.
        raise ValueError("no recognized official review/receipt")
    when, kind = max(decisions, key=lambda item: item[0])
    if kind == "accepted":
        return True, "latest official decision is accepted"
    if kind == "final":
        return False, "final rejection; this does not establish novelty or importance"
    if state == "closed":
        return False, "withdrawn/closed; fresh novelty review is still required"
    if kind == "pending":
        return True, "latest official receipt is pending review (including revisions)"
    until = when + dt.timedelta(hours=48)
    if now < until:
        return True, "revision hold until " + until.isoformat()
    return False, "48-hour hold expired; fresh novelty review is still required"


def gh_json(endpoint, *options):
    result = subprocess.run(
        ["gh", "api", endpoint, *options], check=True, capture_output=True,
        text=True, timeout=45,
    )
    return json.loads(result.stdout)


def fetch_pr(repository, number):
    pr = gh_json(f"repos/{repository}/pulls/{number}")
    # Read every comment page: a revision receipt can supersede an old rejection.
    pages = gh_json(f"repos/{repository}/issues/{number}/comments?per_page=100",
                    "--paginate", "--slurp")
    if not isinstance(pages, list) or any(not isinstance(page, list) for page in pages):
        raise ValueError("invalid paginated comment response")
    return pr, [comment for page in pages for comment in page]


def check(task, registry, now=None):
    repository = registry["repository"]
    if not re.fullmatch(r"[\w.-]+/[\w.-]+", repository):
        raise ValueError("invalid registry repository")
    numbers = registry["tasks"].get(task, [])
    if not isinstance(numbers, list) or any(type(n) is not int or n <= 0 for n in numbers):
        raise ValueError("invalid competing PR list")
    for number in numbers:
        pr, comments = fetch_pr(repository, number)
        held, reason = hold_status(pr, comments, now or dt.datetime.now(dt.timezone.utc))
        url = f"https://github.com/{repository}/pull/{number}"
        if held:
            raise ValueError(f"{task}: {url}: {reason}")
        print(f"ownership: {url}: {reason}")
    if numbers:
        print("ownership: no live hold in the known-conflict registry; novelty is NOT cleared")


def main():
    try:
        registry = json.loads(Path(__file__).with_name("competing-holds.json").read_text())
        check(sys.argv[1], registry)
    except (OSError, ValueError, KeyError, TypeError, IndexError, subprocess.SubprocessError) as exc:
        # Do not echo command stderr (which may include account details).
        if isinstance(exc, subprocess.SubprocessError):
            detail = "live GitHub query failed or timed out; ownership is unverified"
        else:
            detail = str(exc)
        raise SystemExit("ownership: refusing new submission: " + detail)


if __name__ == "__main__":
    main()
