#!/usr/bin/env python3
"""Rolling quota for new task PRs only; callers hold submission.lock on writes."""
import argparse
import re
from datetime import datetime, timedelta, timezone
from pathlib import Path


def recent_submissions(text, now):
    cutoff = now - timedelta(hours=24)
    first_submitted = {}
    for line in text.splitlines():
        if not line.strip():
            continue
        fields = line.split("\t")
        if len(fields) != 3:
            raise ValueError("malformed submission ledger row")
        stamp = datetime.fromisoformat(fields[0].replace("Z", "+00:00"))
        if stamp.tzinfo is None or stamp > now:
            raise ValueError("submission timestamp has no timezone or is in the future")
        pr = fields[2].rstrip("/")
        if not fields[1] or not re.fullmatch(r"https://github\.com/robocurve/benchmark-tasks/pull/[1-9][0-9]*", pr):
            raise ValueError("submission row needs a task folder and canonical task PR URL")
        # Older ledgers may contain revision attempts. A PR consumes a slot only
        # when it was first submitted, even if revised inside this window.
        first_submitted[pr] = min(stamp, first_submitted.get(pr, stamp))
    return sorted(stamp for stamp in first_submitted.values() if stamp > cutoff)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ledger", type=Path)
    args = parser.parse_args()
    try:
        recent = recent_submissions(args.ledger.read_text(), datetime.now(timezone.utc))
    except (ValueError, OSError) as exc:
        raise SystemExit(f"submit: cannot verify shared quota: {exc}")
    if len(recent) >= 20:
        available = recent[-20] + timedelta(hours=24)
        raise SystemExit(f"submit: new-task rolling limit reached ({len(recent)}/20); next slot {available.isoformat()}; revisions are exempt")
    print(f"submit: shared new-task rolling quota {len(recent)}/20 (revisions exempt)")


if __name__ == "__main__":
    main()
