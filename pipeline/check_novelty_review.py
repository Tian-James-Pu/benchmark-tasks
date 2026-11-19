#!/usr/bin/env python3
"""Require a recent, source-matched written novelty review before submission.

This is an evidence gate, not an automatic semantic duplicate detector. The
reviewer must read the nearest live PRs and analyze valid alternate solutions.
"""
import datetime as dt
import hashlib
import json
import pathlib
import sys

root = pathlib.Path(__file__).resolve().parent
folder = pathlib.Path(sys.argv[1])
review_path = root / "novelty-reviews" / (folder.name + ".json")
def fail(message):
    raise SystemExit("novelty: " + message)

if not review_path.exists():
    fail("missing fresh written review: " + str(review_path))
review = json.loads(review_path.read_text())
now = dt.datetime.now(dt.timezone.utc)
try:
    checked = dt.datetime.fromisoformat(review["checked_at"].replace("Z", "+00:00"))
    if checked.tzinfo is None or not 0 <= (now-checked).total_seconds() <= 7200:
        fail("review must be from the past two hours")
except (KeyError, ValueError):
    fail("checked_at must be an explicit UTC timestamp")
known = root / "known_tasks.tsv"
if not known.exists() or now.timestamp()-known.stat().st_mtime > 7200:
    fail("refresh known PRs before reviewing novelty")
if review.get("decision") != "clear":
    fail("review has unresolved duplicate concerns")
for field in ("queries", "nearest_prs", "physical_skill", "valid_shortcuts", "distinct_from_nearest"):
    if not review.get(field):
        fail("review lacks " + field)
actual = hashlib.sha256((folder / "task.py").read_bytes()).hexdigest()
if review.get("task_py_sha256") != actual:
    fail("scene/scorer changed after novelty review")
metadata = json.loads((folder / "task.json").read_text())
for key in ("scores", "media"):
    metadata.pop(key, None)
digest = hashlib.sha256(json.dumps(metadata, sort_keys=True).encode()).hexdigest()
if review.get("task_metadata_sha256") != digest:
    fail("task description changed after novelty review")
print("novelty: recent written skill/shortcut review matches this task")
