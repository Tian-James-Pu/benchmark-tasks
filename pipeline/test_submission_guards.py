import json
import os
from pathlib import Path
import subprocess
import shutil
import tempfile
import unittest
from datetime import datetime, timedelta, timezone

from check_revision_state import revision_allowed
from check_submission_quota import recent_submissions


class SubmissionGuards(unittest.TestCase):
    def test_rolling_window_not_calendar_day(self):
        now = datetime(2026, 11, 17, 3, tzinfo=timezone.utc)
        row = lambda t, pr=1: f"{t.isoformat()}\ttask\thttps://github.com/robocurve/benchmark-tasks/pull/{pr}\n"
        text = row(now - timedelta(hours=24), 1) + row(now - timedelta(hours=23, minutes=59), 2)
        self.assertEqual(len(recent_submissions(text, now)), 1)
        self.assertEqual(len(recent_submissions(''.join(row(now, pr) for pr in range(1, 21)), now)), 20)
        with self.assertRaises(ValueError):
            recent_submissions(row(now + timedelta(seconds=1)), now)
        with self.assertRaises(ValueError):
            recent_submissions("not-a-date\ttask\turl", now)
        with self.assertRaises(ValueError):
            recent_submissions("2026-11-17T02:00:00\ttask\turl", now)
        with self.assertRaises(ValueError):
            recent_submissions(f"{now.isoformat()}\ttask\tnot-a-pr", now)

    def test_historical_revisions_do_not_consume_slots(self):
        now = datetime(2026, 11, 17, 3, tzinfo=timezone.utc)
        row = lambda t, pr: f"{t.isoformat()}\ttask\thttps://github.com/robocurve/benchmark-tasks/pull/{pr}\n"
        # A revision to yesterday's PR costs zero; a recent new PR and its
        # revisions cost one. Reopening as a different PR costs another slot.
        text = row(now, 1) + row(now - timedelta(hours=25), 1)
        text += row(now - timedelta(hours=1), 2) + row(now, 2) * 25
        self.assertEqual(recent_submissions(text, now), [now - timedelta(hours=1)])
        self.assertEqual(len(recent_submissions(text + row(now, 3), now)), 2)

    def test_quota_refuses_twenty_but_allows_nineteen(self):
        with tempfile.TemporaryDirectory() as temp:
            ledger = Path(temp)/"ledger"
            stamp = datetime.now(timezone.utc) - timedelta(minutes=1)
            row = lambda pr: f"{stamp.isoformat()}\ttask\thttps://github.com/robocurve/benchmark-tasks/pull/{pr}\n"
            script = Path(__file__).with_name("check_submission_quota.py")
            ledger.write_text(''.join(row(pr) for pr in range(1, 20)))
            self.assertEqual(subprocess.run(["python3", script, ledger], capture_output=True).returncode, 0)
            ledger.write_text(''.join(row(pr) for pr in range(1, 21)))
            self.assertNotEqual(subprocess.run(["python3", script, ledger], capture_output=True).returncode, 0)

    def test_latest_official_decision_controls_revision(self):
        def pr(*bodies, state="OPEN", labels=()):
            return dict(state=state, labels=[{"name":x} for x in labels], comments=[
                {"body": b, "authorAssociation":"COLLABORATOR"} for b in bodies])
        self.assertTrue(revision_allowed(pr("Received as **TK-1**", "**Not accepted** (TK-1)")))
        self.assertFalse(revision_allowed(pr("**Not accepted**", "Received as **TK-2**")))
        self.assertFalse(revision_allowed(pr("**Not accepted: final**", state="CLOSED")))
        self.assertFalse(revision_allowed(pr("**Not accepted**", labels=["Not accepted: final"])))
        self.assertFalse(revision_allowed(pr()))
        self.assertTrue(revision_allowed(pr("This pull request couldn’t be submitted yet:\n- missing field")))
        p = pr("Received as **TK-1**")
        p["comments"].append({"body":"**Not accepted**", "authorAssociation":"NONE"})
        self.assertFalse(revision_allowed(p))

    def test_failed_preflight_never_runs_git(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            sentinel = root/"git-was-called"
            gh = root/"gh"
            review = {"state":"OPEN", "labels":[], "comments":[
                {"body":"**Not accepted**", "authorAssociation":"COLLABORATOR"}]}
            gh.write_text("#!/bin/sh\nif [ \"$2\" = list ]; then echo 99999; else cat <<'JSON'\n" + json.dumps(review) + "\nJSON\nfi\n")
            git = root/"git"
            git.write_text(f"#!/bin/sh\ntouch '{sentinel}'\nexit 99\n")
            gh.chmod(0o755); git.chmod(0o755)
            env = dict(os.environ, PATH=str(root)+os.pathsep+os.environ["PATH"])
            pipeline = root/"repo"/"pipeline"
            pipeline.mkdir(parents=True)
            for name in ("revise.sh", "submit.sh", "check_submission_quota.py", "check_revision_state.py"):
                shutil.copy2(Path(__file__).with_name(name), pipeline/name)
            stamp = datetime.now(timezone.utc) - timedelta(minutes=1)
            (pipeline/"submitted.tsv").write_text(''.join(
                f"{stamp.isoformat()}\ttask\thttps://github.com/robocurve/benchmark-tasks/pull/{pr}\n"
                for pr in range(1, 21)))
            quota = subprocess.run(["python3", pipeline/"check_submission_quota.py", pipeline/"submitted.tsv"], capture_output=True)
            self.assertNotEqual(quota.returncode, 0)
            script = pipeline/"revise.sh"
            result = subprocess.run(["bash", script, "__missing_guard_test__", "test"], env=env, capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("needs task.json, README.md and task.py", result.stderr)
            self.assertFalse(sentinel.exists())


if __name__ == "__main__":
    unittest.main()
