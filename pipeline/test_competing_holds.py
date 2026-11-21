import contextlib
from datetime import datetime, timedelta, timezone
import io
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from check_competing_holds import check, fetch_pr, hold_status

NOW = datetime(2026, 11, 20, 4, tzinfo=timezone.utc)


def comment(body, age=1, association="COLLABORATOR"):
    return {"body": body, "author_association": association,
            "created_at": (NOW - timedelta(hours=age)).isoformat()}


class CompetingHolds(unittest.TestCase):
    def status(self, *comments, state="open", **fields):
        return hold_status(dict(state=state, **fields), list(comments), NOW)[0]

    def test_revision_receipt_reopens_hold_despite_stale_labels_and_order(self):
        self.assertTrue(self.status(
            comment("Received as **TK-REV**", 1),
            comment("**Not accepted**", 72), labels=[{"name": "Not accepted: final"}],
        ))

    def test_pending_has_no_48_hour_expiry(self):
        self.assertTrue(self.status(comment("Received as **TK-1**", 100)))

    def test_accepted_and_merged_remain_owned(self):
        self.assertTrue(self.status(comment("**Accepted**", 100)))
        self.assertTrue(self.status(state="closed", merged_at="2026-11-18T00:00:00Z"))

    def test_revisable_hold_expires_at_exact_48_hour_boundary(self):
        self.assertTrue(self.status(comment("**Not accepted**", 47.999)))
        self.assertFalse(self.status(comment("**Not accepted**", 48)))
        self.assertFalse(self.status(comment("**Not accepted**", 49)))

    def test_final_and_withdrawn_have_no_hold(self):
        self.assertFalse(self.status(comment("Received as **TK-1**", 2),
                                     comment("**Not accepted: final**", 1)))
        self.assertFalse(self.status(comment("Received as **TK-1**"), state="closed"))

    def test_latest_official_comment_beats_untrusted_comments(self):
        self.assertTrue(self.status(comment("Received as **TK-1**", 2),
                                    comment("**Not accepted: final**", 1, "NONE")))

    def test_unknown_future_and_unzoned_decisions_fail_closed(self):
        for comments in [[], [comment("ordinary discussion")],
                         [comment("**Not accepted**", -1)],
                         [dict(comment("**Not accepted**"), created_at="2026-11-19T00:00:00")]]:
            with self.subTest(comments=comments), self.assertRaises(ValueError):
                hold_status({"state": "open"}, comments, NOW)

    def test_all_comment_pages_are_used(self):
        with patch("check_competing_holds.gh_json", side_effect=[
            {"state": "open"}, [[comment("**Not accepted**", 72)],
                                [comment("Received as **TK-REV**", 1)]]]) as call:
            pr, comments = fetch_pr("robocurve/benchmark-tasks", 1)
        self.assertTrue(hold_status(pr, comments, NOW)[0])
        self.assertEqual(call.call_args.args[-2:], ("--paginate", "--slurp"))

    def test_live_query_failure_and_malformed_json_refuse(self):
        registry = {"repository": "robocurve/benchmark-tasks", "tasks": {"task": [1]}}
        for error in [subprocess.CalledProcessError(1, ["gh"]),
                      subprocess.TimeoutExpired(["gh"], 45),
                      json.JSONDecodeError("bad", "bad", 0)]:
            with self.subTest(error=type(error).__name__), \
                 patch("check_competing_holds.fetch_pr", side_effect=error), \
                 self.assertRaises(type(error)):
                check("task", registry, NOW)

    def test_unlisted_task_never_queries_and_expiry_never_clears_novelty(self):
        registry = {"repository": "robocurve/benchmark-tasks", "tasks": {"task": [1]}}
        with patch("check_competing_holds.fetch_pr") as call:
            check("other", registry, NOW)
            call.assert_not_called()
        out = io.StringIO()
        with patch("check_competing_holds.fetch_pr", return_value=(
                {"state": "open"}, [comment("**Not accepted**", 49)])), \
                contextlib.redirect_stdout(out):
            check("task", registry, NOW)
        self.assertIn("novelty is NOT cleared", out.getvalue())

    def test_shell_checks_new_submissions_but_skips_preflight_and_revision(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            pipeline = root / "pipeline"
            task = root / "tasks/task"
            pipeline.mkdir(); task.mkdir(parents=True)
            for name in ("submit.sh",):
                shutil.copy2(Path(__file__).with_name(name), pipeline / name)
            (pipeline / "submitted.tsv").write_text("")
            for name in ("check_prism_identity.py", "check_novelty_review.py"):
                (pipeline / name).write_text("")
            marker = pipeline / "hold-was-checked"
            (pipeline / "check_competing_holds.py").write_text(
                "from pathlib import Path\n"
                f"Path({str(marker)!r}).touch()\n"
                "raise SystemExit('ownership: known competitor held')\n")
            metadata = dict.fromkeys(["title", "instruction", "setup", "scoring",
                                      "run_limit", "fidelity", "scores"], "test")
            metadata["category"] = "Other"
            (task / "task.json").write_text(json.dumps(metadata))
            (task / "task.py").write_text("# fixture\n")
            (task / "README.md").write_text("fixture\n")
            for mode in ("--check", "", "--preflight", "--revision-check"):
                marker.unlink(missing_ok=True)
                command = ["bash", str(pipeline / "submit.sh"), "task"]
                if mode:
                    command.append(mode)
                result = subprocess.run(command, capture_output=True, text=True)
                with self.subTest(mode=mode):
                    if mode in ("--check", ""):
                        self.assertNotEqual(result.returncode, 0)
                        self.assertTrue(marker.exists())
                        self.assertIn("known competitor held", result.stderr)
                    else:
                        self.assertFalse(marker.exists())
                        if mode == "--preflight":
                            self.assertEqual(result.returncode, 0, result.stderr)
                        else:
                            self.assertIn("missing", result.stderr)  # next ordinary media gate


if __name__ == "__main__":
    unittest.main()
