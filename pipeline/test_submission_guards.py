import json
import os
from pathlib import Path
import subprocess
import shutil
import tempfile
import unittest

from check_revision_state import revision_allowed


class SubmissionGuards(unittest.TestCase):
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
            for name in ("revise.sh", "submit.sh", "check_revision_state.py"):
                shutil.copy2(Path(__file__).with_name(name), pipeline/name)
            script = pipeline/"revise.sh"
            result = subprocess.run(["bash", script, "__missing_guard_test__", "test"], env=env, capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("needs task.json, README.md and task.py", result.stderr)
            self.assertFalse(sentinel.exists())


if __name__ == "__main__":
    unittest.main()
