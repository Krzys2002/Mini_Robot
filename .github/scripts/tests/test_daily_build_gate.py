import importlib.util
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    "daily_build_gate", Path(__file__).resolve().parents[1] / "daily_build_gate.py"
)
gate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gate)


class DailyBuildGateTest(unittest.TestCase):
    def test_latest_checkpoint_across_pages_excludes_current_run(self):
        pages = [{"workflow_runs": [{"id": 1, "created_at": "2026-10-21"}]},
                 {"workflow_runs": [{"id": 2, "created_at": "2026-10-23"},
                                    {"id": 3, "created_at": "2026-10-24"}]}]
        self.assertEqual(gate.previous_checkpoint(pages, "3")["id"], 2)
        self.assertIsNone(gate.previous_checkpoint([], "3"))

    def test_only_merged_master_pull_requests_in_snapshot_trigger_build(self):
        pr = {"merged_at": "2026-10-24", "base": {"ref": "master"},
              "merge_commit_sha": "merge"}
        self.assertTrue(gate.merged_in_commits([[], [pr]], {"merge"}))
        self.assertFalse(gate.merged_in_commits([[pr]], {"direct-push"}))
        self.assertFalse(gate.merged_in_commits([[dict(pr, merged_at=None)]], {"merge"}))
        self.assertFalse(gate.merged_in_commits([[dict(pr, base={"ref": "feature"})]], {"merge"}))

    def run_gate(self, runs, commits, pulls):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "output"
            env = {"GITHUB_REF": "refs/heads/master", "GH_REPO": "owner/repo",
                   "GITHUB_RUN_ID": "3", "GITHUB_SHA": "head", "GITHUB_OUTPUT": str(output)}
            with patch.dict(os.environ, env), patch.object(gate, "api_pages", side_effect=[runs, pulls]) as api, patch.object(gate.subprocess, "run", return_value=subprocess.CompletedProcess([], 0, commits)) as git:
                gate.main()
                return output.read_text(), git.call_args.args[0], api.call_count

    def test_catch_up_uses_last_successful_snapshot_without_time_limit(self):
        runs = [{"workflow_runs": [{"id": 1, "created_at": "2026-10-20", "head_sha": "old"}]}]
        pulls = [[{"merged_at": "2026-10-21", "base": {"ref": "master"}, "merge_commit_sha": "merge"}]]
        output, command, calls = self.run_gate(runs, "merge\n", pulls)
        self.assertEqual(output, "build=true\n")
        self.assertEqual(command, ["git", "rev-list", "head", "^old"])
        self.assertEqual(calls, 2)

    def test_unchanged_snapshot_skips_pull_lookup_and_builds(self):
        output, _, calls = self.run_gate([], "", [])
        self.assertEqual(output, "build=false\n")
        self.assertEqual(calls, 1)

    def test_first_run_uses_one_day_bootstrap_window(self):
        output, command, _ = self.run_gate([], "direct-push\n", [[]])
        self.assertEqual(output, "build=false\n")
        self.assertTrue(command[-1].startswith("--since="))

    def test_other_branches_rejected_before_api_access(self):
        with patch.dict(os.environ, {"GITHUB_REF": "refs/heads/feature"}), patch.object(gate, "api_pages") as api:
            with self.assertRaisesRegex(RuntimeError, "restricted to master"):
                gate.main()
            api.assert_not_called()

    def test_api_failure_is_not_treated_as_no_merges(self):
        with patch.dict(os.environ, {"GITHUB_REF": "refs/heads/master", "GH_REPO": "owner/repo"}), patch.object(gate, "api_pages", side_effect=RuntimeError("API unavailable")):
            with self.assertRaisesRegex(RuntimeError, "API unavailable"):
                gate.main()


if __name__ == "__main__":
    unittest.main()
