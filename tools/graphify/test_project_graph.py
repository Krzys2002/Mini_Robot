"""Offline regression checks for freshness, isolation, and command boundaries."""
import contextlib
import io
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import project_graph as pg


class GraphTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.git("init", "-q")
        self.git("config", "user.email", "test@example.invalid")
        self.git("config", "user.name", "Graph Test")
        self.write(".gitignore", "graphify-out/\nbuild/\n")
        self.write("src/a.cpp", "class A {};\n")
        self.git("add", ".")
        self.git("commit", "-qm", "initial")

    def git(self, *args):
        return pg.git(self.root, *args)

    def write(self, name, content):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")

    def graph(self):
        self.write("graphify-out/graph.json", json.dumps({
            "graph": {"schema_version": 1, "graphify_version": pg.VERSION},
            "nodes": [], "links": []}))

    def stamp(self):
        self.graph()
        self.write("graphify-out/project-snapshot.json", json.dumps({
            "snapshot": pg.snapshot(self.root),
            "graph_sha256": pg.digest(self.root / pg.OUT / "graph.json")}))

    def test_missing_and_corrupt_graph_fall_back(self):
        self.assertFalse(pg.freshness(self.root)[0])
        self.stamp()
        self.assertTrue(pg.freshness(self.root)[0])
        self.write("graphify-out/graph.json", "broken")
        self.assertFalse(pg.freshness(self.root)[0])

    def test_edits_additions_renames_and_deletions_invalidate(self):
        for change in ("edit", "add", "rename", "delete"):
            with self.subTest(change=change):
                self.write("src/a.cpp", "class A {};\n")
                self.stamp()
                if change == "edit":
                    self.write("src/a.cpp", "class B {};\n")
                elif change == "add":
                    self.write("src/new.cpp", "class B {};\n")
                elif change == "rename":
                    (self.root / "src/a.cpp").rename(self.root / "src/renamed.cpp")
                else:
                    (self.root / "src/a.cpp").unlink()
                self.assertFalse(pg.freshness(self.root)[0])

    def test_head_change_invalidates_even_with_same_tree(self):
        self.stamp()
        self.git("commit", "--allow-empty", "-qm", "new head")
        self.assertFalse(pg.freshness(self.root)[0])

    def test_policy_and_lockfile_changes_invalidate(self):
        for name in (".graphifyrc", ".graphifyignore", "tools/graphify/uv.lock", "AGENTS.md"):
            self.stamp()
            self.write(name, "changed")
            self.assertFalse(pg.freshness(self.root)[0])

    def test_outputs_and_ignored_tracked_build_files_do_not_invalidate(self):
        self.write("build/cache.cpp", "generated")
        self.git("add", "-f", "build/cache.cpp")
        self.stamp()
        self.write("build/cache.cpp", "changed")
        self.write("graphify-out/graph.html", "generated")
        self.assertTrue(pg.freshness(self.root)[0])

    def test_different_worktree_cannot_reuse_provenance(self):
        self.stamp()
        second = self.root / "second"
        self.git("worktree", "add", "--detach", str(second), "HEAD")
        (second / pg.OUT).mkdir()
        for name in ("graph.json", "project-snapshot.json"):
            (second / pg.OUT / name).write_bytes((self.root / pg.OUT / name).read_bytes())
        self.assertFalse(pg.freshness(second)[0])

    def test_graph_replacement_invalidates(self):
        self.stamp()
        with (self.root / pg.OUT / "graph.json").open("a") as handle:
            handle.write(" ")
        self.assertFalse(pg.freshness(self.root)[0])

    def test_invalid_graph_metadata_falls_back(self):
        self.stamp()
        self.write("graphify-out/graph.json", '{"nodes": [], "links": [], "graph": null}')
        self.assertFalse(pg.freshness(self.root)[0])

    def test_symlink_content_is_not_read(self):
        target = self.root / "build/private.cpp"
        self.write("build/private.cpp", "outside corpus")
        try:
            (self.root / "link.cpp").symlink_to(target)
        except OSError:
            self.skipTest("symlinks unavailable")
        before = pg.snapshot(self.root)
        target.write_text("changed")
        self.assertEqual(before, pg.snapshot(self.root))

    def test_operation_lock_blocks_concurrent_read_or_write(self):
        with pg.operation_lock(self.root):
            with self.assertRaises(ValueError):
                with pg.operation_lock(self.root):
                    pass
        self.assertFalse((self.root / pg.OUT / ".project-operation.lock").exists())

    def test_failed_build_removes_freshness_stamp(self):
        self.stamp()
        with patch.object(pg, "run_graphify", return_value=1):
            self.assertEqual(pg.rebuild(self.root, "build"), 1)
        self.assertFalse(pg.freshness(self.root)[0])

    def test_changed_sources_during_build_are_never_stamped(self):
        def fake_run(root, args):
            self.graph()
            self.write("src/a.cpp", "changed during extraction")
            return 0
        with patch.object(pg, "run_graphify", side_effect=fake_run):
            with self.assertRaisesRegex(ValueError, "Sources changed"):
                pg.rebuild(self.root, "build")
        self.assertFalse(pg.freshness(self.root)[0])

    def test_clean_build_archives_old_graph_and_stamps_new_graph(self):
        self.stamp()
        with patch.object(pg, "run_graphify", side_effect=lambda *_: self.graph() or 0):
            self.assertEqual(pg.rebuild(self.root, "build"), 0)
        self.assertTrue(pg.freshness(self.root)[0])
        self.assertTrue((self.root / pg.OUT / "previous-graph.json").exists())

    def test_semantic_graph_is_rejected(self):
        self.graph()
        path = self.root / pg.OUT / "graph.json"
        data = json.loads(path.read_text())
        data["nodes"].append({"file_type": "document"})
        path.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, "semantic"):
            pg.graph_data(self.root)

    def test_update_requires_original_identity_and_policy(self):
        self.stamp()
        self.write(".graphifyrc", "language.h=cpp")
        with patch.object(pg, "run_graphify") as run:
            with self.assertRaisesRegex(ValueError, "run build"):
                pg.rebuild(self.root, "update")
            run.assert_not_called()

    def test_update_accepts_source_changes_and_restamps(self):
        self.stamp()
        self.write("src/a.cpp", "class Changed {};")
        with patch.object(pg, "run_graphify", side_effect=lambda *_: self.graph() or 0):
            self.assertEqual(pg.rebuild(self.root, "update"), 0)
        self.assertTrue(pg.freshness(self.root)[0])

    def test_environment_cannot_enable_backend_or_global_query_logging(self):
        with patch.dict(os.environ, {"GRAPHIFY_BACKEND": "openai", "GRAPHIFY_OUT": "/other"}):
            env = pg.environment(self.root)
        self.assertEqual(env["GRAPHIFY_BACKEND"], "none")
        self.assertEqual(env["GRAPHIFY_OUT"], str(self.root / pg.OUT))
        self.assertEqual(env["GRAPHIFY_QUERY_LOG_DISABLE"], "1")

    def test_query_preserves_exit_codes_and_uses_explicit_graph(self):
        self.stamp()
        with patch.object(pg, "repository_root", return_value=self.root), \
             patch.object(pg, "run_graphify", return_value=3) as run:
            self.assertEqual(pg.main(["query", "AmbiguousName"]), 3)
        self.assertEqual(run.call_args.args[1], ["query", "AmbiguousName", "--graph",
                                               str(self.root / pg.OUT / "graph.json")])

    def test_query_rejects_stale_without_running_graphify(self):
        with patch.object(pg, "repository_root", return_value=self.root), \
             patch.object(pg, "run_graphify") as run, contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(pg.main(["query", "anything"]), 2)
            run.assert_not_called()

    def test_exact_id_query_uses_one_symbol_neighborhood(self):
        self.stamp()
        with patch.object(pg, "repository_root", return_value=self.root), \
             patch.object(pg, "graph_data", return_value={"nodes": [{"id": "exact_id"}]}), \
             patch.object(pg, "run_graphify", return_value=0) as run:
            self.assertEqual(pg.main(["query", "exact_id"]), 0)
        self.assertEqual(run.call_args.args[1], ["explain", "exact_id", "--graph",
                                               str(self.root / pg.OUT / "graph.json")])

    def test_missing_environment_is_actionable(self):
        with self.assertRaisesRegex(ValueError, "uv sync"):
            pg.run_graphify(self.root, ["query", "A"])


if __name__ == "__main__":
    unittest.main()
