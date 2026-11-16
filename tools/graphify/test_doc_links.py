"""Regression checks for the explicit, offline technical-document corpus."""
import json
from pathlib import Path
import tempfile
import unittest

from doc_links import add_document_links, references, technical_document


class DocumentTests(unittest.TestCase):
    def test_only_technical_docs_are_candidates(self):
        for name in ("Mini_Robot/docs/overview/transport.md",
                     "Mini_Robot/docs/testing/codecell-c6-hardware-154.md",
                     "Mini_Robot/firmware/ble-lifecycle/README.md"):
            self.assertTrue(technical_document(name), name)
        for name in (".github/README.md", "AGENTS.md", "core arhiteture.md",
                     "Mini_Robot/docs/planning/MILESTONES.md",
                     "Mini_Robot/docs/archive/GITHUB_IMPORT_REPORT.md",
                     "Mini_Robot/docs/development/GITHUB_BLOCKER_AUTOMATION.md",
                     "Mini_Robot/docs/testing/codecell-c6-hardware-154-record.md",
                     "Mini_Robot/docs/testing/review-225-227-229-acceptance.md",
                     "report/report.md", "Pres/slides.md"):
            self.assertFalse(technical_document(name), name)

    def test_reference_resolution_stays_in_available_repository_files(self):
        available = {"src/a.cpp", "T1-ESP-C6/src/Main.cpp"}
        self.assertEqual(references("docs/guide.md", """
[source](../src/a.cpp#L2) `T1-ESP-C6/src/Main.cpp`
[remote](https://github.com/example/src/a.cpp) `/etc/passwd`
`../../outside.cpp` [absolute](/src/a.cpp) [missing](../src/missing.cpp)
""", available), available)

    def test_reachable_docs_only_and_removed_references_disappear(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            docs = {
                "Mini_Robot/docs/overview/direct.md": "# Direct\n[source](../../src/a.cpp)",
                "Mini_Robot/docs/overview/indirect.md": "# Indirect\n[direct](direct.md)",
                "Mini_Robot/docs/overview/alone.md": "# No source\n[remote](https://example.com)",
                "Mini_Robot/docs/overview/cycle1.md": "[cycle](cycle2.md)",
                "Mini_Robot/docs/overview/cycle2.md": "[cycle](cycle1.md)",
                "Mini_Robot/docs/planning/plan.md": "`Mini_Robot/src/a.cpp`",
            }
            for name, content in docs.items():
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(content)
            files = dict.fromkeys([*docs, "Mini_Robot/src/a.cpp"], "hash")
            data = {"graph": {}, "nodes": [{"id": "source", "file_type": "code",
                    "source_file": "Mini_Robot/src/a.cpp"}], "links": []}
            add_document_links(root, data, files)
            self.assertEqual(len(data["nodes"]), 3)
            self.assertEqual({e["relation"] for e in data["links"]}, {"documents", "references"})
            original = json.dumps(data, sort_keys=True)
            add_document_links(root, data, files)
            self.assertEqual(json.dumps(data, sort_keys=True), original)
            (root / "Mini_Robot/docs/overview/direct.md").write_text("# Removed source")
            add_document_links(root, data, files)
            self.assertEqual([n["id"] for n in data["nodes"]], ["source"])
            self.assertEqual(data["links"], [])

    def test_document_symlink_is_not_read(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / "Mini_Robot/README.md"
            path.parent.mkdir()
            path.symlink_to(root / "missing-outside-document")
            data = {"graph": {}, "nodes": [], "links": []}
            add_document_links(root, data, {"Mini_Robot/README.md": "symlink"})
            self.assertFalse(data["nodes"])

    def test_document_points_to_file_instead_of_arbitrary_symbol(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / "Mini_Robot/README.md"
            path.parent.mkdir()
            path.write_text("[source](src/a.cpp)")
            nodes = [{"id": "symbol", "file_type": "code", "label": "Example",
                      "source_file": "Mini_Robot/src/a.cpp"},
                     {"id": "file", "file_type": "code", "label": "a.cpp",
                      "source_file": "Mini_Robot/src/a.cpp", "source_location": "L1"}]
            data = {"graph": {}, "nodes": nodes, "links": []}
            add_document_links(root, data, dict.fromkeys(
                ["Mini_Robot/README.md", "Mini_Robot/src/a.cpp"], "hash"))
            self.assertEqual(data["links"][0]["target"], "file")


if __name__ == "__main__":
    unittest.main()
