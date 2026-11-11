"""Quality audit failures and exact node discovery are reproducible offline."""
import copy
from pathlib import Path
import tempfile
import unittest

from graph_quality import audit_graph, find_nodes
from source_checks import check_cpp_calls, retain_rejections


class QualityTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / "source.cpp").write_text("void run() {}\n")
        self.data = {"nodes": [{"id": "run", "label": "run()", "file_type": "code",
                     "source_file": "source.cpp", "source_location": "L1"}], "links": []}

    def test_valid_graph_passes(self):
        self.assertTrue(audit_graph(self.root, self.data)["passed"])

    def test_duplicate_and_dangling_nodes_fail(self):
        self.data["nodes"].append(copy.deepcopy(self.data["nodes"][0]))
        self.data["links"].append({"source": "run", "target": "absent", "relation": "calls"})
        report = audit_graph(self.root, self.data)
        self.assertFalse(report["passed"])
        self.assertTrue(any("duplicate node" in error for error in report["errors"]))
        self.assertTrue(any("dangling" in error for error in report["errors"]))

    def test_missing_or_invalid_source_locations_fail(self):
        self.data["nodes"][0]["source_location"] = "L100"
        self.assertFalse(audit_graph(self.root, self.data)["passed"])
        self.data["nodes"][0]["source_file"] = "missing.cpp"
        self.assertFalse(audit_graph(self.root, self.data)["passed"])

    def test_excluded_sources_fail(self):
        self.data["nodes"][0]["source_file"] = ".github/workflows/build.yml"
        self.assertFalse(audit_graph(self.root, self.data)["passed"])

    def test_exact_discovery_does_not_expand_into_other_symbols(self):
        self.data["nodes"].append({"id": "run_test", "label": "runTest()"})
        self.assertEqual([n["id"] for n in find_nodes(self.data, "run")], ["run"])
        self.assertEqual([n["id"] for n in find_nodes(self.data, "source.cpp")], ["run"])

    def test_rejections_survive_update_only_while_source_is_unchanged(self):
        self.data["nodes"].append({"id": "type", "label": "Run", "file_type": "code"})
        self.data["links"].append({"source": "run", "target": "type", "relation": "calls",
                                   "source_file": "source.cpp", "source_location": "L1"})
        first = retain_rejections(self.root, check_cpp_calls(self.root, self.data), {})
        self.assertEqual(len(first["rejected_calls"]), 1)
        second = retain_rejections(self.root, check_cpp_calls(self.root, self.data), first)
        self.assertEqual(len(second["rejected_calls"]), 1)
        self.assertEqual(second["rejected_this_operation"], 0)
        (self.root / "source.cpp").write_text("void changed() {}\n")
        third = retain_rejections(self.root, check_cpp_calls(self.root, self.data), second)
        self.assertFalse(third["rejected_calls"])
