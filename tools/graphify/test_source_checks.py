"""The source guard rejects unsupported calls without inventing new bindings."""
from pathlib import Path
import tempfile
import unittest

from source_checks import check_cpp_calls


class SourceChecksTest(unittest.TestCase):
    def check(self, content, target_label, target_file="Mini_Robot/src/types.hpp",
              source="Mini_Robot/src/use.cpp", relation="calls"):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / source
            path.parent.mkdir(parents=True)
            path.write_text(content)
            data = {"nodes": [{"id": "caller"}, {"id": "callee", "label": target_label,
                     "source_file": target_file}],
                    "links": [{"source": "caller", "target": "callee", "relation": relation,
                               "source_file": source, "source_location": "L1"}]}
            return data, check_cpp_calls(root, data)

    def test_case_collision_is_rejected(self):
        data, report = self.check("auto value = markerGeometry(corners);", "MarkerGeometry")
        self.assertFalse(data["links"])
        self.assertEqual(len(report["rejected_calls"]), 1)

    def test_getter_cannot_bind_to_field(self):
        data, _ = self.check("map.profile();", "profile_")
        self.assertFalse(data["links"])

    def test_valid_member_and_multiline_calls_survive(self):
        for content in ("world.updateTrackedPose(id, pose);", "world.updateTrackedPose\n (id, pose);"):
            data, report = self.check(content, ".updateTrackedPose()")
            self.assertEqual(len(data["links"]), 1)
            self.assertFalse(report["rejected_calls"])

    def test_application_and_fixture_do_not_bind_to_host_fakes(self):
        for source in ("Mini_Robot/src/ui/view.cpp", "T1-ESP-C6/src/main.cpp",
                       "T1-ESP-S3/src/main.cpp", "Mini_Robot/firmware/ble-lifecycle/src/main.cpp"):
            data, report = self.check("millis();", "millis()",
                                      "Mini_Robot/tests/firmware/fakes/Arduino.h", source)
            self.assertFalse(data["links"])
            self.assertIn("build context", report["rejected_calls"][0]["reason"])

    def test_host_tests_can_call_their_fakes(self):
        data, _ = self.check("millis();", "millis()", "Mini_Robot/tests/firmware/fakes/Arduino.h",
                             "Mini_Robot/tests/firmware/test.cpp")
        self.assertEqual(len(data["links"]), 1)

    def test_non_call_relationships_are_preserved(self):
        data, _ = self.check("", "profile_", relation="references")
        self.assertEqual(len(data["links"]), 1)


if __name__ == "__main__":
    unittest.main()
