import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location(
    "check_firmware_pins", Path(__file__).resolve().parents[1] / "check_firmware_pins.py"
)
pins = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pins)

RELEASE = ("https://github.com/pioarduino/platform-espressif32/releases/download/"
           "55.03.37/platform-espressif32.zip")
COMMIT = "795616b3fc6fa14c0e23fbdac8aed697d897d303"


class FirmwarePinsTest(unittest.TestCase):
    def manifest(self, body, requirements=None):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        project = Path(temp.name) / "codecell-c6"
        project.mkdir()
        (project / "platformio.ini").write_text(body)
        if requirements is not None:
            (project / "requirements.txt").write_text(requirements)
        return Path(temp.name)

    def test_repository_firmware_manifests_are_pinned(self):
        self.assertEqual(pins.check(), [])

    def test_immutable_sources_are_accepted(self):
        for spec in (RELEASE, f"https://github.com/microbotsio/CodeCell.git#{COMMIT}",
                     f"git+https://example.com/lib#{COMMIT}", "microbotsio/DriveCell@1.1.3",
                     "h2zero/NimBLE-Arduino@2.3.6", "Lib=owner/lib@2.0.0", f"Lib = {RELEASE}",
                     f"https://example.com/owner/lib/archive/{COMMIT}.zip"):
            self.assertIsNone(pins.spec_problem(spec), spec)

    def test_moving_or_unversioned_sources_are_rejected(self):
        for spec in ("espressif32", "espressif32@^6.1.0", "owner/lib@~1.2", "owner/lib@>=1.0",
                     "owner/lib@*", "owner/lib@stable",
                     "https://github.com/pioarduino/platform-espressif32.git",
                     "https://github.com/microbotsio/CodeCell.git#main",
                     "https://github.com/microbotsio/CodeCell.git#v2.0.4",
                     "https://github.com/pioarduino/platform-espressif32/releases/download/"
                     "stable/platform-espressif32.zip",
                     "https://github.com/owner/lib/archive/refs/heads/main.zip",
                     "symlink://../lib", "file:///tmp/lib", "Lib=owner/lib@^2"):
            self.assertIsNotNone(pins.spec_problem(spec), spec)

    def test_manifest_reports_section_key_and_fix(self):
        firmware = self.manifest(
            f"[env:c6]\nplatform = {RELEASE}\nlib_deps =\n"
            "    microbotsio/DriveCell@1.1.3\n    owner/moving@^1.0 ; comment\n")
        problems = pins.check(firmware)
        self.assertEqual(len(problems), 1)
        self.assertIn("[env:c6] lib_deps = owner/moving@^1.0", problems[0])
        self.assertIn("Git commit", problems[0])

    def test_platform_packages_and_shared_env_are_checked(self):
        firmware = self.manifest(
            "[env]\nplatform = espressif32\n"
            f"[env:c6]\nplatform = {RELEASE}\nplatform_packages = tool-esptoolpy\n")
        problems = pins.check(firmware)
        self.assertEqual(len(problems), 2, problems)

    def test_requirements_must_be_exact(self):
        firmware = self.manifest(
            f"[env:c6]\nplatform = {RELEASE}\n",
            "# tools\nplatformio==6.1.19\nclick>=8\nrequests\n")
        problems = pins.check(firmware)
        self.assertEqual(len(problems), 2, problems)
        self.assertTrue(all("name==version" in problem for problem in problems))

    def test_missing_manifests_fail(self):
        with tempfile.TemporaryDirectory() as temp:
            self.assertTrue(pins.check(Path(temp)))


if __name__ == "__main__":
    unittest.main()
