"""Exercise CMake's PlatformIO delegation without an embedded toolchain."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

MODULE = Path(__file__).resolve().parents[3] / "Mini_Robot/cmake/MicroRobotFirmwareTools.cmake"


@unittest.skipUnless(shutil.which("cmake") and shutil.which("ninja"), "requires CMake and Ninja")
class FirmwareToolsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="firmware tools ")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.project = self.root / "project with spaces"
        self.firmware = self.project / "firmware/ble-lifecycle"
        self.firmware.mkdir(parents=True)
        (self.firmware / "platformio.ini").write_text("[env:test]\n")
        self.log = self.root / "invocation.json"
        fake = self.root / "fake_pio.py"
        fake.write_text(f"#!{sys.executable}\nimport json, os, sys\n"
                        f"from pathlib import Path\nPath({str(self.log)!r}).write_text("
                        "json.dumps({'cwd': os.getcwd(), 'args': sys.argv[1:]}))\n")
        fake.chmod(0o755)
        if os.name == "nt":
            command = self.root / "fake_pio.cmd"
            command.write_text(f'@"{sys.executable}" "{fake}" %*\n')
            fake = command
        self.fake = fake
        (self.project / "CMakeLists.txt").write_text(
            'cmake_minimum_required(VERSION 3.24)\nproject(Fixture NONE)\n'
            f'include("{MODULE.as_posix()}")\n')
        self.build = self.root / "build"

    def run_command(self, *args, success=True):
        result = subprocess.run(args, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        if success:
            self.assertEqual(result.returncode, 0, result.stdout)
        else:
            self.assertNotEqual(result.returncode, 0, result.stdout)
        return result.stdout

    def configure(self, *options, success=True):
        return self.run_command("cmake", "-S", str(self.project), "-B", str(self.build),
                                "-G", "Ninja", f"-DMICRO_ROBOT_PLATFORMIO_EXECUTABLE={self.fake}",
                                *options, success=success)

    def test_default_build_never_invokes_firmware(self):
        self.configure()
        self.run_command("cmake", "--build", str(self.build))
        self.assertFalse(self.log.exists())

    def test_explicit_target_passes_exact_project_directory_without_upload(self):
        self.configure()
        self.run_command("cmake", "--build", str(self.build), "--target", "firmware_ble_lifecycle")
        invocation = json.loads(self.log.read_text())
        self.assertEqual(Path(invocation["cwd"]), self.firmware)
        self.assertEqual(invocation["args"], ["run", "--project-dir", str(self.firmware)])

    def test_opt_out_removes_firmware_target(self):
        self.configure("-DMICRO_ROBOT_ENABLE_FIRMWARE_TOOLS=OFF")
        self.run_command("cmake", "--build", str(self.build), "--target",
                         "firmware_ble_lifecycle", success=False)
        self.assertFalse(self.log.exists())

    def test_missing_platformio_does_not_break_configuration(self):
        empty = self.root / "empty tool root"
        empty.mkdir()
        output = self.run_command(
            "cmake", "-S", str(self.project), "-B", str(self.build), "-G", "Ninja",
            f"-DCMAKE_MAKE_PROGRAM={shutil.which('ninja')}",
            f"-DCMAKE_FIND_ROOT_PATH={empty}", "-DCMAKE_FIND_ROOT_PATH_MODE_PROGRAM=ONLY")
        self.assertIn("PlatformIO not found", output)
        self.run_command("cmake", "--build", str(self.build))
        self.assertFalse(self.log.exists())

    def test_colliding_directory_names_are_rejected(self):
        other = self.project / "firmware/ble_lifecycle"
        other.mkdir()
        (other / "platformio.ini").write_text("[env:test]\n")
        output = self.configure(success=False)
        self.assertIn("target name collision", output)


if __name__ == "__main__":
    unittest.main()
