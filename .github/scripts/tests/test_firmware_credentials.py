"""Unit tests for the production credential scan (#165, F7, AT-11)."""
import importlib.util
from pathlib import Path
import unittest

SCAN = Path(__file__).resolve().parents[1] / "check_firmware_credentials.py"
_spec = importlib.util.spec_from_file_location("check_firmware_credentials", SCAN)
scan = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(scan)


def reasons(text, suffix=".cpp"):
    return [reason for _, _, reason in scan.scan_text(text, "x" + suffix, suffix)]


class CredentialScanTests(unittest.TestCase):
    def test_production_tree_has_no_credential_material(self):
        self.assertEqual(scan.scan_tree(), [])
        self.assertGreater(sum(1 for _ in scan.source_files()), 10)

    def test_assigned_credentials_are_rejected(self):
        for line in ('const char* kApPassword = "hunter2hunter2";',
                     'static constexpr auto passphrase{"abc12345"};',
                     'setPassword("secretvalue");',
                     'std::string api_key = "k-123";',
                     '-DWIFI_PASSWORD=\\"letmein\\"'):
            self.assertTrue(reasons(line), line)

    def test_pem_and_long_hex_key_material_is_rejected(self):
        self.assertTrue(reasons("-----BEGIN PRIVATE KEY-----"))
        self.assertTrue(reasons('constexpr char kKey[] = "' + "ab" * 16 + '";'))

    def test_pairing_and_passkey_calls_are_rejected(self):
        for line in ("NimBLEDevice::setSecurityPasskey(123456);",
                     "NimBLEDevice::setSecurityAuth(true, true, true);",
                     "NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);"):
            self.assertTrue(reasons(line), line)

    def test_explicit_no_pairing_settings_and_comments_pass(self):
        self.assertEqual(reasons("NimBLEDevice::setSecurityAuth(false, false, false);"), [])
        self.assertEqual(reasons("NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);"), [])
        self.assertEqual(reasons('// no password: password = "x"'), [])
        self.assertEqual(reasons("# the profile has no secret", ".py"), [])
        self.assertEqual(reasons("; passphrase = \"x\" is never configured", ".ini"), [])

    def test_logging_a_credential_is_rejected(self):
        self.assertTrue(reasons('logf("ap password=%s", password);'))
        self.assertTrue(reasons('Serial.printf("%s", passphrase);'))

    def test_logging_identity_fields_passes(self):
        self.assertEqual(reasons('logf("BOOT robot_id=%lu firmware=%u", id, v);'), [])


if __name__ == "__main__":
    unittest.main()
