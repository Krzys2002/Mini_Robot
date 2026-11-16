"""Unit tests for the CodeCell C6 production image policy (#158)."""
import importlib.util
from pathlib import Path
import unittest

POLICY = Path(__file__).resolve().parents[3] / "Mini_Robot/firmware/codecell-c6/scripts/image_policy.py"
_spec = importlib.util.spec_from_file_location("image_policy", POLICY)
image_policy = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(image_policy)


class ImagePolicyTests(unittest.TestCase):
    def test_runtime_only_image_passes(self):
        nm = "\n".join([
            "42000100 T setup",
            "42000200 T codecell::runtime::ActuatorGate::apply(std::optional<...> const&, ...)",
            "42000300 t vTaskDelayUntil",
            "         U esp_timer_get_time",
        ])
        self.assertEqual(image_policy.violations(nm), [])

    def test_rom_wifi_addresses_and_undefined_references_are_ignored(self):
        nm = "\n".join([
            "40000ba0 A wifi_get_macaddr",
            "4087fcf8 A esp_wifi_cert_tx_bcc",
            "         U esp_wifi_init",
            "         w httpd_start",
        ])
        self.assertEqual(image_policy.violations(nm), [])

    def test_linked_network_http_and_ota_code_is_rejected(self):
        nm = "\n".join([
            "42001000 T esp_wifi_init",
            "42001100 T httpd_start",
            "42001200 T esp_ota_begin",
            "42001300 T WiFiAPClass::softAP(char const*, char const*, int, int, int, bool)",
            "42001400 W WebServer::handleClient()",
            "42001500 T DNSServer::start(unsigned short, String const&, IPAddress const&)",
            "42001600 t lwip_socket",
        ])
        self.assertEqual(image_policy.violations(nm), sorted([
            "esp_wifi_init", "httpd_start", "esp_ota_begin",
            "WiFiAPClass::softAP(char const*, char const*, int, int, int, bool)",
            "WebServer::handleClient()",
            "DNSServer::start(unsigned short, String const&, IPAddress const&)",
            "lwip_socket",
        ]))

    def test_similar_names_are_not_false_positives(self):
        nm = "\n".join([
            "42002000 T modem_clock_hal_enable_wifipwr_clock",
            "42002100 T codecell::runtime::WebServerless::step()",
            "42002200 T esp_wifi_initialized_flag_reader",
        ])
        self.assertEqual(image_policy.violations(nm), [])


if __name__ == "__main__":
    unittest.main()
