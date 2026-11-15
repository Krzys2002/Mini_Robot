"""Production image policy for the CodeCell C6 firmware (#158).

Pure functions (no PlatformIO import) so the rule is unit-tested on the host by
`.github/scripts/tests/test_firmware_image_policy.py`. The PlatformIO glue is
`production_image_policy.py`.

The production image must not contain the retired legacy control paths: Wi-Fi and
access-point networking, HTTP/web servers or clients, DNS captive portals, or OTA
update code. The check reads the linked ELF symbol table (demangled `nm` output),
so code that a source change pulls in transitively is caught as well. ROM symbols
(`A`, absolute) and undefined references are ignored: the C6 boot ROM always
exports Wi-Fi helper addresses, which are not code linked into the image.
"""
import re

# Linked entry points of the stacks that must stay out of the production image.
FORBIDDEN_SYMBOLS = (
    # ESP-IDF Wi-Fi, network interface and TCP/IP stacks.
    re.compile(r"^esp_wifi_(init|start|set_mode|connect)$"),
    re.compile(r"^esp_netif_init$"),
    re.compile(r"^tcpip_init$"),
    re.compile(r"^lwip_(socket|bind|listen)$"),
    # ESP-IDF HTTP server/client and OTA.
    re.compile(r"^httpd_start$"),
    re.compile(r"^esp_http_client_init$"),
    re.compile(r"^esp_(https_)?ota_begin$"),
    # Arduino-ESP32 Wi-Fi, web server, DNS, HTTP client and OTA classes.
    re.compile(r"^(WiFiClass|WiFiGenericClass|WiFiAPClass|WiFiSTAClass|WiFiServer|WiFiClient)::"),
    re.compile(r"^(WebServer|AsyncWebServer|DNSServer|HTTPClient|ArduinoOTAClass|UpdateClass)::"),
)
IGNORED_TYPES = {"A", "U", "w", "v"}


def defined_symbols(nm_output):
    """Yield (type, name) for each symbol line of `nm -C` output that is linked code/data."""
    for line in nm_output.splitlines():
        parts = line.strip().split(None, 2)
        if len(parts) == 3:
            _, kind, name = parts
        elif len(parts) == 2:  # Undefined symbols have no address.
            kind, name = parts
        else:
            continue
        if len(kind) != 1 or kind in IGNORED_TYPES:
            continue
        yield kind, name


def violations(nm_output):
    """Return the sorted, de-duplicated forbidden symbol names linked into the image."""
    found = set()
    for _, name in defined_symbols(nm_output):
        if any(pattern.search(name) for pattern in FORBIDDEN_SYMBOLS):
            found.add(name)
    return sorted(found)
