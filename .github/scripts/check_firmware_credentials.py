"""Reject credential material in the CodeCell C6 production tree (#165, F7, AT-11).

The restricted-lab v1 profile has no keys, passkeys or passwords
(Mini_Robot/docs/protocol/security-profile.md). This scan fails when the production
sources, headers, build scripts and PlatformIO configuration contain a credential-like
assignment, a PEM key block, an enabled pairing/passkey call, or a log/advertising format
that would print a credential. Tests, fixtures and the retired T1 prototypes are outside
the production tree and are not scanned. Run from the repository root:

    python3 .github/scripts/check_firmware_credentials.py
"""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PRODUCTION = ROOT / "Mini_Robot" / "firmware" / "codecell-c6"
SCANNED_DIRECTORIES = ("src", "include", "scripts")
SCANNED_FILES = ("platformio.ini", "CMakeLists.txt", "requirements.txt")
SCANNED_SUFFIXES = {".cpp", ".hpp", ".h", ".c", ".py", ".ini", ".txt", ".cmake", ".csv"}

CREDENTIAL_NAME = r"(?:pass(?:word|phrase|key|wd)?|psk|secret|token|api[_-]?key|private[_-]?key|credential|ssid|auth[_-]?key)"
# A credential-named identifier or key assigned a non-empty string literal.
ASSIGNED_LITERAL = re.compile(
    rf"""(?ix)
    \b[\w.]*{CREDENTIAL_NAME}[\w.]*\b\s*(?:=|:|\(|\{{)\s*(?:\\?")[^"\\\s][^"]*(?:\\?")""")
PEM_BLOCK = re.compile(r"-----BEGIN [A-Z ]*(?:PRIVATE KEY|CERTIFICATE|PUBLIC KEY)-----")
LONG_HEX_LITERAL = re.compile(r'"(?:[0-9a-fA-F]{2}){16,}"')
PAIRING_CALL = re.compile(
    r"\b(?:setSecurityPasskey|setPassKey|setPin|setSecurityAuth\s*\(\s*true|setSecurityInitKey|"
    r"setSecurityRespKey|setSecurityIOCap\s*\(\s*(?!BLE_HS_IO_NO_INPUT_OUTPUT)|"
    r"startSecurity|secureConnection|setBond|saveBond|createBond)\b")
CREDENTIAL_FORMAT = re.compile(
    rf'''(?ix)(?:printf|snprintf|appendf|logf|logLine|print(?:ln)?|write)\s*\([^;]*"[^"]*\b{CREDENTIAL_NAME}\s*=[^"]*%''')
PRINTED_NAME = re.compile(rf'''(?ix)(?:printf|snprintf|appendf|logf|print(?:ln)?)\s*\([^;]*\b(?:get|print)?{CREDENTIAL_NAME}\w*\s*[,)]''')


def source_files():
    for name in SCANNED_FILES:
        path = PRODUCTION / name
        if path.is_file():
            yield path
    for directory in SCANNED_DIRECTORIES:
        base = PRODUCTION / directory
        if not base.is_dir():
            continue
        for path in sorted(base.rglob("*")):
            if path.is_file() and path.suffix in SCANNED_SUFFIXES and "__pycache__" not in path.parts:
                yield path


def code_part(line, suffix):
    """Return the code before a trailing comment; comments describe, they do not carry values."""
    marker = ";" if suffix == ".ini" else "#" if suffix in {".py", ".txt", ".cmake"} else "//"
    index = line.find(marker)
    return line if index < 0 else line[:index]


def scan_text(text, name, suffix=".cpp"):
    findings = []
    for number, raw in enumerate(text.splitlines(), 1):
        if PEM_BLOCK.search(raw):
            findings.append((name, number, "PEM key or certificate block"))
            continue
        line = code_part(raw, suffix)
        if not line.strip():
            continue
        if ASSIGNED_LITERAL.search(line):
            findings.append((name, number, "credential-named value assigned a string literal"))
        elif LONG_HEX_LITERAL.search(line):
            findings.append((name, number, "long hexadecimal literal that may be key material"))
        elif PAIRING_CALL.search(line):
            findings.append((name, number, "pairing, bonding or passkey call (F6 forbids it)"))
        elif CREDENTIAL_FORMAT.search(line) or PRINTED_NAME.search(line):
            findings.append((name, number, "log or advertising output that names a credential"))
    return findings


def scan_tree():
    findings = []
    for path in source_files():
        text = path.read_text(encoding="utf-8", errors="replace")
        findings.extend(scan_text(text, path.relative_to(ROOT).as_posix(), path.suffix))
    return findings


def main():
    findings = scan_tree()
    scanned = sum(1 for _ in source_files())
    if not findings:
        print(f"No credential material in {scanned} production files (restricted-lab-v1: no secrets, F7).")
        return 0
    print("Credential material is forbidden in the restricted-lab v1 production tree (F7, AT-11):", file=sys.stderr)
    for name, number, reason in findings:
        print(f"  {name}:{number}: {reason}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
