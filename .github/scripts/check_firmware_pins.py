"""Reject moving or unpinned dependencies in firmware PlatformIO projects (#155).

Every `platform`, `platform_packages` and `lib_deps` entry under
`Mini_Robot/firmware/*/platformio.ini` must name an immutable source: an exact
registry release, a Git commit, or a versioned release asset. Firmware tool
requirements must be exact `name==version` pins. Run from the repository root:

    python3 .github/scripts/check_firmware_pins.py
"""

import configparser
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
FIRMWARE = ROOT / "Mini_Robot" / "firmware"
PINNED_KEYS = ("platform", "platform_packages", "lib_deps")
GIT_COMMIT = re.compile(r"#[0-9a-f]{40}$")
RELEASE_ASSET = re.compile(r"^https://[^\s#]+/releases/download/v?\d+(?:\.\d+)*[^/\s]*/[^/\s]+$")
COMMIT_ARCHIVE = re.compile(r"^https://[^\s#]+/archive/[0-9a-f]{40}\.(?:zip|tar\.gz)$")
EXACT_VERSION = re.compile(r"^\d+(?:\.\d+)*(?:[-+][0-9A-Za-z.]+)?$")
REGISTRY_NAME = re.compile(r"^(?:[\w.-]+/)?[\w.-]+$")
NAMED_SPEC = re.compile(r"^[\w.-]+\s*=\s*(.+)$")
REQUIREMENT = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*==\d+(?:\.\d+)*(?:[-+.]?[0-9A-Za-z.]+)?$")
FIX = ("pin an exact registry release (owner/name@X.Y.Z), a Git commit "
       "(<url>#<40-hex sha>) or a versioned release asset URL")


def spec_problem(spec):
    """Return why a PlatformIO dependency specification is not immutable."""
    if named := NAMED_SPEC.match(spec):  # "name=source" form
        spec = named.group(1).strip()
    if spec.startswith(("symlink://", "file://")) or spec.startswith(("/", ".", "~")):
        return "local path dependency"
    if spec.startswith(("git+", "git@")) or spec.split("#", 1)[0].endswith(".git"):
        return None if GIT_COMMIT.search(spec) else "Git dependency without a commit SHA"
    if "://" in spec:
        if "#" in spec:
            return "URL with a branch/tag fragment instead of a commit SHA"
        if RELEASE_ASSET.match(spec) or COMMIT_ARCHIVE.match(spec):
            return None
        return "URL is not a versioned release asset or commit archive"
    name, separator, version = spec.partition("@")
    if not separator:
        return "unversioned registry dependency"
    if not REGISTRY_NAME.match(name.strip()):
        return "unrecognized dependency name"
    if not EXACT_VERSION.match(version.strip()):
        return f"version '{version.strip()}' is a range or moving tag"
    return None


def manifest_problems(path):
    parser = configparser.ConfigParser(interpolation=None, strict=False)
    parser.read(path, encoding="utf-8")
    problems = []
    for section in parser.sections():
        if section != "env" and not section.startswith("env:"):
            continue
        for key in PINNED_KEYS:
            for line in parser.get(section, key, fallback="").splitlines():
                for spec in filter(None, (part.strip() for part in line.split(","))):
                    spec = spec.split(";", 1)[0].strip()
                    if spec and (reason := spec_problem(spec)):
                        problems.append(f"{path}: [{section}] {key} = {spec}: {reason}; {FIX}")
    return problems


def requirements_problems(path):
    problems = []
    for number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.split("#", 1)[0].strip()
        if line and not REQUIREMENT.match(line):
            problems.append(f"{path}:{number}: '{line}' is not an exact name==version pin")
    return problems


def check(firmware=FIRMWARE):
    manifests = sorted(firmware.glob("*/platformio.ini"))
    if not manifests:
        return [f"{firmware}: no PlatformIO manifests found"]
    problems = []
    for manifest in manifests:
        problems += manifest_problems(manifest)
        requirements = manifest.parent / "requirements.txt"
        if requirements.is_file():
            problems += requirements_problems(requirements)
    return problems


def main():
    problems = check()
    for problem in problems:
        print(f"error: {problem}", file=sys.stderr)
    if problems:
        print("Firmware dependencies must stay pinned; see "
              "Mini_Robot/firmware/codecell-c6/README.md (dependency review).", file=sys.stderr)
        return 1
    print("Firmware dependency pins: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
