"""PlatformIO extra script: refuse unpinned toolchains and record build provenance.

Runs while PlatformIO reads the build scripts, before anything is compiled:

* PlatformIO Core must equal the `platformio==` pin in `requirements.txt`.
* The resolved platform must equal the release named by the `platform` URL.

Every default build then writes `$BUILD_DIR/firmware-provenance.json` with the
resolved Core, platform, packages, libraries, compiler, C++ standard, firmware
image digest and source revision. It contains no host user name or path.
"""
import hashlib
import json
import platform as host_platform
import re
import subprocess
from pathlib import Path

from platformio import __version__ as CORE_VERSION
from platformio.package.manager.library import LibraryPackageManager

Import("env")  # noqa: F821 - provided by PlatformIO/SCons

PROJECT_DIR = Path(env.subst("$PROJECT_DIR"))
REQUIREMENTS = PROJECT_DIR / "requirements.txt"
INSTALL_HINT = ("python3 -m pip install -r Mini_Robot/firmware/codecell-c6/requirements.txt "
                "(see Mini_Robot/firmware/codecell-c6/README.md, 'Clean installation')")
RELEASE_URL = re.compile(r"/releases/download/v?(\d+)\.(\d+)\.(\d+)/")
PROVENANCE_SCHEMA = 2  # 2: `source.untracked_inputs`; `modified` includes untracked inputs.
# Project-relative paths whose contents can change the image: PlatformIO configuration,
# the pins, sources, headers, libraries, board definitions and the build scripts.
BUILD_INPUTS = ("platformio.ini", "requirements.txt", "src", "include", "lib", "boards", "scripts")


def fail(message):
    print(f"\nError: {message}\n")
    env.Exit(1)


def pinned_core_version():
    for line in REQUIREMENTS.read_text(encoding="utf-8").splitlines():
        match = re.fullmatch(r"platformio==(\S+)", line.split("#", 1)[0].strip())
        if match:
            return match.group(1)
    fail(f"{REQUIREMENTS.name} does not pin platformio==<version>; restore the reviewed pin.")
    return None


def normalized(version):
    """pioarduino tags use zero padding (55.03.37); PlatformIO reports 55.3.37."""
    return ".".join(str(int(part)) for part in re.findall(r"\d+", str(version))[:3])


def expected_platform_version():
    match = RELEASE_URL.search(env.GetProjectOption("platform"))
    if not match:
        fail("The `platform` option must be an exact pioarduino release asset URL "
             "(.../releases/download/<version>/platform-espressif32.zip).")
    return normalized(".".join(match.groups()))


def check_toolchain():
    expected_core = pinned_core_version()
    if CORE_VERSION != expected_core:
        fail(f"PlatformIO Core {CORE_VERSION} is active, but this project is pinned to "
             f"{expected_core}. Install the reviewed toolchain with: {INSTALL_HINT}. "
             "Do not change pins outside the documented dependency review.")
    resolved_platform = normalized(env.PioPlatform().version)
    expected_platform = expected_platform_version()
    if resolved_platform != expected_platform:
        fail(f"Resolved platform {env.PioPlatform().name} {resolved_platform} does not match "
             f"the pinned release {expected_platform}. Remove the stale platform with "
             "`pio pkg uninstall -g -p espressif32` and rebuild.")


def command_output(*command, **kwargs):
    try:
        result = subprocess.run(command, check=True, text=True, stdout=subprocess.PIPE,
                                stderr=subprocess.DEVNULL, **kwargs)
    except (OSError, subprocess.CalledProcessError):
        return None
    return result.stdout.strip()


def source_revision():
    """Commit plus local changes that can reach the image.

    `modified` is true when tracked project files differ from the commit or when an
    untracked, non-ignored file sits under a build input (an extra `src/*.cpp` is
    compiled without being committed). `untracked_inputs` says which of the two it
    was. Untracked files elsewhere in the project, such as bench logs, are not inputs.
    """
    commit = command_output("git", "-C", str(PROJECT_DIR), "rev-parse", "HEAD")
    if commit is None:
        return {"commit": "unknown", "modified": None, "untracked_inputs": None}
    tracked = command_output("git", "-C", str(PROJECT_DIR), "status", "--porcelain",
                             "--untracked-files=no", "--", ".")
    untracked = command_output("git", "-C", str(PROJECT_DIR), "ls-files", "--others",
                               "--exclude-standard", "--", *BUILD_INPUTS)
    if tracked is None or untracked is None:
        return {"commit": commit, "modified": None, "untracked_inputs": None}
    return {"commit": commit, "modified": bool(tracked) or bool(untracked),
            "untracked_inputs": bool(untracked)}


def libraries():
    libdeps = Path(env.subst("$PROJECT_LIBDEPS_DIR")) / env.subst("$PIOENV")
    result = []
    for package in LibraryPackageManager(str(libdeps)).get_installed():
        metadata = package.metadata
        spec = metadata.spec
        source = spec.uri if spec.external else f"registry:{spec.owner}/{spec.name}"
        result.append({"name": metadata.name, "version": str(metadata.version),
                       "source": source})
    return sorted(result, key=lambda entry: entry["name"])


def packages():
    """Platform packages; tools installed from local paths keep no host path."""
    result = []
    for package in env.PioPlatform().dump_used_packages():
        source = package.get("src_url", "")
        if source.startswith("file:"):
            package["src_url"] = "installed by the pinned platform release"
        result.append(package)
    return sorted(result, key=lambda entry: entry["name"])


def sha256(path):
    if not path.is_file():
        return None
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_provenance(target, source, env):
    build_dir = Path(env.subst("$BUILD_DIR"))
    program = env.subst("$PROGNAME")
    compiler = command_output(env.subst("$CXX"), "--version", env=env["ENV"])
    record = {
        "schema": PROVENANCE_SCHEMA,
        "project": PROJECT_DIR.name,
        "environment": env.subst("$PIOENV"),
        "board": env.subst("$BOARD"),
        "source": source_revision(),
        "platformio_core": CORE_VERSION,
        "platform": {"name": env.PioPlatform().name,
                     "version": str(env.PioPlatform().version),
                     "source": env.GetProjectOption("platform")},
        "packages": packages(),
        "libraries": libraries(),
        "compiler": compiler.splitlines()[0] if compiler else None,
        "cxx_standard": [flag for flag in env.Flatten(env.get("CXXFLAGS", []))
                         if str(flag).startswith("-std=")],
        "image": {"file": f"{program}.bin", "sha256": sha256(build_dir / f"{program}.bin")},
        "host": {"system": host_platform.system(), "machine": host_platform.machine(),
                 "python": host_platform.python_version()},
    }
    output = build_dir / "firmware-provenance.json"
    output.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    print(f"Build provenance: {output.relative_to(PROJECT_DIR)}")


check_toolchain()
provenance = env.Alias("provenance", "$BUILD_DIR/${PROGNAME}.bin",
                       env.VerboseAction(write_provenance, "Recording build provenance"))
env.AlwaysBuild(provenance)
env.Default(provenance)
