"""PlatformIO extra script: fail the build if the linked image contains legacy
Wi-Fi/AP/HTTP/OTA code (#158). The rule itself is `image_policy.py`."""
import subprocess
import sys
from pathlib import Path

Import("env")  # noqa: F821 - provided by PlatformIO/SCons

sys.path.insert(0, str(Path(env.subst("$PROJECT_DIR")) / "scripts"))
from image_policy import violations  # noqa: E402


def nm_tool():
    compiler = env.subst("$CC")
    for candidate in (env.subst("$NM"), compiler.replace("gcc", "gcc-nm"), compiler.replace("gcc", "nm")):
        if candidate and env.WhereIs(candidate):
            return env.WhereIs(candidate)
    return None


def check_image(target, source, env):
    elf = Path(env.subst("$BUILD_DIR")) / f"{env.subst('$PROGNAME')}.elf"
    tool = nm_tool()
    if tool is None:
        print("\nError: no nm found next to the compiler; the production image policy cannot run.\n")
        return 1
    result = subprocess.run([tool, "-C", str(elf)], capture_output=True, text=True, check=False)
    if result.returncode != 0:
        print(f"\nError: {Path(tool).name} failed on {elf.name}: {result.stderr.strip()}\n")
        return 1
    found = violations(result.stdout)
    if found:
        print("\nError: the production image links retired Wi-Fi/AP/HTTP/OTA code (#158):")
        for name in found:
            print(f"  {name}")
        print("Remove it, or build a separate experiment environment behind the same safety gate.\n")
        return 1
    print("Production image policy: no Wi-Fi/AP/HTTP/OTA code linked")
    return 0


env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", check_image)
