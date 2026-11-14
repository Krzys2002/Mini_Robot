"""Conservative source checks after upstream AST resolution, before clustering.

This is not a compiler: reject unsupported call targets rather than inventing
replacement bindings. In particular a corpus-wide lookup cannot choose a host
fake for an embedded/application build without a configured build context.
"""
from pathlib import Path
import hashlib
import re

CPP_SUFFIXES = {".cpp", ".hpp", ".h", ".cc", ".cxx", ".hh", ".hxx", ".ino"}
PRODUCTION = ("Mini_Robot/src/", "Mini_Robot/firmware/", "T1-ESP-C6/", "T1-ESP-S3/")


def retain_rejections(root: Path, current: dict, previous: dict) -> dict:
    """Keep known omissions across a no-change update, discard changed evidence."""
    records = {}
    for old in previous.get("rejected_calls", []):
        path = root / old["source_file"]
        if (path.is_file() and not path.is_symlink()
                and old.get("source_sha256") == hashlib.sha256(path.read_bytes()).hexdigest()):
            records[(old["source"], old["target"], old.get("source_location"))] = old
    for item in current["rejected_calls"]:
        item["source_sha256"] = hashlib.sha256((root / item["source_file"]).read_bytes()).hexdigest()
        records[(item["source"], item["target"], item.get("source_location"))] = item
    return {"policy": current["policy"], "rejected_this_operation": len(current["rejected_calls"]),
            "rejected_calls": list(records.values())}


def check_cpp_calls(root: Path, data: dict) -> dict:
    nodes = {node["id"]: node for node in data["nodes"]}
    retained = []
    rejected = []
    lines = {}
    for edge in data["links"]:
        source = edge.get("source_file", "")
        if edge.get("relation") != "calls" or Path(source).suffix not in CPP_SUFFIXES:
            retained.append(edge)
            continue
        target = nodes[edge["target"]]
        reason = None
        if source.startswith(PRODUCTION) and target.get("source_file", "").startswith("Mini_Robot/tests/"):
            reason = "production-to-test binding lacks configured build context"
        else:
            # C++ names are case sensitive. A class/function ID collision or a
            # field substituted for a getter must not certify a different name.
            name = target.get("label", "").removesuffix("()").split("::")[-1].lstrip(".")
            location = re.fullmatch(r"L(\d+)", edge.get("source_location", ""))
            if name.isidentifier() and location:
                if source not in lines:
                    lines[source] = (root / source).read_text(encoding="utf-8").splitlines()
                index = int(location[1]) - 1
                if not 0 <= index < len(lines[source]):
                    reason = "call location outside source"
                elif not re.search(r"\b" + re.escape(name) + r"\s*(?:<[^;{}]*>)?\s*\(", lines[source][index]):
                    # A multi-line call may put its '(' on a following line.
                    window = "\n".join(lines[source][index:index + 3])
                    if not re.search(r"\b" + re.escape(name) + r"\s*(?:<[^;{}]*>)?\s*\(", window):
                        reason = "callee spelling is not supported at the call site"
        if reason:
            rejected.append({"source": edge["source"], "target": edge["target"],
                             "source_file": source, "source_location": edge.get("source_location"),
                             "target_label": target.get("label"), "reason": reason})
        else:
            retained.append(edge)
    data["links"] = retained
    return {"policy": "conservative-cpp-calls-v1", "rejected_calls": rejected}
