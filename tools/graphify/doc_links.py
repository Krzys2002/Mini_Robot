"""Local technical-document navigation from explicit repository references only.

No model extraction, remote links, directory crawling or inferred implementation
claims. Only allowlisted docs with a path to an extracted source file survive.
"""
from pathlib import Path
import posixpath
import re
from urllib.parse import unquote, urlsplit

PROFILE = "code-and-linked-technical-docs"
MARKER = "repository-document-links-v1"


def technical_document(name: str) -> bool:
    path = Path(name)
    if path.suffix.lower() != ".md":
        return False
    if name in {"README.md", "Mini_Robot/README.md", "T1-ESP-C6/README.md",
                "Mini_Robot/firmware/ble-lifecycle/README.md",
                "Mini_Robot/docs/testing/ble-connection-187.md",
                "Mini_Robot/docs/testing/codecell-c6-hardware-154.md"}:
        return True
    return (name.startswith(("Mini_Robot/docs/overview/", "Mini_Robot/docs/protocol/",
                             "Mini_Robot/docs/vision/", "Mini_Robot/docs/development/"))
            and path.name not in {"README.md", "graphify.md"}
            and "github" not in path.name.lower())


def references(name: str, content: str, available: set[str]) -> set[str]:
    """Resolve Markdown destinations and backtick paths, never external URLs."""
    tokens = re.findall(r"\]\(<?([^\s)>]+)>?(?:\s+[^)]*)?\)|`([^`\n]+)`", content)
    found = set()
    for link, literal in tokens:
        try:
            value = urlsplit(link or literal)
        except ValueError:
            continue
        if value.scheme or value.netloc or not value.path or value.path.startswith("/"):
            continue
        target = unquote(value.path)
        candidates = [posixpath.normpath(posixpath.join(posixpath.dirname(name), target))]
        if literal:
            candidates.append(posixpath.normpath(target))
        for candidate in candidates:
            if candidate in available:
                found.add(candidate)
                break
    return found


def add_document_links(root: Path, data: dict, files: dict) -> dict:
    # Remove the old overlay before updates, including deleted docs/references.
    old = {n["id"] for n in data["nodes"] if n.get("extraction") == MARKER}
    data["nodes"] = [n for n in data["nodes"] if n["id"] not in old]
    data["links"] = [e for e in data["links"]
                     if e["source"] not in old and e["target"] not in old]
    source_ids = {}
    for node in data["nodes"]:
        name = node.get("source_file", "")
        if name in files and node.get("file_type") == "code":
            # Prefer a file node; otherwise any extracted symbol anchors the file.
            if (name not in source_ids or node.get("type") == "file"
                    or (node.get("source_location") == "L1"
                        and node.get("label", "").endswith(Path(name).name))):
                source_ids[name] = node["id"]
    docs = {name for name in files if technical_document(name)
            and not (root / name).is_symlink()}
    available = docs | source_ids.keys()
    refs = {name: references(name, (root / name).read_text(encoding="utf-8"), available)
            for name in sorted(docs)}
    # Directed reachability: each retained doc must ultimately reference code.
    connected = set(source_ids)
    while True:
        added = {name for name, targets in refs.items() if targets & connected} - connected
        if not added:
            break
        connected.update(added)
    ids = dict(source_ids)
    for name in sorted(docs & connected):
        node_id = "technical_doc:" + name
        ids[name] = node_id
        title = next((line.lstrip("# ") for line in (root / name).read_text(
            encoding="utf-8").splitlines() if line.startswith("# ")), Path(name).stem)
        data["nodes"].append({"id": node_id, "label": title, "type": "file",
                              "file_type": "document", "source_file": name,
                              "extraction": MARKER})
    for name in sorted(docs & connected):
        for target in sorted(refs[name] & connected):
            data["links"].append({"source": ids[name], "target": ids[target],
                                  "relation": "documents" if target in source_ids else "references",
                                  "confidence": "EXTRACTED", "confidence_score": 1.0,
                                  "source_file": name, "extraction": MARKER})
    data["graph"]["repository_profile"] = PROFILE
    return data
