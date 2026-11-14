"""Small offline navigation helpers and repeatable graph/source integrity audit."""
from collections import Counter
import copy
from pathlib import Path
import re

from doc_links import MARKER, references, technical_document
from source_checks import check_cpp_calls


def find_nodes(data: dict, term: str) -> list[dict]:
    def canonical(value):
        return value.removesuffix("()").lstrip(".")
    exact = [n for n in data["nodes"] if n["id"] == term
             or canonical(n.get("label", "")) == canonical(term)]
    matches = exact or [n for n in data["nodes"]
                        if term.casefold() in (n.get("label", "") + " "
                                               + n.get("source_file", "")).casefold()]
    return sorted(matches, key=lambda n: (not bool(n.get("source_file")),
                  n.get("source_file", ""), n.get("source_location", ""), n["id"]))


def audit_graph(root: Path, data: dict) -> dict:
    errors = []
    nodes = {n["id"]: n for n in data["nodes"]}
    if len(nodes) != len(data["nodes"]):
        errors.append("duplicate node IDs")
    triples = [(e["source"], e["target"], e.get("relation")) for e in data["links"]]
    if len(triples) != len(set(triples)):
        errors.append("duplicate edge triples")
    lines = {}
    forbidden = (".github/", ".agents/", "Pres/", "report/", "Mini_Robot/output/",
                 "Mini_Robot/docs/planning/", "Mini_Robot/docs/archive/",
                 "Mini_Robot/docs/testing/evidence/")
    for node in data["nodes"]:
        for path_key, line_key in (("source_file", "source_location"),
                                   ("definition_file", "definition_location")):
            name = node.get(path_key, "")
            if not name:
                continue
            path = root / name
            if (Path(name).is_absolute() or ".." in Path(name).parts
                    or name.startswith(forbidden) or path.is_symlink()
                    or not path.is_file()):
                errors.append("invalid/excluded source: " + name)
                continue
            location = re.fullmatch(r"L(\d+)", node.get(line_key, ""))
            if location:
                if name not in lines:
                    lines[name] = path.read_text(encoding="utf-8", errors="replace").splitlines()
                if not 1 <= int(location[1]) <= len(lines[name]):
                    errors.append("invalid location: " + name + ":" + location[0])
        if node.get("file_type") == "document" and not technical_document(node.get("source_file", "")):
            errors.append("unapproved document: " + node["id"])
    available = {n.get("source_file", "") for n in nodes.values()} - {""}
    checked_docs = 0
    doc_refs = {}
    for edge in data["links"]:
        if edge["source"] not in nodes or edge["target"] not in nodes:
            errors.append("dangling edge: " + str(edge))
            continue
        if edge.get("extraction") == MARKER:
            source = nodes[edge["source"]].get("source_file", "")
            target = nodes[edge["target"]].get("source_file", "")
            path = root / source
            if not technical_document(source) or not path.is_file() or path.is_symlink():
                errors.append("invalid document edge owner: " + source)
                continue
            if source not in doc_refs:
                doc_refs[source] = references(source, path.read_text(encoding="utf-8"), available)
            checked_docs += 1
            if target not in doc_refs[source]:
                errors.append("unsupported document reference: " + source + " -> " + target)
    if not any(e["source"] not in nodes or e["target"] not in nodes for e in data["links"]):
        rejected = check_cpp_calls(root, copy.deepcopy(data))["rejected_calls"]
        errors.extend("unsupported call: " + str(e) for e in rejected)
    return {"passed": not errors, "nodes": len(data["nodes"]), "edges": len(data["links"]),
            "source_files": len(available), "documents": sum(n.get("file_type") == "document" for n in nodes.values()),
            "explicit_document_edges_checked": checked_docs,
            "nodes_without_source": sum(not n.get("source_file") for n in nodes.values()),
            "confidence_counts": dict(Counter(e.get("confidence", "unknown") for e in data["links"])),
            "errors": sorted(set(errors)),
            "limits": ["Integrity and lexical evidence checks, not compiler-verified precision/recall",
                       "Missing calls, runtime Qt dispatch and configured build ownership need source inspection"]}
