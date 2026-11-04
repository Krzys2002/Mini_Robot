#!/usr/bin/env python3
"""Optional, local-only Graphify entry point. No imports from Graphify itself."""
from __future__ import annotations

import argparse
import contextlib
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

VERSION = "0.9.102"
FORMAT = 1
OUT = "graphify-out"


def git(root: Path, *args: str) -> bytes:
    return subprocess.check_output(["git", "-C", str(root), *args])


def repository_root() -> Path:
    # The script location, rather than the caller's cwd, identifies the worktree.
    return Path(git(Path(__file__).resolve().parent, "rev-parse", "--show-toplevel")
                .decode().strip()).resolve()


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def excluded(name: str) -> bool:
    parts = Path(name).parts
    return any(p in {OUT, ".git", ".idea", ".pio", ".venv", "__pycache__",
                     "build", "out", "_deps"} or p.startswith("cmake-build-")
               for p in parts)


def snapshot(root: Path) -> dict:
    # Over-approximate the graph corpus: also hash docs/build contracts so a
    # change to navigation policy or target ownership invalidates the snapshot.
    names = git(root, "ls-files", "--cached", "--others", "--exclude-standard", "-z")
    ignored = set(git(root, "ls-files", "--cached", "--ignored",
                      "--exclude-standard", "-z").split(b"\0"))
    files = {}
    for raw in sorted(set(names.split(b"\0")) - ignored):
        if not raw:
            continue
        name = os.fsdecode(raw)
        path = root / name
        if excluded(name):
            continue
        # Never follow symlinks into a different worktree or an external file.
        if path.is_symlink():
            files[name] = "symlink:" + os.readlink(path)
        elif path.is_file():
            files[name] = digest(path)
    return {"format": FORMAT, "root": str(root),
            "head": git(root, "rev-parse", "HEAD").decode().strip(),
            "graphify_version": VERSION, "profile": "code-only", "files": files}


def graph_data(root: Path) -> dict:
    data = json.loads((root / OUT / "graph.json").read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not isinstance(data.get("nodes"), list):
        raise ValueError("invalid graph nodes")
    metadata = data.get("graph")
    if (not isinstance(data.get("links"), list) or not isinstance(metadata, dict)
            or not metadata.get("schema_version")
            or any(not isinstance(n, dict) for n in data["nodes"])):
        raise ValueError("missing graph links or schema version")
    if metadata.get("graphify_version") != VERSION:
        raise ValueError("unexpected Graphify graph version")
    if any(n.get("file_type") in {"document", "paper", "image", "video"}
           for n in data["nodes"]):
        raise ValueError("semantic content found; run build for a clean code-only graph")
    return data


def freshness(root: Path) -> tuple[bool, str]:
    try:
        saved = json.loads((root / OUT / "project-snapshot.json").read_text())
        graph_data(root)
        if saved["graph_sha256"] != digest(root / OUT / "graph.json"):
            return False, "graph changed outside the wrapper"
        if saved["snapshot"] != snapshot(root):
            return False, "checkout, content, or policy changed"
        return True, "fresh (code-only; documentation is not indexed)"
    except (OSError, ValueError, KeyError, TypeError):
        return False, "missing, incompatible, or corrupt graph/provenance"


def python_executable(root: Path) -> Path:
    env = root / "tools" / "graphify" / ".venv"
    return env / ("Scripts/python.exe" if os.name == "nt" else "bin/python")


def environment(root: Path) -> dict:
    env = dict(os.environ)
    env.update(GRAPHIFY_BACKEND="none", GRAPHIFY_QUERY_LOG_DISABLE="1",
               GRAPHIFY_NO_TIPS="1", GRAPHIFY_OUT=str(root / OUT))
    return env


def run_graphify(root: Path, args: list[str]) -> int:
    executable = python_executable(root)
    if not executable.exists():
        raise ValueError("Tool environment missing. Run uv sync --project tools/graphify --locked "
                         "from the repository root. Use normal source search meanwhile.")
    installed = subprocess.check_output(
        [str(executable), "-c", "from importlib.metadata import version; print(version('graphifyy'))"],
        text=True).strip()
    if installed != VERSION:
        raise ValueError(f"Expected Graphify {VERSION}, found {installed}; sync the locked environment.")
    return subprocess.call([str(executable), "-m", "graphify", *args],
                           cwd=root, env=environment(root))


@contextlib.contextmanager
def operation_lock(root: Path):
    out = root / OUT
    out.mkdir(exist_ok=True)
    lock = out / ".project-operation.lock"
    try:
        fd = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
    except FileExistsError as exc:
        raise ValueError("Another graph operation holds the lock. If its process exited, "
                         "remove graphify-out/.project-operation.lock and retry.") from exc
    try:
        with os.fdopen(fd, "w") as handle:
            handle.write(str(os.getpid()))
        yield
    finally:
        lock.unlink(missing_ok=True)


def rebuild(root: Path, command: str) -> int:
    before = snapshot(root)
    stamp = root / OUT / "project-snapshot.json"
    if command == "update":
        # Incremental updates may replace code, but may not adopt a different
        # worktree, extraction policy, or externally written graph.
        saved = json.loads(stamp.read_text())
        previous = saved["snapshot"]
        if not isinstance(previous, dict) or not isinstance(previous.get("files"), dict):
            raise ValueError("Corrupt provenance; run build.")
        keys = ("format", "root", "graphify_version", "profile")
        policy = (".graphifyrc", ".graphifyignore", "tools/graphify/uv.lock",
                  "tools/graphify/pyproject.toml", "tools/graphify/project_graph.py")
        if (any(previous.get(k) != before[k] for k in keys)
                or any(previous["files"].get(p) != before["files"].get(p) for p in policy)
                or saved["graph_sha256"] != digest(root / OUT / "graph.json")):
            raise ValueError("Graph identity or extraction policy changed; run build.")
    stamp.unlink(missing_ok=True)
    if command == "build":
        # Build from an empty graph: upstream --force --code-only preserves an
        # existing semantic tier, which would violate this profile's contract.
        # Archive previous output locally for recovery; never delete user files.
        old = root / OUT / "graph.json"
        if old.exists():
            old.replace(root / OUT / "previous-graph.json")
        args = ["extract", str(root), "--code-only", "--force"]
    else:
        graph_data(root)  # update needs a compatible existing graph
        args = ["update", str(root)]
    result = run_graphify(root, args)
    if result:
        return result
    if command == "build":
        result = run_graphify(root, ["cluster-only", str(root), "--no-label"])
        if result:
            return result
    graph_data(root)
    after = snapshot(root)
    if before != after:
        raise ValueError("Sources changed during extraction; graph remains stale. Retry update.")
    record = {"snapshot": after, "graph_sha256": digest(root / OUT / "graph.json"),
              "built_at_unix": time.time()}
    temporary = stamp.with_suffix(".tmp")
    temporary.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    temporary.replace(stamp)
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["build", "update", "status", "query", "path",
                                             "explain", "affected"])
    parser.add_argument("terms", nargs="*")
    args = parser.parse_args(argv)
    counts = {"build": 0, "update": 0, "status": 0, "query": 1,
              "path": 2, "explain": 1, "affected": 1}
    if len(args.terms) != counts[args.command] or any(t.startswith("-") for t in args.terms):
        parser.error(f"{args.command} expects {counts[args.command]} quoted term(s), no options")
    try:
        root = repository_root()
        with operation_lock(root):
            if args.command == "status":
                fresh, reason = freshness(root)
                print(reason)
                return 0 if fresh else 2
            if args.command in {"build", "update"}:
                return rebuild(root, args.command)
            fresh, reason = freshness(root)
            if not fresh:
                raise ValueError(f"Graph unavailable: {reason}. Run build/update, or use source search.")
            return run_graphify(root, [args.command, *args.terms, "--graph",
                                       str(root / OUT / "graph.json")])
    except (OSError, ValueError, KeyError, TypeError, subprocess.CalledProcessError) as exc:
        print(f"Graphify: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
