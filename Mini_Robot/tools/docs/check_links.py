"""Validate local links/anchors in maintained guides, without network access."""
from __future__ import annotations

import argparse
import html
from pathlib import Path
import re
import sys
import unicodedata
from urllib.parse import unquote, urlsplit


def prose(text: str) -> str:
    """Ignore fenced/indented code while preserving line numbers for diagnostics."""
    lines = []
    fence = None
    for line in text.splitlines():
        match = re.match(r"^ {0,3}(`{3,}|~{3,})", line)
        if fence:
            if match and match[1][0] == fence[0] and len(match[1]) >= len(fence):
                fence = None
            lines.append("")
        elif match:
            fence = match[1]
            lines.append("")
        else:
            lines.append("" if line.startswith(("    ", "\t")) else line)
    return "\n".join(lines)


def anchors(text: str) -> set[str]:
    text = prose(text)
    result = set()
    for tag in re.finditer(r"<([a-z][a-z0-9-]*)\b([^>]*)>", text, re.I):
        for attribute, value in re.findall(r'\s(id|name)\s*=\s*["\']([^"\']+)["\']', tag[2], re.I):
            if attribute.lower() == "id" or tag[1].lower() == "a":
                result.add(value)
    used = set()
    lines = text.splitlines()
    headings = []
    for index, line in enumerate(lines):
        match = re.match(r"^ {0,3}#{1,6}\s+(.+?)\s*#*\s*$", line)
        if match:
            headings.append(match[1])
        elif index and re.fullmatch(r" {0,3}(?:=+|-+)\s*", line) and lines[index - 1].strip():
            headings.append(lines[index - 1].strip())
    for heading in headings:
        heading = re.sub(r"\[([^]]+)\]\([^)]*\)", r"\1", heading)
        heading = html.unescape(re.sub(r"<[^>]+>", "", heading)).lower()
        slug = "".join(c for c in heading if c in "-_ " or unicodedata.category(c)[0] not in "PS")
        slug = slug.strip().replace(" ", "-")
        candidate, suffix = slug, 0
        while candidate in used:
            suffix += 1
            candidate = f"{slug}-{suffix}"
        used.add(candidate)
        result.add(candidate)
    return result


def destinations(text: str):
    """Yield Markdown inline/reference destinations, including images and <paths>."""
    for number, line in enumerate(prose(text).splitlines(), 1):
        # Inline code can contain literal Markdown examples.
        line = re.sub(r"(`+).*?\1", "", line)
        reference = re.match(r"^ {0,3}\[[^]]+\]:\s*(.*)", line)
        candidates = [reference[1]] if reference else []
        for match in re.finditer(r"\]\(\s*", line):
            rest = line[match.end():]
            if rest.startswith("<"):
                candidates.append(rest)
                continue
            depth, end = 0, 0
            while end < len(rest):
                c = rest[end]
                if c == "\\":
                    end += 2
                    continue
                if c == "(":
                    depth += 1
                elif c == ")":
                    if not depth:
                        break
                    depth -= 1
                elif c.isspace() and not depth:
                    break
                end += 1
            candidates.append(rest[:end])
        for candidate in candidates:
            if candidate.startswith("<"):
                target = candidate[1:].split(">", 1)[0]
            else:
                target = candidate.split(maxsplit=1)[0] if candidate.strip() else ""
            if target:
                yield number, re.sub(r"\\([() ])", r"\1", target)


def maintained_files(root: Path) -> list[Path]:
    files = [root / "README.md", root / "AGENTS.md", root / "docs/README.md"]
    for directory in ("docs/overview", "docs/development", "docs/vision", "docs/protocol", "tools/docs"):
        files.extend((root / directory).rglob("*.md"))
    return sorted({p.resolve() for p in files if p.is_file()})


def check(root: Path, files: list[Path]) -> list[str]:
    errors = []
    cache = {}
    # GitHub root-relative links start at the repository root, one level above
    # the CMake project directory in this repository.
    repository = root.parent if (root.parent / ".github").is_dir() else root
    for source in files:
        text = source.read_text(encoding="utf-8")
        for line, value in enumerate(prose(text).splitlines(), 1):
            # A bare ======= is also a valid setext heading underline.
            if re.match(r"^(<<<<<<< |>>>>>>> )", value):
                errors.append(f"{source}:{line}: unresolved conflict marker")
        for line, raw in destinations(text):
            url = urlsplit(raw)
            if url.scheme or url.netloc:
                continue  # Never contact external services during documentation checks.
            path = unquote(url.path)
            if not path:
                target = source
            else:
                base = repository if path.startswith("/") else source.parent
                target = (base / path.lstrip("/")).resolve()
            if not target.exists():
                errors.append(f"{source}:{line}: missing local target: {raw}")
                continue
            if url.fragment and target.suffix.lower() == ".md":
                if target not in cache:
                    cache[target] = anchors(target.read_text(encoding="utf-8"))
                if unquote(url.fragment) not in cache[target]:
                    errors.append(f"{source}:{line}: missing Markdown anchor: {raw}")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--inventory", type=Path, help="Also check the build-local generated source map")
    args = parser.parse_args()
    files = maintained_files(args.root.resolve())
    if args.inventory:
        if not args.inventory.is_file():
            parser.error(f"inventory does not exist: {args.inventory}")
        files.append(args.inventory.resolve())
    if not files:
        parser.error("no maintained Markdown files found; check --root")
    errors = check(args.root.resolve(), files)
    if errors:
        print("\n".join(errors), file=sys.stderr)
        return 1
    print(f"Documentation links/anchors passed ({len(files)} Markdown files).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
