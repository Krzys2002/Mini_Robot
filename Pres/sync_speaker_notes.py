#!/usr/bin/env python3
"""Sync speaker_notes.md sections into Beamer \note{...} blocks in pres.tex."""

from pathlib import Path
import re


ROOT = Path(__file__).resolve().parent
NOTES_PATH = ROOT / "speaker_notes.md"
TEX_PATH = ROOT / "pres.tex"


def latex_escape(text: str) -> str:
    replacements = {
        "\\": r"\textbackslash{}",
        "&": r"\&",
        "%": r"\%",
        "$": r"\$",
        "#": r"\#",
        "_": r"\_",
        "{": r"\{",
        "}": r"\}",
        "~": r"\textasciitilde{}",
        "^": r"\textasciicircum{}",
    }
    return "".join(replacements.get(ch, ch) for ch in text)


def extract_notes(markdown: str) -> list[str]:
    pattern = re.compile(
        r"^##\s+Slide\s+\d+:[^\n]*\n(?P<body>.*?)(?=^##\s+Slide\s+\d+:|\Z)",
        re.M | re.S,
    )
    notes = []
    for match in pattern.finditer(markdown):
        body = match.group("body").strip()
        notes.append(latex_escape(body))
    return notes


def find_note_blocks(tex: str) -> list[tuple[int, int]]:
    blocks = []
    search_from = 0
    marker = r"\note{"
    while True:
        start = tex.find(marker, search_from)
        if start == -1:
            break
        pos = start + len(marker)
        depth = 1
        while pos < len(tex) and depth:
            char = tex[pos]
            prev = tex[pos - 1] if pos else ""
            if char == "{" and prev != "\\":
                depth += 1
            elif char == "}" and prev != "\\":
                depth -= 1
            pos += 1
        if depth:
            raise ValueError(f"Unclosed note block starting at byte {start}")
        blocks.append((start, pos))
        search_from = pos
    return blocks


def main() -> None:
    notes = extract_notes(NOTES_PATH.read_text(encoding="utf-8"))
    tex = TEX_PATH.read_text(encoding="utf-8")
    blocks = find_note_blocks(tex)

    if len(notes) != len(blocks):
        raise SystemExit(
            f"Expected {len(blocks)} slide notes in {NOTES_PATH.name}, "
            f"found {len(notes)}."
        )

    updated = []
    last = 0
    for (start, end), note in zip(blocks, notes):
        updated.append(tex[last:start])
        updated.append("\\note{\n")
        updated.append(note)
        updated.append("\n}")
        last = end
    updated.append(tex[last:])

    TEX_PATH.write_text("".join(updated), encoding="utf-8")
    print(f"Synced {len(notes)} notes from {NOTES_PATH.name} into {TEX_PATH.name}.")


if __name__ == "__main__":
    main()
