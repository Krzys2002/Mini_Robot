#!/usr/bin/env sh
set -eu

python3 sync_speaker_notes.py
pdflatex -interaction=nonstopmode pres.tex
pdflatex -interaction=nonstopmode pres.tex
