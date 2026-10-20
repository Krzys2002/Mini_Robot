# Project plan presentation

The ten-slide Beamer presentation covers the desktop control system, M1–M5
deliverables, the Gantt chart, planned expansion through M8, and validation.

- `Micro-Robot-Project-Plan.tex`: editable LaTeX source, including the TikZ Gantt chart.
- `Micro-Robot-Project-Plan.pdf`: compiled presentation for review and presenting.

Build from the `Mini_Robot` directory with a TeX Live installation containing
Beamer, Latin Modern, PGF/TikZ, and `latexmk`:

```sh
latexmk -pdf -interaction=nonstopmode -halt-on-error \
  -outdir=build/presentation-plan/latex \
  output/project-plan/Micro-Robot-Project-Plan.tex
cp build/presentation-plan/latex/Micro-Robot-Project-Plan.pdf \
  output/project-plan/Micro-Robot-Project-Plan.pdf
```

Intermediate TeX files stay in the ignored `build/` directory. The PDF is a
deliberately versioned presentation deliverable.

Content follows [the milestone roadmap](../../docs/planning/MILESTONES.md) and
[the project instructions](../../AGENTS.md). The Gantt chart reproduces the
[10 October 2026 schedule snapshot](../../docs/archive/GITHUB_PROJECT_GANTT.md),
not current completion status. That snapshot omits later Linux additions.
M6–M8 remain undated expansion plans.
