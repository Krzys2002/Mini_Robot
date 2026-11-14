# Local graph lifecycle

Use this workflow for code relationship navigation alongside root `AGENTS.md`
and the [Graphify guide](../../Mini_Robot/docs/development/graphify.md). This is
an explicitly read instruction file, not a Codex auto-run hook. The wrapper,
instructions, policy and tests travel through Git; `.venv/` and `graphify-out/`
are ignored and must be created independently in every worktree.

## Select the task worktree

Run `git rev-parse --show-toplevel` and work from that root. Read applicable
instructions and any assigned issue first. Never use a graph or environment from
another checkout, even if its branch/source looks identical. The wrapper locates
its root from its own script path; from `Mini_Robot/` use `../tools/graphify/`.

```sh
python3 tools/graphify/project_graph.py status
```

Fresh: navigate below. Missing/stale/error (exit 2): choose the appropriate
operation below; read the diagnostic rather than assuming every error is stale.
Graph setup is optional for ordinary coding. If unavailable, use `rg` and source
inspection and report the graph limitation once; do not repeatedly retry.

## Create or recreate in a new worktree

Prerequisites: Git, Python 3.12 and uv 0.13.7. Install uv using an approved package
installation method if absent. Initial setup can download packages and requires
network permission. Do not run `graphify install` or modify global agent files.

```sh
uv sync --project tools/graphify --locked
python3 -m unittest discover -s tools/graphify -p 'test_*.py'
python3 tools/graphify/project_graph.py build
python3 tools/graphify/project_graph.py status
python3 tools/graphify/project_graph.py audit
```

For an offline setup, supply an existing uv package cache, not another worktree's
environment: `uv sync --project tools/graphify --locked --offline`. If packages
are absent, fall back to source inspection until downloads are available.

## Navigate and verify

```sh
python3 tools/graphify/project_graph.py find 'WorldModel'
python3 tools/graphify/project_graph.py explain '<returned exact node ID>'
python3 tools/graphify/project_graph.py affected '<returned exact node ID>'
python3 tools/graphify/project_graph.py path '<ID A>' '<ID B>'
```

`find` accepts a symbol or source-path substring. Prefer IDs for ambiguous
classes/constructors. `query '<exact ID>'` shows that symbol's neighborhood;
free-text query can broaden/truncate results. Exit 3 lists ambiguity; exit 4
means no match, not proof of absent functionality. Verify source lines, edge
direction/confidence, actual target ownership and test bodies before editing.
Use `rg` directly for exact strings and compiler errors. Inspect known omissions
in `graphify-out/source-checks.json`; missing/rejected calls remain unresolved.

## Update after coherent edits or a branch/HEAD change

For compatible provenance and unchanged extraction policy:

```sh
python3 tools/graphify/project_graph.py update
python3 tools/graphify/project_graph.py status
python3 tools/graphify/project_graph.py audit
```

One agent owns refreshes in a shared worktree. Do not refresh during another
agent's edits. Check status again after commits: HEAD is part of provenance, so
a commit can invalidate a graph even without a source change.

## Redo extraction or recover

Use `build`, then `status` and `audit` for a missing/corrupt graph, foreign
provenance, a changed `.graphifyignore`/`.graphifyrc`, changed wrapper/policy code,
or an intentional tool-version/lock change. A clean build archives the old graph
locally; do not edit graph JSON or copy provenance to bypass freshness checks.
Re-run locked `uv sync` if the environment is missing or the installed version
differs from the pin. Never silently upgrade Graphify.

After a failed/concurrent build, stop changing source, inspect the error and
refresh once with the operation it requests. If a lock is left by a killed
process, verify that process has exited before removing only that stale lock.
Do not delete a live lock or an entire graph/build directory as routine recovery.
If recovery fails, use source inspection and report the failure.

## Finish

For graph-tooling changes, run offline tests plus real `build`, `update`, `status`,
`audit`, `find`, exact-symbol and representative path/impact probes. Test a fresh
worktree when changing setup or worktree isolation. Keep corpus/backend limits
from `AGENTS.md`; neither a passing audit nor tests establish complete call
coverage, physical motor safety or a calibrated control loop.

Record commands actually executed and distinguish software/GUI/hardware checks.
Commit only requested source, policy, tests and instructions. Never commit
graphs, caches, environments, secrets or machine-specific symlinks. Refresh the
local graph after the final commit. Do not push, publish artifacts or invoke
manual GitHub diagnostics unless explicitly requested.
