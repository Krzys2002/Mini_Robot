# Repository agent instructions

The main application is `Mini_Robot/`. Read `Mini_Robot/AGENTS.md` before
editing that subtree. Preserve unrelated user edits. Direct maintenance work
without an assigned issue does not require invented issue dependencies or a PR.
Firmware directories are separate PlatformIO projects; do not add them to desktop
production targets. Build/test recipes are in `Mini_Robot/docs/development/`.

## Optional code graph

Graphify is a derived navigation aid. Applicable instructions and assigned issues
define scope; source and tests establish implementation; live GitHub data
establishes project state; recorded execution establishes validation. Use
`Mini_Robot/docs/overview/capabilities.md` for current integration limits, not
planning catalogs, archives, or the historical `core arhiteture.md` sketch.

- Read applicable instructions and the assigned issue/dependencies first.
- For structural exploration, check `python3 tools/graphify/project_graph.py status`
  from the Git worktree root. A missing/stale graph must not block work: use `rg`
  and source inspection, or explicitly build/update the graph when worthwhile.
- Use scoped query/path/explain/affected commands to locate relevant code/tests.
  Exact strings and errors can use `rg` directly. Verify source before editing;
  an absent edge does not prove independence. Pass this guidance to exploring
  subagents; they share this worktree's graph and must not refresh concurrently.
- After coherent edits, refresh with `update`; use `build` for first use,
  recovery, or changed extraction policy. Every worktree owns its ignored
  `graphify-out/`. Never reuse another checkout's output or commit generated graphs.
- Keep backend `none` and code-only extraction. Semantic indexing, hosted upload,
  saved-answer ingestion, hooks, and global agent changes are outside this setup.
- Do not infer issue completion, calibrated closed-loop operation, physical
  stopping, or firmware safety from graph results. Read live GitHub blockers;
  graph impact suggestions do not replace required build/test suites.

Commands, coverage limits, and recovery: [Graphify guide](Mini_Robot/docs/development/graphify.md).
