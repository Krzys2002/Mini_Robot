# GitHub blocker Status automation

The workflow synchronizes the existing **Status** field in [Krzys2002 Project #4](https://github.com/users/Krzys2002/projects/4) with native GitHub Issue dependencies for `Krzys2002/Mini_Robot`.

## Architecture

- `.github/workflows/sync-issue-blockers.yml` runs when an issue is opened, closed, or reopened; every 30 minutes (`7,37 * * * *`, UTC); and manually.
- `.github/scripts/sync_issue_blockers.py` uses Python 3.10+ standard-library code and the authenticated GitHub CLI. GitHub-hosted Ubuntu runners provide Python and `gh`.
- Every run inventories all Project fields and items with GraphQL pagination, then checks native blockers for repository issues whose Status is Todo or Blocked. A full scan also catches dependencies added or removed since the preceding run, including when no issue event occurred.
- The script resolves Project, field, option, and item identifiers at runtime. It updates only the existing Status field. It never edits issue fields, creates issues or milestones, or changes Project configuration/views.
- Each individual issue failure is logged and processing continues. Any incomplete synchronization returns a nonzero exit code. Configuration or inventory failures stop the run before any mutation. Subsequent runs retry from current GitHub data.

## Transition rules

| Current Status | Native blocker states | Result |
| --- | --- | --- |
| Todo | At least one OPEN | Blocked |
| Todo | No OPEN blockers, including no dependencies | Unchanged |
| Blocked | At least one OPEN | Unchanged |
| Blocked | No OPEN blockers, including no dependencies | Todo |
| In progress or Done | Any | Unchanged |
| Backlog, Review, any other option, or unset | Any | Unchanged |

Only native dependency state is used. Markdown links, labels, milestone order, and issue descriptions are ignored. An issue's own closed state does not cause a Done transition. A manually selected Blocked status is restored to Todo when it has no open native blockers.

The existing field was inspected before implementation: Todo, Blocked, In progress, and Done already exist, alongside Backlog and Review. No settings update was needed. Option names are matched case-insensitively so the existing `In progress` spelling is retained; IDs are preserved. If a required option later disappears, the workflow fails rather than creating or replacing fields/options.

Concurrency prevents overlapping runs of this workflow. Immediately before a mutation, the script re-reads the item, verifies its Project and issue identity, and respects its fresh Status. GitHub's mutation offers no compare-and-set condition: a manual edit in the short interval between that read and the write can race with the automation. Blocker state is also a snapshot; later changes are reconciled on the next run.

## APIs and pagination

- `GET /repos/{owner}/{repo}/issues/{issue_number}/dependencies/blocked_by?per_page=100&page=N`: reads native blockers and each returned issue's `state`. Pages are read until fewer than 100 results are returned; any OPEN blocker makes the dependency unresolved. Unknown or inaccessible blocker states fail safely for that issue.
- `POST /graphql`: reads `user.projectV2`, `fields`, `items`, `fieldValueByName`, and the item recheck. Field and item connections use `first:100`, `after`, and `pageInfo` cursors.
- `updateProjectV2ItemFieldValue`: writes `value.singleSelectOptionId` for the existing Status field only. Successful mutations confirm the expected item ID.

REST calls use `Accept: application/vnd.github+json` and `X-GitHub-Api-Version: 2026-03-10`. Commands send GraphQL payloads through stdin; credentials are never included in command arguments or committed files.

References: [native dependency REST API](https://docs.github.com/en/rest/issues/issue-dependencies?apiVersion=2026-03-10), [Projects automation authentication](https://docs.github.com/en/issues/planning-and-tracking-with-projects/automating-your-project/automating-projects-using-actions).

## Authentication and configuration

The workflow reuses the repository's existing **`PROJECT_TOKEN`** Actions secret. The existing `daily-report.yml` already uses this secret for Project operations; its stored value and exact scopes cannot be inspected. A new token is not required if its current permissions cover these operations.

Required access:

- Repository Issues: read, including native dependencies and every blocker that must be read.
- The owner's user Projects: read and write for Project #4.
- For a classic PAT, the `project` scope covers Project access; add `repo` when private repository issues must be read. A fine-grained PAT needs the owner's account **Projects: Read and write**, and selected repository **Issues: Read** (plus automatic Metadata read). Ensure the token's resource owner and repository selection include this repository and any private blocker repositories.

The default `GITHUB_TOKEN` is used by checkout with `contents: read`; it is not used to update the user-owned Project. Workflow `permissions` do not grant a PAT additional access. Authorization policies, token expiry, or missing repository access may still prevent a run.

Set or rotate `PROJECT_TOKEN` under repository **Settings → Secrets and variables → Actions → Repository secrets**. Never paste a token into a workflow, documentation, logs, or CLI arguments. The token is exposed only to the synchronization step, after unit tests run. API errors redact known token values and common GitHub token formats.

The workflow explicitly sets owner `Krzys2002` and number `4`. It preserves existing repository variables, including `PROJECT_OWNER=@me`; `@me` is a CLI alias and is not used as a GraphQL user login here. Only issues belonging to the executing repository and included in Project #4 are eligible; other repository items, pull requests, and drafts are skipped.

## Local tests and preview

Run from the Git repository root (the directory containing `.github`):

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s .github/scripts/tests -v
python3 .github/scripts/sync_issue_blockers.py --dry-run
```

The preview uses your existing `gh` authentication and reads live data without mutations. Tests require no token or network and cover the seven requested scenarios, idempotence, field/item/dependency pagination, protected fresh statuses, scope filtering, dry-run behavior, missing configuration, API failures, and credential redaction.

## Activation and manual execution

Push the implementation branch only after maintainer approval, review it, and merge into the default branch (`master`). Issue and scheduled triggers use the default branch. Scheduled runs apply changes automatically after activation; GitHub cron may be delayed, and cannot guarantee a 30-minute maximum latency.

For a manual preview, open **Actions → Sync issue blockers → Run workflow**, choose the appropriate ref, and keep `dry_run` checked (the default). To apply changes, run again with `dry_run` unchecked. Once the workflow exists on the default branch, the equivalent CLI commands are:

```sh
gh workflow run sync-issue-blockers.yml --repo Krzys2002/Mini_Robot --ref master -f dry_run=true
gh workflow run sync-issue-blockers.yml --repo Krzys2002/Mini_Robot --ref master -f dry_run=false
gh run list --repo Krzys2002/Mini_Robot --workflow sync-issue-blockers.yml
gh run view RUN_ID --repo Krzys2002/Mini_Robot --log
```

A dry-run validates reads and transition decisions, but cannot prove the stored secret has Project write access. The first applying run validates that permission. To test live behavior, use suitable existing Project issues with explicit maintainer authorization: preview/apply with an open blocker, close the blocker and rerun, then verify the dependent item's Status. Local automated tests exercise all requested cases without modifying live issue state or dependencies.

## Logs and troubleshooting

The script emits one JSON log entry per change, proposed change, skip, or error, followed by a summary with `scanned`, `skipped`, `changed`, `would_change`, and `failed` counts.

| Symptom | Check |
| --- | --- |
| Missing `PROJECT_TOKEN` | Configure the repository secret; no default-token fallback is used. |
| Project inaccessible / GraphQL permission error | Token owner, account Projects permissions, repository access, expiry, and authorization policy. |
| REST 403/404 or dependency read failure | Issues read permission and visibility of the affected repository/blockers; leave its Status untouched until reads succeed. |
| Missing Status field/options | Restore the existing field/required options through normal maintainer configuration; the script does not alter settings. |
| Write denied but preview succeeds | Grant the token account Projects write access. |
| Nonzero run after some changes | Inspect per-issue errors, fix the cause, and rerun; successful transitions are idempotent. |
| No update after adding/removing a dependency | Run manually or wait for the next full scheduled scan; dependencies do not have a dedicated trigger here. |
| Rate limit, timeout, or transient API failure | Inspect the error and rerun after access recovers; each CLI request has a 45-second timeout and the job a 15-minute limit. |
| Issue skipped | Confirm Project membership, repository, and current Status. Protected/other statuses are intentional skips. |

No dependency relationships, issue descriptions, milestones, assignees, deadlines, priorities, or Project views are changed by this workflow.
