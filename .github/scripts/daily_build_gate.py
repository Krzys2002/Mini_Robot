"""Build master once daily only when it contains newly merged pull requests."""

import json
import os
import subprocess
from pathlib import Path


def api_pages(endpoint):
    result = subprocess.run(
        ["gh", "api", "--paginate", "--slurp", endpoint],
        check=True, text=True, capture_output=True,
    )
    return json.loads(result.stdout)


def previous_checkpoint(pages, current_run_id):
    """Successful no-op checks also advance the checkpoint; failures do not."""
    runs = [run for page in pages for run in page["workflow_runs"]
            if str(run["id"]) != str(current_run_id)]
    return max(runs, key=lambda run: run["created_at"], default=None)


def merged_in_commits(pages, commits):
    return any(pr.get("merged_at") and pr["base"]["ref"] == "master"
               and pr.get("merge_commit_sha") in commits
               for page in pages for pr in page)


def main():
    if os.environ["GITHUB_REF"] != "refs/heads/master":
        raise RuntimeError("Daily builds are restricted to master")
    repo = os.environ["GH_REPO"]
    pages = api_pages(
        f"repos/{repo}/actions/workflows/daily-build-test.yml/runs"
        "?branch=master&event=schedule&status=success&per_page=100"
    )
    previous = previous_checkpoint(pages, os.environ["GITHUB_RUN_ID"])
    if previous is None:
        # Until one complete daily suite succeeds, validate the actual current
        # baseline regardless of merge age or the duration of an Actions outage.
        should_build = True
    else:
        result = subprocess.run(
            ["git", "rev-list", os.environ["GITHUB_SHA"], f'^{previous["head_sha"]}'],
            check=True, text=True, capture_output=True,
        )
        commits = set(result.stdout.splitlines())
        should_build = bool(commits) and merged_in_commits(
            api_pages(f"repos/{repo}/pulls?state=closed&base=master&per_page=100"), commits,
        )
    with Path(os.environ["GITHUB_OUTPUT"]).open("a") as output:
        output.write(f"build={str(should_build).lower()}\n")
    print("Validate baseline/new master merges: run builds" if should_build else "No new master merges: skip builds")


if __name__ == "__main__":
    main()
