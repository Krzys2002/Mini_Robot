#!/usr/bin/env python3
"""Isolated Unix process RSS/time evidence for the opt-in workspace SVD audit.

Each child runs exactly one C++ case so RUSAGE_CHILDREN is not contaminated by
another mode. RSS includes linked-library/runtime overhead; U dimensions provide
the decomposition allocation evidence. No platform-independent timing guarantee.
"""
import argparse
import json
import os
import platform
import resource
import subprocess
import sys
import time


def child(binary, mode):
    env = dict(os.environ, MICRO_ROBOT_WORKSPACE_SVD_AUDIT=mode)
    start = time.monotonic()
    run = subprocess.run([binary, '--gtest_filter=WorkspaceSolverMemoryAudit.*'], env=env, capture_output=True, text=True)
    usage = resource.getrusage(resource.RUSAGE_CHILDREN)
    result = {
        'mode': mode,
        'elapsed_seconds': time.monotonic() - start,
        'peak_rss_bytes': usage.ru_maxrss * (1 if sys.platform == 'darwin' else 1024),
        'exit_code': run.returncode,
        'output': run.stdout + run.stderr,
    }
    print(json.dumps(result))
    return run.returncode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', required=True)
    parser.add_argument('--child', choices=['legacy-full-1525', 'thin-1525', 'thin-4096', 'solver-1525', 'solver-4096'])
    args = parser.parse_args()
    if args.child:
        return child(args.binary, args.child)
    results = []
    for mode in ['legacy-full-1525', 'thin-1525', 'thin-4096', 'solver-1525', 'solver-4096']:
        run = subprocess.run([sys.executable, __file__, '--binary', args.binary, '--child', mode], capture_output=True, text=True)
        if not run.stdout:
            raise RuntimeError(run.stderr)
        results.append(json.loads(run.stdout))
    print(json.dumps({'platform': platform.platform(), 'binary': os.path.abspath(args.binary), 'results': results}, indent=2))
    return 0 if all(result['exit_code'] == 0 for result in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
