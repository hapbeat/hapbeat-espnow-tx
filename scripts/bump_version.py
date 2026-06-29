#!/usr/bin/env python3
"""Bump a single env's release version in firmware-versions.json (DEC-035).

Writes the new version back to the file and PRINTS the git tag command to run —
it never tags or commits automatically (tagging is a deliberate, hardware-verified
step). Versions are per-env: bumping one env never moves another.

Usage:
  python scripts/bump_version.py <env> <patch|minor|major>
  python scripts/bump_version.py <env> --set X.Y.Z
  python scripts/bump_version.py --list

Examples:
  python scripts/bump_version.py m5stack_audio_tx patch   # 0.1.0 -> 0.1.1
  python scripts/bump_version.py esp32 --set 0.2.0
"""
from __future__ import annotations

import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
VERSIONS_PATH = os.path.join(HERE, "..", "firmware-versions.json")
SEMVER_RE = re.compile(r"^\d+\.\d+\.\d+$")


def _load() -> dict:
    with open(VERSIONS_PATH, encoding="utf-8") as f:
        return json.load(f)


def _save(data: dict) -> None:
    with open(VERSIONS_PATH, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
        f.write("\n")


def _bump(version: str, part: str) -> str:
    major, minor, patch = (int(x) for x in version.split("."))
    if part == "major":
        return f"{major + 1}.0.0"
    if part == "minor":
        return f"{major}.{minor + 1}.0"
    if part == "patch":
        return f"{major}.{minor}.{patch + 1}"
    raise ValueError(f"unknown bump part: {part}")


def main(argv: list[str]) -> int:
    data = _load()
    repo_short = data.get("_repoShort", "fw")
    envs = data.get("envs", {})

    if not argv or argv[0] in ("-h", "--help"):
        print(__doc__)
        return 0

    if argv[0] == "--list":
        width = max((len(e) for e in envs), default=0)
        for e, v in envs.items():
            print(f"  {e.ljust(width)}  {v}")
        return 0

    if len(argv) < 2:
        print("error: need <env> <patch|minor|major>  or  <env> --set X.Y.Z",
              file=sys.stderr)
        return 2

    env_name, op = argv[0], argv[1]
    if env_name not in envs:
        print(f"error: env '{env_name}' not in firmware-versions.json. "
              f"Known: {', '.join(envs)}", file=sys.stderr)
        return 2

    cur = envs[env_name]
    if op == "--set":
        if len(argv) < 3 or not SEMVER_RE.match(argv[2]):
            print("error: --set needs a strict semver X.Y.Z", file=sys.stderr)
            return 2
        new = argv[2]
    elif op in ("patch", "minor", "major"):
        new = _bump(cur, op)
    else:
        print(f"error: unknown op '{op}' (patch|minor|major|--set)",
              file=sys.stderr)
        return 2

    if new == cur:
        print(f"no change: {env_name} already {cur}")
        return 0

    envs[env_name] = new
    _save(data)
    tag = f"{repo_short}/{env_name}/v{new}"
    print(f"{env_name}: {cur} -> {new}")
    print("firmware-versions.json updated. To release (after hardware verify):")
    print(f"  git add firmware-versions.json && git commit -m 'release: {tag}'")
    print(f"  git tag {tag} && git push origin {tag}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
