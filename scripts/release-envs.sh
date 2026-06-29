#!/usr/bin/env bash
#
# Release one version across MULTIPLE envs at once (DEC-035) — for a shared-code
# change that ships several envs at the same number. Sets each env in
# firmware-versions.json, then prints (or runs) the commit + per-env namespaced
# tag + push sequence.
#
# Tags are namespaced per env: <repoShort>/<env>/vX.Y.Z (this repo: tx/...).
# The tag must point at the commit that carries the bumped
# firmware-versions.json, so the order is: edit versions.json -> commit -> tag.
#
# Usage:
#   scripts/release-envs.sh <X.Y.Z> <env> [<env> ...]      # dry-run (prints steps)
#   scripts/release-envs.sh <X.Y.Z> --all                  # every env in versions.json
#   scripts/release-envs.sh <X.Y.Z> <env> ... --execute     # actually commit+tag+push
#
# Default is DRY-RUN. Per the distribution policy, only pass --execute AFTER the
# build is verified on hardware.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
VERSIONS="$ROOT/firmware-versions.json"
PY="${PYTHON:-python}"

VER="${1:-}"
shift || true
if [[ ! "$VER" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
  echo "error: first arg must be a strict semver X.Y.Z" >&2
  exit 2
fi

EXECUTE=0
ENVS=()
for a in "$@"; do
  case "$a" in
    --execute) EXECUTE=1 ;;
    --all)     mapfile -t ENVS < <("$PY" -c "import json;print('\n'.join(json.load(open(r'$VERSIONS'))['envs']))") ;;
    *)         ENVS+=("$a") ;;
  esac
done
if [[ ${#ENVS[@]} -eq 0 ]]; then
  echo "error: no envs given (pass env names or --all)" >&2
  exit 2
fi

REPO_SHORT="$("$PY" -c "import json;print(json.load(open(r'$VERSIONS')).get('_repoShort','fw'))")"

for e in "${ENVS[@]}"; do
  "$PY" "$HERE/bump_version.py" "$e" --set "$VER" >/dev/null
done
echo "firmware-versions.json: set ${ENVS[*]} -> $VER"

TAGS=()
for e in "${ENVS[@]}"; do TAGS+=("$REPO_SHORT/$e/v$VER"); done

COMMIT_MSG="release: $VER (${ENVS[*]})"
if [[ $EXECUTE -eq 1 ]]; then
  git -C "$ROOT" add firmware-versions.json
  git -C "$ROOT" commit -m "$COMMIT_MSG"
  for t in "${TAGS[@]}"; do git -C "$ROOT" tag "$t"; echo "tagged $t"; done
  git -C "$ROOT" push origin "${TAGS[@]}"
  echo "pushed: ${TAGS[*]}"
else
  echo
  echo "DRY-RUN — versions.json edited but NOT committed/tagged. To release:"
  echo "  git add firmware-versions.json && git commit -m \"$COMMIT_MSG\""
  for t in "${TAGS[@]}"; do echo "  git tag $t"; done
  echo "  git push origin ${TAGS[*]}"
  echo "(or re-run with --execute, AFTER hardware verification)"
fi
