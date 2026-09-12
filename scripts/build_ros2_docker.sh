#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOCAL_CA="${ROWER_LOCAL_CA:-/usr/local/share/ca-certificates/mirea-usergate-inspect.crt}"

if ! command -v docker >/dev/null 2>&1; then
  echo "ERROR: docker is not installed. Run scripts/setup_docker_bookworm.sh first." >&2
  exit 1
fi

cd "$REPO_ROOT"

BUILD_ARGS=(
  build
  -t rower-ros2:jazzy
  -f docker/Dockerfile
)

if [[ -r "$LOCAL_CA" ]]; then
  echo "Using local TLS-inspection CA for container build: $LOCAL_CA"
  BUILD_ARGS+=(--secret "id=local_ca,src=$LOCAL_CA")
else
  echo "No local TLS-inspection CA found; using standard public CA store."
fi

BUILD_ARGS+=(.)
docker "${BUILD_ARGS[@]}"

echo

echo "Built image rower-ros2:jazzy"
