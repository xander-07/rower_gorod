#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if ! command -v docker >/dev/null 2>&1; then
  echo "ERROR: docker is not installed. Run scripts/setup_docker_bookworm.sh first." >&2
  exit 1
fi

cd "$REPO_ROOT"
docker build -t rower-ros2:jazzy -f docker/Dockerfile .

echo

echo "Built image rower-ros2:jazzy"
