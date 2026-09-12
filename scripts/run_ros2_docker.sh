#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${ROWER_ROS_IMAGE:-rower-ros2:jazzy}"
BASE_DEV="${ROWER_BASE_DEV:-/dev/ttyAMA0}"
LIDAR_DEV="${ROWER_LIDAR_DEV:-/dev/ttyUSB0}"
DIALOUT_GID="$(getent group dialout | cut -d: -f3 || true)"

for dev in "$BASE_DEV" "$LIDAR_DEV"; do
  if [[ ! -e "$dev" ]]; then
    echo "ERROR: device not found: $dev" >&2
    exit 1
  fi
done

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "ERROR: Docker image $IMAGE not found. Run scripts/build_ros2_docker.sh first." >&2
  exit 1
fi

ARGS=(
  run --rm -it
  --network host
  --device "$BASE_DEV:/dev/ttyAMA0"
  --device "$LIDAR_DEV:/dev/rower_lidar"
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
  -v "$REPO_ROOT:/workspace/rower_gorod"
  -w /workspace/rower_gorod
  --user "$(id -u):$(id -g)"
)

if [[ -n "$DIALOUT_GID" ]]; then
  ARGS+=(--group-add "$DIALOUT_GID")
fi

ARGS+=("$IMAGE" bash)

exec docker "${ARGS[@]}"
