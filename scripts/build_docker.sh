#!/usr/bin/env bash
# Build the Tracailer ROS 2 Jazzy development image.
#
# Usage:
#   ./scripts/build_docker.sh [--no-cache] [--tag <name>]

set -euo pipefail

IMAGE_NAME="${TRACAILER_IMAGE:-tracailer:jazzy}"
DOCKERFILE="docker/Dockerfile"
BUILD_CONTEXT="."

EXTRA_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-cache)
            EXTRA_ARGS+=("--no-cache")
            shift
            ;;
        --tag|-t)
            IMAGE_NAME="$2"
            shift 2
            ;;
        -h|--help)
            sed -n '2,6p' "$0"
            exit 0
            ;;
        *)
            EXTRA_ARGS+=("$1")
            shift
            ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

USER_UID="$(id -u)"
USER_GID="$(id -g)"

echo "[build_docker] Building image '${IMAGE_NAME}' from ${DOCKERFILE}"
echo "[build_docker] UID=${USER_UID} GID=${USER_GID}"

docker build \
    -f "${DOCKERFILE}" \
    -t "${IMAGE_NAME}" \
    --build-arg USERNAME=ros \
    --build-arg USER_UID="${USER_UID}" \
    --build-arg USER_GID="${USER_GID}" \
    "${EXTRA_ARGS[@]}" \
    "${BUILD_CONTEXT}"

echo "[build_docker] Done. Image: ${IMAGE_NAME}"
