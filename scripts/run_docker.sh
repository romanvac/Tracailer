#!/usr/bin/env bash
# Run the Tracailer ROS 2 Jazzy dev container with X11 forwarding (Linux host).
#
# Usage:
#   ./scripts/run_docker.sh               # interactive shell
#   ./scripts/run_docker.sh <command...>  # run a specific command inside
#
# Environment overrides:
#   TRACAILER_IMAGE      image tag (default: tracailer:jazzy)
#   TRACAILER_CONTAINER  container name (default: tracailer-dev)

set -euo pipefail

IMAGE_NAME="${TRACAILER_IMAGE:-tracailer:jazzy}"
CONTAINER_NAME="${TRACAILER_CONTAINER:-tracailer-dev}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# ---------------------------------------------------------------------------
# X11 forwarding setup (Linux host)
# ---------------------------------------------------------------------------
# Allow the local Docker user to connect to the host X server. `xhost +local:`
# (or +local:docker) is the common idiom and avoids sharing cookies. If xhost
# is not installed we fall back to an XAUTHORITY file.
XSOCK="/tmp/.X11-unix"
XAUTH="/tmp/.docker.xauth"

if command -v xhost >/dev/null 2>&1; then
    xhost +local:docker >/dev/null 2>&1 || xhost +local: >/dev/null 2>&1 || true
fi

# Build a fresh XAUTHORITY cookie usable from inside the container.
if command -v xauth >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ]; then
    touch "${XAUTH}"
    xauth nlist "${DISPLAY}" 2>/dev/null \
        | sed -e 's/^..../ffff/' \
        | xauth -f "${XAUTH}" nmerge - 2>/dev/null || true
    chmod 644 "${XAUTH}" || true
fi

# GPU passthrough (Intel / AMD via /dev/dri). NVIDIA users can set
# TRACAILER_GPU_ARGS="--gpus all" to enable the NVIDIA runtime.
GPU_ARGS=(--device=/dev/dri:/dev/dri)
if [ -n "${TRACAILER_GPU_ARGS:-}" ]; then
    # shellcheck disable=SC2206
    GPU_ARGS=(${TRACAILER_GPU_ARGS})
fi

# Remove any stale container with the same name.
if docker ps -a --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
    docker rm -f "${CONTAINER_NAME}" >/dev/null
fi

echo "[run_docker] Image:      ${IMAGE_NAME}"
echo "[run_docker] Container:  ${CONTAINER_NAME}"
echo "[run_docker] DISPLAY:    ${DISPLAY:-<unset>}"
echo "[run_docker] Workspace:  ${REPO_ROOT} -> /workspaces/tracailer"

exec docker run -it --rm \
    --name "${CONTAINER_NAME}" \
    --network host \
    --ipc host \
    --privileged \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY="${XAUTH}" \
    -e ROS_DISTRO=jazzy \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -v "${XSOCK}:${XSOCK}:rw" \
    -v "${XAUTH}:${XAUTH}:rw" \
    -v "${REPO_ROOT}:/workspaces/tracailer" \
    "${GPU_ARGS[@]}" \
    -w /workspaces/tracailer \
    "${IMAGE_NAME}" \
    "$@"
