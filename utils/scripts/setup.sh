#!/usr/bin/env bash
set -euo pipefail

# ----------------------------------
# Argument parsing
# ----------------------------------
SILENT=0

for arg in "$@"; do
    case "$arg" in
        --silent)
            SILENT=1
            ;;
    esac
done

log() {
    if [[ "${SILENT}" -eq 0 ]]; then
        echo "$@"
    fi
}

# If silent, redirect once globally (PTY-safe)
if [[ "${SILENT}" -eq 1 ]]; then
    exec >/dev/null 2>&1
fi

# ----------------------------------
# Configuration
# ----------------------------------
PKG_NAME="turtlebot3_control"

# ----------------------------------
# Resolve paths (relative, enforced)
# ----------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

TB3_REPO="${BASE_DIR}/tb3"
PUBLIC_PKG="${TB3_REPO}/src/${PKG_NAME}"

TB3_WS="${HOME}/turtlebot3_ws"
TB3_SRC="${TB3_WS}/src/turtlebot3"
TB3_DST_PKG="${TB3_SRC}/${PKG_NAME}"

BACKUP_DIR="${TB3_SRC}/.${PKG_NAME}.backup"

# ----------------------------------
# Sanity checks
# ----------------------------------
[[ -d "${TB3_REPO}" ]] || {
    echo "[setup.sh] ERROR: tb3 repo not found at ${TB3_REPO}" >&2
    exit 1
}

[[ -d "${PUBLIC_PKG}" ]] || {
    echo "[setup.sh] ERROR: ${PKG_NAME} not found in tb3 repo" >&2
    exit 1
}

[[ -d "${TB3_WS}" ]] || {
    echo "[setup.sh] ERROR: TurtleBot3 workspace not found at ${TB3_WS}" >&2
    exit 1
}

[[ -d "${TB3_SRC}" ]] || {
    echo "[setup.sh] ERROR: turtlebot3 src folder not found at ${TB3_SRC}" >&2
    exit 1
}

# ----------------------------------
# Backup existing package
# ----------------------------------
if [[ -d "${TB3_DST_PKG}" ]]; then
    log "[setup.sh] Backing up existing ${PKG_NAME}"
    rm -rf "${BACKUP_DIR}"
    mv "${TB3_DST_PKG}" "${BACKUP_DIR}"
fi

# ----------------------------------
# Deploy package
# ----------------------------------
log "[setup.sh] Deploying ${PKG_NAME}"

rsync -a \
    --exclude build \
    --exclude install \
    --exclude log \
    --exclude __pycache__ \
    --exclude "*.pyc" \
    "${PUBLIC_PKG}/" "${TB3_DST_PKG}/"

# ----------------------------------
# Build (with rollback)
# ----------------------------------
log "[setup.sh] Building ${PKG_NAME}"

export AMENT_TRACE_SETUP_FILES=""

set +u
source /opt/ros/humble/setup.bash
set -u

cd "${TB3_WS}"

set +e
colcon build --symlink-install --packages-select "${PKG_NAME}"
BUILD_RC=$?
set -e

if [[ ${BUILD_RC} -ne 0 ]]; then
    echo "[setup.sh] ERROR: build failed, rolling back" >&2
    rm -rf "${TB3_DST_PKG}"

    if [[ -d "${BACKUP_DIR}" ]]; then
        mv "${BACKUP_DIR}" "${TB3_DST_PKG}"
        echo "[setup.sh] Restored previous ${PKG_NAME}" >&2
    fi

    exit 1
fi

# ----------------------------------
# Cleanup + source workspace
# ----------------------------------
rm -rf "${BACKUP_DIR}"

export AMENT_TRACE_SETUP_FILES=""
set +u
source "${TB3_WS}/install/setup.bash"
set -u

log "[setup.sh] Done."
log "Run:"
log "  ros2 run ${PKG_NAME} marl_controller"

log ""
log "[marl_controller runtime flags]"
log "  [scan_lines] [print_hz]     Positional args (defaults: 36 2.0)"
log "  --scan_lines <int>          Number of laser scan divisions"
log "  --print_hz <float>          Telemetry print frequency"
log "  --degrees_yaw               Print yaw in degrees"
log "  --help                      Show help"
