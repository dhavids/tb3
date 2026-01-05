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
        *)
            ;;
    esac
done

log() {
    if [[ "${SILENT}" -eq 0 ]]; then
        echo "$@"
    fi
}

# ----------------------------------
# Configuration (edit only if needed)
# ----------------------------------
TB3_GIT_URL="https://github.com/dhavids/tb3.git"
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
if [[ ! -d "${BASE_DIR}" ]]; then
    echo "[setup.sh] ERROR: BASE_DIR does not exist" >&2
    exit 1
fi

if [[ ! -d "${TB3_WS}" ]]; then
    echo "[setup.sh] ERROR: TurtleBot3 workspace not found at ${TB3_WS}" >&2
    exit 1
fi

if [[ ! -d "${TB3_SRC}" ]]; then
    echo "[setup.sh] ERROR: TurtleBot3 src folder not found at ${TB3_SRC}" >&2
    exit 1
fi

# ----------------------------------
# Clone or update tb3 repo
# ----------------------------------
if [[ ! -d "${TB3_REPO}" ]]; then
    log "[setup.sh] tb3 repo not found, cloning..."
    cd "${BASE_DIR}"
    git clone "${TB3_GIT_URL}" tb3 ${SILENT:+>/dev/null}
else
    if [[ ! -d "${TB3_REPO}/.git" ]]; then
        echo "[setup.sh] ERROR: ${TB3_REPO} exists but is not a git repo" >&2
        exit 1
    fi

    log "[setup.sh] Updating tb3 repo from git"
    cd "${TB3_REPO}"

    BRANCH="$(git symbolic-ref --short HEAD 2>/dev/null || true)"
    if [[ -z "${BRANCH}" ]]; then
        echo "[setup.sh] ERROR: tb3 repo is in detached HEAD state" >&2
        exit 1
    fi

    git fetch origin ${SILENT:+>/dev/null}
    git pull --ff-only origin "${BRANCH}" ${SILENT:+>/dev/null}
fi

# ----------------------------------
# Verify package exists in public repo
# ----------------------------------
if [[ ! -d "${PUBLIC_PKG}" ]]; then
    echo "[setup.sh] ERROR: ${PKG_NAME} not found in tb3 repo after update" >&2
    exit 1
fi

# ----------------------------------
# Backup existing package (rollback safety)
# ----------------------------------
if [[ -d "${TB3_DST_PKG}" ]]; then
    log "[setup.sh] Backing up existing ${PKG_NAME}"
    rm -rf "${BACKUP_DIR}"
    mv "${TB3_DST_PKG}" "${BACKUP_DIR}"
fi

# ----------------------------------
# Deploy new package
# ----------------------------------
log "[setup.sh] Deploying ${PKG_NAME} to TurtleBot3 workspace"

rsync -av \
    --exclude build \
    --exclude install \
    --exclude log \
    --exclude __pycache__ \
    --exclude "*.pyc" \
    "${PUBLIC_PKG}/" "${TB3_DST_PKG}/" \
    ${SILENT:+>/dev/null}

# ----------------------------------
# Build package (with rollback)
# ----------------------------------
log "[setup.sh] Building ${PKG_NAME}"

export AMENT_TRACE_SETUP_FILES=""

set +u
source /opt/ros/humble/setup.bash
set -u

cd "${TB3_WS}"

set +e
colcon build --symlink-install --packages-select "${PKG_NAME}" ${SILENT:+>/dev/null}
BUILD_RC=$?
set -e

if [[ ${BUILD_RC} -ne 0 ]]; then
    echo "[setup.sh] ERROR: build failed, rolling back" >&2

    rm -rf "${TB3_DST_PKG}"

    if [[ -d "${BACKUP_DIR}" ]]; then
        mv "${BACKUP_DIR}" "${TB3_DST_PKG}"
        echo "[setup.sh] Restored previous ${PKG_NAME}" >&2
    else
        echo "[setup.sh] WARNING: no backup package to restore" >&2
    fi

    exit 1
fi

# ----------------------------------
# Cleanup backup + source workspace
# ----------------------------------
rm -rf "${BACKUP_DIR}"

export AMENT_TRACE_SETUP_FILES=""
set +u
source "${TB3_WS}/install/setup.bash"
set -u

log "[setup.sh] Done."
log "[setup.sh] Run:"
log "  ros2 run ${PKG_NAME} marl_controller"
log "  Allowed flags: --scan_lines, --print_hz, --degrees_yaw, --help"
