#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

SRC_ROOT="${BASE_DIR}/turtlebot3_il"
DST_ROOT="${BASE_DIR}/tb3"

if [[ ! -d "${SRC_ROOT}" || ! -d "${DST_ROOT}" ]]; then
    echo "[setup.sh] ERROR: Expected tb3 and turtlebot3_il to be siblings under ${BASE_DIR}"
    exit 1
fi

SRC_PKG="${SRC_ROOT}/dev_ws/src/turtlebot3/turtlebot3_control"
DST_PKG="${DST_ROOT}/src/turtlebot3_control"

# ---- sanity checks ----
if [[ ! -d "${SRC_ROOT}" ]]; then
    echo "[update.sh] ERROR: turtlebot3_il not found at ${SRC_ROOT}"
    exit 1
fi

if [[ ! -d "${DST_ROOT}" ]]; then
    echo "[update.sh] ERROR: tb3 repo not found at ${DST_ROOT}"
    exit 1
fi

if [[ ! -d "${SRC_PKG}" ]]; then
    echo "[update.sh] ERROR: source package not found at ${SRC_PKG}"
    exit 1
fi

echo "[update.sh] Updating turtlebot3_control"
echo "  from: ${SRC_PKG}"
echo "  to:   ${DST_PKG}"

# Remove old copy
if [[ -d "${DST_PKG}" ]]; then
    echo "[update.sh] Removing existing destination package"
    rm -rf "${DST_PKG}"
fi

# Copy source only
rsync -av \
    --exclude build \
    --exclude install \
    --exclude log \
    --exclude __pycache__ \
    --exclude "*.pyc" \
    --exclude ".git" \
    "${SRC_PKG}/" "${DST_PKG}/"

echo "[update.sh] Done."
echo "[update.sh] Remember to commit and push the tb3 repo."
