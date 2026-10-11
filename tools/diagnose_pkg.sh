#!/usr/bin/env bash
# Diagnose why a ROS 2 package is not found by ros2 launch / ros2 run.
#
# Written for the specific case of dummy_vc_calibration not being found even
# though the workspace has been sourced. Read-only: builds nothing, changes
# nothing.
#
# Usage:
#   bash tools/diagnose_pkg.sh                       # checks dummy_vc_calibration
#   bash tools/diagnose_pkg.sh <package_name>

set -u

PKG="${1:-dummy_vc_calibration}"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "=================================================================="
echo " package : $PKG"
echo " repo    : $WS"
echo "=================================================================="

# ---------------------------------------------------------------------------
echo
echo "[1] Is the package source present?"
SRC="$WS/dummy_vc_ws/src/$PKG"
if [ -d "$SRC" ]; then
    echo "    source dir : $SRC"
    for f in package.xml setup.py setup.cfg; do
        if [ -f "$SRC/$f" ]; then
            echo "      OK  $f"
        else
            echo "      MISSING  $f"
        fi
    done
    if [ -f "$SRC/setup.py" ]; then
        echo "    entry_points found in setup.py:"
        grep -A6 "console_scripts" "$SRC/setup.py" | sed 's/^/      /'
    fi
else
    echo "    MISSING: $SRC"
    echo "    -> the source tree does not contain this package at all."
    echo "       Did you 'git pull' in the repo root? The package was added"
    echo "       in commit 3fd7be1."
fi

# ---------------------------------------------------------------------------
echo
echo "[2] Is a COLCON_IGNORE in the way?"
FOUND_IGNORE=0
d="$WS/dummy_vc_ws"
while [ "$d" != "/" ]; do
    if [ -f "$d/COLCON_IGNORE" ]; then
        echo "    FOUND: $d/COLCON_IGNORE"
        FOUND_IGNORE=1
    fi
    [ "$d" = "$WS" ] && break
    d="$(dirname "$d")"
done
if [ -f "$WS/dummy_vc_ws/src/$PKG/COLCON_IGNORE" ]; then
    echo "    FOUND: $WS/dummy_vc_ws/src/$PKG/COLCON_IGNORE"
    FOUND_IGNORE=1
fi
[ "$FOUND_IGNORE" -eq 0 ] && echo "    none found (good)"

# ---------------------------------------------------------------------------
echo
echo "[3] What did colcon actually install?"
INST="$WS/dummy_vc_ws/install"
if [ -d "$INST" ]; then
    echo "    install dir : $INST"
    echo "    packages present:"
    ls -1 "$INST" 2>/dev/null | sed 's/^/      /'
    if [ -d "$INST/$PKG" ]; then
        echo
        echo "    $PKG IS installed."
        echo "    share/ contents:"
        find "$INST/$PKG/share" -maxdepth 3 2>/dev/null | sed 's/^/      /'
    else
        echo
        echo "    $PKG is NOT installed -> this is why ros2 cannot find it."
    fi
else
    echo "    MISSING: $INST  (workspace has never been built)"
fi

# ---------------------------------------------------------------------------
echo
echo "[4] Is the package registered in the ament index?"
IDX="$INST/$PKG/share/ament_index/resource_index/packages/$PKG"
if [ -f "$IDX" ]; then
    echo "    OK  $IDX"
else
    echo "    MISSING: $IDX"
    echo "    -> colcon installs a marker file per package. Without it,"
    echo "       ros2 pkg / ros2 launch cannot see the package."
fi

# ---------------------------------------------------------------------------
echo
echo "[5] Environment"
echo "    AMENT_PREFIX_PATH:"
if [ -n "${AMENT_PREFIX_PATH:-}" ]; then
    echo "$AMENT_PREFIX_PATH" | tr ':' '\n' | sed 's/^/      /'
else
    echo "      (empty!)  You have NOT sourced anything in this shell."
fi
echo "    ROS_DISTRO: ${ROS_DISTRO:-<unset>}"

# ---------------------------------------------------------------------------
echo
echo "[6] Does ros2 pkg see it?"
if command -v ros2 >/dev/null 2>&1; then
    if ros2 pkg prefix "$PKG" >/dev/null 2>&1; then
        echo "    YES -> $(ros2 pkg prefix "$PKG")"
    else
        echo "    NO  (ros2 pkg prefix $PKG failed)"
    fi
else
    echo "    ros2 not on PATH"
fi

# ---------------------------------------------------------------------------
echo
echo "[7] Build log for this package"
LOGDIR="$WS/dummy_vc_ws/log/latest_build/$PKG"
if [ -d "$LOGDIR" ]; then
    echo "    $LOGDIR"
    echo "    stdout/stderr tails:"
    for f in stdout_stderr.log stderr.log; do
        if [ -f "$LOGDIR/$f" ]; then
            echo "    --- $f (last 20 lines)"
            tail -20 "$LOGDIR/$f" | sed 's/^/      /'
        fi
    done
else
    echo "    no build log -> colcon never even tried to build this package."
    echo "    available latest_build entries:"
    ls -1 "$WS/dummy_vc_ws/log/latest_build" 2>/dev/null | sed 's/^/      /' \
        || echo "      (no log/latest_build directory)"
fi

echo
echo "=================================================================="
echo " Likely fix"
echo "=================================================================="
cat <<'EOF'
 If [3] says the package is not installed and [7] says there is no build
 log, colcon never saw it. Run from the workspace root:

     cd dummy_vc_ws
     rm -rf build install log
     colcon build --symlink-install
     source install/setup.bash

 Removing build/install/log first matters: a stale install tree from before
 the package existed can otherwise mask the problem.

 A common cause of "colcon silently skips a package" is that the directory
 was copied in while the install tree still had a cached package list. A
 clean rebuild resolves it.
EOF
