#!/usr/bin/env bash
# Send one joint a delta, then report the resulting pose.
#
# Why a script: the driver works in three coordinate spaces (see
# docs or config/dummy_controller.yaml). Sending a raw ros2 topic pub with a
# hand-computed angle is an easy way to command real hardware to a wrong
# position. This script reads the current pose, applies the delta in URDF
# space, and lets the driver's own inverse mapping handle the rest.
#
# Usage:
#   bash tools/test_motion.sh --list          # joints and limits
#   bash tools/test_motion.sh 1 20            # joint 1 by +20 degrees
#   bash tools/test_motion.sh 2 -15 --yes     # skip confirmation
#
# Safety: keep a hand on the power switch the first time. A delta of
# 10-30 degrees is a sensible test.
set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ "${1:-}" = "--list" ]; then
    python3 "$REPO/tools/print_joints.py"
    exit 0
fi

JOINT="${1:-}"
DELTA="${2:-}"
ASSUME_YES=0
for a in "$@"; do [ "$a" = "--yes" ] && ASSUME_YES=1; done

if [ -z "$JOINT" ] || [ -z "$DELTA" ]; then
    echo "usage: bash tools/test_motion.sh <joint 1-6> <delta degrees> [--yes]"
    echo "       bash tools/test_motion.sh --list"
    exit 1
fi

echo "=================================================================="
echo " joint : dummy2_joint$JOINT       delta : $DELTA degrees (URDF space)"
echo "=================================================================="
echo

if [ "$ASSUME_YES" -ne 1 ]; then
    read -r -p "Send it? Keep a hand on the power switch. [y/N] " ans
    case "$ans" in
        y|Y|yes|YES) ;;
        *) echo 'aborted'; exit 1 ;;
    esac
fi

python3 "$REPO/tools/send_joint_delta.py" "$JOINT" "$DELTA"
rc=$?
if [ $rc -ne 0 ]; then
    echo "send failed (exit $rc)"
    exit $rc
fi

echo
echo 'Waiting 4 s for the motion to settle, then reading the pose chain...'
sleep 4
python3 "$REPO/tools/debug_pose_chain.py" --timeout 8
