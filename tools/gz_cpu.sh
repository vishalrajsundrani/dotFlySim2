#!/usr/bin/env bash
# =============================================================================
# gz_cpu.sh — what the Gazebo SERVER is costing, as a percentage of one core.
#
# This is the measurement behind SPIKE-1, and the one to reach for whenever a
# camera change is supposed to have made a difference. Rendering cost is the
# only honest way to tell a camera that is genuinely off from one that is
# merely not being looked at: topic rates cannot distinguish them, because a
# sensor nobody subscribes to publishes nothing either way.
#
#   gz_cpu.sh [seconds]        default 10
#
# Counts the SERVER only. The GUI client is a separate `gz sim` process and
# including it would swamp the signal with window redraws.
# =============================================================================
set -eo pipefail
WINDOW="${1:-10}"

PID=""
for p in $(pgrep -r DRSW -f '[g]z sim' 2>/dev/null); do
    # tr, because /proc/<pid>/cmdline is NUL-separated.
    if tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null | grep -q -- ' -s '; then
        PID="$p"; break
    fi
done
[ -z "$PID" ] && { echo "no gz sim server running"; exit 1; }

read_ticks() { awk '{print $14+$15}' "/proc/$1/stat"; }

A="$(read_ticks "$PID")"
sleep "$WINDOW"
B="$(read_ticks "$PID")"
HZ="$(getconf CLK_TCK)"
awk -v a="$A" -v b="$B" -v hz="$HZ" -v w="$WINDOW" \
    'BEGIN { printf "%.1f\n", (b-a)/hz/w*100 }'
