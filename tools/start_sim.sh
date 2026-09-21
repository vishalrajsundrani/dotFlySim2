#!/usr/bin/env bash
# =============================================================================
# start_sim.sh — bring up one composed simulation, in the foreground.
#
# walkerd runs this as the `sim` unit. It reads what to start from
# ~/gz_runtime/compose.json, which tools/compose_sim.py has just written, so
# this script has NO opinion about which drone, world or camera profile is in
# play -- that decision was made on walker's screens.
#
# FOREGROUND, ON PURPOSE. PX4 is exec'd last and holds this process open, so
# walkerd's supervision (is it alive? what is its exit code? what is it
# printing?) works on the ordinary process it already knows how to manage. The
# three helpers run in the background and are killed by the trap below, so a
# stopped `sim` unit leaves nothing behind.
#
# WHY THE ORDER IS THIS ORDER
#   1. gz sim first, and WAIT for the world -- PX4 in standalone mode expects
#      to find a running world and exits if it never appears.
#   2. the XRCE agent before PX4, so PX4's first uORB announcements are heard;
#      starting it late means /fmu topics that exist but never populate.
#   3. ros_gz_bridge before PX4 so /clock is flowing when PX4 starts, which
#      matters because everything downstream runs on simulated time.
#   4. PX4 last, in the foreground.
# =============================================================================
set -Eeuo pipefail

RUNTIME="${SIM_RUNTIME:-$HOME/gz_runtime}"
COMPOSE="$RUNTIME/compose.json"
[ -f "$COMPOSE" ] || { echo "start_sim: $COMPOSE is missing -- compose_sim.py has not run" >&2; exit 78; }

# `set -u` is off while ROS's setup.bash is sourced: it reads
# $AMENT_TRACE_SETUP_FILES with no default, which under -u is a fatal unbound
# variable naming a file nobody here wrote.
set +u
source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ws/install/setup.bash" ] && source "$HOME/ws/install/setup.bash"
source "$HOME/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh"
set -u

# Read the composition. python3 rather than a yaml/jq dependency.
read_json() { python3 -c "import json,sys;print(json.load(open('$COMPOSE')).get('$1',''))"; }
read_spawn() { python3 -c "import json;print(json.load(open('$COMPOSE'))['spawn'].get('$1',''))"; }

DRONE="$(read_json drone)"
WORLD_SDF="$(read_json world_sdf)"
GZ_WORLD="$(read_json gz_world_name)"
PX4_MODEL="$(read_json px4_sim_model)"
SERVER_CONFIG="$(read_json server_config)"
BRIDGE_YAML="$(read_json bridge_yaml)"
LAT="$(read_spawn lat)"; LON="$(read_spawn lon)"; ALT="$(read_spawn alt)"

# The composed directory goes FIRST so a name resolves to the composed copy,
# not to the pristine source beside it.
export GZ_SIM_RESOURCE_PATH="$RUNTIME:$HOME/gz_models:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SERVER_CONFIG_PATH="$SERVER_CONFIG"
export PX4_GZ_STANDALONE=1
export PX4_GZ_NO_FOLLOW=1

PIDS=()
cleanup() {
    trap - EXIT INT TERM
    for pid in ${PIDS+"${PIDS[@]}"}; do kill "$pid" 2>/dev/null || true; done
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[sim] drone=$DRONE world=$GZ_WORLD model=$PX4_MODEL"
echo "[sim] resource path: $RUNTIME (composed) then $HOME/gz_models"

# ── 1. Gazebo ────────────────────────────────────────────────────────────────
echo "[sim] 1/4 starting Gazebo: $WORLD_SDF"
GZ_ARGS=(-r -s --verbose=1 "$WORLD_SDF")
[ "${SIM_HEADLESS:-1}" = "1" ] && GZ_ARGS=(--headless-rendering "${GZ_ARGS[@]}")
gz sim "${GZ_ARGS[@]}" &
PIDS+=($!)

echo "[sim]     waiting for the world to finish loading..."
for i in $(seq 1 120); do
    if gz service -l 2>/dev/null | grep -q "/world/$GZ_WORLD/control"; then
        echo "[sim]     Gazebo ready after ${i}s"
        break
    fi
    kill -0 "${PIDS[0]}" 2>/dev/null || { echo "[sim] Gazebo exited during start-up" >&2; exit 1; }
    sleep 1
    [ "$i" = "120" ] && { echo "[sim] Gazebo never advertised /world/$GZ_WORLD/control" >&2; exit 1; }
done

# ── 2. the XRCE agent ────────────────────────────────────────────────────────
echo "[sim] 2/4 starting Micro XRCE-DDS agent on udp4:8888"
MicroXRCEAgent udp4 -p 8888 &
PIDS+=($!)
sleep 2

# ── 3. the ROS <-> Gazebo bridge ─────────────────────────────────────────────
echo "[sim] 3/4 starting ros_gz_bridge with $BRIDGE_YAML"
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$BRIDGE_YAML" &
PIDS+=($!)
sleep 2

# ── 4. PX4, in the foreground ────────────────────────────────────────────────
# The parameter pokes that used to live in launch_sim.sh are gone: everything
# they set (NAV_DLL_ACT, MIS_TAKEOFF_ALT, the performance envelope) is now in
# the drone's walker.toml and lands in the generated airframe, which PX4 reads
# at boot. One place, per drone, instead of a script that knew about one model.
echo "[sim] 4/4 starting PX4 SITL ($PX4_MODEL) at $LAT,$LON alt $ALT"
cd "$HOME/PX4-Autopilot/build/px4_sitl_default"

# THE pxh SHELL NEEDS A TTY, AND COSTS A LOT WITHOUT ONE.
#
# PX4's interactive console redraws its "pxh> " prompt continuously. On a real
# terminal that is invisible and useful -- you can type `commander status` or
# `param show` straight into the running autopilot. On a PIPE it is a disaster:
# measured here, 22 MB of prompt escape sequences in under a minute, burning
# CPU that the simulation needs.
#
# So: keep the shell when walkerd has given this unit a PTY (the normal case,
# and what makes `walker-attach sim` genuinely useful), and use daemon mode
# when there is no terminal -- CI, a redirected log, a detached start.
PX4_ARGS=(-s etc/init.d-posix/rcS)
if [ -t 0 ]; then
    echo "[sim]     tty present: the pxh console is live in this terminal"
else
    echo "[sim]     no tty: starting PX4 in daemon mode (-d), no pxh console"
    PX4_ARGS=(-d "${PX4_ARGS[@]}")
fi

exec env \
    PX4_HOME_LAT="$LAT" PX4_HOME_LON="$LON" PX4_HOME_ALT="$ALT" \
    PX4_SIM_MODEL="$PX4_MODEL" PX4_GZ_WORLD="$GZ_WORLD" \
    ./bin/px4 "${PX4_ARGS[@]}"
