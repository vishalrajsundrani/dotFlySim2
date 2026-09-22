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

# ── stale instance guard ─────────────────────────────────────────────────────
# PX4 refuses to start with "PX4 server already running for instance 0" when
# /tmp/px4-sock-0 exists, and that socket outlives a PX4 that was killed rather
# than shut down. Under walkerd every unit is a supervised child, so this should
# not happen -- but it DOES happen after a `docker exec` someone ran by hand, or
# a crash, and the resulting error names neither the file nor the fix.
#
# So: if the socket is there but no PX4 owns it, it is debris. Remove it.
if [ -S /tmp/px4-sock-0 ] && ! pgrep -r DRSW -f "[b]in/px4" >/dev/null 2>&1; then
    echo "[sim] removing a stale /tmp/px4-sock-0 left by a PX4 that is no longer running"
    rm -f /tmp/px4-sock-0
fi
if pgrep -r DRSW -f "[b]in/px4" >/dev/null 2>&1; then
    echo "[sim] REFUSING to start: a PX4 instance is already running." >&2
    echo "[sim] Stop it first -- under walker that is the sim unit (s)." >&2
    exit 75
fi

echo "[sim] drone=$DRONE world=$GZ_WORLD model=$PX4_MODEL"
echo "[sim] resource path: $RUNTIME (composed) then $HOME/gz_models"

# ── 1. Gazebo ────────────────────────────────────────────────────────────────
# ── is there a screen to draw on? ───────────────────────────────────────────
# A simulation you cannot see is the right answer for CI and the wrong answer
# for a person, so the default follows the environment rather than a constant.
#
#   SIM_HEADLESS=1   never open a window (CI, a remote shell, a benchmark)
#   SIM_HEADLESS=0   always open one, and fail loudly if it cannot
#   unset            open one when DISPLAY is set and actually answers
#
# The probe matters: DISPLAY can be set and still be unusable -- no X11 socket
# mounted, xhost refusing this container, a stale value inherited from a
# previous session. Gazebo's GUI then dies a few seconds after start with an
# Ogre error, long after the "starting" message scrolled past, and the
# simulation appears to have come up fine. Better to find out here and say so.
want_gui() {
    [ "${SIM_HEADLESS:-}" = "1" ] && return 1
    if [ -z "${DISPLAY:-}" ]; then
        [ "${SIM_HEADLESS:-}" = "0" ] && echo "[sim] SIM_HEADLESS=0 but DISPLAY is unset" >&2
        return 1
    fi
    if command -v xdpyinfo >/dev/null 2>&1; then
        if ! timeout 5 xdpyinfo >/dev/null 2>&1; then
            echo "[sim] DISPLAY=$DISPLAY is set but not answering; running headless." >&2
            echo "[sim] On the host:  xhost +local:   (walker does this for you)" >&2
            return 1
        fi
    fi
    return 0
}

if want_gui; then
    GUI=1
    echo "[sim] display $DISPLAY is available: the Gazebo window will open"
else
    GUI=0
    echo "[sim] running headless (no Gazebo window)"
fi

echo "[sim] 1/4 starting Gazebo: $WORLD_SDF"
# The SERVER always runs with -s. Whether pixels reach a window is the GUI
# client's business, and the two are separate processes in gz Harmonic.
#
# --headless-rendering is NOT the same as "no GUI": it selects an offscreen
# rendering path (EGL) for the server's own sensor rendering. With a window we
# want the normal path, because the cameras and the GUI then share one GL
# context instead of fighting for the GPU.
GZ_ARGS=(-r -s --verbose=1 "$WORLD_SDF")
[ "$GUI" = "0" ] && GZ_ARGS=(--headless-rendering "${GZ_ARGS[@]}")
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

# ── 1b. the Gazebo window ────────────────────────────────────────────────────
# Started AFTER the world is loaded, not before: a GUI client that connects to
# a server still parsing a 300-tower world shows an empty scene and sometimes
# gives up entirely.
#
# It is deliberately NOT in PIDS with a plain kill: closing the window must not
# stop the simulation, and stopping the simulation should take the window with
# it. The trap kills the whole process group, which covers it.
if [ "$GUI" = "1" ]; then
    echo "[sim]     opening the Gazebo window"
    gz sim -g > /tmp/gz_gui.log 2>&1 &
    GUI_PID=$!
    # A GUI that dies immediately is worth reporting now, while the reason is
    # still the last thing in its log, rather than leaving someone wondering
    # where the window went.
    sleep 4
    if ! kill -0 "$GUI_PID" 2>/dev/null; then
        echo "[sim] the Gazebo window failed to open. Last lines:" >&2
        tail -5 /tmp/gz_gui.log 2>/dev/null | sed "s/^/[sim]   /" >&2
        echo "[sim] the simulation is still running; this is only the window." >&2
    fi
fi

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

# ── 3b. the payload rig ──────────────────────────────────────────────────────
# WITHOUT THESE THE PAYLOAD CAMERAS FILM THE WRONG PLACE.
#
# The gimbal is a SEPARATE Gazebo model, because gz's set_pose only sticks on a
# top-level model and a stabilised gimbal therefore cannot be a link of the
# drone. Nothing moves it by itself: the relay watches the aircraft's pose and
# the stabiliser teleports the rig to follow it, 50 times a second.
#
# Skip them and the simulation looks completely healthy -- the drone flies, the
# cameras render, frames arrive on the wrapper surface -- while every payload
# image shows the patch of ground the aircraft took off from.
#
# Split into two processes on purpose: a busy gz-transport subscription and a
# stream of gz-transport service requests starve each other in one process
# (measured in V1: ~150 Hz alone, ~40% success when combined).
SIMSUPPORT="$HOME/ws/src/simsupport"
if [ -d "$SIMSUPPORT" ]; then
    echo "[sim]     starting the gimbal pose relay + stabiliser"
    python3 "$SIMSUPPORT/gimbal_pose_relay.py" &
    PIDS+=($!)
    sleep 1
    python3 "$SIMSUPPORT/gimbal_stabilizer.py" &
    PIDS+=($!)

    # The LRF's noise model. The sensor's own <noise> block is removed in the
    # model so this can shape returns the way the real rangefinder behaves.
    python3 "$SIMSUPPORT/lrf_noise_model.py" &
    PIDS+=($!)
else
    echo "[sim]     WARNING: $SIMSUPPORT is not mounted; the payload rig will" >&2
    echo "[sim]     not follow the aircraft and payload images will be wrong." >&2
fi

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
