# =============================================================================
# dotFlySim2 — UAV simulation environment
# Base:  Ubuntu 24.04 LTS (Noble Numbat)
# Stack: ROS 2 Jazzy + Gazebo Harmonic + PX4 SITL
# =============================================================================
#
# HOW THIS DIFFERS FROM VERSION 1's IMAGE, IN ONE PLACE
# -----------------------------------------------------
# V1 baked the content into the image: the m4e model, the powerline world, the
# 4900_gz_m4e airframe, the GUI scripts, the bridge. That is exactly why V1's
# drone and world are not selectable — choosing a different one meant editing
# the Dockerfile and paying a rebuild.
#
# V2 inverts it. The image holds only what is EXPENSIVE AND STABLE:
#
#     the OS, ROS 2, Gazebo, the PX4 source tree and its compiled SITL binary,
#     QGroundControl, the Micro XRCE-DDS agent, the message packages
#     (px4_msgs, psdk_interfaces), and the toolchains a C++ project needs.
#
# Everything an operator can CHOOSE or EDIT is mounted at run time:
#
#     models/  worlds/  projects/  bridge/  walker/  walkerd/  tools/
#     config/  bags/
#
# So adding a world, a drone, a mission or a bag never touches this file, and
# editing the bridge or walker never costs a rebuild. The only reasons to
# rebuild are: a new apt/pip dependency, a PX4 patch, or a new PX4 version.
#
# WHAT WAS REMOVED FROM V1's IMAGE, AND WHY
# -----------------------------------------
#   COPY models/ + worlds/           mounted now, and scanned by walker
#   the 4900_gz_m4e airframe         generated at run time from a model's
#                                    walker.yaml, which is what makes the
#                                    drone selectable at all
#   gui/drone_controller.py          D1: the Tkinter panel is not ported
#   python3-tk                       nothing in V2 uses Tk
#   gui/bridge_panel.py, rviz_panels D6: replaced by walker and by a new,
#                                    much smaller RViz panel
#   camera_switcher supervisor/unit  walkerd supervises every process now
#   launch_sim.sh                    walkerd composes and launches the sim
#   ROS_DOMAIN_ID=42 + SUBNET        no Manifold, so discovery is localhost
#
# LAYER ORDER IS LOAD-BEARING (do not "tidy" this)
# ------------------------------------------------
# Docker invalidates every layer BELOW a changed one. `make px4_sitl_default`
# near the bottom costs ~10 minutes. So anything likely to change lives AFTER
# it, not before, even where that reads oddly — see FIX LAYER 6b and the
# vision-libraries layer at the very end. V1 learned this; V2 keeps it.
#
# OFFLINE / PINNED BUILDS
# -----------------------
# Drop either of these into vendor/ and the build uses them instead of
# reaching out to GitHub:
#
#     vendor/PX4-Autopilot.zip
#     vendor/QGroundControl-x86_64.AppImage
#
# IMPORTANT for PX4-Autopilot.zip: PX4's build needs its git submodules and
# `git describe` for PX4_GIT_TAG. A plain "Download ZIP" from GitHub's web UI
# has neither and CANNOT build. Produce the archive from a recursive clone,
# keeping .git:
#
#     git clone --recursive https://github.com/PX4/PX4-Autopilot.git
#     zip -qr PX4-Autopilot.zip PX4-Autopilot
#
# The RUN below verifies this and falls back to a real clone rather than
# failing ten minutes into the compile.
# =============================================================================

FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

LABEL maintainer="dotFly Engineering"
LABEL description="dotFlySim2 — simulation environment driven by the walker TUI"
LABEL stack="ROS2-Jazzy + Gazebo-Harmonic + PX4-SITL"

# =============================================================================
# STEP 1: System base packages
#
# Changes from V1: python3-tk dropped (D1 removed the only Tk consumer);
# procps and psmisc added explicitly — walkerd's supervision and probes use
# pgrep/pkill/fuser and inheriting them by accident from another package is
# how a minimal-base rebuild breaks mysteriously later.
# =============================================================================
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
        software-properties-common \
        curl \
        git \
        wget \
        sudo \
        lsb-release \
        gnupg2 \
        locales \
        tzdata \
        build-essential \
        cmake \
        python3 \
        python3-pip \
        python3-venv \
        nano \
        htop \
        unzip \
        zip \
        procps \
        psmisc \
        libfuse2t64 \
    && rm -rf /var/lib/apt/lists/*

# Locale (required for ROS 2)
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# =============================================================================
# STEP 2: ROS 2 Jazzy repository
# =============================================================================
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# =============================================================================
# STEP 3: ROS 2 Jazzy desktop + the Gazebo bridge
#
# ONE MIDDLEWARE, EVERYWHERE: Fast DDS. Every process in this stack — PX4's
# MicroXRCEAgent, ros_gz_bridge, the camera relay, the PSDK bridge, walkerd,
# RViz, rosbag2 — must agree. Mixing Fast DDS with CycloneDDS is not supported
# by ROS 2: topic type hashes and the service request/reply topic naming
# differ, so endpoints either fail to match, or match and then misbehave.
# rmw-fastrtps-cpp is installed explicitly (and cyclonedds is not) so there is
# no ambiguity about which RMW is actually present.
#
# rosbag2-storage-mcap is explicit for the same reason: V2 records mcap by
# default and "which storage plugins exist" should not depend on what the
# desktop metapackage happened to pull in this month.
# =============================================================================
RUN apt-get update && apt-get install -y \
        ros-jazzy-desktop \
        ros-jazzy-ros-gz \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-cv-bridge \
        ros-jazzy-image-transport-plugins \
        ros-jazzy-rmw-fastrtps-cpp \
        ros-jazzy-rosbag2 \
        ros-jazzy-rosbag2-storage-mcap \
        python3-rosdep \
        python3-colcon-common-extensions \
        python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# =============================================================================
# STEP 4: QGroundControl dependencies
# =============================================================================
RUN apt-get update && apt-get install -y \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-libav \
        gstreamer1.0-gl \
        libxcb-xinerama0 \
        libxkbcommon-x11-0 \
        libxcb-cursor-dev \
    && rm -rf /var/lib/apt/lists/*

# =============================================================================
# STEP 5: PX4 build dependencies (toolchain, generators, Gazebo headers)
# =============================================================================
RUN apt-get update && apt-get install -y \
        astyle \
        libxml2-utils \
        shellcheck \
        libxml2-dev \
        libxslt1-dev \
        zlib1g-dev \
        python3-jinja2 \
        python3-jsonschema \
        python3-packaging \
        python3-toml \
        python3-numpy \
        python3-empy \
        python3-ply \
        python3-setuptools \
    && rm -rf /var/lib/apt/lists/*

# =============================================================================
# STEP 6: Non-root developer user
# =============================================================================
RUN usermod  -l developer -d /home/developer -m ubuntu && \
    groupmod -n developer ubuntu && \
    echo "developer ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    usermod -aG dialout,video,render developer

USER developer
WORKDIR /home/developer

# =============================================================================
# STEP 7: Micro XRCE-DDS Agent
#
# PX4 v1.14+ speaks uXRCE-DDS internally; this agent is what turns that into
# the ROS 2 /fmu/in and /fmu/out topics the bridge converts. Without it there
# is no telemetry at all, and the symptom (a C++ project waiting forever) says
# nothing about the cause — which is why walkerd probes for it by name.
# =============================================================================
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
        --branch v2.4.3 --depth 1 /tmp/uxrce-agent && \
    cd /tmp/uxrce-agent && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    sudo make install && \
    sudo ldconfig && \
    rm -rf /tmp/uxrce-agent

# =============================================================================
# STEP 8: Workspace + the two message packages
#
# These are the ONLY ROS packages built into the image. They are interface
# definitions: they change rarely, everything else compiles against them, and
# a colcon build of them at container start would cost a minute on every run.
#
# Everything else that gets built — the C++ projects, the RViz panel — is
# mounted and built by walker on demand, because those DO change.
# =============================================================================
RUN mkdir -p /home/developer/ws/src /home/developer/ws/config

RUN bash -c 'cd /home/developer/ws/src && \
    git clone https://github.com/PX4/px4_msgs.git && \
    cd /home/developer/ws && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build --packages-select px4_msgs'

# psdk_interfaces: the DJI message/service/action package, vendored. Both the
# bridge (its registry imports these types directly) and every C++ project
# depend on it, so it is a message-only package with no PSDK core dependency.
COPY --chown=developer:developer psdk_interfaces /home/developer/ws/src/psdk_interfaces
RUN bash -c 'cd /home/developer/ws && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build --packages-select psdk_interfaces'

# =============================================================================
# STEP 9: Python runtime dependencies
#
# V1 pip-installed opencv-python, Pillow and numpy<2 with
# --break-system-packages. V2 does not:
#
#   * walker is stdlib curses only, by decision D2, and it runs on the HOST
#     anyway — nothing here serves it.
#   * walkerd is stdlib + rclpy (already present from ros-jazzy-desktop).
#   * the camera relay republishes sensor_msgs/Image without decoding it,
#     so it needs no image library at all.
#   * cv_bridge's Python half wants python3-opencv, which apt provides
#     properly packaged — so it is installed from apt rather than fighting
#     PEP 668 with --break-system-packages.
#
# The result is an image with no pip-installed packages, which is one whole
# class of "works on my machine" gone.
# =============================================================================
RUN sudo apt-get update && sudo apt-get install -y \
        python3-opencv \
    && sudo rm -rf /var/lib/apt/lists/*

# =============================================================================
# STEP 10: walker entry points
#
# walkerd and walker-attach live in the MOUNTED walkerd/ directory so they can
# be edited without a rebuild. What the image provides is two stable shims, so
# that `docker exec dotflysim2 walker-attach sim` works regardless of where the
# source happens to sit, and so a terminal command line stays short enough to
# read in a window title.
#
# The shims deliberately fail loudly if the mount is missing: a silent
# "command not found" from inside a terminal window that just opened and
# closed again is the worst possible diagnostic.
# =============================================================================
USER root
RUN mkdir -p /opt/walker/bin && \
    printf '%s\n' \
        '#!/usr/bin/env bash' \
        '# Shim: run the mounted walkerd supervisor.' \
        '# set -u is OFF around the ROS overlay on purpose: setup.bash reads' \
        '# $AMENT_TRACE_SETUP_FILES with no default, which under -u is a fatal' \
        '# unbound-variable error naming a file nobody here wrote.' \
        'set -eo pipefail' \
        'WD=/home/developer/ws/src/walkerd' \
        'if [ ! -d "$WD" ]; then' \
        '    echo "walkerd: $WD is not mounted into this container." >&2' \
        '    echo "         walker starts the container with that mount; if you" >&2' \
        '    echo "         started it by hand, add -v <repo>/walkerd:$WD" >&2' \
        '    exit 78' \
        'fi' \
        'source /opt/ros/jazzy/setup.bash' \
        '[ -f /home/developer/ws/install/setup.bash ] && source /home/developer/ws/install/setup.bash' \
        'exec python3 -B "$WD/__main__.py" "$@"' \
        > /opt/walker/bin/walkerd && \
    printf '%s\n' \
        '#!/usr/bin/env bash' \
        '# Shim: attach this terminal to a running walkerd unit.' \
        'set -eo pipefail' \
        'WD=/home/developer/ws/src/walkerd' \
        'if [ ! -d "$WD" ]; then' \
        '    echo "walker-attach: $WD is not mounted into this container." >&2' \
        '    exit 78' \
        'fi' \
        'exec python3 -B "$WD/attach.py" "$@"' \
        > /opt/walker/bin/walker-attach && \
    chmod +x /opt/walker/bin/walkerd /opt/walker/bin/walker-attach && \
    ln -sf /opt/walker/bin/walkerd      /usr/local/bin/walkerd && \
    ln -sf /opt/walker/bin/walker-attach /usr/local/bin/walker-attach
USER developer

# =============================================================================
# STEP 11: Shell startup
#
# `docker exec ... bash -c` is NON-interactive and does not read .bashrc, so
# nothing may DEPEND on what is set here — the load-bearing variables are real
# ENV further down. This exists to make `walker shell` and an attached terminal
# comfortable to work in.
#
# The V1 aliases for starting things by hand (sim, agent, bridge, gui,
# launch_sim) are gone on purpose: in V2 walkerd starts every process, and an
# alias that starts a second, unsupervised copy of the simulation is a trap.
# What remains is diagnostics.
# =============================================================================
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/developer/.bashrc && \
    echo "[ -f ~/ws/install/setup.bash ] && source ~/ws/install/setup.bash" >> /home/developer/.bashrc && \
    echo "export APPIMAGE_EXTRACT_AND_RUN=1" >> /home/developer/.bashrc && \
    echo "alias ws='cd ~/ws'" >> /home/developer/.bashrc && \
    echo "alias units='walkerd status'" >> /home/developer/.bashrc && \
    echo "alias wrapper='ros2 topic list | grep /wrapper/psdk_ros2/'" >> /home/developer/.bashrc && \
    echo "alias fmu='ros2 topic list | grep /fmu/'" >> /home/developer/.bashrc

# =============================================================================
# STEP 12: rosdep
# =============================================================================
RUN sudo rosdep init 2>/dev/null || true && \
    rosdep update

# =============================================================================
# STEP 13: Qt5 widget toolchain
#
# Kept — and in V2 it is actually used. D6 replaces V1's source-only
# dotfly_rviz_panels with a single small panel (walker_rviz_panel/) that walker
# builds on demand, and building it needs these headers present. rviz2,
# rviz_common and rviz_default_plugins already ship with ros-jazzy-desktop.
# =============================================================================
RUN sudo apt-get update && sudo apt-get install -y \
        qtbase5-dev \
    && sudo rm -rf /var/lib/apt/lists/*

# =============================================================================
# ENV: display
# =============================================================================
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1
ENV LIBGL_ALWAYS_SOFTWARE=0

# =============================================================================
# ENV: ROS 2 middleware and discovery
#
# Real Docker ENV, not just .bashrc, because `docker exec -it ... bash -c` does
# NOT source .bashrc and every unit walkerd starts goes through exactly that.
#
# THE NO-MULTICAST RULE, in the second of its two places.
# ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST is applied by ROS 2 Jazzy above the
# vendor configuration, so even a process that ignores FASTRTPS_DEFAULT_-
# PROFILES_FILE still cannot discover anything off this host. config/dds.xml
# is the first place, and is mounted rather than copied so it can be inspected
# and tuned without a rebuild.
#
# Domain 0, not V1's 42: 42 existed to share a domain with the Manifold's
# psdk_wrapper. With discovery confined to localhost the number isolates
# nothing, so the default is one less thing to get wrong.
# =============================================================================
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_DOMAIN_ID=0
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/home/developer/ws/config/dds.xml

# PX4 is always launched standalone in V2: walkerd starts Gazebo itself, with
# the composed model set and the chosen world, and only then starts PX4 against
# the already-running world. PX4 spawning its own Gazebo would race the
# composer and ignore the camera profile.
ENV PX4_GZ_STANDALONE=1
ENV PX4_GZ_NO_FOLLOW=1

# =============================================================================
# EXPOSED PORTS — documentation only in V2.
#
# V2 runs on the DEFAULT bridge network, not --network host, and every one of
# these is container-internal: QGC, PX4's MAVLink instances, the XRCE agent and
# the video stream all live in here together. Nothing needs to be published to
# the host, and not publishing it is what stops the simulation appearing on
# your LAN. The list is kept because it is the clearest statement of which
# ports mean what.
# =============================================================================
EXPOSE 8888/udp
EXPOSE 14540/udp
EXPOSE 14541/udp
EXPOSE 14550/udp
EXPOSE 18570/udp
EXPOSE 5600/udp

WORKDIR /home/developer

# #############################################################################
# #                                                                           #
# #        FINAL SECTION: PX4 + QGROUNDCONTROL ACQUISITION                    #
# #                                                                           #
# #  Everything above is independent of PX4 and QGroundControl, so it stays   #
# #  cached across rebuilds. Everything below needs them present.             #
# #                                                                           #
# #############################################################################

# Optional-file COPY. A COPY whose sources are only globs fails when nothing
# matches, so vendor/.keep — which is committed and therefore always exists —
# keeps the source set non-empty. The two archives are then picked up only if
# you actually supplied them.
COPY --chown=developer:developer ["vendor/.keep", "vendor/PX4-Autopilot.zip*", "vendor/QGroundControl-x86_64.AppImage*", "/home/developer/vendor/"]

# -----------------------------------------------------------------------------
# PX4-Autopilot: local archive if supplied, otherwise a recursive clone.
# -----------------------------------------------------------------------------
RUN set -eu; \
    cd /home/developer; \
    clone_px4() { \
        echo "### PX4: cloning from GitHub (recursive)"; \
        rm -rf /home/developer/PX4-Autopilot; \
        git clone https://github.com/PX4/PX4-Autopilot.git --recursive \
            /home/developer/PX4-Autopilot; \
    }; \
    if [ -f vendor/PX4-Autopilot.zip ]; then \
        echo "### PX4: found vendor/PX4-Autopilot.zip -- skipping GitHub clone"; \
        rm -rf .px4unzip PX4-Autopilot; \
        mkdir -p .px4unzip; \
        unzip -q vendor/PX4-Autopilot.zip -d .px4unzip; \
        root="$(dirname "$(find .px4unzip -maxdepth 3 -name Makefile -print -quit)")"; \
        if [ -z "$root" ] || [ "$root" = "." ]; then \
            echo "### PX4: no Makefile inside the archive -- not a PX4 tree"; \
            rm -rf .px4unzip; \
            clone_px4; \
        else \
            mv "$root" PX4-Autopilot; \
            rm -rf .px4unzip; \
            if [ -d PX4-Autopilot/.git ]; then \
                echo "### PX4: archive carries .git -- syncing submodules"; \
                git -C PX4-Autopilot submodule update --init --recursive; \
            elif [ -f PX4-Autopilot/src/modules/mavlink/mavlink/message_definitions/v1.0/common.xml ]; then \
                echo "### PX4: no .git, but submodule content is vendored -- proceeding"; \
            else \
                echo "### PX4: archive has neither .git nor submodule content;"; \
                echo "###      it cannot build. Falling back to a clone."; \
                clone_px4; \
            fi; \
        fi; \
    else \
        echo "### PX4: no vendor/PX4-Autopilot.zip present"; \
        clone_px4; \
    fi; \
    test -f /home/developer/PX4-Autopilot/Makefile

# -----------------------------------------------------------------------------
# QGroundControl: local AppImage if supplied, otherwise download.
# -----------------------------------------------------------------------------
RUN set -eu; \
    cd /home/developer; \
    if [ -f vendor/QGroundControl-x86_64.AppImage ]; then \
        echo "### QGC: using vendor/QGroundControl-x86_64.AppImage -- skipping download"; \
        cp vendor/QGroundControl-x86_64.AppImage QGroundControl-x86_64.AppImage; \
    else \
        echo "### QGC: downloading latest release"; \
        wget -q https://github.com/mavlink/qgroundcontrol/releases/latest/download/QGroundControl-x86_64.AppImage \
            -O QGroundControl-x86_64.AppImage; \
    fi; \
    chmod +x QGroundControl-x86_64.AppImage; \
    rm -rf /home/developer/vendor

# -----------------------------------------------------------------------------
# PX4's own dependency installer. Needs the PX4 tree to exist.
# -----------------------------------------------------------------------------
RUN cd /home/developer/PX4-Autopilot && \
    bash ./Tools/setup/ubuntu.sh --no-nuttx 2>&1 || true

# =============================================================================
# RUNTIME MODEL AND WORLD DIRECTORIES — the layer that makes drone and world
# selection possible at all.
#
# PX4 does NOT resolve a Gazebo model through GZ_SIM_RESOURCE_PATH when it
# spawns one. px4-rc.gzsim builds an <include><uri> from an ABSOLUTE path:
#
#     ${PX4_GZ_MODELS}/${PX4_SIM_MODEL#*gz_}/model.sdf
#
# V1 satisfied that by symlinking gz_models/m4e into this directory AT BUILD
# TIME — which is precisely why V1's aircraft cannot be changed without a
# rebuild.
#
# V2 creates the directories here, empty and writable, and tools/compose_sim.py
# symlinks the COMPOSED model into them at run time, immediately before PX4 is
# started. The same applies to worlds. Nothing model-specific is baked in.
#
# Keep these directories owned by developer: the composer runs as that user and
# a root-owned directory here fails at run time with a permission error that
# reads as if Gazebo, not Docker, were at fault.
# =============================================================================
RUN mkdir -p /home/developer/PX4-Autopilot/Tools/simulation/gz/models && \
    mkdir -p /home/developer/PX4-Autopilot/Tools/simulation/gz/worlds && \
    mkdir -p /home/developer/gz_runtime && \
    chown -R developer:developer \
        /home/developer/PX4-Autopilot/Tools/simulation/gz \
        /home/developer/gz_runtime

# =============================================================================
# FIX LAYER 1: PX4 v1.18+ ships a gazebo-models submodule with no
# CMakeLists.txt, and the ExternalProject_Add step fails without one. Drop in a
# dummy so the build proceeds; the Gazebo plugins that matter are compiled into
# the main PX4 binary anyway.
# =============================================================================
RUN cat > /home/developer/PX4-Autopilot/Tools/simulation/gz/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.10)
project(px4_gazebo_models NONE)
# Dummy CMakeLists.txt for the PX4-gazebo-models submodule (v1.18+).
install(DIRECTORY models/ DESTINATION share/px4_gazebo_models/models)
install(DIRECTORY worlds/ DESTINATION share/px4_gazebo_models/worlds)
EOF

# =============================================================================
# FIX LAYER 2: Disable Gazebo's camera-follow. PX4 auto-tracks the drone on
# spawn, which fights every attempt to look at anything else in the world.
# Replacing the guard with "if false" means the gz call that sets
# track_mode: FOLLOW is never made. PX4_GZ_NO_FOLLOW is also set as ENV above;
# this is the belt to that braces, because the env var only works if the guard
# is still the one being read.
# =============================================================================
RUN sed -i 's/if \[ -z "\${PX4_GZ_NO_FOLLOW}" \]/if false/' \
    /home/developer/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim

# =============================================================================
# FIX LAYER 3: Selective single-stream mode for GstCameraSystem.
#
# Stock PX4 auto-streams EVERY type="camera" sensor in the world onto
# sequential UDP ports from 5600, and resolves each sensor's topic by
# reconstructing it from the name and link rather than honouring an explicit
# <topic> override. The payload models use custom <topic>s throughout, so the
# stock plugin subscribes to topics nobody publishes and no video ever
# streams.
#
# This patch fixes topic resolution and adds a selective mode that keeps
# exactly one stream on port 5600, switched at run time through a message on
# selectTopic — which is what lets walker's Cameras screen choose the lens QGC
# sees. It also carries a multi-sink (tee) feature that is currently DISABLED;
# see the server.config note below for why.
#
# These files are COMPILED INTO libGstCameraSystem.so, so they must land here,
# BEFORE the SITL build below. Copying them any later would silently build the
# stock plugin and discard the patch.
# =============================================================================
COPY --chown=developer:developer px4-patches/gz_plugins/gstreamer/GstCameraSystem.cpp \
     /home/developer/PX4-Autopilot/src/modules/simulation/gz_plugins/gstreamer/GstCameraSystem.cpp
COPY --chown=developer:developer px4-patches/gz_plugins/gstreamer/GstCameraSystem.hpp \
     /home/developer/PX4-Autopilot/src/modules/simulation/gz_plugins/gstreamer/GstCameraSystem.hpp

# =============================================================================
# FIX LAYER 4: Pre-build the PX4 SITL binary so it ships inside the image.
# ~10 minutes, and the reason every cheap layer lives below this one.
# =============================================================================
RUN bash -c 'source /opt/ros/jazzy/setup.bash && cd /home/developer/PX4-Autopilot && make px4_sitl_default'

# =============================================================================
# FIX LAYER 5: gz_env.sh — the environment PX4's own scripts source.
#
# PX4 generates this file itself in some configurations and not in others, and
# when it is missing the failure is a model that never spawns with no
# explanation. Writing it explicitly makes it deterministic.
#
# Note GZ_SIM_RESOURCE_PATH here does NOT mention gz_runtime: walkerd puts the
# composed directory FIRST on that variable when it launches Gazebo, so the
# composed model wins over anything else with the same name. This file only
# has to supply PX4's own paths.
# =============================================================================
RUN mkdir -p /home/developer/PX4-Autopilot/build/px4_sitl_default/rootfs && \
    printf '%s\n' \
        '#!/usr/bin/env bash' \
        'export PX4_GZ_MODELS=/home/developer/PX4-Autopilot/Tools/simulation/gz/models' \
        'export PX4_GZ_WORLDS=/home/developer/PX4-Autopilot/Tools/simulation/gz/worlds' \
        'export PX4_GZ_PLUGINS=/home/developer/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_plugins' \
        'export PX4_GZ_SERVER_CONFIG=/home/developer/PX4-Autopilot/src/modules/simulation/gz_bridge/server.config' \
        'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$PX4_GZ_MODELS:$PX4_GZ_WORLDS' \
        'export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:$PX4_GZ_PLUGINS' \
        'export GZ_SIM_SERVER_CONFIG_PATH=$PX4_GZ_SERVER_CONFIG' \
        > /home/developer/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh && \
    chmod +x /home/developer/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh

# =============================================================================
# FIX LAYER 6: GstCameraSystem's server.config.
#
# Copied here rather than beside the .cpp/.hpp above because, unlike those,
# nothing compiles this — gz sim reads it fresh at its own start-up through
# GZ_SIM_SERVER_CONFIG_PATH. Putting it after the SITL build means editing it
# costs a copy, not a ten-minute recompile.
#
# multiSink is false. Enabling it ("Failed to link tee branch 1 (QGC)") throws
# an unhandled exception inside this in-process Gazebo plugin and aborts the
# whole gz sim process, not merely the video stream. Leave it false until that
# tee-branch bug is actually fixed; single-sink mode is unaffected.
#
# This file also carries the <plugin> list gz sim loads — the Sensors system
# among them, which is what renders cameras at all. If cameras ever go
# completely dark, confirm GZ_SIM_SERVER_CONFIG_PATH is set and pointing here
# before suspecting the model.
# =============================================================================
COPY --chown=developer:developer px4-patches/gz_bridge/server.config \
     /home/developer/PX4-Autopilot/src/modules/simulation/gz_bridge/server.config

# =============================================================================
# FIX LAYER 7: C++ vision and optimisation libraries.
#
# Without these the image can fly the aircraft but not perceive with it:
# ros-jazzy-cv-bridge drags in OpenCV's shared objects, but no C++ headers or
# CMake config, and no Eigen, Ceres or GeographicLib at all. Anything doing
# visual-inertial work then has to apt-get inside a running container and lose
# it on the next teardown.
#
#   libopencv-dev         feature detection, matching, image handling
#   libeigen3-dev         linear algebra; also a Ceres dependency
#   libceres-dev          the bundle-adjustment solver
#   libgeographiclib-dev  geodetic to local ENU, for the GNSS factor's anchor
#
# These are what demo_gnss_stereo_inertial (M9) needs. It sits at the very
# bottom for the layer-cache reason in the header: new apt packages must never
# invalidate FIX LAYER 4.
# =============================================================================
RUN sudo apt-get update && sudo apt-get install -y \
        libopencv-dev \
        libeigen3-dev \
        libceres-dev \
        libgeographiclib-dev \
    && sudo rm -rf /var/lib/apt/lists/*

# walkerd is PID 1's job only when walker starts the container; by default this
# is an ordinary shell so the image is usable on its own for debugging.
CMD ["/bin/bash"]
