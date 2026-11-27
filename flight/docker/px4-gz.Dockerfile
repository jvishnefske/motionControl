# syntax=docker/dockerfile:1
#
# PX4 SITL + Gazebo Harmonic, for crates/flight-launch (Px4Sitl, GazeboSim).
#
#   docker build -f docker/px4-gz.Dockerfile -t flight-px4-gz:latest docker
#   (or: nix run .#build-px4-image, just px4-image)
#
# Ubuntu 24.04: PX4's Tools/setup/ubuntu.sh supports 24.04 and 22.04 and
# installs gz-harmonic from packages.osrfoundation.org; PX4's own
# Tools/setup/Dockerfile is on 24.04 too. Harmonic is the minimum
# px4-rc.gzsim accepts (gz-sim 8).
#
# Both SIH (sihsim_*) and Gazebo (gz_*) airframes run from the one build:
# `make px4_sitl` builds px4 with gz_bridge (gz-transport is found), and
# px4_gz_plugins builds the system plugins server.config loads
# (libOpticalFlowSystem.so, libGstCameraSystem.so).
#
# Run with --network host (MAVLink UDP 14550/14540, gz-transport discovery
# on GZ_IP=127.0.0.1). The entrypoint dispatches:
#   px4 [args]   PX4 SITL instance 0 (env: PX4_SYS_AUTOSTART, PX4_SIM_MODEL, ...)
#   gz  [args]   gz with PX4's models/worlds/plugins on the gz paths
#   anything     exec'd as is (e.g. bash)
FROM ubuntu:24.04

# Latest stable PX4 release at the time of writing (2026-11-24).
ARG PX4_TAG=v1.18.0

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    TZ=UTC \
    RUNS_IN_DOCKER=true \
    GZ_IP=127.0.0.1

RUN apt-get update -y --quiet \
 && apt-get install -y --quiet --no-install-recommends \
      ca-certificates git gnupg lsb-release sudo wget tzdata \
 && rm -rf /var/lib/apt/lists/*

# Shallow clone of the tag and its submodules (GitHub serves submodule
# commits by sha). `git describe` still finds the tag for PX4's version.
RUN git clone --depth 1 --branch "${PX4_TAG}" \
      --recurse-submodules --shallow-submodules --jobs 8 \
      https://github.com/PX4/PX4-Autopilot.git /opt/PX4-Autopilot \
 && git config --global --add safe.directory '*'

WORKDIR /opt/PX4-Autopilot

# Build tools, Python requirements and Gazebo Harmonic; no NuttX toolchain.
RUN bash Tools/setup/ubuntu.sh --no-nuttx \
 && rm -rf /var/lib/apt/lists/*

RUN make px4_sitl \
 && cmake --build build/px4_sitl_default --target px4_gz_plugins \
 && test -x build/px4_sitl_default/bin/px4 \
 && test -f build/px4_sitl_default/rootfs/gz_env.sh

COPY flight-entrypoint.sh /usr/local/bin/flight-entrypoint
RUN chmod 0755 /usr/local/bin/flight-entrypoint

# Documentation only: --network host is what flight-launch uses.
EXPOSE 14550/udp 14540/udp 14580/udp 18570/udp

ENTRYPOINT ["/usr/local/bin/flight-entrypoint"]
CMD ["px4"]
