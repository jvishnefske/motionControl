#!/bin/sh
# Entrypoint of flight-px4-gz (docker/px4-gz.Dockerfile).
#
#   px4 [args]  PX4 SITL instance 0 from build/px4_sitl_default: daemon mode
#               (no pxh shell on stdin), working dir rootfs, startup files
#               from etc. Extra args replace the defaults.
#   gz [args]   gz with gz_env.sh sourced (PX4_GZ_MODELS/WORLDS/PLUGINS,
#               GZ_SIM_RESOURCE_PATH, GZ_SIM_SYSTEM_PLUGIN_PATH,
#               GZ_SIM_SERVER_CONFIG_PATH).
#   other       exec'd unchanged.
set -e

PX4_DIR=/opt/PX4-Autopilot
BUILD="$PX4_DIR/build/px4_sitl_default"

# gz_env.sh is a bash script but only uses POSIX syntax apart from the
# macOS branch, which is skipped on Linux. PX4 in standalone gz mode needs
# PX4_GZ_MODELS from it as well (px4-rc.gzsim does not source it then).
# shellcheck disable=SC1091
. "$BUILD/rootfs/gz_env.sh"

cmd="${1:-px4}"
[ $# -gt 0 ] && shift

case "$cmd" in
  px4)
    cd "$BUILD/rootfs"
    if [ $# -eq 0 ]; then
      exec "$BUILD/bin/px4" -d -w "$BUILD/rootfs" "$BUILD/etc"
    fi
    exec "$BUILD/bin/px4" "$@"
    ;;
  gz)
    exec gz "$@"
    ;;
  *)
    exec "$cmd" "$@"
    ;;
esac
