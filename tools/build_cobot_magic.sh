#!/usr/bin/env bash
# Build only this checkout's ROS workspaces.  It never reads from or writes to
# another Cobot_Magic/cobot_magic checkout.
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "$script_dir/.." && pwd)"
camera_workspace="$project_root/camera_ws"
piper_workspace="$project_root/Piper_ros_private-ros-noetic"

usage() {
    cat <<'EOF'
Usage: bash tools/build_cobot_magic.sh

Builds the RealSense D435 packages (realsense2_camera and
realsense2_description), then the Piper workspace in this checkout.
astra_camera is intentionally not built because this robot does not use it.
EOF
}

for argument in "$@"; do
    case "$argument" in
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "ERROR: Unknown argument: $argument" >&2
            usage >&2
            exit 2
            ;;
    esac
done

for required_dir in "$camera_workspace/src" "$piper_workspace/src"; do
    if [[ ! -d "$required_dir" ]]; then
        echo "ERROR: Expected workspace source directory does not exist: $required_dir" >&2
        exit 1
    fi
done

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
    echo "ERROR: ROS Noetic is required at /opt/ros/noetic/setup.bash" >&2
    exit 1
fi

source /opt/ros/noetic/setup.bash

for required_command in catkin_make catkin_init_workspace; do
    if ! command -v "$required_command" >/dev/null 2>&1; then
        echo "ERROR: Required ROS command is unavailable: $required_command" >&2
        exit 1
    fi
done

echo "Building checkout: $project_root"
echo "Building camera workspace: $camera_workspace"
pushd "$camera_workspace" >/dev/null
# astra_camera has an additional libudev-dev dependency and is not used on this
# robot.  Whitelist only the RealSense D435 packages needed by Cobot Magic.
catkin_make --only-pkg-with-deps realsense2_camera realsense2_description
popd >/dev/null

if [[ ! -f "$piper_workspace/src/CMakeLists.txt" ]]; then
    echo "Initialising Piper catkin workspace"
    catkin_init_workspace "$piper_workspace/src"
fi

echo "Building Piper workspace: $piper_workspace"
pushd "$piper_workspace" >/dev/null
source "$camera_workspace/devel/setup.bash"
catkin_make
popd >/dev/null

for setup_file in "$camera_workspace/devel/setup.bash" "$piper_workspace/devel/setup.bash"; do
    if [[ ! -f "$setup_file" ]]; then
        echo "ERROR: Build did not produce $setup_file" >&2
        exit 1
    fi
done

echo "Build succeeded.  Source these files before launching ROS nodes:"
echo "  source /opt/ros/noetic/setup.bash"
echo "  source $camera_workspace/devel/setup.bash"
echo "  source $piper_workspace/devel/setup.bash"
