#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO_REQUIRED=jazzy
APT_UPDATED=0

fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
info() { printf '\n==> %s\n' "$*"; }

[[ -r /etc/os-release ]] || fail 'This installer requires Ubuntu 24.04.'
# shellcheck disable=SC1091
source /etc/os-release
[[ "${ID}" == ubuntu && "${VERSION_ID}" == 24.04 ]] || \
  fail "Detected ${PRETTY_NAME}; Ubuntu 24.04 is required."

if grep -qi microsoft /proc/version; then
  info 'WSL2 detected. The Gazebo GUI requires WSLg DISPLAY or WAYLAND support.'
fi

if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != "${ROS_DISTRO_REQUIRED}" ]]; then
  fail "ROS_DISTRO=${ROS_DISTRO} is active. Start a clean shell before installing Jazzy."
fi

apt_update_once() {
  if (( APT_UPDATED == 0 )); then
    sudo apt-get update
    APT_UPDATED=1
  fi
}

if [[ ! -e /opt/ros/jazzy/setup.bash ]]; then
  info 'Configuring the official ROS 2 apt repository'
  apt_update_once
  sudo apt-get install -y locales software-properties-common curl ca-certificates
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  sudo add-apt-repository -y universe

  ROS_APT_SOURCE_URL="$(curl -fsSLI -o /dev/null -w '%{url_effective}' \
    https://github.com/ros-infrastructure/ros-apt-source/releases/latest)"
  ROS_APT_SOURCE_VERSION="${ROS_APT_SOURCE_URL##*/}"
  [[ -n "${ROS_APT_SOURCE_VERSION}" ]] || fail 'Could not resolve ros-apt-source release.'
  ROS_APT_DEB="/tmp/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb"
  curl -fL -o "${ROS_APT_DEB}" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb"
  sudo dpkg -i "${ROS_APT_DEB}"
  APT_UPDATED=0
fi

PACKAGES=(
  curl
  build-essential
  python3-matplotlib
  python3-rosdep
  python3-colcon-common-extensions
  ros-jazzy-ros-base
)

missing=()
for package in "${PACKAGES[@]}"; do
  dpkg-query -W -f='${db:Status-Abbrev}' "${package}" 2>/dev/null | grep -q '^ii ' || missing+=("${package}")
done
if (( ${#missing[@]} )); then
  info "Installing ${#missing[@]} missing dependency packages"
  apt_update_once
  sudo apt-get install -y "${missing[@]}"
else
  info 'All apt dependencies are already installed; skipping apt update.'
fi

if [[ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  info 'Initializing rosdep'
  sudo rosdep init
fi
info 'Updating rosdep indexes'
rosdep update

BASHRC="${HOME}/.bashrc"
JAZZY_SOURCE='source /opt/ros/jazzy/setup.bash'
if ! grep -Fqx "${JAZZY_SOURCE}" "${BASHRC}" 2>/dev/null; then
  if grep -Eq '^[[:space:]]*(source|\.)[[:space:]]+/opt/ros/(humble|iron|rolling|kilted|foxy)/setup\.(bash|sh)' "${BASHRC}" 2>/dev/null; then
    info 'Another ROS distribution is sourced in ~/.bashrc; not adding Jazzy automatically.'
  else
    printf '\n%s\n' "${JAZZY_SOURCE}" >> "${BASHRC}"
  fi
fi

# ROS environment hooks are not uniformly nounset-safe.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

info 'Resolving workspace dependencies'
rosdep install --from-paths "${WORKSPACE_DIR}/src" --ignore-src -r -y

info "Building ${WORKSPACE_DIR}"
cd "${WORKSPACE_DIR}"
colcon build --symlink-install

info 'Environment setup and workspace build completed successfully.'
printf 'Next: source %q && ros2 launch tabletop_sim tabletop_sim.launch.py\n' \
  "${WORKSPACE_DIR}/install/setup.bash"
