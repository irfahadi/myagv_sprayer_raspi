#!/usr/bin/env bash
# One-shot installer for the myAGV Raspberry Pi 4B (Ubuntu 22.04 arm64, ROS 2 Humble).
# Run as the normal user (not root):  bash setup/install_raspi.sh
set -euo pipefail

WS="$HOME/myagv_sprayer_raspi"
cd "$(dirname "$0")/.."

echo "==> apt: ROS 2 Humble base + Nav2 + slam_toolbox + tools"
sudo apt update
sudo apt install -y \
  ros-humble-ros-base ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-cv-bridge ros-humble-image-transport ros-humble-compressed-image-transport \
  ros-humble-rmw-cyclonedds-cpp ros-humble-diagnostic-msgs ros-humble-visualization-msgs \
  python3-opencv python3-yaml python3-vcstool python3-colcon-common-extensions \
  python3-rosdep git v4l-utils

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

echo "==> YDLidar SDK"
if ! ldconfig -p | grep -q ydlidar_sdk; then
  tmp=$(mktemp -d)
  git clone --depth 1 https://github.com/YDLIDAR/YDLidar-SDK.git "$tmp/YDLidar-SDK"
  cmake -S "$tmp/YDLidar-SDK" -B "$tmp/build" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$tmp/build" -j4
  sudo cmake --install "$tmp/build"
  sudo ldconfig
fi

echo "==> external drivers (myagv_ros2, ydlidar_ros2_driver)"
vcs import src < myagv_sprayer.repos || true

echo "==> udev rules (lidar -> /dev/ydlidar, camera stays /dev/video0)"
sudo tee /etc/udev/rules.d/99-myagv-sprayer.rules >/dev/null <<'EOF'
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="ydlidar"
KERNEL=="ttyACM*", MODE:="0666"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout,video "$USER"

echo "==> build"
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --skip-keys "gazebo_ros gazebo_ros_pkgs gazebo_plugins rviz2 joint_state_publisher_gui"
colcon build --symlink-install --packages-ignore myagv_sprayer_gazebo

echo "==> shell env"
grep -q "myagv_sprayer_raspi/setup/env.sh" "$HOME/.bashrc" || \
  echo "source $PWD/setup/env.sh" >> "$HOME/.bashrc"

echo
echo "Done. Re-login (group change), then:"
echo "  ros2 launch myagv_sprayer_bringup hardware.launch.py"
echo "Optional autostart:  sudo cp setup/myagv-sprayer.service /etc/systemd/system/ && sudo systemctl enable --now myagv-sprayer"
