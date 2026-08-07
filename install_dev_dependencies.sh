#!/bin/bash

set -e
set -o pipefail

sudo apt update

# Install ROS2 Humble (skip if already installed)
if [ -f /opt/ros/humble/setup.bash ]; then
    echo "ROS 2 Humble already installed at /opt/ros/humble — skipping base install."
else
    sudo apt install -y software-properties-common
    sudo add-apt-repository -y universe
    sudo apt update && sudo apt install curl -y
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
    sudo dpkg -i /tmp/ros2-apt-source.deb
    sudo apt update && sudo apt install -y ros-dev-tools
    sudo apt install -y ros-humble-desktop
    echo 'source /opt/ros/humble/setup.bash' >> $HOME/.bashrc
fi
source /opt/ros/humble/setup.bash

# install pip
sudo apt install -y python3-pip libglfw3-dev portaudio19-dev

pip install wandb glfw openai[realtime] pyaudio pydub numpy sounddevice --break-system-packages

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Install dependencies
cd $SCRIPT_DIR/ros2_ws
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init || true
rosdep update
rosdep install --from-paths src -y --ignore-src --skip-keys=libcamera

# Install additional ROS2 packages
sudo apt-get install -y ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-teleop-twist-joy ros-humble-foxglove-bridge ros-humble-xacro ros-humble-hardware-interface
sudo apt-get install -y ros-humble-vision-msgs ros-humble-camera-calibration ros-humble-image-transport-plugins ros-humble-theora-image-transport ros-humble-compressed-depth-image-transport ros-humble-compressed-image-transport
sudo apt-get install -y ros-humble-foxglove-bridge
sudo apt-get install -y ros-humble-joy-linux

# Upgrade packages near the end since it takes a long time
sudo apt upgrade -y

# Build ROS2 workspace
source /opt/ros/humble/setup.bash
bash $SCRIPT_DIR/ros2_ws/build.sh

echo "source $SCRIPT_DIR/ros2_ws/install/setup.bash" >> $HOME/.bashrc
echo 'export RCUTILS_COLORIZED_OUTPUT=1' >> $HOME/.bashrc
source $HOME/.bashrc