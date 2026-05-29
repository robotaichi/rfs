# ROS2 Installation Guide

This guide describes how to install and setup ROS2 Jazzy Jalisco on Ubuntu 24.04.

## Prerequisites for Installation
Install the required packages and add the universe repository:
```bash
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe
```

Next, retrieve the GPG key for the ROS2 repository and add it to your sources list:
```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

## Installing ROS2
Install ROS2 Desktop packages:
```bash
sudo apt update
sudo apt install ros-jazzy-desktop
```

## Installing ROS2 Development Tools
In ROS2, `colcon` is used as the build tool. Install these development tools:
```bash
sudo apt update && sudo apt install ros-dev-tools
```

## Environment Setup
Source the setup script and add it to your `.bashrc` so that it runs automatically in new terminals:
```bash
cd
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Initializing ROS2
Initialize and update `rosdep` to manage dependencies:
```bash
sudo rosdep init
rosdep update
```

## Creating a ROS2 Workspace
Create a workspace directory for development. In this guide, we use a directory named `colcon_ws` (you can choose any other name such as `ros2_ws`):
```bash
cd
mkdir -p colcon_ws/src
cd colcon_ws
colcon build
```
After this, you can create or download packages inside the `src` directory to proceed with development.

## Detailed Configurations
To ensure that built packages in your workspace are automatically referenced when opening new terminals, add the workspace setup script to your `.bashrc`:
```bash
echo "source \$HOME/colcon_ws/install/setup.bash" >> ~/.bashrc
```
This allows referencing the built packages inside the workspace immediately upon opening a terminal.
