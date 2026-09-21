## Installation and Setup

This guide provides step-by-step instructions for setting up the complete software environment.

### Prerequisites

- Ubuntu 22.04 LTS (Jammy Jellyfish)
- Ubuntu Pro account (free for personal use) for RT kernel access

### 1. Real-Time Kernel Installation

For optimal robot control performance, install the RT kernel via Ubuntu Pro:
```bash
# Sign up for Ubuntu Pro and attach PC
sudo pro attach <your-token>

# Enable RT kernel
sudo pro enable realtime-kernel

# Reboot system
sudo reboot
```

### 2. ROS 2 Humble Installation
```bash
# Set up sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Install ROS 2 Humble
sudo apt update && sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools

# Environment setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. UR ROS 2 Driver Installation
```bash
sudo apt-get install ros-humble-ur
```

### 4. Python Dependencies Installation
```bash
# Install Python development tools
sudo apt install python3-dev python3-pip

# Install required packages from requirements.txt
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone git@github.com:AbdulShahzeb/ros-3d-printer-ur.git
pip install -r requirements.txt
```

### 5. Custom UR10e Models Installation

The Robotics Toolbox doesn't include UR e-series models by default. Install custom models:
```bash
chmod +x install_ur_models.sh
./install_ur_models.sh
```

### 6. Micro-ROS Setup

For Arduino Portenta H7:

1. Install Arduino IDE 2.x from https://www.arduino.cc/en/software
2. Add Arduino Mbed OS Portenta Boards via Board Manager
3. Install the `AccelStepper` library via Library Manager
4. Download latest `micro_ros_arduino` for Humble from [Releases](https://github.com/micro-ROS/micro_ros_arduino/releases)
5. Upload the downloaded ZIP to the IDE using `Sketch -> Include library -> Add .ZIP Library...`
6. Upload `DRV8825_microros.ino` to Portenta H7
7. Install micro-ROS agent on PC: https://github.com/micro-ROS/micro_ros_setup

### 8. Build This Package
```bash
cd ~/ros2_ws/
colcon build --symlink-install
source install/setup.bash
```

### Usage
```bash
# First time setup (only do this once)
cd ~/ros2_ws/src/ros-3d-printer-ur
cp local/ur_ros2_driver.sh local/ur10e_calibration.yaml local/microros.sh ~
cd ~
chmod +x ur_ros2_driver.sh && chmod +x microros.sh

# Terminal 1: Start UR driver
./ur_ros2_driver.sh

# Terminal 2: Start micro-ROS agent
./microros.sh

# Terminal 3: Launch control system
ros2 launch move_robot ur_master.launch.py
```
