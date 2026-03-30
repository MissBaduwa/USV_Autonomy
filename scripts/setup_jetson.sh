#!/bin/bash
# COMPLETE JETSON SETUP SCRIPT

echo "🚀 COMPLETE JETSON NANO SETUP FOR USV"
echo "====================================="

# Update system
echo "📦 Updating system..."
sudo apt-get update
sudo apt-get upgrade -y

# Install ROS 2 Humble
echo "🤖 Installing ROS 2 Humble..."
sudo apt-get install -y software-properties-common
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update
sudo apt-get install -y ros-humble-ros-base ros-humble-cv-bridge ros-humble-usb-cam

# Install GPS driver
echo "📍 Installing GPS driver..."
sudo apt-get install -y ros-humble-nmea-navsat-driver

# Install Python packages
echo "🐍 Installing Python packages..."
pip3 install --upgrade pip
pip3 install numpy opencv-python-headless
pip3 install transforms3d scipy
pip3 install onnxruntime-gpu
pip3 install geographiclib mpu6050-raspberrypi

# Create workspace
echo "📁 Creating ROS 2 workspace..."
mkdir -p ~/usv_ws/src
cp -r ~/jetson_usv_final/src/* ~/usv_ws/src/
cp -r ~/jetson_usv_final/msg ~/usv_ws/src/
cp ~/jetson_usv_final/package.xml ~/usv_ws/src/
cp ~/jetson_usv_final/CMakeLists.txt ~/usv_ws/src/

# Build workspace
echo "🔨 Building workspace..."
cd ~/usv_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select usv_autonomy
source install/setup.bash

# Setup permissions
echo "🔧 Setting up permissions..."
sudo usermod -a -G dialout $USER
sudo usermod -a -G video $USER

# Create model directory
mkdir -p ~/usv_ws/models
cp ~/jetson_usv_final/models/water_surface_detection.onnx ~/usv_ws/models/

echo ""
echo "✅ SETUP COMPLETE!"
echo ""
echo "NEXT STEPS:"
echo "1. Reboot: sudo reboot"
echo "2. Configure GPS: bash ~/jetson_usv_final/scripts/configure_gps.sh"
echo "3. Test accelerometer: bash ~/jetson_usv_final/scripts/setup_accel.sh"
echo "4. Launch USV: ros2 launch usv_autonomy usv_jetson.launch.py"
