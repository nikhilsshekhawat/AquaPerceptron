
#!/bin/bash

# ROV Vision System Setup Script
echo "Setting up ROV Vision System..."

# Update system packages
sudo apt-get update

# Install Python dependencies
echo "Installing Python dependencies..."
pip install -r requirements.txt

# Install Intel RealSense SDK
echo "Installing Intel RealSense SDK..."
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg

# Install Arduino IDE for servo controller programming
echo "Installing Arduino IDE..."
wget -O arduino.tar.xz https://downloads.arduino.cc/arduino-1.8.19-linux64.tar.xz
tar -xf arduino.tar.xz
sudo mv arduino-1.8.19 /opt/arduino
sudo /opt/arduino/install.sh

# Setup permissions for serial communication
sudo usermod -a -G dialout $USER

echo "Setup complete! Please reboot your system."
echo "After reboot, run: python rov_vision_system.py"
