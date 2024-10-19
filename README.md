# Inter-IIT_Doc
General Docs &amp; instructions for Inter-IIT IdeaForge PS

## PX4 Environment setup with ROS2 & Gazebo
ROS2 humble should already be installed on ubuntu22.04. If not, then install it from [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
### 1. Update linux packages :
```
sudo apt update
sudo apt upgrade -y
```
### 2. Install PX4 setup :
```
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
sudo reboot
```

### 3. Install ROS2 dependencies :
```
pip install --user -U empy==3.3.4 pyros-genmsg setuptools
sudo apt install python3-colcon-common-extensions
sudo apt install ros-humble-desktop python3-argcomplete
sudo apt install ros-dev-tools
```

### 4. Install XRCE-DDS Agent :
```
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

### 5. Create a ros2 workspace with basic packages for px4 :
```
mkdir -p ~/inter-iit_ws/src/
cd ~/inter-iit_ws/src/
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
cd ..
colcon build
echo 'source ~/inter-iit_ws/install/setup.bash' >> ~/.bashrc
#if failed, re-run the build command
```

### 6. Installing Ggroundcontrol (GCS) :
```
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
cd ~
```

Now download the app-image file from [here](https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage).   
Change the permissions to run the ground control file and run it.
```
chmod +x ./QGroundControl.AppImage
./QGroundControl.AppImage
```
### 6. Testing the basic setup :

Launch gazebo environment & SITL

```
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```   

Run the XRCE-DDS Agent   
```
MicroXRCEAgent udp4 -p 8888
```

In the SITL terminal, give the following command to arm & takeoff the drone
```
commander takeoff
```

### 7. Running the PX4_Offboard control setup :
```
cd ~/inter-iit_ws/src
git clone https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example.git
cd ..
colcon build --packages-select px4_offboard
source install/setup.bash
ros2 launch px4_offboard offboard_velocity_control.launch.py
```