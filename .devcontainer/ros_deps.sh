#!/bin/bash

# install turtlebot3 deps
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src/
git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/turtlebot3_ws
colcon build --symlink-install
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
echo 'export LDS_MODEL=LDS-01' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc

# install audio_common
# mkdir ~/ros2_ws/src -p
# cd ~/ros2_ws/src
# git clone https://github.com/mgonzs13/audio_common
# cd ~/ros2_ws
# rosdep install --from-paths src --ignore-src -r -y
# colcon build --symlink-install
# source ~/ros2_ws/install/setup.sh