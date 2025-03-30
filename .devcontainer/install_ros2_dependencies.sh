#!/bin/bash

# install turtlebot3 deps
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src/
git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/turtlebot3_ws
colcon build --symlink-install --parallel-workers 2
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
echo 'export LDS_MODEL=LDS-01' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc

# install champ deps
mkdir -p ~/champ_ws/src
cd ~/champ_ws/src
git clone --recursive https://github.com/chvmp/champ -b ros2
git clone https://github.com/chvmp/champ_teleop -b ros2
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
echo 'source ~/champ_ws/install/setup.bash' >> ~/.bashrc

# install audio_common
# mkdir ~/ros2_ws/src -p
# cd ~/ros2_ws/src
# git clone https://github.com/mgonzs13/audio_common
# cd ~/ros2_ws
# rosdep install --from-paths src --ignore-src -r -y
# colcon build --symlink-install
# source ~/ros2_ws/install/setup.sh

# install turlebot4 sim
# sudo apt install ros-galactic-turtlebot4-simulator ros-galactic-irobot-create-nodes -y
# sudo apt install ros-dev-tools -y
# sudo apt-get update && sudo apt-get install wget
# sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
# wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
# sudo apt-get update && sudo apt-get install ignition-edifice -y