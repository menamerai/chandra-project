#!/bin/bash

sudo apt update
sudo apt-get update

# configure git
if [ -z "${GIT_USER_NAME}" ]; then
  echo "GIT_USER_NAME is not set. Please set it in the devcontainer.json file."
  exit 1
fi
if [ -z "${GIT_USER_EMAIL}" ]; then
  echo "GIT_USER_EMAIL is not set. Please set it in the devcontainer.json file."
  exit 1
fi
git config --global user.email "${GIT_USER_EMAIL}"
git config --global user.name "${GIT_USER_NAME}"

# install turtlebot3 deps
# mkdir -p ~/turtlebot3_ws/src
# cd ~/turtlebot3_ws/src/
# git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
# git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
# git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
# git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
# cd ~/turtlebot3_ws
# colcon build --symlink-install --parallel-workers 2
# echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
# echo 'export LDS_MODEL=LDS-01' >> ~/.bashrc
# echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
# echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
# echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc

# Create ROS 2 workspace and clone Mini Pupper ROS repository
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
if ! [ -d "mini_pupper_ros" ]; then
  git clone https://github.com/mangdangroboticsclub/mini_pupper_ros.git -b ros2-dev mini_pupper_ros
fi
vcs import < mini_pupper_ros/.minipupper.repos --recursive

# Install dependencies and build the ROS 2 packages
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
sudo apt install -y ros-humble-teleop-twist-keyboard ros-humble-teleop-twist-joy
sudo apt install -y ros-humble-v4l2-camera ros-humble-image-transport-plugins
sudo apt install -y ros-humble-rqt*
pip3 install simple_pid
colcon build --symlink-install
echo 'export ROBOT_MODEL=mini_pupper_2' >> ~/.bashrc
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc