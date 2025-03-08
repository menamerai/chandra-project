#!/bin/bash

mkdir ~/ros2_ws/src -p
cd ~/ros2_ws/src
git clone https://github.com/mgonzs13/audio_common
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source ~/ros2_ws/install/setup.sh
echo "source ~/ros2_ws/install/setup.sh" >> ~/.bashrc