#!/bin/bash

sudo apt-get update
sudo apt-get install -y ffmpeg
sudo apt install build-essential gcc cmake ninja-build -y

# Install ROS 2 dependencies
bash ./.devcontainer/install_ros2_dependencies.sh

git config --global user.email "${GIT_USER_EMAIL}"
git config --global user.name "${GIT_USER_NAME}"