#!/bin/bash

sudo apt-get update
sudo apt-get install -y ffmpeg
sudo apt install build-essential gcc cmake ninja-build -y

# Run the OpenSSL installation script
./.devcontainer/install_openssl1.1.sh

# Install ROS 2 dependencies
# ./devcontainer/install_ros2_dependencies.sh

git config --global user.email "${GIT_USER_EMAIL}"
git config --global user.name "${GIT_USER_NAME}"