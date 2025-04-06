#!/bin/bash

sudo apt-get update
sudo apt-get install -y ffmpeg
sudo apt install build-essential gcc cmake wget python3-pip -y

git config --global user.email "${GIT_USER_EMAIL}"
git config --global user.name "${GIT_USER_NAME}"