#!/bin/bash

sudo apt-get update
sudo apt-get install -y ffmpeg

git config --global user.email "${GIT_USER_EMAIL}"
git config --global user.name "${GIT_USER_NAME}"