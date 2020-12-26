#!/bin/bash

# gtsam library for graph slam
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update
sudo apt install libgtsam-dev

# Eigen libraries to deal with the linear algebra
git clone https://gitlab.com/libeigen/eigen.git ../include

# opencv c++ install
cd ~/

# Install minimal prerequisites (Ubuntu 18.04 as reference)
sudo apt update && sudo apt install -y cmake g++ wget unzip
# Download and unpack sources
wget -O opencv.zip https://github.com/opencv/opencv/archive/master.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/master.zip
unzip opencv.zip
unzip opencv_contrib.zip

cd opencv-master/

# Create build directory and switch into it
mkdir -p build && cd build
# Configure
cmake -DOPENCV_EXTRA_MODULES_PATH=~/opencv_contrib-master/modules ~/opencv-master
# Build
make -j4 

sudo make install