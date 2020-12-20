#!/bin/bash

# gtsam library for graph slam
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update
sudo apt install libgtsam-dev

# Eigen libraries to deal with the linear algebra
git clone https://gitlab.com/libeigen/eigen.git ../include
