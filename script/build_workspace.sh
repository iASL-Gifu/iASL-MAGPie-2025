#!/bin/bash

# ROS2ワークスペースのルートへ移動
cd /home/jetson-orin/MAGPie

# packageをbuild
colcon build --symlink-install

# source
source install/setup.bash
