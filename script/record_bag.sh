#!/bin/bash

# ROS2ワークスペースのルートへ移動
cd /home/jetson-orin/MAGPie

# ROS2の環境をsource
source install/setup.bash

cd /home/jetson-orin/rosbag

ros2 bag record /odom_vesc /scan /sensors/imu/raw /early_fusion/odom