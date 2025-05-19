#!/bin/bash

# デフォルトのマップ名
DEFAULT_MAP_NAME="gifu_univ_7th"

# 引数が指定されていればそれを使用、なければデフォルトを使用
MAP_NAME=${1:-$DEFAULT_MAP_NAME}

# ROS2ワークスペースのルートへ移動
cd /home/jetson-orin/MAGPie

# ROS2の環境をsource
source install/setup.bash

# base_systemを起動
ros2 launch stack_master base_system_launch.xml map_name:=$MAP_NAME
