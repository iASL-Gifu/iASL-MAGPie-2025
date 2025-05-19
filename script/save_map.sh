#!/bin/bash

# 引数からマップ名を取得（指定がなければ日付にする）
MAP_NAME=${1:-$(date +%m%d_%H%M)}

cd /home/jetson-orin/MAGPie
source install/setup.bash

ros2 run nav2_map_server map_saver_cli -f ~/map/${MAP_NAME}
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/jetson-orin/map/${MAP_NAME}.pbstream'}"