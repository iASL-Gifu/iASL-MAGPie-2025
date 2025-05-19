#!/bin/bash

# エラーハンドリング：Ctrl+Cで安全に全プロセスを止める
trap 'echo "Stopping all..."; kill $NODE1_PID $NODE2_PID $TF1_PID $TF2_PID $TF3_PID $TF4_PID; exit 0' SIGINT

# Cartographerの設定
SLAM_CONFIG_PATH="/home/jetson-orin/MAGPie/src/iASL-MAGPie-2025/stack_master/config/NUC2/slam"
CONFIG_BASENAME="f110_2d.lua"
USE_SIM_TIME="true"

# ROS 2 ワークスペースの環境設定
cd /home/jetson-orin/MAGPie
source install/setup.bash

# --- Static TF publishers ---

# base_link → laser
ros2 run tf2_ros static_transform_publisher \
  0.5 0.0 0.12 0.0 0.0 0.0 1.0 base_link laser &
TF1_PID=$!

# base_link → camera
ros2 run tf2_ros static_transform_publisher \
  0.35 0.0 0.17 0.0 0.0 0.0 1.0 base_link camera &
TF2_PID=$!

# map → odom
ros2 run tf2_ros static_transform_publisher \
  0.0 0.0 0.0 0.0 0.0 0.0 1.0 map odom &
TF3_PID=$!

# base_link → imu
ros2 run tf2_ros static_transform_publisher \
  0.07 0.0 0.12 0.0 0.0 0.0 1.0 base_link imu &
TF4_PID=$!

# --- Cartographerノード ---

ros2 run cartographer_ros cartographer_node \
  -configuration_directory "$SLAM_CONFIG_PATH" \
  -configuration_basename "$CONFIG_BASENAME" \
  --minloglevel 2 \
  --ros-args \
  -r __node:=cartographer_node \
  -r odom:=early_fusion/odom \
  -r imu:=sensors/imu/raw \
  -p use_sim_time:=$USE_SIM_TIME &
NODE1_PID=$!

ros2 run cartographer_ros cartographer_occupancy_grid_node \
  -resolution 0.05 \
  --ros-args \
  -r __node:=cartographer_occupancy_grid_node \
  -p use_sim_time:=$USE_SIM_TIME &
NODE2_PID=$!

# --- 全ノードが終了するまで待機 ---
wait $NODE1_PID $NODE2_PID $TF1_PID $TF2_PID $TF3_PID $TF4_PID
