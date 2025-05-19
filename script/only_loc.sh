#!/bin/bash

# === デフォルト設定 ===
LOC_ALGORITHM="slam"         # デフォルトはslam
USE_SIM_TIME="false"
MAP_PATH="/home/jetson-orin/MAGPie/maps"
MAP_NAME="map2025_05_18"
SLAM_CONFIG_PATH="/home/jetson-orin/MAGPie/install/cartographer_config/config"

# === 引数（1つ目）を loc_algorithm として受け取る ===
if [ $# -ge 1 ]; then
  LOC_ALGORITHM="$1"
fi

# === Ctrl+Cで全ノードを停止する ===
trap 'echo "[INFO] Stopping all..."; kill $(jobs -p); exit 0' SIGINT

# === ROS2環境を準備 ===
cd /home/jetson-orin/MAGPie
source install/setup.bash

# === ローカライゼーションの起動 ===
if [ "$LOC_ALGORITHM" = "slam" ]; then
  echo "[INFO] Starting Cartographer SLAM localization..."

  ros2 run cartographer_ros cartographer_node \
    -configuration_directory "$SLAM_CONFIG_PATH" \
    -configuration_basename "f110_2d_loc.lua" \
    -load_state_filename "$MAP_PATH/$MAP_NAME.pbstream" \
    --minloglevel 2 \
    --ros-args \
    -r odom:=early_fusion/odom \
    -p use_sim_time:=$USE_SIM_TIME \
    -r __node:=cartographer_node &

  ros2 run cartographer_ros cartographer_occupancy_grid_node \
    -resolution 0.05 \
    --ros-args \
    -p use_sim_time:=$USE_SIM_TIME \
    -r __node:=cartographer_occupancy_grid_node &

elif [ "$LOC_ALGORITHM" = "amcl" ]; then
  echo "[INFO] Starting AMCL (als_ros2) localization..."

  ros2 launch als_ros2 mcl.launch.xml \
    odom_name:=/early_fusion/odom \
    use_gl_pose_sampler:=True \
    use_mrf_failure_detector:=false \
    use_sim_time:=$USE_SIM_TIME &

  ros2 run nav2_map_server map_server \
    --ros-args \
    -p yaml_filename:="$MAP_PATH/$MAP_NAME.yaml" \
    -p use_sim_time:=$USE_SIM_TIME \
    -r __node:=map_server &

  ros2 run nav2_lifecycle_manager lifecycle_manager \
    --ros-args \
    -p autostart:=True \
    -p node_names:="[map_server]" \
    -p use_sim_time:=$USE_SIM_TIME \
    -r __node:=lifecycle_manager &

  ros2 run map_repeater map_repeater_node \
    --ros-args \
    -p repeat_rate:=1.0 \
    -r __node:=map_repeater_node &

else
  echo "[ERROR] Unknown localization algorithm: $LOC_ALGORITHM"
  echo "Usage: $0 [slam|amcl]"
  exit 1
fi

# === ノードが終了するまで待機 ===
wait
