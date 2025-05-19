#!/bin/bash

MODE=$1  # "real" or "sim"

cd /home/jetson-orin/MAGPie/src/iASL-MAGPie-2025

if [ "$MODE" = "sim" ]; then
  echo "[INFO] Initializing all submodules (simulation mode)"
  git submodule update --init --recursive
else
  echo "[INFO] Initializing only real-robot submodules"
  git submodule update --init \
    base_system/f1tenth_system \
    sensors/vesc \
    planner/global_planner/global_planner/global_racetrajectory_optimization \
    localization/als_ros2
fi
