#!/usr/bin/env bash
# sim_start.sh - 启动四旋翼仿真（Gazebo GUI + rviz2）
#
# 用法:
#   ./sim_start.sh                              # 默认带 GUI 和 rviz2
#   ./sim_start.sh gui:=false rviz:=false       # 无头模式
#
# 说明:
#   - 前台运行；按 Ctrl+C 停止，launch 会连带关闭所有子进程（推荐停止方式）
#   - 启动前自动检查残留实例，避免多实例导致的 tf2 "jump back in time" 警告
set -e
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")/../.."
cd ros2_ws
source install/setup.bash

STALE=$(ps -eo cmd | grep -E "quadcopter_(sim|control)\.launch|gz sim -s -r.*quadcopter_world|quadcopter_pose_tf|attitude_control_node" \
  | grep -v grep | wc -l)
if [ "$STALE" -gt 0 ]; then
  echo "[sim_start] 检测到已有仿真进程（$STALE 个），继续启动会造成多实例冲突。"
  echo "[sim_start] 请先清理：./sim_stop.sh"
  exit 1
fi

echo "[sim_start] ros2 launch quadcopter_description quadcopter_sim.launch.py $*"
echo "[sim_start] 按 Ctrl+C 停止（会连带关闭所有子进程）"
exec ros2 launch quadcopter_description quadcopter_sim.launch.py "$@"
