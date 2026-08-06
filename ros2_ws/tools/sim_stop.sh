#!/usr/bin/env bash
# sim_stop.sh - 清理所有四旋翼仿真相关进程（可安全重复执行）
#
# 用法: ./sim_stop.sh
echo "[sim_stop] 正在清理四旋翼仿真进程..."

kill_by_pattern() {
  local pattern="$1"
  local pids
  pids=$(ps -eo pid,cmd | grep -E "$pattern" | grep -v grep | awk '{print $1}')
  if [ -n "$pids" ]; then
    echo "[sim_stop] 终止: $pattern -> $pids"
    kill -9 $pids 2>/dev/null || true
  fi
}

kill_by_pattern "quadcopter_(sim|control|jump)\.launch"
kill_by_pattern "gz sim -s -r.*quadcopter_world"
kill_by_pattern "gz sim -g -r"
kill_by_pattern "ros_gz_bridge/parameter_bridge /world/quadcopter_world"
kill_by_pattern "ros_gz_bridge/parameter_bridge /model/quadcopter"
kill_by_pattern "robot_state_publisher --ros-args --params-file /tmp/launch_params"
kill_by_pattern "quadcopter_pose_tf"
kill_by_pattern "quadcopter_demo_node"
kill_by_pattern "attitude_control_node"
kill_by_pattern "attitude_setpoint_node"
kill_by_pattern "jump_control_node"
kill_by_pattern "leg_phase_node"
kill_by_pattern "rviz2 -d.*quadcopter"

sleep 1
LEFT=$(ps -eo cmd | grep -E "quadcopter|gz sim -s -r.*quadcopter_world" | grep -v grep | wc -l)
echo "[sim_stop] 清理完成，剩余相关进程: $LEFT"
