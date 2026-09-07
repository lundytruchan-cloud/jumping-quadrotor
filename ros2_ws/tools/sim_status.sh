#!/usr/bin/env bash
# sim_status.sh - 查看仿真运行状态（进程 / 实例数 / ROS 节点）
#
# 用法: ./sim_status.sh
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")/../.."
cd ros2_ws
source install/setup.bash

echo "=== 仿真进程 ==="
ps -eo pid,lstart,cmd | grep -E "quadcopter_(sim|control)\.launch|gz sim -s -r.*quadcopter_world|gz sim -g -r|ros_gz_bridge/parameter_bridge /(world/)?quadcopter|robot_state_publisher --ros-args --params-file /tmp/launch_params|quadcopter_pose_tf|quadcopter_demo_node|attitude_control_node|attitude_setpoint_node|rviz2 -d.*quadcopter" \
  | grep -v grep || echo "（无相关进程）"

SERVER_COUNT=$(ps -eo cmd | grep -E "gz sim -s -r.*quadcopter_world" | grep -v grep | wc -l)
echo ""
echo "=== gz 服务端实例数: $SERVER_COUNT ==="
if [ "$SERVER_COUNT" -gt 1 ]; then
  echo "警告: 检测到多个仿真实例！这是 tf2 \"jump back in time\" 警告的常见原因。"
  echo "请运行 ./sim_stop.sh 清理后重新启动。"
fi

echo ""
echo "=== ROS 节点 ==="
ros2 daemon stop >/dev/null 2>&1 || true
ros2 node list 2>/dev/null || echo "（无节点，仿真未运行）"
