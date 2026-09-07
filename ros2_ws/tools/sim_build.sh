#!/usr/bin/env bash
# sim_build.sh - 构建并测试 quadcopter_description + quadcopter_gz_plugins
#
# 用法:
#   ./sim_build.sh        # 只构建并测试本包（快）
#   ./sim_build.sh all    # 构建整个工作空间
set -e
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")/../.."
cd ros2_ws

if [ "${1:-}" = "all" ]; then
  colcon build --symlink-install
else
  colcon build --packages-select quadcopter_gz_plugins quadcopter_description --symlink-install
fi
colcon test --packages-select quadcopter_gz_plugins quadcopter_description
colcon test-result --verbose
