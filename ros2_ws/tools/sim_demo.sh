#!/usr/bin/env bash
# sim_demo.sh - 电机起转演示 + 数据记录
#
# 用法:
#   ./sim_demo.sh                        # 默认: 转速 0.25 / 斜坡 5s / 保持 25s
#   ./sim_demo.sh 0.3 3 20               # 自定义转速、斜坡、保持
#   ./sim_demo.sh 0.25 5 25 /tmp/a.csv   # 自定义日志路径
set -e
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")/../.."
cd ros2_ws
source install/setup.bash

# ROS2 参数要求浮点型（整数会被解析为 INTEGER 导致类型冲突），统一补 .0
to_float() {
  case "$1" in
    *.*) echo "$1" ;;
    *)   echo "$1.0" ;;
  esac
}

SPEED=$(to_float "${1:-0.25}")
RAMP=$(to_float "${2:-5.0}")
HOLD=$(to_float "${3:-25.0}")
LOG=${4:-../docs/data/task3_spin_log.csv}

mkdir -p ../docs/data
rm -f "$LOG"
echo "[sim_demo] motor_speed=$SPEED ramp_time=$RAMP hold_time=$HOLD"
echo "[sim_demo] 日志: $LOG"
timeout 150 ros2 run quadcopter_description quadcopter_demo_node --ros-args \
  -p motor_speed:="$SPEED" \
  -p ramp_time:="$RAMP" \
  -p hold_time:="$HOLD" \
  -p log_file:="$LOG" \
  -p use_sim_time:=True
echo "[sim_demo] 完成，日志行数: $(wc -l < "$LOG")"
