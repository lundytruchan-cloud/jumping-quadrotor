# 仿真常用脚本（ros2_ws/tools/）

统一通过相对路径从仓库根目录调用，避免 PowerShell -> wsl 转义与路径空格问题：

```bash
wsl -e bash -c "./ros2_ws/tools/sim_start.sh gui:=false"
```

Windows PowerShell 里也可以直接用 `tools/sim.ps1`：

```powershell
Set-ExecutionPolicy -Scope CurrentUser RemoteSigned   # 只需执行一次
.\ros2_ws\tools\sim.ps1 start
.\ros2_ws\tools\sim.ps1 status
```

如果不想改执行策略，也可以用一行命令绕过：
`powershell -ExecutionPolicy Bypass -File ros2_ws\tools\sim.ps1 status`

| 脚本 | 作用 | 常用示例 |
| --- | --- | --- |
| `sim_start.sh` | 启动仿真（Gazebo GUI + rviz2），前台运行，启动前检查残留实例 | `./sim_start.sh`；无头：`./sim_start.sh gui:=false rviz:=false` |
| `sim_stop.sh` | 清理所有四旋翼仿真进程（可重复执行） | `./sim_stop.sh` |
| `sim_status.sh` | 查看进程 / gz 实例数 / ROS 节点，多实例时给出警告 | `./sim_status.sh` |
| `sim_demo.sh` | 电机起转演示 + CSV 记录 | `./sim_demo.sh 0.25 5 25` |
| `sim_build.sh` | colcon 构建 + 测试 | `./sim_build.sh`；全工作空间：`./sim_build.sh all` |
| `sim_photos.sh` | 无头渲染模型五视图到 `docs/data/task3_photos/` | `./sim_photos.sh` |
| `sim_drop.sh` | 自由落体掉落仿真 + 阶段 CSV 记录 | `./sim_drop.sh 0.5 5` |
| `sim_drop_photos.sh` | 无头渲染掉落截图（空中/最大压缩五视图） | `./sim_drop_photos.sh` |
| `sim_attitude.sh` | 姿态控制验证：悬停 → 姿态阶跃 → 零推力弹道 + 指标/出图 | `./sim_attitude.sh 5 5 5`；加 `true` 参数可开 rviz2 |
| `sim_hop.sh` | 高层跳跃控制器验证：原地/圆/阶跃轨迹 + 指标/出图 | `./sim_hop.sh spot 12`；`./sim_hop.sh circle 10`；`./sim_hop.sh step 10` |

## 注意事项

- 不要同时启动两套仿真实例：同一 ROS 域下多实例各有独立仿真时钟，
  rviz2 会反复输出 `Detected jump back in time. Clearing TF buffer.`。
  停止用 `sim_stop.sh`，或在前台 launch 窗口按 Ctrl+C 完整退出。
- 脚本用 apply_patch 创建（LF 行尾）；如需新增脚本，保持同样方式，不要用
  Windows 记事本或 here-string 管道，否则 CRLF 会破坏 bash 脚本。
