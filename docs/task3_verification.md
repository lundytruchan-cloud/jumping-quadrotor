# 任务三验证记录：四旋翼 xacro 建模 + Gazebo/rviz2

日期：2026-08-06

## 1. 验收对照

| 验收项 | 结果 | 证据 |
| --- | --- | --- |
| Gazebo 中模型可正常加载起转 | 通过 | 电机转速指令 0.25×2200 rad/s 时，四桨关节速度稳定在 ±550 rad/s（`docs/data/task3_spin_log.csv`，保持段 1270 行） |
| 模型稳定不炸裂 | 通过 | 35 s 仿真内位姿恒定：x/y≈0、z=0.040999 m，无 NaN、无漂移、无异常数值；`docs/task3_spin_verification.png` |
| rviz2 可显示位姿 | 通过 | TF 树完整：`world → base_link`（z=0.041）+ `base_link → prop0..3_link`；`ros2 topic echo /tf` 可查 |
| 截图存档 docs/ | 部分完成 | Gazebo 模型五视图 + 数值图已存 docs/；rviz2 窗口截图受当前会话限制未能截取（见 §6） |
| 仿真可复现 | 通过 | 固定 world（1 ms 步长、RTF 1.0）、固定 spawn 位姿（0, 0, 0.041）、launch 一键启动 |

## 2. 模型规格（参考 Crazyflie 2.1）

| 参数 | 值 |
| --- | --- |
| 总质量 | 27 g（base_link 16 g + 4×桨电机组件 2.75 g） |
| 对角轴距 | 92 mm（电机距中心 46 mm，x/y 分量 32.53 mm） |
| 螺旋桨 | 直径 45 mm、厚 1 mm |
| 电机 | 直径 7 mm、高 6 mm |
| 机身 | 30×30×12 mm 中心盒体 + 4 腿（长 35 mm、半径 1.5 mm） |
| 关节 | 4 个连续转动关节（`propN_joint`，轴 z） |
| 电机模型 | `gz::sim::systems::MulticopterMotorModel`：maxRotVelocity=2200 rad/s、motorConstant=3e-8、momentConstant=3e-5（近似值，任务 6 与控制器联合标定） |

模型源文件：`ros2_ws/src/quadcopter_description/urdf/quadcopter.urdf.xacro`

## 3. 数值验证结果

演示节点以 0→0.25 归一化转速斜坡（5 s）→ 保持 25 s → 回零，记录 1751 行（约 50 Hz）：

- 保持段四桨速度：**549.9999999999993 rad/s**（= 0.25 × 2200），桨 0/1 为正、桨 2/3 为负（对角反转抵消反扭矩）。
- 位姿：x = 5.7e-20 m、y = -1.0e-20 m、**z = 0.0409994 m**（模型落在四腿上，与 spawn 高度 0.041 一致），姿态单位四元数，全程无 NaN。
- 无炸裂：速度/位姿全程有界且收敛，无数值发散。

数据与图：
- `docs/data/task3_spin_log.csv`（原始数据）
- `docs/task3_spin_verification.png`（指令转速 vs 实际转速、位姿时序）
- `docs/task3_model_render.png`（按 URDF 参数绘制的结构图）

## 4. 截图

Gazebo 模型五视图（由 gz-sim `ModelPhotoShoot` 系统插件无头渲染，ogre 引擎；文件位于 `docs/data/task3_photos/`）：

- `task3_gazebo_perspective.png`
- `task3_gazebo_top.png`
- `task3_gazebo_front.png`
- `task3_gazebo_side.png`
- `task3_gazebo_back.png`

> 注：按 ModelPhotoShoot 文档顺序命名（perspective/top/front/side/back）。模型无法直接查看图片，已通过像素统计确认五张图均含模型内容；构图请以实际查看为准。

## 5. 复现步骤

```bash
# WSL2 Ubuntu 24.04
source /opt/ros/jazzy/setup.bash
cd <repo>/ros2_ws
colcon build --packages-select quadcopter_description
source install/setup.bash

# 一键启动：Gazebo（server+GUI）+ 生成模型 + rviz2
ros2 launch quadcopter_description quadcopter_sim.launch.py

# 电机起转演示 + 数据记录（另一个终端）
ros2 run quadcopter_description quadcopter_demo_node \
  --ros-args -p motor_speed:=0.25 -p ramp_time:=5.0 -p hold_time:=25.0 \
  -p log_file:=<输出 CSV 路径> -p use_sim_time:=True
```

launch 内部流程：启动 gz server → 3 s 后 xacro→URDF→`gz sdf -p`→SDF → `ros_gz_sim create` 生成模型 → 桥接 `/clock`、`/joint_states`、`/quadcopter/pose` → robot_state_publisher + `world→base_link` TF → rviz2。

## 6. 已知限制

1. **rviz2 窗口截图未存档**：当前 Codex 会话无交互桌面访问权（`SESSIONNAME` 为空、`CopyFromScreen` 句柄无效），无法截取 rviz2 窗口。已提供数值证据（TF 树、关节状态）；用户可在本地运行 `ros2 launch quadcopter_description quadcopter_sim.launch.py` 查看 rviz2 画面（固定系 world，Grid/TF/RobotModel）。
2. **电机常数与惯量**：k_f/k_m、maxRotVelocity 为量级近似，悬停配平与姿态响应将在任务 6 联合控制器标定。
3. **渲染引擎**：ogre2 在本环境编译着色器失败（WSL2 GL 驱动限制），拍照与 GUI 使用 ogre 引擎；不影响物理仿真。
4. **无 ros2_control**：本任务直接用 gz-sim 系统插件（MulticopterMotorModel + JointStatePublisher + PosePublisher）实现起转与状态输出，任务 6 起再评估是否引入 ros2_control 方案。
