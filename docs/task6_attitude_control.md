# 任务六：姿态 PID 与姿态优先推力分配（低层控制）

> 参考论文：Bai et al., *An agile monopedal hopping quadcopter with synergistic
> hybrid locomotion*, Sci. Robot. 9, eadi8912 (2024)。本文档对应任务六
> （低层姿态控制，论文控制架构第 5 点与式 34–35）。

## 1. 目标与验收

- 实现四旋翼姿态 PID（四元数姿态误差 → 期望角速度 → 速率环力矩）。
- 实现**姿态优先推力分配**：`f = argmin |T_d − T|`，约束 `fᵢ ≥ 0` 且
  `τ_p,d = τ_p`（式 34–35）。
- 支持 **hover / zero_thrust** 两种模式切换（零推力进入纯弹道）。
- 仿真验证：悬停稳定、roll/pitch/yaw 阶跃收敛、零推力段纯弹道；
  rviz2 可视化。

验收指标（`scripts/task6_analyze_attitude.py` 自动判定）：

| 指标 | 目标 | 实测 | 结论 |
| --- | --- | --- | --- |
| 悬停平均 roll/pitch | < 2° | 0.001°/0.000° | PASS |
| 悬停最大姿态误差 | < 8° | 0.005° | PASS |
| 悬停 z / xy 漂移 | < 0.30 m | 0.097 m / 0.000 m | PASS |
| roll 阶跃收敛（15°） | < 5 s | 0.85 s，稳态 0.0003° | PASS |
| pitch 阶跃收敛（−10°） | < 5 s | 0.84 s，稳态 0.0008° | PASS |
| yaw 阶跃收敛（30°） | < 5 s | 0.30 s，稳态 0.000° | PASS |
| 回正收敛（0°） | < 5 s | 0.20 s | PASS |
| 零推力最大电机转速 | < 5 rad/s | 0.0 rad/s | PASS |
| 零推力下落高度 | > 0.05 m | 0.58 m | PASS |

完整数据：`docs/data/task6_attitude_control.csv`（100 Hz 遥测）、
`docs/data/task6_attitude_summary.csv`（指标表）、
`docs/data/task6_attitude_verification.png`（曲线图）。

## 2. 控制架构与数据流

```
Gazebo IMU (500 Hz)
   │  ros_gz_bridge ──> /quadcopter/imu (sensor_msgs/Imu)
   ▼
attitude_control_node (100 Hz)
   ├─ 姿态误差 e_R = axis-angle(q_ref⁻¹ ⊗ q_cur)，最短路径
   ├─ ω_des = kp_att · e_R
   ├─ τ = kp_rate·(ω_des − ω) + ki_rate·∫(ω_des − ω)dt   （抗饱和）
   ├─ T_d = hover：mg + 最小高度保持；zero_thrust：0
   ├─ f = T_alloc/4 + M·τ，T_alloc = max(T_d, T_min(τ))
   └─ ω_motor = clamp(√(fᵢ/k_f), 0, ω_max)
        │  actuator_msgs ──> /quadcopter/motor_speed
        ▼
   ros_gz_bridge ──> gz.msgs.Actuators（Gazebo MulticopterMotorModel）
```

`quadcopter_control.launch.py` 复用 `quadcopter_sim.launch.py`（Gazebo + rviz2 +
TF），额外加入 IMU 桥、电机指令桥与控制器节点。`attitude_setpoint_node` 是
验证驱动：悬停 → roll → pitch → yaw → 回正 → 零推力，按固定时刻发布设定点。

## 3. 姿态优先推力分配（论文式 34–35）

四电机推力 `f ∈ R⁴₊`，期望体轴力矩 `τ_d` 与期望总推力 `T_d`。姿态优先：
力矩必须精确满足，总推力在可行域内尽量接近 `T_d`：

```
min |T_d − T|     s.t.   fᵢ ≥ 0,   A_t·f = τ_d
```

对标准 X 布局，力矩零空间恰好是“四电机等量加减推力”，于是有闭式解：

```
T_min(τ) = max(0, −4·minᵢ(M·τ)ᵢ)
T_alloc  = max(T_d, T_min(τ))
f        = T_alloc/4 + M·τ
```

`M` 是力矩矩阵的零和伪逆（4×3），实现见
`ros2_ws/src/quadcopter_description/quadcopter_description/thrust_allocation.py`。
物理几何（`a = 0.032527 m`，`c = 0.006 m`，即 `τ_z/fᵢ`）：

```
τx = a·(f0 + f1 − f2 − f3)
τy = a·(−f0 + f1 + f2 − f3)
τz = c·(−f0 + f1 − f2 + f3)
```

## 4. 关键实现细节

### 4.1 电机模型标定与转向修正

- `motor_constant = 3e-8 N/(rad/s)²`：悬停约 1686 rad/s（2/3 最大转速）。
- `moment_constant = 0.006 m`：`τ_z = moment_constant × fᵢ`，按 Crazyflie
  实测推力/反扭矩比标定（原 3e-5 过小，yaw 无法控制）。
- **转向修正**：原 URDF 中 0/1 同为 CCW、2/3 同为 CW，导致 roll 与 yaw
  力矩线性相关（分配矩阵奇异）。修正为标准 X4 排布 CCW/CW/CCW/CW。

### 4.2 IMU 与桥接

- URDF `base_link` 增加 500 Hz IMU（`gz-sim-imu-system`，ENU 参考系）。
- `ros_gz_bridge`：`gz.msgs.IMU → sensor_msgs/Imu`
  （`/quadcopter/imu`），`actuator_msgs/Actuators → gz.msgs.Actuators`
  （`/quadcopter/motor_speed`）。

### 4.3 悬停 / 零推力切换

- 话题 `/quadcopter/control_mode`（`std_msgs/String`）：
  `hover` / `zero_thrust`。
- hover：`T_d = mg` 加一个最小高度保持
  （`T_d = mg + kp·(z_d − z) − kd·v_z`，仅用于让悬停试验有稳定高度；
  任务七将替换为论文式 27–29 的高度控制）。
- zero_thrust：`T_d = 0, τ = 0`，四电机转速归零，机体进入纯弹道。
- 安全逻辑：IMU 丢失/超时超过 0.2 s 立即归零电机。

### 4.4 PID 参数（仿真标定）

| 参数 | 值 | 说明 |
| --- | --- | --- |
| kp_att | 15 | 姿态环：误差 → 期望角速度（1/s） |
| kp_rate | 1.2e-3 | 速率环比例（N·m·s/rad） |
| ki_rate | 1.0e-3 | 速率环积分（消除悬挂腿常值力矩） |
| integral_limit | 1e-2 | 积分抗饱和上限（N·m） |
| max_torque | 0.05 | 单轴力矩限幅（N·m） |

调参记录：首版 `kp_att=25, kp_rate=5e-4` 在 roll/pitch 小惯量轴上出现
~7 rad/s 极限环（±10°），yaw 因惯量大仍稳定；提高速率环阻尼
（`kp_rate→1.2e-3`）并降低姿态增益（`kp_att→15`）后，三轴阶跃均在
0.9 s 内收敛且无稳态误差。

## 5. 复现方法

```bash
# 构建 + 全部测试
wsl -e bash -c "./ros2_ws/tools/sim_build.sh"

# 无头验证（悬停 5s + 每轴阶跃 5s + 零推力 5s，空中生成 0.8m）
wsl -e bash -c "./ros2_ws/tools/sim_attitude.sh"

# 带 rviz2 可视化（交互式 WSLg 会话）
wsl -e bash -c "./ros2_ws/tools/sim_attitude.sh 5 5 5 ../docs/data/task6_attitude_control.csv 0.8 true"

# 或直接启动带控制器的仿真 + rviz2
wsl -e bash -c "./ros2_ws/tools/sim_start.sh"   # 基础仿真
ros2 launch quadcopter_description quadcopter_control.launch.py rviz:=true
```

分析脚本独立可跑：
`python3 scripts/task6_analyze_attitude.py --csv docs/data/task6_attitude_control.csv`
（阶段边界由设定点/模式变化自动识别，无需手工对齐时间）。

## 6. 已知限制与后续

- 悬停用最小高度保持（P-D 于 z），不是完整位置/高度控制器；任务七将实现
  论文式 22–33 的高层跳跃控制并替换它。
- 未加水平位置控制，阶跃试验中 xy 漂移很小（<0.1 mm）但理论上会缓慢漂移。
- 电机常数为仿真级近似；真机需按 Crazyflie 固件参数重新辨识。
- 本会话无 DISPLAY，rviz2 无法自动化截图；交互式 WSLg 会话中
  `sim_attitude.sh ... true` 可打开 rviz2 查看机体与 TF。
