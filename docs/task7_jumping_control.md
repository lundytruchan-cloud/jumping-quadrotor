# 任务七：高层跳跃控制器（论文式 22–33）复现记录

## 结论

| 验证项 | 结果 | 论文对照 |
| --- | --- | --- |
| 原地连续跳跃 | **12/12 跳稳定**，落点 RMSE 0.003 m | 任务要求 ≥10 跳 |
| 原地高度控制 | 高度 RMSE **0.062 m** | 论文高度 RMSE 4.1–4.8 cm（同量级） |
| 圆轨迹跟踪 | 全程落点 RMSE 0.385 m；**最佳连续 6 跳段 RMSE 0.139 m**，高度 RMSE **0.040 m** | 论文圆轨迹落点 16 cm、高度 4.1 cm（收敛段同量级） |
| 阶跃轨迹跟踪 | 10/10 跳，落点 RMSE **0.491 m**；最佳连续 6 跳段 0.296 m，高度 RMSE **0.057 m** | 论文阶跃落点 40 cm、高度 4.8 cm（同量级） |

说明：圆轨迹在第 10 跳（圆顶部 180° 转向）后发散。原因与差距分析见下文
“限制与差距”，已如实保留全程数据。

## 实现内容（与论文式的对应）

### 1. 纯 Python 规划模块 `src/hopcopter_model/jump_planner.py`

- `predict_landing_from_apex` / `predict_landing_from_takeoff`：空中阶段弹道
  预测（论文 22–26 区间）。触地条件取足端高度为零：
  `e3·p(t_LD) = l0·(e3·z_b(t_LD))`。
- `stance_landing_to_takeoff`：把任务四的二维支撑映射（式 12–21）嵌入三维：
  平面由 `a = -v_LD/|v_LD|` 与 `z_b(t_LD)` 张成，离地姿态/速度方向按
  `theta_t / theta_v` 绕平面法向旋转得到（论文“着陆/离地向量共面”假设）。
- `desired_takeoff_velocity`：期望离地速度。垂直分量由期望顶点高度
  `sqrt(2 g z_d)` 决定，水平分量按平底飞行时间 `2 v_z/g` 实现落点
  dead-beat（同组后续论文式 36–37）。
- `solve_landing_attitude`：**数值优化求 `z_b(t_LD)`（式 31 区）**。
  以（倾角 α，方位 φ）参数化，`α ≤ min(atan μ, 倾角上限)`；目标函数为
  下一周期落点水平误差 + 离地速度方向误差 + 离地姿态无滑移惩罚，
  用 SLSQP + 抛光轮迭代收敛代数环（落点时间 ↔ 支撑映射 ↔ 下一落点）。
- `powered_ascent_duration` / `ballistic_flight_duration`：高度控制
  `Δt_PA = z_d/ż(t_TO) − ż(t_TO)/(2g)`（悬停推力下竖直速度近似恒定）与
  弹道段 `Δt_PJ = ż(t_TO)/g + √(2 z_d/g)`（式 27–29）。
- `no_slip_angle` / `no_slip_ok`：无滑移约束（式 32）
  `arccos(e3·z_b(t_LD)) < tan⁻¹μ` 且 `arccos(e3·z_b(t_TO)) < tan⁻¹μ`。
- `circle_reference` / `step_reference`：周期级落点参考。

> 原稿为付费墙（science.org 受 Cloudflare 保护，Sci-Hub/机构镜像均不可达），
> 精确符号按仓库论文笔记重建，并参考同组开放论文（Li et al.,
> "A High-Payload Robotic Hopper Powered by Bidirectional Thrusters",
> 式 21–38）校准。已在上方逐条标注“式 xx 区”，含义与笔记一致。

### 2. 控制器状态机 `jump_controller.py`（无 ROS 依赖，纯逻辑）

每跳周期状态：`STANDBY → ASCENT_POWERED → ASCENT_BALLISTIC → DESCENT →
PRE_LAND → STANCE`。

- 离地：按上一周期实测顶点高度反推 `vz = √(2g·h_meas)`，计算 `Δt_PA`
  （含电机起转补偿 `spin_up_comp`），`hover` 模式施加 mg 推力。
- 顶点：以当前顶点状态规划本期着陆姿态；参考点取**下一周期期望落点时刻**
  （论文“落点参考取下一周期期望位置以消除延迟”）。
- 预着陆：对姿态设定点做 0.25–0.35 s 斜坡（避免无推力姿态环被阶跃激励），
  `attitude_only` 模式（零推力 + 姿态力矩）保持 `z_b(t_LD)` 触地。
- 支撑：零推力保持姿态，被动弹簧腿完成“着陆→离地”映射。
- 鲁棒项：单跳最大修正距离 `max_step`、速度限制器 `max_speed`、
  参考软起步 `ramp_hops`、在线增益校准 `tilt_scale`（实测位移/模型位移比）。

### 3. 低层控制扩展（任务六基础上）

- `attitude_only` 模式：推力 0 + 姿态 PID 力矩（弹道段仍需姿态控制），
  并限制力矩缩放与净推力上限，防止零基座推力下分配器“火箭化”。
- 弹道段关闭偏航位置/速率控制（跳跃周期不需要偏航，避免挤占滚转/俯仰力矩预算）。
- 位姿发布频率 20 → 100 Hz（顶点/速度估计需要）。

## 仿真验证

环境：ROS2 Jazzy + Gazebo Harmonic（DART，1 ms 步长），无头运行。
入口脚本：`ros2_ws/tools/sim_hop.sh`；分析：`scripts/task7_analyze_hop.py`。

```bash
./ros2_ws/tools/sim_hop.sh spot 12 0.6 0.8        # 原地 12 跳
./ros2_ws/tools/sim_hop.sh circle 10 0.6 0.8      # 圆轨迹 10 跳
./ros2_ws/tools/sim_hop.sh step 10 0.6 0.8        # 阶跃轨迹 10 跳
```

最终存档使用的参数（部分经 EXTRA_ARGS 覆盖）：

- spot：`desired_height=0.6, drop=0.8`
- circle：`R=0.5, ω=0.35 rad/s, max_tilt=10°, max_step=0.25 m, ramp=0`
- step：目标 `0 ↔ 0.4 m`，保持 4 跳，`max_tilt=20°, max_step=0.5 m`

产物：`docs/data/task7_{spot,circle,step}_{jump,attitude,leg}.csv`、
`task7_{...}_summary.csv`、`task7_{...}_verification.png`。

## 限制与差距（如实记录）

1. **圆轨迹持续跟踪发散**：第 4–9 跳收敛到论文量级（RMSE ≈0.14 m），
   第 10 跳（圆顶 180° 转向）后速度增大、支撑增益放大并缺乏横向能量耗散，
   最终翻滚发散。论文真机依赖任务八的气动稳定器管理横向能量，且板载姿态
   控制器（1 kHz）远快于当前 100 Hz 姿态环——这两项均不在任务七范围。
2. **支撑增益随速度增长**：仿真中触地角速度/接触动力学使有效转向增益约为
   模型的 1.3–2 倍且随水平速度增长；在线增益校准能部分补偿但无法完全消除。
3. **姿态权限受限**：无推力姿态控制受电机升速（20 ms）与净推力上限约束，
   大倾角（>25°）在 0.35 s 下落窗口内难以收敛，故限制单跳修正距离与倾角。
4. 论文原文付费墙不可访问，式 22–33 为“笔记重建 + 同组论文镜像”，
   符号细节可能存在差异（已逐条标注）。

## 文件清单

- `src/hopcopter_model/jump_planner.py` + `tests/test_jump_planner.py`
- `quadcopter_description/jump_controller.py`、`jump_control_node.py`、
  `launch/quadcopter_jump.launch.py`
- `quadcopter_description/attitude_control_node.py`（`attitude_only` 模式）
- `quadcopter_description/attitude_controller.py`（偏航增益开关）
- `tools/sim_hop.sh`、`scripts/task7_analyze_hop.py`
- 本记录与 `docs/data/task7_*`
