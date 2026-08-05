# 四旋翼可跳跃飞行器复现——任务路线图

> 参考论文：Hopcopter（Sci. Robot. 9, eadi8912, 2024），笔记见 docs/hopcopter-paper-notes.md。

## 全局固定决策（发布任务时不要改动）

- 环境：Windows 11 + WSL2 Ubuntu 24.04；**ROS2 Jazzy + Gazebo Harmonic**（2026 年主流 LTS 组合）。
- 工作空间：`<仓库>/ros2_ws`（WSL 中为 `/mnt/c/Users/Larry/Documents/ChatGPT/New project/ros2_ws`；`build/ install/ log/` 已被 .gitignore 排除，只提交源码）。
- 分支：新任务一律用 `codex/<任务名>` 分支；提交信息用 `feat/fix/docs/chore/refactor/test` 前缀。
- 每个任务结束：构建/测试通过 → 提交 → 推送私有仓库 jumping-quadrotor。
- 大文件规则：原始视频/动捕大包不提交仓库；Zenodo 小包（StanceDynamics/DropTest 等）按需下载，分析脚本与结果图入仓库。

---

## 任务 0：WSL2 + Ubuntu 24.04 安装（人工完成，不发布）

**目标**：让 Windows 具备 Linux 开发环境。

**步骤**：
1. 管理员 PowerShell 运行：`wsl --install -d Ubuntu-24.04`
2. 重启电脑。
3. 首次进入 Ubuntu 设置用户名/密码。
4. `wsl --update` 确认内核最新。

**验收**：`wsl -l -v` 显示 Ubuntu-24.04 且 VERSION = 2；`uname -a` 正常输出。

---

## 任务 1：ROS2 Jazzy + Gazebo Harmonic + colcon 工作空间 + AGENTS.md 回填

**前置**：任务 0。

**步骤**：
1. WSL Ubuntu 中安装 ROS2 Jazzy 桌面版与 colcon：
   ```
   sudo apt update && sudo apt upgrade -y
   sudo apt install -y software-properties-common
   sudo add-apt-repository universe
   sudo apt install -y ros-jazzy-desktop python3-rosdep python3-colcon-common-extensions
   ```
2. 安装 Gazebo Harmonic 集成包：`sudo apt install -y ros-jazzy-ros-gz`
3. 环境写入 `~/.bashrc`：`echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc`
4. 建工作空间并验证空构建：
   ```
   mkdir -p <仓库>/ros2_ws/src && cd <仓库>/ros2_ws
   colcon build
   ```
5. 更新项目 `AGENTS.md`：
   - 技术栈行写明「ROS2 Jazzy + Gazebo Harmonic（Ubuntu 24.04 / WSL2）」。
   - 「验证命令」占位符替换为实际命令：`colcon build`、`colcon test --packages-select <pkg>`、`colcon test-result --verbose`、`ros2 launch <pkg> <launch>`。
6. 提交推送（`docs:` 前缀）。

**验收**：`ros2 --version` 输出 Jazzy；`gz sim --version` 正常；空工作空间 `colcon build` 成功；AGENTS.md 占位符已回填并推送。

**发布用提示词**：
> 任务一：在 WSL2 Ubuntu 24.04 中安装 ROS2 Jazzy 桌面版与 Gazebo Harmonic（ros-jazzy-ros-gz）及 colcon；在仓库 <仓库>/ros2_ws 建立 colcon 工作空间并验证空构建；把项目 AGENTS.md 的「验证命令」占位符回填为实际命令（colcon build / colcon test --packages-select <pkg> / colcon test-result --verbose / ros2 launch），并在技术栈行写明 Jazzy + Harmonic。所有 Linux 命令通过 `wsl` 执行；完成后面向 codex/ 分支提交并推送。

---

## 任务 2：ROS2 包骨架与全链路验证

**前置**：任务 1。

**步骤**：
1. 在 `<仓库>/ros2_ws/src` 创建 Python 包：`ros2 pkg create --build-type ament_python hello_quadcopter`
2. 写一个最小节点（发布定时消息/坐标变换），添加 `setup.cfg` 入口点与一个 launch 文件。
3. 全链路验证：`colcon build --packages-select hello_quadcopter` → `colcon test` → `colcon test-result --verbose` → `ros2 launch` → `ros2 node list` 可见节点。
4. 提交推送（`feat:` 前缀）。

**验收**：构建、测试、启动全链路通过，节点可被 `ros2 node list` 发现。

**发布用提示词**：
> 任务二：在 <仓库>/ros2_ws/src 用 `ros2 pkg create --build-type ament_python hello_quadcopter` 创建 Python 包，写一个最小节点与 launch 文件；跑通 colcon build → colcon test → colcon test-result --verbose → ros2 launch → ros2 node list 全链路；通过后 codex/ 分支提交推送。

---

## 任务 3：四旋翼 URDF/xacro 建模 + Gazebo 空载仿真

**前置**：任务 2。

**步骤**：
1. 用 xacro 建四旋翼模型（机身 + 4 旋翼 + 惯性参数，参考 Crazyflie 2.1：质量约 27 g、对角线轴距约 92 mm）。
2. 编写 launch：Gazebo 加载模型 + rviz2 可视化。
3. 验证电机可起转、模型稳定不炸裂；截图/录像存档 `docs/`。
4. 提交推送（`feat:` 前缀）。

**验收**：Gazebo 中模型可正常加载起转；rviz2 可显示位姿；仿真可复现（固定参数与初始条件）。

**发布用提示词**：
> 任务三：用 xacro 建四旋翼模型（参考 Crazyflie 2.1 尺寸/质量），写 Gazebo 加载 + rviz2 可视化 launch；验证模型稳定起转不炸裂，截图存档 docs/；通过后 codex/ 分支提交推送。

---

## 任务 4：支撑阶段动力学复现（核心里程碑 1）

**前置**：任务 1；下载 Zenodo `StanceDynamics.zip`（0.9 MB）与 `DropTest.zip`（4.9 MB）。

**步骤**：
1. 通读 `run_this.m`、`evaluate_data.m`、`derive_eqns.mlx`，建立「脚本 ↔ 论文式 12–21」的对应关系。
2. 用 Python（numpy/scipy）翻译：支撑阶段轴向 ODE（式 12，弹簧 + 库仑阻尼）、角动量守恒（式 13–14）、着陆→离地映射（式 15–21）。
3. 使用论文辨识参数：k/m = 5.00×10³ s⁻²、f_c/m = 12.7 m/s²、l_p = 1.97 cm、l₀ = 22 cm，复现图 2C 映射表（θ_LD 0–25°，‖ṗ_LD‖ 1–3.5 m/s）。
4. 按 `evaluate_data.m` 的数据管线（足端偏移 −0.22 m、足高阈值检测 t_LD/t_TO、局部线性拟合速度）处理论文动捕数据，生成着陆-离地数据集。
5. 验证：模型预测 Δψ、θ_TO 与实测对比，误差量级应接近论文（RMSE 1.0°/1.6°）。
6. 产出图（映射等高线、预测 vs 实测散点）存入 `docs/`；脚本入仓库（如 `src/hopcopter_model/`）；提交推送（`feat:` 前缀）。

**验收**：Python 模型在论文参数下生成与图 2C 一致的映射；对论文数据预测误差 <2°；脚本、数据、图全部入仓库。

**发布用提示词**：
> 任务四：从 Zenodo（10.5281/zenodo.10777420）下载 StanceDynamics.zip 与 DropTest.zip；把支撑阶段动力学（论文式 12–21）翻译为 Python 模块（src/hopcopter_model/），用论文参数（k/m=5.00e3 s⁻²、f_c/m=12.7 m/s²、l_p=1.97 cm、l0=0.22 m）复现图 2C 着陆→离地映射；按 evaluate_data.m 管线处理动捕数据并验证预测误差（Δψ/θ_TO 应 <2°）；产出图与脚本入仓库并提交推送。

---

## 任务 5：混合动力学仿真（空中 + 支撑阶段切换）

**前置**：任务 3、4。

**步骤**：
1. 在 URDF/Gazebo 中加入被动弹簧-阻尼腿（论文式 7 参数；Gazebo 无原生弹簧，需自写插件或先以简化实现验证动力学）。
2. 实现「空中弹道 → 着陆 → 支撑 → 离地」阶段切换与状态输出。
3. 自由落体掉落仿真，对照论文掉落实验：支撑时间约 32 ms、腿收缩 4.7–5.5 cm。
4. 数据与截图存档；提交推送（`feat:` 前缀）。

**验收**：掉落仿真支撑时间与收缩量与论文一致（±20%）。

**发布用提示词**：
> 任务五：在 Gazebo/URDF 中加入被动弹簧-阻尼腿（论文式 7，Gazebo 无原生弹簧需自写插件或先简化实现）；实现「空中弹道→着陆→支撑→离地」阶段切换；自由落体掉落仿真对照论文（支撑时间约 32 ms、腿收缩 4.7–5.5 cm，±20%）；数据与截图入仓库并提交推送。

---

## 任务 6：低层姿态控制 + 姿态优先推力分配

**前置**：任务 3。

**步骤**：
1. 实现姿态 PID 控制（论文低层控制部分）。
2. 实现姿态优先推力分配（式 34–35：f = argmin|T_d−T|，约束 fᵢ ≥ 0 且 τ_p,d = τ_p），支持悬停/零推力（弹道段）切换。
3. 仿真验证：悬停稳定、姿态阶跃响应无发散；rviz2 可视化。

**验收**：姿态阶跃收敛、稳态误差小；零推力段可进入纯弹道。

**发布用提示词**：
> 任务六：实现四旋翼姿态 PID 与姿态优先推力分配（论文式 34–35，支持悬停/零推力切换）；仿真验证悬停稳定与姿态阶跃收敛，rviz2 可视化；通过后提交推送。

---

## 任务 7：高层跳跃控制器（核心里程碑 2）

**前置**：任务 4、5、6。

**步骤**：
1. 实现式 22–33：着陆状态预测（弹道自由落体）、支撑映射复用（任务 4 模块）、数值优化求期望着陆姿态 z_b(t_LD)（式 31）、高度控制（Δt_PA/Δt_PJ，式 27–29）、无滑移约束（式 32）。
2. 实现周期级轨迹跟踪（落点参考取下一周期期望位置）。
3. 仿真验证：原地连续跳跃 ≥10 跳稳定；圆/阶跃轨迹跟踪对照论文图 3（落点 RMSE 量级 16/40 cm、高度 RMSE 4–5 cm）。

**验收**：连续跳跃稳定；轨迹跟踪 RMSE 在论文量级；数据存档。

**发布用提示词**：
> 任务七：实现高层跳跃控制器（论文式 22–33）：着陆状态预测、复用任务四的支撑映射、数值优化期望着陆姿态 z_b(t_LD)（式 31）、高度控制 Δt_PA/Δt_PJ、无滑移约束（式 32）；仿真验证原地连续跳跃 ≥10 跳，圆/阶跃轨迹跟踪 RMSE 对照论文（落点 16/40 cm、高度 4–5 cm 量级）；数据存档并提交推送。

---

## 任务 8：气动稳定器 + Poincaré 稳定性分析

**前置**：任务 7；下载 Zenodo `AerodynamicStabilizer.zip`（0.2 MB）与 `StabilityTests.zip`（0.7 MB）。

**步骤**：
1. 实现气动面模型（3 个 39 cm² 铰接面、平板理论，可激活/停用）与伺服控制逻辑。
2. 构建 Poincaré 映射（θ_z = arccos(−z_wᵀṗ(t_LD)/‖ṗ(t_LD)‖)，分析 θ_z|k+1/θ_z|k 稳定区域），对照论文图 6C。
3. 仿真复现三组实验：仅姿态控制 / 仅稳定器 / 两者组合；对照 StabilityTests 数据结论。

**验收**：组合策略的稳定区域与论文一致；无位置反馈下连续跳跃 ≥30 跳（仿真）。

**发布用提示词**：
> 任务八：实现气动稳定器（3 个 39 cm² 铰接面、平板理论、可激活/停用）与控制逻辑；构建 Poincaré 映射并对照论文图 6C 稳定区域；仿真复现「仅姿态/仅稳定器/组合」三组实验（可参考 Zenodo StabilityTests 数据）；无位置反馈连续跳跃 ≥30 跳后提交推送。

---

## 任务 9：性能验收与论文基准对照

**前置**：任务 7、8。

**步骤**：
1. 复现图 4A 跳跃周期-高度曲线、跳跃敏捷度 ν = 2hf（论文：2.38 m/s @ 1.63 m）。
2. 复现轨迹跟踪 RMSE（图 3）与续航占空比（仿真估测，论文 0.28）。
3. 产出对照表 `docs/benchmark.md`，逐项记录目标值、仿真值、差距与原因。

**验收**：各指标达到论文量级（敏捷度 ≥2 m/s 量级、跟踪 RMSE 同量级）；差距均有书面分析。

**发布用提示词**：
> 任务九：性能验收：复现图 4A 跳跃周期-高度曲线、敏捷度 ν=2hf、轨迹跟踪 RMSE 与续航占空比（仿真估测）；产出 docs/benchmark.md 对照表（目标值/仿真值/差距原因），提交推送。

---

## 任务 10（可选·真机）：Crazyflie 2.1 硬件复现

**前置**：仿真任务全部完成；硬件采购（Crazyflie 2.1、碳纤杆 Ø1.8/2 mm、橡胶带、轴承 VXB 681、3D 打印件、可选高速相机/动捕）。

**步骤**：
1. 按论文 Fig 1B 结构装配被动伸缩腿（腿长 22 cm、预拉伸 1.97 cm）。
2. 掉落实验 + 高速相机跟踪（复用 DropTest 管线）辨识 k/m、f_c/m、l_p。
3. 板载姿态控制（Crazyflie 固件）→ 高层跳跃控制器（任务 7 移植）→ 气动稳定器（任务 8）。
4. 数据与视频存档，对照仿真。

**验收**：真机连续跳跃 ≥10 跳、跳跃高度 ≥0.5 m。

**发布用提示词**：
> 任务十（真机）：按论文 Fig 1B 装配 Crazyflie 2.1 + 被动伸缩腿（腿长 22 cm、预拉伸 1.97 cm）；高速相机掉落实验辨识 k/m、f_c/m、l_p；移植板载姿态控制 → 高层跳跃控制器 → 气动稳定器；记录数据/视频并对照仿真，提交推送。

---

## 跨任务提醒

- 每个任务发布时附上本文件路径与任务编号，新对话会自动加载 AGENTS.md 与技能（ros2-development、robotics-testing、robotics-design-patterns 等）。
- 遇到安装/编译问题先查官方文档（docs.ros.org、gazebosim.org），再求助。
- 结果优先存档：图、数据、脚本是复现的「产物」，都要进仓库。
