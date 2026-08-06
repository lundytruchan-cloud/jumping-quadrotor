# AGENTS.md

## 项目状态

- 全新 git 仓库，当前阶段目标：**四旋翼可跳跃飞行器复现**（学习项目，非全局目标）。
- 技术栈：ROS2 Jazzy + Gazebo Harmonic（`ros-jazzy-ros-gz`，Gazebo Sim 8.x）（Python/C++ 节点）、rviz2 可视化；控制算法（PID/LQR 等）仿真优先验证。

## 工作流

- 新分支统一使用 `codex/` 前缀（如 `codex/jumping-mechanism`）。
- 提交信息使用类型前缀：`feat` / `fix` / `docs` / `chore` / `refactor` / `test`。
- 保持小粒度提交：一次提交只做一件事，提交信息写明动机。
- 改动前先阅读本文件与 README，遵循仓库既有约定。

## Shell 执行约定（重要）

- **交互操作**：先 `wsl` 进入 Ubuntu 的 bash 再敲命令；不在 PowerShell 内联写复杂
  Linux 命令（`$`、引号、分号、括号在 PowerShell → wsl → bash 三层传递中极易被
  转义破坏，已多次踩坑）。
- **脚本化操作**：Linux 命令一律写成 `ros2_ws/tools/*.sh`（用 apply_patch 创建，
  保证 LF 行尾；不要用 here-string 管道或 Windows 编辑器，CRLF 会破坏 bash 脚本），
  从仓库根目录以相对路径调用：`wsl -e bash -c "./ros2_ws/tools/xxx.sh"`（相对路径
  天然避开 `New project` 空格问题）。
- **固定脚本**：`sim_start.sh`（启动仿真）、`sim_stop.sh`（清理进程）、
  `sim_status.sh`（状态/多实例检查）、`sim_demo.sh`（电机起转+记录）、
  `sim_build.sh`（构建+测试）、`sim_photos.sh`（模型五视图渲染）；Windows 下可用
  `ros2_ws/tools/sim.ps1 start|stop|status|demo|build|photos` 免转义调用
  （首次需 `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`，或
  `powershell -ExecutionPolicy Bypass -File ...` 绕过）。详见
  `ros2_ws/tools/README.md`。
- **已知坑**：不要同时启动两套仿真实例——同一 ROS 域下多实例各有独立仿真时钟，
  rviz2 会反复输出 `Detected jump back in time. Clearing TF buffer.`；停止仿真用
  Ctrl+C 让 launch 完整退出，或用 `sim_stop.sh` 清理。

## 图片与视频协作（重要）

- 当前模型不支持图像输入（已实测验证）：无法直接查看图片、截图、视频。
- 任务中需要读取图片内容（论文图、仿真截图、图表、rviz2/Gazebo 画面）时：
  - 优先用文本/数值数据代替图片：`ros2 topic echo`、日志、CSV、数据文件、程序化提取的数值。
  - 必须依赖图片时，**明确告知用户**「需要看哪张图、关注什么」，请用户用文字描述关键信息（坐标轴含义、曲线趋势、数值、异常点）。
  - 生成图表或截图给用户查看时，同时导出对应数据（CSV/日志），便于双方核对。
- 视频同理：无法观看，需抽帧并请用户描述，或读取配套数据文件。
- 需要模型生成图片/图表时照常生成，交付给用户查看，模型本身不做视觉核验。

## 质量门槛

- 改动完成前必须验证：`colcon build` 通过、`ros2 test` / pytest 通过、节点可启动、仿真可复现。
- 新增功能/修复应附带对应测试（单元测试优先，仿真验证次之）。
- 不改动与任务无关的文件；不引入未使用的依赖。

## 验证命令

> 基于 WSL2 Ubuntu 24.04，先 `source /opt/ros/jazzy/setup.bash`，进入 `ros2_ws` 后另加 `source install/setup.bash`。
- 构建：`colcon build`（在 `ros2_ws` 下执行）
- 测试：`colcon test --packages-select <package>`，查看结果：`colcon test-result --verbose`
- 启动仿真：`ros2 launch <package> <launch-file>`

## 安全与规范

- 不提交 `.env`、密钥、令牌；敏感配置一律走环境变量。
- 仿真产物与临时文件（`build/`、`install/`、`log/`、`__pycache__/` 等）由 `.gitignore` 排除，禁止 `git add -f` 加入。
- 破坏性操作（删除、重置、覆盖）前先与用户确认。
