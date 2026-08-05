# AGENTS.md

## 项目状态

- 全新 git 仓库，当前阶段目标：**四旋翼可跳跃飞行器复现**（学习项目，非全局目标）。
- 技术栈：ROS2（Python/C++ 节点）、Gazebo 仿真、rviz2 可视化；控制算法（PID/LQR 等）仿真优先验证。
- ROS2 发行版与 Gazebo 版本确定后，请回填下方「验证命令」的具体命令。

## 工作流

- 新分支统一使用 `codex/` 前缀（如 `codex/jumping-mechanism`）。
- 提交信息使用类型前缀：`feat` / `fix` / `docs` / `chore` / `refactor` / `test`。
- 保持小粒度提交：一次提交只做一件事，提交信息写明动机。
- 改动前先阅读本文件与 README，遵循仓库既有约定。

## 质量门槛

- 改动完成前必须验证：`colcon build` 通过、`ros2 test` / pytest 通过、节点可启动、仿真可复现。
- 新增功能/修复应附带对应测试（单元测试优先，仿真验证次之）。
- 不改动与任务无关的文件；不引入未使用的依赖。

## 验证命令

> 待技术栈确定后回填，例如：
> - 构建：`colcon build`
> - 测试：`ros2 test` / `pytest`
> - 启动仿真：`ros2 launch <package> <launch-file>`

## 安全与规范

- 不提交 `.env`、密钥、令牌；敏感配置一律走环境变量。
- 仿真产物与临时文件（`build/`、`install/`、`log/`、`__pycache__/` 等）由 `.gitignore` 排除，禁止 `git add -f` 加入。
- 破坏性操作（删除、重置、覆盖）前先与用户确认。
