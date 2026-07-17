# Cobot Magic

本仓库包含 ALOHA/ACT 训练与推理代码、ROS 相机工作空间、数据采集脚本，以及 TiPToP bridge 相关代码。

## 目录说明

- `aloha-devel/`：ACT、robomimic 训练与推理代码。
- `camera_ws/`：ROS 相机工作空间和相机驱动源码。
- `collect_data/`：数据采集和回放脚本；Piper SDK demo 位于其 Git submodule 中。
- `tools/`：CAN、相机和工作空间辅助脚本。

## Piper SDK demo（Git submodule）

`collect_data/piper_sdk_demo` 是 AgileX Piper 机械臂的 CAN/SDK 示例，负责机械臂使能、状态读取、关节控制、主从臂配置和 MIT 模式配置。它不是训练日志或模型权重。

主仓库只记录子模块提交指针；当前固定版本为 `f88fa63`。首次克隆时初始化子模块：

```bash
git clone --recurse-submodules https://github.com/Str0keOOOO/Cobot_Magic.git
cd Cobot_Magic
```

已有工作区补齐子模块：

```bash
git submodule sync --recursive
git submodule update --init --recursive
git submodule status
```

更新到上游 `master` 后，需要在主仓库提交新的子模块指针：

```bash
git -C collect_data/piper_sdk_demo fetch origin
git -C collect_data/piper_sdk_demo checkout master
git -C collect_data/piper_sdk_demo pull --ff-only
git add collect_data/piper_sdk_demo
git commit -m "Update Piper SDK demo"
```

当前本地子模块还有未提交的定制：删除部分 CAN/配置/读取示例，并修改了 `piper_joint_ctrl.py`、`piper_status.py`。这些改动不会随主仓库的子模块指针自动同步；如果需要让其他人复现，应在子模块中提交并推送，或导出补丁：

```bash
git -C collect_data/piper_sdk_demo diff > piper_sdk_demo.local.patch
```

## 运行和编译产物

以下目录由本地运行或编译生成，不应提交到 Git：

- `camera_ws/build/`、`camera_ws/devel/`
- `Piper_ros_private-ros-noetic/build/`、`Piper_ros_private-ros-noetic/devel/`
- Python 的 `__pycache__/`、`*.egg-info/`、`.bridge_deps/`
- `*.log`、`**/output.txt`、训练 checkpoint 和数据文件

ROS 工作空间编译（在 ROS Noetic 环境中）：

```bash
cd camera_ws
catkin_make
source devel/setup.bash

cd ../Piper_ros_private-ros-noetic
catkin_make
source devel/setup.bash
```

需要从头生成时，可先删除对应的 `build/` 和 `devel/`，再重新执行 `catkin_make`：

```bash
rm -rf camera_ws/build camera_ws/devel
rm -rf Piper_ros_private-ros-noetic/build Piper_ros_private-ros-noetic/devel
```

TiPToP bridge 的 Python 依赖和可编辑安装：

```bash
python3 -m pip install -r requirements-tiptop-bridge.txt
python3 -m pip install -e .
```

训练数据、模型权重和推理输出建议放在仓库外部，例如 `~/cobot_magic_data/` 或 `~/cobot_magic_checkpoints/`，避免误提交设备数据和大文件。
