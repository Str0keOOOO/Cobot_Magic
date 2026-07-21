# TiPToP Cobot Magic 客户端

这个目录运行在机械臂上位机，用于将现有的 ROS、机械臂和右腕相机接入 TiPToP 推理服务器。它不会启动或修改 ROS Master、机械臂驱动和相机驱动。

## 一、启动机械臂与三相机

### 0. 配置 CAN

每次开机，或每次重新插拔 CAN 模块后，都必须先执行：

```bash
cd Cobot_Magic/Piper_ros_private-ros-noetic/
bash can_config.sh
```

### 1. 终端一：启动 ROS Master

```bash
roscore
```

### 2. 终端二：启动双从臂

启动前先将机械臂断电重启，并**拔掉主臂的航空插头**。

```bash
cd /home/agilex/xuchenfei/Cobot_Magic/Piper_ros_private-ros-noetic
source devel/setup.bash
roslaunch piper start_ms_piper.launch mode:=1 auto_enable:=true
```

### 3. 终端三：启动三台 Intel RealSense 相机

```bash
cd /home/agilex/xuchenfei/Cobot_Magic/camera_ws
source devel/setup.bash
roslaunch realsense2_camera multi_camera.launch
```

## 二、TiPToP 桥接服务：端口、能力与启动

在第一部分的 ROS、双从臂和三相机均已启动后，再启动本客户端。客户端对外提供的是 **ZeroMQ + MessagePack RPC**，不是 HTTP 或视频流服务。GPU 侧需要实现同一套 RPC 客户端；这些是本桥接层提供的能力，并不代表 TiPToP 会自动调用。

### 1. 集中启动命令

首次使用时，在上位机安装客户端依赖（ALOHA不要安装）：

```bash
cd /home/agilex/xuchenfei/Cobot_Magic
python3 -m pip install -r tiptop_client/requirements.txt
```

然后在两个新终端中分别运行以下命令：

```bash
# 终端四：控制桥接
cd /home/agilex/xuchenfei/Cobot_Magic
python3 -m tiptop_client controller-server --config tiptop_client/config.yaml
```

```bash
# 终端五：相机桥接
cd /home/agilex/xuchenfei/Cobot_Magic
python3 -m tiptop_client camera-server --config tiptop_client/config.yaml
```

### 2. 端口

监听地址和端口由 `config.yaml` 的 `bind_host` 与 `port` 决定，默认如下。GPU 推理服务器应通过 SSH 隧道访问它们，不能直接暴露到不受信任的网络。

| 默认地址 | 服务 | 用途 |
| --- | --- | --- |
| `tcp://<controller.bind_host>:5555` | 控制桥接 | 读取机械臂状态，并接收高层夹爪或关节轨迹命令。 |
| `tcp://<camera_server.bind_host>:5556` | 相机桥接 | 读取已缓存的右腕相机数据和相机内参。 |

所有请求必须包含 `protocol_version: "1.0"`、非空 `request_id`、`op` 和字典类型的 `params`。所有响应均返回 `protocol_version`、对应的 `request_id`、`success`、`result` 和 `error`。

### 3. 控制桥接能力：5555

| `op` | 参数 | 成功返回/实际行为 |
| --- | --- | --- |
| `ping` | 无 | 服务名和协议版本，用于连通性检查。 |
| `health` | 无 | ROS 是否初始化、是否收到关节状态、状态新鲜度、当前是否正在执行轨迹、自由度、订阅/发布 topic、伺服频率，以及是否配置关节限位。 |
| `get_joint_positions` | 无 | 当前 6 个机械臂关节位置，字段为 `joint_positions`，单位为弧度。关节状态未收到或过期时会失败。 |
| `open_gripper` | 可选 `speed`、`force`，范围均为 `0` 到 `1` | 将夹爪移动到已配置的打开位置。`speed` 会影响插补时长；Piper 当前 ROS 控制消息不支持实际力控制，因此返回会标记 `force_supported: false`。 |
| `close_gripper` | 同上 | 将夹爪移动到已配置的关闭位置；`force` 同样只是记录请求值，不会改变实际夹爪力。 |
| `execute_joint_impedance_path` | 必填 `joint_confs`、`joint_vels`、`durations` | 执行完整的高层关节轨迹，不接收网络逐伺服指令。`joint_confs` 和 `joint_vels` 必须是 `N×6`；位置单位为弧度、速度单位为弧度/秒；`durations` 有 `N` 个值，语义由配置中的 `trajectory_duration_mode` 决定。执行前会校验有限数、关节限位、速度、首 waypoint 与当前姿态的误差，以及每段实际插补速度。一次只允许一条运动命令。 |
| `stop` | 无 | 停止桥接层的轨迹发布并发布当前位置保持命令。**这不是硬件急停（E-stop）**，返回会明确 `safety_stop_interface_available: false`。 |

### 4. 相机桥接能力：5556

当前 `config.yaml` 只启用 `right_wrist` 相机。服务实现支持多个已配置相机，但读取内参或画面时都必须指定相机 `serial`。

| `op` | 参数 | 成功返回/实际行为 |
| --- | --- | --- |
| `ping` | 无 | 服务名和协议版本。 |
| `health` | 无 | 每台已配置相机的运行状态。 |
| `list_cameras` | 无 | 已配置相机的 `namespace`、`serial` 和 `role`。 |
| `get_intrinsics` | 必填 `serial` | 不依赖图像到达顺序，返回 `serial`、`K_color: float32[3,3]`、`distortion_color: float32[5]`、`K_ir: float32[3,3]`、`baseline_ir`（米）和 `T_color_from_ir: float32[4,4]`。`K_color`/`D` 来自 color CameraInfo，`K_ir` 来自 IR1 CameraInfo，baseline 从 IR2 的 `abs(P[0,3] / P[0,0])` 计算，TF 查询方向为 color optical frame ← IR1 optical frame。 |
| `read_camera` | 必填 `serial` | **最新一份 RGB、IR1、IR2 同步缓存快照**：`serial`、有限的 ROS `header.stamp` 时间戳 `timestamp`、RGB `uint8[H,W,3]`（RGB 通道顺序）、`ir_left: uint8[H,W]` 和 `ir_right: uint8[H,W]`。三张图的空间分辨率必须完全相同；IR 不会复制为三通道。 |

`multi_camera.launch` 已对 `/camera_r` 明确启用 color、infra1 和 infra2 的
`640×480@30fps` 流，并打开 RealSense `enable_sync`。桥接服务使用
`/camera_r/color/image_raw`、`/camera_r/infra1/image_rect_raw` 和
`/camera_r/infra2/image_rect_raw`；不要用 ZED 或用伪彩色 IR 数据替代这些流。