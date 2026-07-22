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

### 2. 控制桥接能力：`tcp://<controller.bind_host>:5555`

| `op` | 参数 | 成功返回/实际行为 |
| --- | --- | --- |
| `ping` | 无 | 服务名和协议版本，用于连通性检查。 |
| `health` | 无 | ROS 是否初始化、是否收到关节状态、状态新鲜度、当前是否正在执行轨迹、自由度、订阅/发布 topic 和伺服频率。 |
| `get_joint_positions` | 无 | 当前 6 个机械臂关节位置，字段为 `joint_positions`，单位为弧度。关节状态未收到或过期时会失败。 |
| `open_gripper` | 可选 `speed`、`force`，范围均为 `0` 到 `1` | 将夹爪移动到已配置的打开位置。`speed` 会影响插补时长；Piper 当前 ROS 控制消息不支持实际力控制，因此返回会标记 `force_supported: false`。 |
| `close_gripper` | 同上 | 将夹爪移动到已配置的关闭位置；`force` 同样只是记录请求值，不会改变实际夹爪力。 |
| `execute_joint_impedance_path` | 必填 `joint_confs`、`joint_vels`、`durations` | 执行完整的高层关节轨迹，不接收网络逐伺服指令。`joint_confs` 和 `joint_vels` 必须是 `N×6`；位置单位为弧度、速度单位为弧度/秒；`durations` 有 `N` 个值，语义由配置中的 `trajectory_duration_mode` 决定。执行前会校验数组形状、有限值与时长合法性；一次只允许一条运动命令。 |
| `stop` | 无 | 停止桥接层的轨迹发布并发布当前位置保持命令。**这不是硬件急停（E-stop）**，返回会明确 `safety_stop_interface_available: false`。 |

### 3. 相机桥接能力：`tcp://<camera_server.bind_host>:5556`

当前 `config.yaml` 只启用 `right_wrist` 相机。服务实现支持多个已配置相机，但读取内参或画面时都必须指定相机 `serial`。

| `op` | 参数 | 成功返回/实际行为 |
| --- | --- | --- |
| `ping` | 无 | 服务名和协议版本。 |
| `health` | 无 | 每台已配置相机的运行状态。 |
| `list_cameras` | 无 | 已配置相机的 `namespace`、`serial` 和 `role`。 |
| `get_intrinsics` | 必填 `serial` | 只返回静态标定：`serial`、`K_color: float32[3,3]`、`K_ir: float32[3,3]`、`baseline_ir`（米）、`T_color_from_ir: float32[4,4]`、`distortion_color: float32[5]`。CameraInfo 和 TF 独立缓存，因此不等待图像。`T_color_from_ir` 是 `color optical frame ← IR1 optical frame`，满足 `point_color = T_color_from_ir @ point_ir`。baseline 优先为 IR1←IR2 TF 平移范数，IR2 `P[0,3]/P[0,0]` 仅作可校验的回退。 |
| `read_camera` | 必填 `serial`；可选 `enable_depth`（布尔，默认 `false`） | 只返回最新同步动态帧：`serial`、color header 的有限 `timestamp`、RGB `uint8[H,W,3]`（RGB 顺序）、`ir1: uint8[H,W]`（左 IR）、`ir2: uint8[H,W]`（右 IR）。三张图必须同分辨率，IR 不复制为三通道。仅当 `enable_depth: true` 时还返回对齐 RGB 的米制 `depth: float32[H,W]`；`enable_depth: false` 时不返回 `depth`。若请求了深度但该相机未在配置中启用深度流，返回错误 `DEPTH_UNAVAILABLE`。 |

```python
get_intrinsics = {"serial": "339222070351", "K_color": Kc, "K_ir": Ki,
                  "baseline_ir": 0.055, "T_color_from_ir": Tci,
                  "distortion_color": Dc}
read_camera = {"serial": "339222070351", "timestamp": 1710000000.0,
               "rgb": rgb, "ir1": ir_left, "ir2": ir_right}
# enable_depth=true 时 read_camera 另有 depth（float32 米制、与 RGB 对齐）。
```
