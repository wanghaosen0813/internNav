# LIMO 部署笔记

## 当前目标

将 InternNav 的真实机器人链路部署到 LIMO 小车上，采用以下架构：

- 小车端运行 ROS2 Foxy。
- PC 端运行 InternVLA-N1 的 HTTP 推理服务。
- 小车与 PC 之间通过 HTTP 通信。

## 已确认环境

### 小车端

- 小车已确认可进入 ROS2 Foxy 环境。
- 已确认安装如下 ROS2 包：
  - `limo_base`
  - `limo_bringup`
  - `limo_description`
  - `limo_msgs`
  - `limo_visions`
  - `orbbec_camera`
  - `orbbec_camera_msgs`
  - `realsense2_camera`
  - `realsense2_camera_msgs`

### PC 端

- 当前部署方案下，PC 端不需要 ROS2。
- PC 端仅负责运行 Python 推理服务。
- PC 与小车当前通过网线直连。
- 当前 IP 已确认：
  - PC：`192.168.100.10`
  - 小车：`192.168.100.20`

## 小车端已确认可用的启动命令

### 启动底盘

在小车端进入 ROS2 Foxy 后执行：

```bash
ros2 launch limo_base limo_base.launch.py
```

该命令已经验证可正常启动。

### 启动 Orbbec Dabai 相机

在小车端另一个 ROS2 Foxy 终端执行：

```bash
ros2 launch orbbec_camera dabai.launch.py
```

该命令已经验证可正常启动。

## 已确认 ROS2 话题

### 底盘相关话题

- `/cmd_vel`
- `/odom`
- `/imu`

### 相机相关话题

- `/camera/color/image_raw`
- `/camera/color/camera_info`
- `/camera/depth/image_raw`
- `/camera/depth/camera_info`
- `/camera/depth/points`
- `/camera/ir/image_raw`
- `/camera/ir/camera_info`

## 重要说明

### `/camera/depth_to_color` 不是深度图像

已确认 `/camera/depth_to_color` 的消息类型是：

- `orbbec_camera_msgs/msg/Extrinsics`

因此它不是可直接订阅的深度图话题。

### 第一版部署使用的话题

当前第一版客户端应直接订阅：

- RGB 图像：`/camera/color/image_raw`
- Depth 图像：`/camera/depth/image_raw`
- 里程计：`/odom`
- 控制话题：`/cmd_vel`

## 当前相机输入特性

从启动日志中确认：

- 彩色图像：`640x480 @ 30 FPS`
- 深度图像：`640x400 @ 30 FPS`

这说明当前 RGB 与 Depth 在分辨率上并不是天然严格对齐的。

因此第一阶段建议策略为：

1. 先直接使用 `/camera/color/image_raw` 与 `/camera/depth/image_raw`。
2. 先打通整条 HTTP 推理控制链路。
3. 后续如有需要，再进一步处理深度对齐问题。

## 推荐运行架构

```text
LIMO 小车（ROS2 客户端）
  -> 订阅 RGB / Depth / Odom
  -> 通过 HTTP 向 PC 发请求

PC（InternVLA-N1 推理服务）
  -> 运行模型推理
  -> 返回 discrete_action 或 trajectory

LIMO 小车（ROS2 客户端）
  -> 发布 /cmd_vel
```

## 当前已完成的 PC 端准备

### 模型权重已确认存在

路径：

- `/home/whs/wanghaosen/code/InternNav/checkpoints/InternVLA-N1`

### GPU 与 PyTorch 已确认可用

在 `internnav` 环境中已确认：

- `torch.cuda.is_available() == True`
- GPU：`NVIDIA GeForce RTX 4090`

### Python 依赖已检查

在 `internnav` 环境中已确认：

- `PIL`：正常
- `requests`：正常
- `transformers`：正常
- `flash_attn`：正常
- `cv2`：正常
- `numpy`：正常
- `flask`：已补装完成

### 服务端脚本参数已验证

已验证以下命令可正常解析参数：

```bash
source /home/whs/anaconda3/etc/profile.d/conda.sh
conda activate internnav
cd /home/whs/wanghaosen/code/InternNav
python scripts/realworld/http_internvla_server.py --help
```

## 当前推荐启动步骤

### A. 小车端启动 ROS2 相关节点

#### 1. 启动底盘

```bash
ros2 launch limo_base limo_base.launch.py
```

#### 2. 启动相机

```bash
ros2 launch orbbec_camera dabai.launch.py
```

### B. PC 端启动推理服务

```bash
source /home/whs/anaconda3/etc/profile.d/conda.sh
conda activate internnav
cd /home/whs/wanghaosen/code/InternNav

python scripts/realworld/http_internvla_server.py \
  --device cuda:0 \
  --model_path checkpoints/InternVLA-N1 \
  --resize_w 384 \
  --resize_h 384 \
  --num_history 8 \
  --plan_step_gap 4
```

### C. 小车端启动 HTTP 客户端

当前客户端默认已适配如下话题：

- RGB：`/camera/color/image_raw`
- Depth：`/camera/depth/image_raw`
- Odom：`/odom`
- CmdVel：`/cmd_vel`

启动命令建议为：

```bash
python scripts/realworld/http_internvla_client.py \
  --server_url http://192.168.100.10:5801/eval_dual
```

## 常用检查命令

### 检查关键话题

```bash
ros2 topic list | grep -E "image|depth|camera|odom|cmd_vel"
```

### 检查里程计

```bash
ros2 topic echo /odom --once
```

### 检查控制话题类型

```bash
ros2 topic info /cmd_vel
```

### 检查图像话题类型

```bash
ros2 topic info /camera/color/image_raw
ros2 topic info /camera/depth/image_raw
```

## 已完成代码适配

### 服务端已修改

文件：

- `/home/whs/wanghaosen/code/InternNav/scripts/realworld/http_internvla_server.py`

已完成：

- 不再写死 `instruction`。
- 改为从请求中读取指令内容。
- 补上了 `plan_step_gap` 参数。

### 小车客户端已修改

文件：

- `/home/whs/wanghaosen/code/InternNav/scripts/realworld/http_internvla_client.py`

已完成：

- 适配当前 LIMO 的 ROS2 话题。
- 默认话题已改为：
  - RGB：`/camera/color/image_raw`
  - Depth：`/camera/depth/image_raw`
  - Odom：`/odom`
  - CmdVel：`/cmd_vel`
- 增加了 `server_url`、`instruction`、topic 等启动参数。

## 当前联调进展

### PC 端推理服务已跑通

当前已确认：

- 模型可正常加载。
- warmup 已可正常执行。
- Flask 服务已成功监听 `5801` 端口。
- 当前 PC 端已有服务实例在运行。

说明：

- 启动日志中出现的 `slow image processor`、`temperature/top_p/top_k` 警告当前不是阻塞问题。
- 如果再次启动时提示 `Address already in use`，说明旧服务仍在运行，不需要重复拉起。

### 小车端 Python 环境已准备

当前在小车端使用：

```bash
python3 -m venv --system-site-packages ~/venvs/internnav_limo
source ~/venvs/internnav_limo/bin/activate
```

说明：

- 这里使用 `--system-site-packages`，目的是兼容 ROS2 自带的 `rclpy`、`cv_bridge` 等系统 Python 包。
- 不建议在小车端使用 conda 管理这一条 ROS2 控制链。

当前已确认安装：

- `casadi`
- `requests`
- `scipy`

### 小车端客户端已成功连上 PC 服务

当前在小车端运行：

```bash
cd /home/agilex/InternNav/realworld
python http_internvla_client.py \
  --server_url http://192.168.100.10:5801/eval_dual
```

已确认客户端启动后可正常打印：

- 订阅的话题
- 服务端 URL
- HTTP 返回结果

当前已实际收到的服务端返回示例：

```json
{"discrete_action":[3,3,3,3]}
```

这说明以下链路已经打通：

1. 小车端读取 RGB / Depth / Odom
2. 小车端向 PC 发 HTTP 请求
3. PC 端完成推理并返回动作
4. 小车端将返回动作转换为 `/cmd_vel`

## 当前控制侧排查结论

### `/cmd_vel` 当前无冲突

已确认：

- `/cmd_vel` 当前只有 1 个发布者：`limo_manager`
- `/cmd_vel` 当前只有 1 个订阅者：`limo_base`

因此当前不存在多个节点抢占控制的问题。

### 当前客户端发出的角速度很小

实际观测到的 `/cmd_vel` 示例：

- `linear.x = 0.0`
- `angular.z = -0.0068`

这类角速度过小，底盘几乎不会产生明显转向。

### 底盘原地转向能力已确认

通过手动测试已确认：

- 前进命令可执行。
- 使用更大的纯角速度命令后，小车可以原地转向。

参考对比代码：

- `/home/whs/wanghaosen/code/ApexNav/real_world_test_example/real_world_test_limo.py`

其中 ApexNav 的原地转圈逻辑就是直接发布：

- `linear.x = 0.0`
- `angular.z = 1.0`

这说明 LIMO 本身支持原地转向，当前主要问题不是底盘能力，而是我们当前客户端输出的转向控制量太小，且表现略有顿挫。

## 当前待处理问题

### 转向动作偏顿

当前现象：

- 小车已经可以接收动作并原地转向。
- 但真实表现仍有“一顿一顿”的感觉。

当前判断：

- 问题更偏向控制平滑性，而不是通信或底盘驱动错误。
- 目前离散动作到 `/cmd_vel` 的转换还不够适合 LIMO 的实车表现。

### 下一步建议

建议下一步对以下文件做 LIMO 平滑控制适配：

- `/home/whs/wanghaosen/code/InternNav/scripts/realworld/http_internvla_client.py`
- `/home/whs/wanghaosen/code/InternNav/scripts/realworld/controllers.py`

优化方向：

- 提高原地转向时的有效角速度下限。
- 对角速度做平滑与限幅。
- 让离散转向动作映射成更适合 LIMO 的稳定旋转控制。

## 当前状态

- 底盘启动：已确认
- 相机启动：已确认
- ROS2 关键话题：已确认
- PC 端环境准备：已确认
- 服务端代码适配：已完成
- 小车客户端适配：已完成
- PC 推理服务启动：已完成
- 小车 HTTP 联调：已完成
- 底盘原地转向能力：已确认
- 当前剩余问题：控制平滑性待优化
- 默认使用以下话题：
  - RGB：`/camera/color/image_raw`
  - Depth：`/camera/depth/image_raw`
  - Odom：`/odom`
  - CmdVel：`/cmd_vel`
- 增加了 `server_url`、`instruction`、topic 等启动参数。

## 最新控制适配记录

已在 `scripts/realworld/http_internvla_client.py` 中增加 LIMO 原地转向最小角速度逻辑：

- `turn_in_place_omega = 1.0`
- `turn_in_place_linear_threshold = 0.05`
- `turn_in_place_angular_deadband = 0.02`

目的：

- 当客户端输出的 `angular.z` 只有 `0.02 ~ 0.03` 量级时，LIMO 基本不会转动。
- 参考 ApexNav 的 `real_world_test_limo.py`，原地转圈可用角速度是 `1.0 rad/s`。
- 因此现在在“原地转向”场景下，将过小角速度提升到最小有效值。

## 最新实验记录

### 任务下发已验证

已在小车端通过以下方式显式下发任务：

```bash
python http_internvla_client.py \
  --server_url http://192.168.100.10:5801/eval_dual \
  --instruction "Find the chair and stop near it."
```

当前确认：

- 客户端可以将 `instruction` 正常发送到 PC 端服务。
- 服务端会持续返回离散动作。
- 返回动作会在不同时间段发生变化，例如：
  - `[3,3,3,3]`
  - `[2,2,2,2]`

这说明当前不仅部署链路打通，任务层也已经成功接入。

### 小车端可视化已验证

当前已确认可用的可视化方式：

```bash
ros2 run rqt_image_view rqt_image_view
ros2 run rqt_plot rqt_plot
```

推荐查看：

- 图像：`/camera/color/image_raw`
- 控制曲线：`/cmd_vel/angular/z`

当前已经能够同时观察：

- 小车实时图像
- `/cmd_vel` 角速度变化

### 当前策略现象

目前最典型的现象是：

- 小车会原地左右转动
- 但通常不向前走，也不向后走
- 当前更像是在执行“搜索椅子”的动作，而不是“锁定椅子后靠近”

结合当前输出判断：

- 当前模型大多数时候仍在输出左右转相关离散动作
- 尚未稳定输出前进动作 `[1]`

因此目前更准确的状态应表述为：

- 部署已完成
- 任务下发已完成
- 可视化已完成
- 但任务闭环仍未完成，当前仍停留在“搜索阶段”

### 当前控制参数测试结论

在把原地转向最小角速度提升后，已观测到 `/cmd_vel` 变为：

- `angular.z = -1.0`
- `angular.z = 1.0`

这说明：

- 原地转向放大逻辑已生效
- 但在当前场景下，左右切换较频繁，导致原地搜索动作较明显

### 当前剩余问题更新

当前待解决的问题已经从“部署是否成功”变为：

1. 模型是否能稳定识别并锁定画面中的椅子
2. 锁定目标后是否能输出前进动作而不只是原地搜索
3. 转向搜索动作是否需要进一步做防抖和平滑处理


python http_internvla_client.py \
  --server_url http://192.168.100.10:5801/eval_dual \
  --instruction "If you see a chair, move straight toward the chair and stop close to it. If you do not see a chair, turn slowly to search." \
  --turn_in_place_omega 0.4 \
  --turn_in_place_angular_deadband 0.0
## 最新可视化增强记录

已在 `scripts/realworld/http_internvla_client.py` 中增加两类调试可视化输出：

### 1. 调试图像话题

默认发布：

- `/internnav/debug_image`

图像中会叠加：

- 当前 `discrete_action`
- 当前轨迹点数量
- `pixel_goal` 红色标记点

### 2. 轨迹 Path 话题

默认发布：

- `/internnav/debug_path`

该话题会把当前局部轨迹以 `nav_msgs/Path` 的形式发布，坐标系为 `odom`。

### 查看方法

在小车端：

```bash
ros2 run rqt_image_view rqt_image_view
```

选择：

- `/internnav/debug_image`

在 RViz2 中可添加：

- `Path`，Topic 选择 `/internnav/debug_path`
- `TF`
- `Odometry`，Topic 选择 `/odom`

说明：

- 当前仍然没有独立的地图话题发布。
- 但现在已经可以直接看到模型的 `pixel_goal` 和当前局部轨迹。



## 运行时目标发现提示

当前是否找到椅子的权威判断以 PC 端服务日志为准。

PC 端服务终端会输出明显的状态标志：

- `[SEARCHING_FOR_CHAIR]`：当前还在搜索椅子
- `[FOUND_CHAIR_CANDIDATE]`：当前已经给出像素目标点，可视为发现椅子候选目标
- `[TRACKING_TARGET]`：当前已经输出局部轨迹，开始跟踪目标方向
- `[MOVING_TOWARD_TARGET]`：当前离散动作里已经包含前进动作

这些日志会保存到 PC 端：

- `test_data/时间戳目录/server_runtime.log`

日志中会额外保存：

- 状态切换标志
- 每次 HTTP 返回的原始 JSON：`[RAW_RESPONSE] ...`

如果需要查看本次运行日志，可在 PC 端执行：

```bash
ls test_data
tail -f test_data/时间戳目录/server_runtime.log
```
