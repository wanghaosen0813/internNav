# LIMO 最终部署启动手册

## 目的

本手册只记录当前已经验证可用的启动步骤，用于实机部署和重复启动。

说明：

- 过程排障、问题记录、测试结论继续写入 `LIMO部署笔记.md`。
- 本手册只保留最终可执行步骤，不记录长篇分析。

## 当前部署架构

- PC 端：运行 InternVLA-N1 HTTP 推理服务
- 小车端：运行 ROS2 Foxy、底盘、相机、HTTP 客户端
- 通信方式：小车通过 HTTP 请求 PC 推理服务

## 当前网络信息

- PC：`192.168.100.10`
- 小车：`192.168.100.20`

## 重要路径

### PC 端

- 项目目录：`/home/whs/wanghaosen/code/InternNav`
- 模型目录：`/home/whs/wanghaosen/code/InternNav/checkpoints/InternVLA-N1`

### 小车端

- 客户端目录：`/home/agilex/InternNav/realworld`
- 虚拟环境：`~/venvs/internnav_limo`

## 小车端已确认话题

- RGB：`/camera/color/image_raw`
- Depth：`/camera/depth/image_raw`
- Odom：`/odom`
- CmdVel：`/cmd_vel`

## 一次完整启动顺序

### 第 0 步：如有旧服务，先清理 `5801` 端口

在 PC 上执行：

```bash
lsof -i:5801
```

如果看到旧的 `python` 服务仍在监听 `5801`，先结束该进程：

```bash
kill PID
```

例如：

```bash
kill 17046
```

如果普通 `kill` 后仍未退出，可再执行：

```bash
kill -9 PID
```

然后再次确认端口已经释放：

```bash
lsof -i:5801
```

如果没有输出，说明端口已释放，可以继续启动新的服务端。

也可以直接使用一条命令清理旧服务：

```bash
pkill -f "python scripts/realworld/http_internvla_server.py"
```

### 第 1 步：PC 端启动推理服务

在 PC 上执行：

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

启动成功标志：

- 能看到 Flask 监听 `5801`
- 终端中出现类似如下信息：

```text
* Running on all addresses (0.0.0.0)
* Running on http://127.0.0.1:5801
```

### 第 2 步：小车端启动底盘

在小车 ROS2 Foxy 终端执行：

```bash
ros2 launch limo_base limo_base.launch.py
```

### 第 3 步：小车端启动相机

在小车另一个 ROS2 Foxy 终端执行：

```bash
ros2 launch orbbec_camera dabai.launch.py
```

### 第 4 步：小车端激活 Python 虚拟环境

在小车新的终端中，先进入 ROS2 Foxy，再执行：

```bash
source ~/venvs/internnav_limo/bin/activate
```

### 第 5 步：小车端启动 HTTP 客户端

在小车端执行：

```bash
cd /home/agilex/InternNav/realworld
python http_internvla_client.py \
  --server_url http://192.168.100.10:5801/eval_dual
```

启动成功标志：

- 能看到以下日志字段：
  - `RGB topic`
  - `Depth topic`
  - `Odom topic`
  - `CmdVel topic`
  - `Server URL`
- 能看到服务端返回结果，例如：

```text
response {"discrete_action":[3,3,3,3]}
```

## 小车端首次环境准备

如果小车还没有创建虚拟环境，执行：

```bash
python3 -m venv --system-site-packages ~/venvs/internnav_limo
source ~/venvs/internnav_limo/bin/activate
python -m pip install -U pip
python -m pip install casadi requests scipy
```

说明：

- 使用 `--system-site-packages` 是为了兼容 ROS2 的系统 Python 包。
- 不建议在小车端使用 conda 管理这条 ROS2 控制链。

## 常用检查命令

### 检查 PC 端服务端口

在 PC 上执行：

```bash
lsof -i:5801
```

### 检查小车关键话题

```bash
ros2 topic list | grep -E "image|depth|camera|odom|cmd_vel"
```

### 检查 `/cmd_vel` 是否有冲突

```bash
ros2 topic info /cmd_vel -v
```

当前已确认正常情况为：

- 发布者：`limo_manager`
- 订阅者：`limo_base`

### 查看当前控制输出

```bash
ros2 topic echo /cmd_vel
```

## 部署完成验收标准

满足以下 4 条时，可认为当前部署已经完成：

### 1. PC 端推理服务在线

在 PC 上执行：

```bash
lsof -i:5801
```

验收标准：

- 能看到 `python` 进程监听 `5801`
- 服务端没有退出

### 2. 小车端关键 ROS2 话题在线

在小车上执行：

```bash
ros2 topic list | grep -E "image|depth|camera|odom|cmd_vel"
```

验收标准：

- 看到 `/camera/color/image_raw`
- 看到 `/camera/depth/image_raw`
- 看到 `/odom`
- 看到 `/cmd_vel`

### 3. 小车客户端能收到服务端返回

查看小车端客户端终端输出。

验收标准：

- 能看到 `response {...}`
- 能看到 `idx: ... after http ...`

示例：

```text
response {"discrete_action":[3,3,3,3]}
idx: 0 after http 0.20
```

### 4. 小车端能够向底盘发布控制

在小车上执行：

```bash
ros2 topic info /cmd_vel -v
```

验收标准：

- 发布者是 `limo_manager`
- 订阅者是 `limo_base`

补充检查：

```bash
ros2 topic echo /cmd_vel
```

若能持续看到控制消息输出，则说明客户端控制链路正常。

结论：

- 以上 4 条全部满足时，说明部署主链路已经打通。
- 之后若仍有问题，优先按“控制调优问题”处理，而不是按“部署失败”处理。

## 当前已知限制

当前版本已经完成部署与任务下发，但仍有以下已知限制：

- 小车当前能够接收任务并执行原地搜索。
- 当前最常见现象是原地左右转动。
- 在当前场景下，模型尚未稳定输出前进动作，因此通常不会主动靠近目标物体。

这说明：

- 部署成功不等于任务闭环成功。
- 当前版本已经完成“系统可运行”，但还没有完成“稳定找到并接近目标物体”。

## 当前推荐可视化方式

在小车端可使用以下命令：

```bash
ros2 run rqt_image_view rqt_image_view
ros2 run rqt_plot rqt_plot
```

推荐查看：

- 图像话题：`/camera/color/image_raw`
- 控制曲线：`/cmd_vel/angular/z`

用途：

- `rqt_image_view`：确认当前画面中是否真的出现目标物体
- `rqt_plot`：观察当前是否在持续左右切换角速度

## 常见问题处理

### 1. PC 端提示 `Address already in use`

说明：

- `5801` 端口已经有一个服务在运行
- 先不要重复启动，先检查端口占用：

```bash
lsof -i:5801
```

### 2. 小车端提示缺少 Python 包

先激活虚拟环境：

```bash
source ~/venvs/internnav_limo/bin/activate
```

然后再安装缺失依赖。

### 3. 小车端能收到动作，但转向很弱

当前已知：

- LIMO 支持原地转向
- 当前客户端控制链已打通
- 若转向偏弱或发顿，属于后续控制调优问题，不属于部署失败

## 当前版本结论

当前版本已经完成：

- PC 推理服务部署
- 小车底盘与相机启动
- 小车客户端连接 PC 服务
- HTTP 动作返回与 `/cmd_vel` 发布

当前剩余问题：

- 控制平滑性仍需后续优化

## 当前原地转向参数说明

当前客户端已增加以下参数，用于解决 LIMO 原地转向角速度过小的问题：

```bash
--turn_in_place_omega 1.0
--turn_in_place_linear_threshold 0.05
--turn_in_place_angular_deadband 0.02
```

含义：

- 当 `linear.x` 足够接近 0 且 `angular.z` 不为 0 时，客户端会把过小的角速度提升到至少 `1.0 rad/s`。
- 这个值参考了 ApexNav 中 `real_world_test_limo.py` 的原地转圈角速度。

如果后续觉得转向过猛，可优先调小：

- `--turn_in_place_omega 0.6`
- 或 `--turn_in_place_omega 0.8`
## 当前服务端相机内参

PC 服务端当前使用的小车真实彩色相机内参来自：

- `/camera/color/camera_info`

当前参数：

- 分辨率：`640 x 480`
- `fx = 489.2552490234375`
- `fy = 489.2552490234375`
- `cx = 317.91510009765625`
- `cy = 216.17910766601562`

说明：

- 如重新部署服务端，请确保 `scripts/realworld/http_internvla_server.py` 中保持这组内参。
- 如果后续更换相机分辨率或更换相机设备，需要重新读取 `camera_info` 并同步更新服务端。
## 当前调试可视化话题

当前客户端启动后会额外发布：

- 调试图像：`/internnav/debug_image`
- 调试轨迹：`/internnav/debug_path`

### 查看调试图像

在小车端执行：

```bash
ros2 run rqt_image_view rqt_image_view
```

选择：

- `/internnav/debug_image`

图像中可看到：

- 当前动作
- `pixel_goal` 红色标记点
- 当前轨迹点数量

### 查看轨迹

在小车端执行：

```bash
rviz2
```

在 RViz2 中添加：

- `Path`，Topic 选择 `/internnav/debug_path`
- `TF`
- `Odometry`，Topic 选择 `/odom`

说明：

- 当前这套链路默认还不会发布地图。
- 但已经可以可视化当前局部轨迹和模型给出的像素目标。



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

## 客户端取图链路更新（压缩图像优先）

当前小车端客户端已改为更接近 ApexNav 的实机风格：

- 默认优先订阅压缩彩色图：`/camera/color/image_raw/compressed`
- 默认优先订阅压缩深度图：`/camera/depth/image_raw/compressedDepth`
- 不再严格依赖 `ApproximateTimeSynchronizer`
- 改为分别缓存最新 RGB / Depth 帧，再按时间差和低频间隔处理
- 如果上一帧仍在处理，则直接跳过新帧，相当于“忙则丢帧”

### 新的推荐启动命令

```bash
python http_internvla_client.py \
  --server_url http://192.168.100.10:5801/eval_dual \
  --instruction "The chair is in front of you. Move toward the chair and stop near it." \
  --frame_process_interval 0.3 \
  --sync_slop 0.3
```

### 如需切回原始 raw 话题

```bash
python http_internvla_client.py \
  --server_url http://192.168.100.10:5801/eval_dual \
  --instruction "The chair is in front of you. Move toward the chair and stop near it." \
  --no-use_compressed_rgb \
  --no-use_compressed_depth \
  --rgb_topic /camera/color/image_raw \
  --depth_topic /camera/depth/image_raw
```



## 当前推荐的小车端纯净环境

小车端所有 ROS2 终端统一使用下面这套方式启动：

```bash
bash --noprofile --norc
export ROS_LOCALHOST_ONLY=1
source /opt/ros/foxy/setup.bash
```

说明：

- 这样可以避免 DDS 误选 `wlan0 + IPv6`
- 小车端 WiFi 建议保持关闭
- 小车端 ROS2 节点只做本机通信，PC 与小车之间走 HTTP，不走跨机 ROS2

## 当前相机推荐启动命令

不要再使用默认的 `dabai.launch.py` 无参启动。当前推荐使用低负载模式：

```bash
bash --noprofile --norc
export ROS_LOCALHOST_ONLY=1
source /opt/ros/foxy/setup.bash
ros2 launch orbbec_camera dabai.launch.py \
  enable_ir:=false \
  enable_point_cloud:=false \
  enable_colored_point_cloud:=false \
  enable_d2c_viewer:=false \
  color_fps:=15 \
  depth_fps:=15
```

当前这组参数已经验证会生效：

- color = `15 FPS`
- depth = `15 FPS`
- `IR` 关闭

## 当前推荐的小车端客户端启动命令

```bash
bash --noprofile --norc
export ROS_LOCALHOST_ONLY=1
source /opt/ros/foxy/setup.bash
source ~/venvs/internnav_limo/bin/activate
cd /home/agilex/InternNav/realworld
python http_internvla_client.py \
  --server_url http://192.168.100.10:5801/eval_dual \
  --instruction "The chair is in front of you. Move toward the chair and stop near it." \
  --no-use_compressed_rgb \
  --no-use_compressed_depth \
  --rgb_topic /camera/color/image_raw \
  --depth_topic /camera/depth/image_raw \
  --sync_queue_size 10 \
  --sync_slop 0.3 \
  --frame_process_interval 0.3 \
  --reuse_depth_max_age 1.0
```

参数含义：

- `--no-use_compressed_rgb` / `--no-use_compressed_depth`
  - 当前 Orbbec 实测先用 raw 话题更稳
- `--sync_queue_size 10`
  - 使用近似时间同步时保留小缓冲
- `--sync_slop 0.3`
  - RGB/Depth 最大允许时间差 `0.3s`
- `--frame_process_interval 0.3`
  - 客户端低频处理，减轻小车端负担
- `--reuse_depth_max_age 1.0`
  - depth 短时断流时，允许复用最近 1 秒内的有效 depth，避免立刻停住

## 当前完整复现顺序

### 1. PC 端启动推理服务

```bash
source /home/whs/anaconda3/etc/profile.d/conda.sh
conda activate internnav
cd /home/whs/wanghaosen/code/InternNav
TOKENIZERS_PARALLELISM=false python scripts/realworld/http_internvla_server.py \
  --device cuda:0 \
  --model_path checkpoints/InternVLA-N1 \
  --resize_w 384 \
  --resize_h 384 \
  --num_history 8 \
  --plan_step_gap 4
```

### 2. 小车端启动底盘

```bash
bash --noprofile --norc
export ROS_LOCALHOST_ONLY=1
source /opt/ros/foxy/setup.bash
ros2 launch limo_base limo_base.launch.py
```

### 3. 小车端启动相机

```bash
bash --noprofile --norc
export ROS_LOCALHOST_ONLY=1
source /opt/ros/foxy/setup.bash
ros2 launch orbbec_camera dabai.launch.py \
  enable_ir:=false \
  enable_point_cloud:=false \
  enable_colored_point_cloud:=false \
  enable_d2c_viewer:=false \
  color_fps:=15 \
  depth_fps:=15
```

### 4. 小车端确认关键话题

```bash
bash --noprofile --norc
export ROS_LOCALHOST_ONLY=1
source /opt/ros/foxy/setup.bash
ros2 topic list
```

至少应看到：

- `/camera/color/image_raw`
- `/camera/depth/image_raw`
- `/odom`
- `/cmd_vel`

### 5. 小车端启动客户端

```bash
bash --noprofile --norc
export ROS_LOCALHOST_ONLY=1
source /opt/ros/foxy/setup.bash
source ~/venvs/internnav_limo/bin/activate
cd /home/agilex/InternNav/realworld
python http_internvla_client.py \
  --server_url http://192.168.100.10:5801/eval_dual \
  --instruction "The chair is in front of you. Move toward the chair and stop near it." \
  --no-use_compressed_rgb \
  --no-use_compressed_depth \
  --rgb_topic /camera/color/image_raw \
  --depth_topic /camera/depth/image_raw \
  --sync_queue_size 10 \
  --sync_slop 0.3 \
  --frame_process_interval 0.3 \
  --reuse_depth_max_age 1.0
```

## 当前推荐验收标准

这轮最新稳定版本建议用下面几条验收：

- 客户端终端里 `idx` 持续增长，不停在 `0/1/4`
- `client_runtime.log` 中 `depth_count` 持续增长
- `client_runtime.log` 中即使出现 `Reusing recent depth ...`，系统也仍继续推进
- `server_runtime.log` 持续出现 `[TRACKING_TARGET]`
- 小车能持续向前靠近，而不是只动一下就停

## 当前日志位置

PC 端日志统一保存到：

- `/home/whs/wanghaosen/code/InternNav/test_data/<timestamp>/client_runtime.log`
- `/home/whs/wanghaosen/code/InternNav/test_data/<timestamp>/server_runtime.log`

查看最新一轮：

```bash
cd /home/whs/wanghaosen/code/InternNav/test_data
ls -1dt * | head
```

跟踪最新客户端日志：

```bash
tail -f /home/whs/wanghaosen/code/InternNav/test_data/最新目录/client_runtime.log
```

跟踪最新服务端日志：

```bash
tail -f /home/whs/wanghaosen/code/InternNav/test_data/最新目录/server_runtime.log
```
