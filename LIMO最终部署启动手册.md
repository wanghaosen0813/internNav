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
