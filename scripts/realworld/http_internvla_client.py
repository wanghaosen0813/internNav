import argparse
import copy
import io
import json
import math
import subprocess
import threading
import time

import cv2
from collections import deque
from enum import Enum

import numpy as np
import rclpy
import requests
import message_filters
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from PIL import Image as PIL_Image, ImageDraw
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

from controllers import Mpc_controller, PID_controller
from thread_utils import ReadWriteLock

DEFAULT_INSTRUCTION = (
    "Turn around and walk out of this office. Turn towards your slight right at the chair. "
    "Move forward to the walkway and go near the red bin. You can see an open door on your right side, "
    "go inside the open door. Stop at the computer monitor"
)

frame_data = {}
frame_idx = 0


class ControlMode(Enum):
    PID_MODE = 1
    MPC_MODE = 2


policy_init = True
mpc = None
pid = PID_controller(Kp_trans=1.5, Kd_trans=0.0, Kp_yaw=0.9, Kd_yaw=0.0, max_v=0.10, max_w=0.2)
http_idx = -1
first_running_time = 0.0
last_pixel_goal = None
last_s2_step = -1
manager = None
current_control_mode = ControlMode.MPC_MODE
trajs_in_world = None


desired_v, desired_w = 0.0, 0.0
rgb_depth_rw_lock = ReadWriteLock()
odom_rw_lock = ReadWriteLock()
mpc_rw_lock = ReadWriteLock()
stop_event = threading.Event()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--server_url', type=str, default='http://127.0.0.1:5801/eval_dual')
    parser.add_argument('--instruction', type=str, default=DEFAULT_INSTRUCTION)
    parser.add_argument('--rgb_topic', type=str, default=None)
    parser.add_argument('--depth_topic', type=str, default=None)
    parser.add_argument('--odom_topic', type=str, default='/odom')
    parser.add_argument('--cmd_vel_topic', type=str, default='/cmd_vel')
    parser.add_argument('--sync_queue_size', type=int, default=20)
    parser.add_argument('--sync_slop', type=float, default=0.3)
    parser.add_argument('--use_compressed_rgb', dest='use_compressed_rgb', action='store_true')
    parser.add_argument('--no-use_compressed_rgb', dest='use_compressed_rgb', action='store_false')
    parser.add_argument('--use_compressed_depth', dest='use_compressed_depth', action='store_true')
    parser.add_argument('--no-use_compressed_depth', dest='use_compressed_depth', action='store_false')
    parser.set_defaults(use_compressed_rgb=True, use_compressed_depth=True)
    parser.add_argument('--turn_in_place_omega', type=float, default=0.5)
    parser.add_argument('--turn_in_place_linear_threshold', type=float, default=0.01)
    parser.add_argument('--turn_in_place_angular_deadband', type=float, default=0.02)
    parser.add_argument('--turn_direction_hold_sec', type=float, default=0.6)
    parser.add_argument('--debug_image_topic', type=str, default='/internnav/debug_image')
    parser.add_argument('--debug_path_topic', type=str, default='/internnav/debug_path')
    parser.add_argument('--frame_process_interval', type=float, default=0.3)
    parser.add_argument('--rgb_callback_min_interval', type=float, default=0.1)
    parser.add_argument('--reuse_depth_max_age', type=float, default=1.0)
    return parser.parse_args()


def build_client_log_url(server_url):
    return server_url.rsplit('/', 1)[0] + '/client_log'


def post_client_log(url, message):
    try:
        requests.post(url, json={'message': message}, timeout=2)
    except Exception:
        pass


def _sample_topic_hz(topic, duration, client_log_url):
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'hz', '--window', '50', topic],
            capture_output=True, text=True, timeout=duration,
        )
        output = (result.stdout or result.stderr).strip()
    except subprocess.TimeoutExpired as exc:
        output = (exc.stdout or '').strip()
    except Exception as exc:
        output = f'error: {exc}'
    post_client_log(client_log_url, f'[topic_hz] {topic}: {output}')


def start_topic_hz_logging(rgb_topic, depth_topic, client_log_url, duration=10):
    for topic in (rgb_topic, depth_topic):
        threading.Thread(
            target=_sample_topic_hz,
            args=(topic, duration, client_log_url),
            daemon=True,
        ).start()


def dual_sys_eval(image_bytes, depth_bytes, url, instruction):
    global policy_init, http_idx, first_running_time
    data = {
        'reset': policy_init,
        'idx': http_idx,
        'instruction': instruction,
    }
    json_data = json.dumps(data)

    policy_init = False
    files = {
        'image': ('rgb_image', image_bytes, 'image/jpeg'),
        'depth': ('depth_image', depth_bytes, 'image/png'),
    }
    start = time.time()
    response = requests.post(url, files=files, data={'json': json_data}, timeout=100)
    response.raise_for_status()
    print(f'response {response.text}')
    if manager is not None:
        manager.emit_client_log(f'response {response.text}')
    http_idx += 1
    if http_idx == 0:
        first_running_time = time.time()
    print(f'idx: {http_idx} after http {time.time() - start}')
    if manager is not None:
        manager.emit_client_log(f'idx: {http_idx} after http {time.time() - start}')

    return json.loads(response.text)


def control_thread():
    global desired_v, desired_w
    while not stop_event.is_set():
        global current_control_mode
        if current_control_mode == ControlMode.MPC_MODE:
            odom_rw_lock.acquire_read()
            odom = manager.odom.copy() if manager.odom else None
            odom_rw_lock.release_read()
            if mpc is not None and manager is not None and odom is not None:
                local_mpc = mpc
                opt_u_controls, opt_x_states = local_mpc.solve(np.array(odom))
                v, w = opt_u_controls[0, 0], opt_u_controls[0, 1]

                desired_v, desired_w = v, w
                manager.move(v, 0.0, w, allow_turn_boost=False)
        elif current_control_mode == ControlMode.PID_MODE:
            odom_rw_lock.acquire_read()
            odom = manager.odom.copy() if manager.odom else None
            odom_rw_lock.release_read()
            homo_odom = manager.homo_odom.copy() if manager.homo_odom is not None else None
            vel = manager.vel.copy() if manager.vel is not None else None
            homo_goal = manager.homo_goal.copy() if manager.homo_goal is not None else None

            if homo_odom is not None and vel is not None and homo_goal is not None:
                v, w, e_p, e_r = pid.solve(homo_odom, homo_goal, vel)
                if v < 0.0:
                    v = 0.0
                desired_v, desired_w = v, w
                manager.move(v, 0.0, w, allow_turn_boost=True)

        time.sleep(0.1)


def planning_thread():
    global trajs_in_world

    while not stop_event.is_set():
        start_time = time.time()
        desired_time = 0.3
        time.sleep(0.05)

        if not manager.new_image_arrived:
            time.sleep(0.01)
            continue
        manager.new_image_arrived = False
        rgb_depth_rw_lock.acquire_read()
        rgb_bytes = copy.deepcopy(manager.rgb_bytes)
        depth_bytes = copy.deepcopy(manager.depth_bytes)
        infer_rgb = copy.deepcopy(manager.rgb_image)
        infer_depth = copy.deepcopy(manager.depth_image)
        rgb_time = manager.rgb_time
        rgb_depth_rw_lock.release_read()
        odom_rw_lock.acquire_read()
        min_diff = 1e10
        odom_infer = None
        latest_odom = copy.deepcopy(manager.odom) if manager.odom is not None else None
        for odom in manager.odom_queue:
            diff = abs(odom[0] - rgb_time)
            if diff < min_diff:
                min_diff = diff
                odom_infer = copy.deepcopy(odom[1])
        odom_rw_lock.release_read()
        if odom_infer is None and latest_odom is not None:
            odom_infer = latest_odom
            manager.emit_client_log('[odom] Fallback to latest odom for planning')

        if odom_infer is not None and rgb_bytes is not None and depth_bytes is not None:
            global frame_data
            frame_data[http_idx] = {
                'infer_rgb': copy.deepcopy(infer_rgb),
                'infer_depth': copy.deepcopy(infer_depth),
                'infer_odom': copy.deepcopy(odom_infer),
            }
            if len(frame_data) > 100:
                del frame_data[min(frame_data.keys())]
            response = dual_sys_eval(rgb_bytes, depth_bytes, manager.server_url, manager.instruction)
            global current_control_mode
            if 'trajectory' in response:
                trajectory = response['trajectory']
                trajs_in_world = []
                odom = odom_infer
                traj_len = np.linalg.norm(trajectory[-1][:2])
                print(f'traj len {traj_len}')
                manager.emit_client_log(f'traj len {traj_len}')
                for i, traj in enumerate(trajectory):
                    if i < 3:
                        continue
                    x_, y_, yaw_ = odom[0], odom[1], odom[2]

                    w_T_b = np.array(
                        [
                            [np.cos(yaw_), -np.sin(yaw_), 0, x_],
                            [np.sin(yaw_), np.cos(yaw_), 0, y_],
                            [0.0, 0.0, 1.0, 0],
                            [0.0, 0.0, 0.0, 1.0],
                        ]
                    )
                    w_P = (w_T_b @ (np.array([traj[0], traj[1], 0.0, 1.0])).T)[:2]
                    trajs_in_world.append(w_P)
                trajs_in_world = np.array(trajs_in_world)
                print(f'{time.time()} update traj')
                manager.emit_client_log(f'{time.time()} update traj')

                manager.last_trajs_in_world = trajs_in_world
                manager.publish_debug_path(trajs_in_world)
                manager.publish_debug_image(infer_rgb, response)
                mpc_rw_lock.acquire_write()
                global mpc
                if mpc is None:
                    mpc = Mpc_controller(np.array(trajs_in_world))
                else:
                    mpc.update_ref_traj(np.array(trajs_in_world))
                manager.request_cnt += 1
                mpc_rw_lock.release_write()
                current_control_mode = ControlMode.MPC_MODE
            elif 'discrete_action' in response:
                actions = response['discrete_action']
                if actions != [5] and actions != [9]:
                    manager.incremental_change_goal(actions)
                    manager.publish_debug_image(infer_rgb, response)
                    current_control_mode = ControlMode.PID_MODE
        else:
            skip_msg = f'skip planning. odom_infer: {odom_infer is not None} rgb_bytes: {rgb_bytes is not None} depth_bytes: {depth_bytes is not None}'
            print(skip_msg)
            manager.emit_client_log(skip_msg)
            time.sleep(0.1)

        time.sleep(max(0, desired_time - (time.time() - start_time)))


class LimoManager(Node):
    def __init__(self, args):
        super().__init__('limo_manager')
        self.server_url = args.server_url
        self.client_log_url = build_client_log_url(args.server_url)
        self.instruction = args.instruction
        self.turn_in_place_omega = args.turn_in_place_omega
        self.turn_in_place_linear_threshold = args.turn_in_place_linear_threshold
        self.turn_in_place_angular_deadband = args.turn_in_place_angular_deadband
        self.turn_direction_hold_sec = args.turn_direction_hold_sec
        self.frame_process_interval = args.frame_process_interval
        self.rgb_callback_min_interval = max(0.0, args.rgb_callback_min_interval)
        self.reuse_depth_max_age = max(0.0, args.reuse_depth_max_age)

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

        self.use_compressed_rgb = args.use_compressed_rgb
        self.use_compressed_depth = args.use_compressed_depth
        self.sync_slop = args.sync_slop
        self.frame_pair_queue_size = max(2, args.sync_queue_size)
        self.rgb_topic = args.rgb_topic or (
            '/camera/color/image_raw/compressed' if self.use_compressed_rgb else '/camera/color/image_raw'
        )
        self.depth_topic = args.depth_topic or (
            '/camera/depth/image_raw/compressedDepth' if self.use_compressed_depth else '/camera/depth/image_raw'
        )

        rgb_msg_type = CompressedImage if self.use_compressed_rgb else Image
        depth_msg_type = CompressedImage if self.use_compressed_depth else Image

        self.rgb_sub = message_filters.Subscriber(self, rgb_msg_type, self.rgb_topic, qos_profile=qos_profile)
        self.depth_sub = message_filters.Subscriber(self, depth_msg_type, self.depth_topic, qos_profile=qos_profile)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=self.frame_pair_queue_size, slop=self.sync_slop
        )
        self.sync.registerCallback(self.sync_callback)
        self.rgb_sub.registerCallback(self.rgb_monitor_callback)
        self.depth_sub.registerCallback(self.depth_monitor_callback)
        self.odom_sub = self.create_subscription(Odometry, args.odom_topic, self.odom_callback, qos_profile)
        self.control_pub = self.create_publisher(Twist, args.cmd_vel_topic, 5)
        self.debug_image_pub = self.create_publisher(Image, args.debug_image_topic, 5)
        self.debug_path_pub = self.create_publisher(Path, args.debug_path_topic, 5)
        self.input_health_timer = self.create_timer(1.0, self.monitor_input_health)

        self.cv_bridge = CvBridge()
        self.rgb_image = None
        self.rgb_bytes = None
        self.depth_image = None
        self.depth_bytes = None
        self.new_image_arrived = False
        self.new_vis_image_arrived = False
        self.rgb_time = 0.0
        self.depth_time = 0.0
        self.last_rgb_time = 0.0
        self.last_depth_time = 0.0
        self.last_processed_frame_time = 0.0
        self.last_sync_receive_time = None
        self.last_rgb_receive_time = None
        self.last_depth_receive_time = None
        self.last_rgb_msg_stamp = None
        self.last_depth_msg_stamp = None
        self.last_rgb_msg_gap = None
        self.last_depth_msg_gap = None
        self.last_sync_pair_dt = None
        self.last_rgb_reuse_time = 0.0
        self.last_reused_depth_age = None
        self.reuse_depth_count = 0
        self.last_good_depth_image = None
        self.last_good_depth_bytes = None
        self.last_good_depth_time = None
        self.rgb_frame_count = 0
        self.depth_frame_count = 0
        self.sync_frame_count = 0
        self.sync_drop_count = 0
        self.processing_sync = False

        self.odom = None
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.request_cnt = 0
        self.odom_cnt = 0
        self.odom_queue = deque(maxlen=50)
        self.odom_timestamp = 0.0

        self.last_s2_step = -1
        self.last_trajs_in_world = None
        self.last_all_trajs_in_world = None
        self.homo_odom = None
        self.homo_goal = None
        self.vel = None
        self.last_turn_sign = 0
        self.last_turn_sign_time = 0.0
        self.debug_log_times = {}

        self.get_logger().info(f'RGB topic: {self.rgb_topic}')
        self.emit_client_log(f'RGB topic: {self.rgb_topic}')
        self.get_logger().info(f'Depth topic: {self.depth_topic}')
        self.emit_client_log(f'Depth topic: {self.depth_topic}')
        self.get_logger().info(f'Odom topic: {args.odom_topic}')
        self.emit_client_log(f'Odom topic: {args.odom_topic}')
        self.get_logger().info(f'CmdVel topic: {args.cmd_vel_topic}')
        self.emit_client_log(f'CmdVel topic: {args.cmd_vel_topic}')
        self.get_logger().info(f'Server URL: {self.server_url}')
        self.emit_client_log(f'Server URL: {self.server_url}')
        self.get_logger().info(f'Debug image topic: {args.debug_image_topic}')
        self.emit_client_log(f'Debug image topic: {args.debug_image_topic}')
        self.get_logger().info(f'Debug path topic: {args.debug_path_topic}')
        self.emit_client_log(f'Debug path topic: {args.debug_path_topic}')
        self.get_logger().info(f'Frame process interval: {self.frame_process_interval}s')
        self.emit_client_log(f'Frame process interval: {self.frame_process_interval}s')
        self.get_logger().info(f'RGB callback min interval: {self.rgb_callback_min_interval}s')
        self.emit_client_log(f'RGB callback min interval: {self.rgb_callback_min_interval}s')
        self.get_logger().info(f'Reuse depth max age: {self.reuse_depth_max_age}s')
        self.emit_client_log(f'Reuse depth max age: {self.reuse_depth_max_age}s')
        self.get_logger().info(f'Use compressed RGB: {self.use_compressed_rgb}')
        self.emit_client_log(f'Use compressed RGB: {self.use_compressed_rgb}')
        self.get_logger().info(f'Use compressed Depth: {self.use_compressed_depth}')
        self.emit_client_log(f'Use compressed Depth: {self.use_compressed_depth}')
        self.get_logger().info(f'RGB/Depth pairing slop: {self.sync_slop}s')
        self.emit_client_log(f'RGB/Depth pairing slop: {self.sync_slop}s')
        self.get_logger().info(f'Frame pair queue size: {self.frame_pair_queue_size}')
        self.emit_client_log(f'Frame pair queue size: {self.frame_pair_queue_size}')

        start_topic_hz_logging(self.rgb_topic, self.depth_topic, self.client_log_url, duration=10)

    @staticmethod
    def _stamp_to_sec(stamp):
        return stamp.sec + stamp.nanosec / 1.0e9

    def _log_throttled(self, key, period_sec, message, level='info'):
        now = time.time()
        last_time = self.debug_log_times.get(key, 0.0)
        if now - last_time < period_sec:
            return
        self.debug_log_times[key] = now
        if level != 'info':
            message = f'[WARN] {message}'
        self.get_logger().info(message)
        self.emit_client_log(message)

    def emit_client_log(self, message):
        post_client_log(self.client_log_url, message)

    def _decode_rgb(self, rgb_msg):
        if self.use_compressed_rgb:
            image = None
            try:
                image = self.cv_bridge.compressed_imgmsg_to_cv2(rgb_msg, desired_encoding='rgb8')
            except Exception:
                image = None
            if image is None:
                np_arr = np.frombuffer(rgb_msg.data, np.uint8)
                bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if bgr is None:
                    raise ValueError('Failed to decode compressed RGB image')
                image = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            return image[:, :, :]
        return self.cv_bridge.imgmsg_to_cv2(rgb_msg, 'rgb8')[:, :, :]

    def _decode_depth(self, depth_msg):
        if self.use_compressed_depth:
            depth = None
            try:
                depth = self.cv_bridge.compressed_imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            except Exception:
                depth = None
            if depth is None:
                data = depth_msg.data
                png_magic = b"\x89PNG\r\n\x1a\n"
                start = data.find(png_magic)
                if start != -1:
                    np_arr = np.frombuffer(data[start:], np.uint8)
                    depth = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            if depth is None:
                raise ValueError('Failed to decode compressed depth image')
            return depth
        return self.cv_bridge.imgmsg_to_cv2(depth_msg, 'passthrough')

    def rgb_monitor_callback(self, rgb_msg):
        now = time.time()
        rgb_time = self._stamp_to_sec(rgb_msg.header.stamp)
        if self.last_rgb_msg_stamp is not None:
            self.last_rgb_msg_gap = rgb_time - self.last_rgb_msg_stamp
        self.last_rgb_msg_stamp = rgb_time
        self.last_rgb_receive_time = now
        self.rgb_frame_count += 1
        gap_str = 'None' if self.last_rgb_msg_gap is None else f'{self.last_rgb_msg_gap:.3f}s'
        self._log_throttled(
            'rgb_heartbeat',
            5.0,
            f'[frame] RGB heartbeat count={self.rgb_frame_count} last_rgb_stamp={rgb_time:.3f} last_rgb_stamp_gap={gap_str}',
        )

        if self.processing_sync:
            return
        if self.last_good_depth_bytes is None or self.last_good_depth_image is None or self.last_good_depth_time is None:
            return
        if self.reuse_depth_max_age <= 0.0:
            return
        if self.last_processed_frame_time > 0.0 and rgb_time - self.last_processed_frame_time < self.frame_process_interval:
            return
        if self.last_rgb_reuse_time > 0.0 and now - self.last_rgb_reuse_time < self.rgb_callback_min_interval:
            return
        depth_age = rgb_time - self.last_good_depth_time
        if depth_age <= self.sync_slop:
            return
        if depth_age > self.reuse_depth_max_age:
            return

        try:
            raw_image = self._decode_rgb(rgb_msg)
            image = PIL_Image.fromarray(raw_image)
            image_bytes = io.BytesIO()
            image.save(image_bytes, format='JPEG')
            image_bytes.seek(0)

            rgb_depth_rw_lock.acquire_write()
            self.rgb_image = raw_image
            self.rgb_bytes = image_bytes
            self.rgb_time = rgb_time
            self.last_rgb_time = rgb_time
            self.depth_image = self.last_good_depth_image.copy()
            self.depth_bytes = copy.deepcopy(self.last_good_depth_bytes)
            self.depth_time = self.last_good_depth_time
            self.last_depth_time = self.last_good_depth_time
            rgb_depth_rw_lock.release_write()

            self.last_processed_frame_time = rgb_time
            self.last_rgb_reuse_time = now
            self.last_reused_depth_age = depth_age
            self.reuse_depth_count += 1
            self.new_vis_image_arrived = True
            self.new_image_arrived = True
            self._log_throttled(
                'reuse_depth',
                1.0,
                f'[frame] Reusing recent depth. reuse_count={self.reuse_depth_count} depth_age={depth_age:.3f}s rgb_t={rgb_time:.3f} depth_t={self.last_good_depth_time:.3f}',
                level='warn',
            )
        except Exception as exc:
            self._log_throttled('reuse_depth_error', 2.0, f'[frame] Failed RGB+recent-depth fallback: {exc}', level='warn')

    def depth_monitor_callback(self, depth_msg):
        now = time.time()
        depth_time = self._stamp_to_sec(depth_msg.header.stamp)
        if self.last_depth_msg_stamp is not None:
            self.last_depth_msg_gap = depth_time - self.last_depth_msg_stamp
        self.last_depth_msg_stamp = depth_time
        self.last_depth_receive_time = now
        self.depth_frame_count += 1
        gap_str = 'None' if self.last_depth_msg_gap is None else f'{self.last_depth_msg_gap:.3f}s'
        self._log_throttled(
            'depth_heartbeat',
            2.0,
            f'[frame] Depth heartbeat count={self.depth_frame_count} last_depth_stamp={depth_time:.3f} last_depth_stamp_gap={gap_str}',
        )

    def sync_callback(self, rgb_msg, depth_msg):
        rgb_time = self._stamp_to_sec(rgb_msg.header.stamp)
        depth_time = self._stamp_to_sec(depth_msg.header.stamp)
        pair_time = max(rgb_time, depth_time)
        pair_dt = abs(rgb_time - depth_time)

        if self.processing_sync:
            self.sync_drop_count += 1
            self._log_throttled(
                'sync_busy_drop',
                2.0,
                f'[frame] Sync busy, dropping synchronized frame. sync_dropped={self.sync_drop_count} pair_dt={pair_dt:.3f}s',
                level='warn',
            )
            return

        if (
            self.last_processed_frame_time > 0.0
            and pair_time - self.last_processed_frame_time < self.frame_process_interval
        ):
            self._log_throttled(
                'waiting_interval',
                2.0,
                f'[frame] Waiting for next process window. since_last={pair_time - self.last_processed_frame_time:.3f}s target={self.frame_process_interval:.3f}s pair_dt={pair_dt:.3f}s',
            )
            return

        self.processing_sync = True
        self.last_sync_receive_time = time.time()
        self.last_sync_pair_dt = pair_dt
        self._log_throttled(
            'pair_success',
            1.0,
            f'[frame] Paired RGB/Depth successfully. pair_dt={pair_dt:.3f}s sync_count={self.sync_frame_count + 1} sync_dropped={self.sync_drop_count}',
        )
        try:
            raw_image = self._decode_rgb(rgb_msg)
            self.rgb_image = raw_image
            image = PIL_Image.fromarray(self.rgb_image)
            image_bytes = io.BytesIO()
            image.save(image_bytes, format='JPEG')
            image_bytes.seek(0)

            raw_depth = self._decode_depth(depth_msg)
            raw_depth = np.nan_to_num(raw_depth, nan=0.0, posinf=0.0, neginf=0.0)
            if raw_depth.dtype == np.uint16:
                depth_m = raw_depth.astype(np.float32) / 1000.0
            else:
                depth_m = raw_depth.astype(np.float32)
            depth_m[depth_m < 0] = 0
            self.depth_image = depth_m
            depth = (np.clip(self.depth_image * 10000.0, 0, 65535)).astype(np.uint16)
            depth = PIL_Image.fromarray(depth)
            depth_bytes = io.BytesIO()
            depth.save(depth_bytes, format='PNG')
            depth_bytes.seek(0)

            rgb_depth_rw_lock.acquire_write()
            self.rgb_bytes = image_bytes
            self.rgb_time = rgb_time
            self.last_rgb_time = self.rgb_time
            self.depth_bytes = depth_bytes
            self.depth_time = depth_time
            self.last_depth_time = self.depth_time
            rgb_depth_rw_lock.release_write()

            self.last_good_depth_image = depth_m.copy()
            self.last_good_depth_bytes = copy.deepcopy(depth_bytes)
            self.last_good_depth_time = depth_time
            self.last_processed_frame_time = pair_time
            self.new_vis_image_arrived = True
            self.new_image_arrived = True
            self.sync_frame_count += 1
        except Exception as exc:
            self._log_throttled('sync_process_error', 2.0, f'[frame] Failed to process synchronized RGB-D pair: {exc}', level='warn')
        finally:
            self.processing_sync = False

    def monitor_input_health(self):
        if stop_event.is_set():
            return
        now = time.time()
        last_rgb_age = -1.0 if self.last_rgb_receive_time is None else now - self.last_rgb_receive_time
        last_depth_age = -1.0 if self.last_depth_receive_time is None else now - self.last_depth_receive_time
        last_sync_age = -1.0 if self.last_sync_receive_time is None else now - self.last_sync_receive_time
        pair_dt = 'None' if self.last_sync_pair_dt is None else f'{self.last_sync_pair_dt:.3f}s'
        depth_gap = 'None' if self.last_depth_msg_gap is None else f'{self.last_depth_msg_gap:.3f}s'
        rgb_gap = 'None' if self.last_rgb_msg_gap is None else f'{self.last_rgb_msg_gap:.3f}s'
        reused_depth_age = 'None' if self.last_reused_depth_age is None else f'{self.last_reused_depth_age:.3f}s'
        self._log_throttled(
            'input_health',
            2.0,
            f'[frame] Health rgb_count={self.rgb_frame_count} depth_count={self.depth_frame_count} sync_count={self.sync_frame_count} sync_dropped={self.sync_drop_count} reuse_depth_count={self.reuse_depth_count} last_rgb_age={last_rgb_age:.3f}s last_depth_age={last_depth_age:.3f}s last_sync_age={last_sync_age:.3f}s last_pair_dt={pair_dt} last_reused_depth_age={reused_depth_age} last_rgb_gap={rgb_gap} last_depth_gap={depth_gap}',
        )

    def publish_debug_image(self, rgb_image, response):
        if rgb_image is None:
            return

        debug_image = PIL_Image.fromarray(np.asarray(rgb_image, dtype=np.uint8)).convert('RGB')
        draw = ImageDraw.Draw(debug_image)

        overlay_lines = []
        if 'discrete_action' in response:
            overlay_lines.append(f"action: {response['discrete_action']}")
        if 'trajectory' in response:
            overlay_lines.append(f"traj_pts: {len(response['trajectory'])}")
        if 'pixel_goal' in response and len(response['pixel_goal']) == 2:
            pixel_x = int(response['pixel_goal'][0])
            pixel_y = int(response['pixel_goal'][1])
            cross_half = 8
            draw.ellipse((pixel_x - 6, pixel_y - 6, pixel_x + 6, pixel_y + 6), outline='red', width=3)
            draw.line((pixel_x - cross_half, pixel_y, pixel_x + cross_half, pixel_y), fill='red', width=3)
            draw.line((pixel_x, pixel_y - cross_half, pixel_x, pixel_y + cross_half), fill='red', width=3)
            overlay_lines.append(f'pixel_goal: ({pixel_x}, {pixel_y})')

        if overlay_lines:
            draw.rectangle((8, 8, 430, 32 + 22 * len(overlay_lines)), outline='yellow', width=2)
            for idx, line in enumerate(overlay_lines):
                draw.text((16, 16 + idx * 22), line, fill='yellow')

        msg = self.cv_bridge.cv2_to_imgmsg(np.array(debug_image), encoding='rgb8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'
        self.debug_image_pub.publish(msg)

    def publish_debug_path(self, traj_points):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'

        for point in traj_points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.debug_path_pub.publish(path_msg)

    def odom_callback(self, msg):
        self.odom_cnt += 1
        odom_rw_lock.acquire_write()
        zz = msg.pose.pose.orientation.z
        ww = msg.pose.pose.orientation.w
        yaw = math.atan2(2 * zz * ww, 1 - 2 * zz * zz)
        self.odom = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]
        self.odom_queue.append((time.time(), copy.deepcopy(self.odom)))
        self.odom_timestamp = time.time()
        self.linear_vel = msg.twist.twist.linear.x
        self.angular_vel = msg.twist.twist.angular.z
        odom_rw_lock.release_write()

        rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        self.homo_odom = np.eye(4)
        self.homo_odom[:2, :2] = rotation
        self.homo_odom[:2, 3] = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.vel = [msg.twist.twist.linear.x, msg.twist.twist.angular.z]

        if self.odom_cnt == 1:
            self.homo_goal = self.homo_odom.copy()

    def incremental_change_goal(self, actions):
        if self.homo_goal is None:
            raise ValueError('Please initialize homo_goal before change it!')
        homo_goal = self.homo_odom.copy()
        for each_action in actions:
            if each_action == 0:
                pass
            elif each_action == 1:
                yaw = math.atan2(homo_goal[1, 0], homo_goal[0, 0])
                homo_goal[0, 3] += 0.15 * np.cos(yaw)
                homo_goal[1, 3] += 0.15 * np.sin(yaw)
            elif each_action == 2:
                angle = math.radians(10)
                rotation_matrix = np.array(
                    [[math.cos(angle), -math.sin(angle), 0], [math.sin(angle), math.cos(angle), 0], [0, 0, 1]]
                )
                homo_goal[:3, :3] = np.dot(rotation_matrix, homo_goal[:3, :3])
            elif each_action == 3:
                angle = -math.radians(10.0)
                rotation_matrix = np.array(
                    [[math.cos(angle), -math.sin(angle), 0], [math.sin(angle), math.cos(angle), 0], [0, 0, 1]]
                )
                homo_goal[:3, :3] = np.dot(rotation_matrix, homo_goal[:3, :3])
        self.homo_goal = homo_goal

    def move(self, vx, vy, vyaw, allow_turn_boost=True):
        if stop_event.is_set() or not rclpy.ok():
            return
        request = Twist()
        request.linear.x = vx
        request.linear.y = 0.0

        # For LIMO, only apply aggressive turn-in-place boosting during search-like PID turning.
        if allow_turn_boost and abs(vx) < self.turn_in_place_linear_threshold and abs(vyaw) > self.turn_in_place_angular_deadband:
            now = time.time()
            turn_sign = 1 if vyaw >= 0.0 else -1
            if (
                self.last_turn_sign != 0
                and turn_sign != self.last_turn_sign
                and (now - self.last_turn_sign_time) < self.turn_direction_hold_sec
            ):
                turn_sign = self.last_turn_sign
            vyaw = turn_sign * max(abs(vyaw), self.turn_in_place_omega)
            self.last_turn_sign = turn_sign
            self.last_turn_sign_time = now
        else:
            self.last_turn_sign = 0

        request.angular.z = vyaw
        try:
            self.control_pub.publish(request)
        except Exception:
            return


if __name__ == '__main__':
    args = parse_args()
    control_thread_instance = threading.Thread(target=control_thread)
    planning_thread_instance = threading.Thread(target=planning_thread)
    control_thread_instance.daemon = True
    planning_thread_instance.daemon = True
    rclpy.init()

    try:
        manager = LimoManager(args)
        control_thread_instance.start()
        planning_thread_instance.start()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        time.sleep(0.2)
        if manager is not None:
            manager.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
