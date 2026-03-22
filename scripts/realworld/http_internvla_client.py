import argparse
import copy
import io
import json
import math
import threading
import time
from collections import deque
from enum import Enum

import numpy as np
import rclpy
import requests
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry, Path
from PIL import Image as PIL_Image, ImageDraw
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

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


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--server_url', type=str, default='http://127.0.0.1:5801/eval_dual')
    parser.add_argument('--instruction', type=str, default=DEFAULT_INSTRUCTION)
    parser.add_argument('--rgb_topic', type=str, default='/camera/color/image_raw')
    parser.add_argument('--depth_topic', type=str, default='/camera/depth/image_raw')
    parser.add_argument('--odom_topic', type=str, default='/odom')
    parser.add_argument('--cmd_vel_topic', type=str, default='/cmd_vel')
    parser.add_argument('--sync_queue_size', type=int, default=5)
    parser.add_argument('--sync_slop', type=float, default=0.1)
    parser.add_argument('--turn_in_place_omega', type=float, default=0.5)
    parser.add_argument('--turn_in_place_linear_threshold', type=float, default=0.01)
    parser.add_argument('--turn_in_place_angular_deadband', type=float, default=0.02)
    parser.add_argument('--turn_direction_hold_sec', type=float, default=0.6)
    parser.add_argument('--debug_image_topic', type=str, default='/internnav/debug_image')
    parser.add_argument('--debug_path_topic', type=str, default='/internnav/debug_path')
    parser.add_argument('--frame_process_interval', type=float, default=0.3)
    return parser.parse_args()


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
    http_idx += 1
    if http_idx == 0:
        first_running_time = time.time()
    print(f'idx: {http_idx} after http {time.time() - start}')

    return json.loads(response.text)


def control_thread():
    global desired_v, desired_w
    while True:
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

    while True:
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
        for odom in manager.odom_queue:
            diff = abs(odom[0] - rgb_time)
            if diff < min_diff:
                min_diff = diff
                odom_infer = copy.deepcopy(odom[1])
        odom_rw_lock.release_read()

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
            print(
                f'skip planning. odom_infer: {odom_infer is not None} rgb_bytes: {rgb_bytes is not None} depth_bytes: {depth_bytes is not None}'
            )
            time.sleep(0.1)

        time.sleep(max(0, desired_time - (time.time() - start_time)))


class LimoManager(Node):
    def __init__(self, args):
        super().__init__('limo_manager')
        self.server_url = args.server_url
        self.instruction = args.instruction
        self.turn_in_place_omega = args.turn_in_place_omega
        self.turn_in_place_linear_threshold = args.turn_in_place_linear_threshold
        self.turn_in_place_angular_deadband = args.turn_in_place_angular_deadband
        self.turn_direction_hold_sec = args.turn_direction_hold_sec
        self.frame_process_interval = args.frame_process_interval

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

        rgb_sub = Subscriber(self, Image, args.rgb_topic, qos_profile=qos_profile)
        depth_sub = Subscriber(self, Image, args.depth_topic, qos_profile=qos_profile)
        self.syncronizer = ApproximateTimeSynchronizer([rgb_sub, depth_sub], args.sync_queue_size, args.sync_slop)
        self.syncronizer.registerCallback(self.rgb_depth_callback)
        self.odom_sub = self.create_subscription(Odometry, args.odom_topic, self.odom_callback, qos_profile)
        self.control_pub = self.create_publisher(Twist, args.cmd_vel_topic, 5)
        self.debug_image_pub = self.create_publisher(Image, args.debug_image_topic, 5)
        self.debug_path_pub = self.create_publisher(Path, args.debug_path_topic, 5)

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

        self.get_logger().info(f'RGB topic: {args.rgb_topic}')
        self.get_logger().info(f'Depth topic: {args.depth_topic}')
        self.get_logger().info(f'Odom topic: {args.odom_topic}')
        self.get_logger().info(f'CmdVel topic: {args.cmd_vel_topic}')
        self.get_logger().info(f'Server URL: {self.server_url}')
        self.get_logger().info(f'Debug image topic: {args.debug_image_topic}')
        self.get_logger().info(f'Debug path topic: {args.debug_path_topic}')
        self.get_logger().info(f'Frame process interval: {self.frame_process_interval}s')



    def rgb_depth_callback(self, rgb_msg, depth_msg):
        rgb_time = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec / 1.0e9
        if (
            self.last_processed_frame_time > 0.0
            and rgb_time - self.last_processed_frame_time < self.frame_process_interval
        ):
            return

        raw_image = self.cv_bridge.imgmsg_to_cv2(rgb_msg, 'rgb8')[:, :, :]
        self.rgb_image = raw_image
        image = PIL_Image.fromarray(self.rgb_image)
        image_bytes = io.BytesIO()
        image.save(image_bytes, format='JPEG')
        image_bytes.seek(0)

        raw_depth = self.cv_bridge.imgmsg_to_cv2(depth_msg, '16UC1')
        raw_depth = np.nan_to_num(raw_depth, nan=0.0, posinf=0.0, neginf=0.0)
        self.depth_image = raw_depth.astype(np.float32) / 1000.0
        self.depth_image[self.depth_image < 0] = 0
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
        self.depth_time = depth_msg.header.stamp.sec + depth_msg.header.stamp.nanosec / 1.0e9
        self.last_depth_time = self.depth_time
        rgb_depth_rw_lock.release_write()

        self.last_processed_frame_time = rgb_time
        self.new_vis_image_arrived = True
        self.new_image_arrived = True

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
        self.control_pub.publish(request)


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
        if manager is not None:
            manager.destroy_node()
        rclpy.shutdown()
