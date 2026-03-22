import argparse
import json
import os
import time
from datetime import datetime

import numpy as np

os.environ.setdefault('TOKENIZERS_PARALLELISM', 'false')

from flask import Flask, jsonify, request
from PIL import Image

from internnav.agent.internvla_n1_agent_realworld import InternVLAN1AsyncAgent

app = Flask(__name__)
idx = 0
start_time = time.time()
output_dir = ''
DEFAULT_INSTRUCTION = (
    "Turn around and walk out of this office. Turn towards your slight right at the chair. "
    "Move forward to the walkway and go near the red bin. You can see an open door on your right side, "
    "go inside the open door. Stop at the computer monitor"
)
run_log_path = ''
client_run_log_path = ''
last_state = None


def classify_json_output(json_output):
    if 'pixel_goal' in json_output:
        return 'FOUND_CHAIR_CANDIDATE'
    if 'trajectory' in json_output:
        return 'TRACKING_TARGET'
    if 'discrete_action' in json_output:
        actions = json_output['discrete_action']
        if actions and all(action in (2, 3) for action in actions):
            return 'SEARCHING_FOR_CHAIR'
        if 1 in actions:
            return 'MOVING_TOWARD_TARGET'
        return 'ACTION'
    return 'UNKNOWN_RESPONSE'


def summarize_json_output(json_output):
    state = classify_json_output(json_output)
    if state == 'FOUND_CHAIR_CANDIDATE':
        traj_pts = len(json_output.get('trajectory', []))
        return f"[FOUND_CHAIR_CANDIDATE] pixel_goal={json_output['pixel_goal']} traj_pts={traj_pts}"
    if state == 'TRACKING_TARGET':
        return f"[TRACKING_TARGET] traj_pts={len(json_output['trajectory'])}"
    if state == 'SEARCHING_FOR_CHAIR':
        return f"[SEARCHING_FOR_CHAIR] discrete_action={json_output['discrete_action']}"
    if state == 'MOVING_TOWARD_TARGET':
        return f"[MOVING_TOWARD_TARGET] discrete_action={json_output['discrete_action']}"
    if state == 'ACTION':
        return f"[ACTION] discrete_action={json_output['discrete_action']}"
    return f"[UNKNOWN_RESPONSE] {json_output}"


def log_server_event(message):
    global run_log_path
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    formatted = f'[{timestamp}] {message}'
    print(formatted)
    if run_log_path:
        with open(run_log_path, 'a', encoding='utf-8') as f:
            f.write(formatted + '\n')


def log_client_event(message):
    global client_run_log_path
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    formatted = f'[{timestamp}] {message}'
    print(formatted)
    if client_run_log_path:
        with open(client_run_log_path, 'a', encoding='utf-8') as f:
            f.write(formatted + '\n')


def log_server_state(json_output):
    global last_state
    state = classify_json_output(json_output)
    summary = summarize_json_output(json_output)
    if state != last_state:
        banner = '=' * 24
        log_server_event(f'{banner} [{state}] {banner}')
        last_state = state
    log_server_event(summary)


@app.route("/client_log", methods=['POST'])
def client_log():
    global client_run_log_path
    payload = request.get_json(silent=True) or {}
    message = payload.get('message')
    if not message:
        return jsonify({'status': 'ignored'}), 400
    if not client_run_log_path:
        client_run_log_path = os.path.join(agent.save_dir, 'client_runtime.log')
    log_client_event(message)
    return jsonify({'status': 'ok'})


@app.route("/eval_dual", methods=['POST'])
def eval_dual():
    global idx, output_dir, start_time, run_log_path, client_run_log_path, last_state
    start_time = time.time()

    image_file = request.files['image']
    depth_file = request.files['depth']
    json_data = request.form['json']
    data = json.loads(json_data)

    image = Image.open(image_file.stream)
    image = image.convert('RGB')
    image = np.asarray(image)

    depth = Image.open(depth_file.stream)
    depth = depth.convert('I')
    depth = np.asarray(depth)
    depth = depth.astype(np.float32) / 10000.0
    print(f"read http data cost {time.time() - start_time}")

    camera_pose = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    instruction = data.get('instruction', DEFAULT_INSTRUCTION)
    policy_init = data.get('reset', False)
    if policy_init:
        start_time = time.time()
        idx = 0
        output_dir = 'output/runs' + datetime.now().strftime('%m-%d-%H%M')
        os.makedirs(output_dir, exist_ok=True)
        print("init reset model!!!")
        agent.reset()
        run_log_path = os.path.join(agent.save_dir, 'server_runtime.log')
        client_run_log_path = os.path.join(agent.save_dir, 'client_runtime.log')
        last_state = None
        log_server_event(f'[START] instruction={instruction}')
        log_server_event(f'[START] runtime_log={run_log_path}')
        log_client_event(f'[START] runtime_log={client_run_log_path}')

    if not run_log_path:
        run_log_path = os.path.join(agent.save_dir, 'server_runtime.log')
    if not client_run_log_path:
        client_run_log_path = os.path.join(agent.save_dir, 'client_runtime.log')
        client_run_log_path = os.path.join(agent.save_dir, 'client_runtime.log')

    idx += 1

    look_down = False
    t0 = time.time()

    dual_sys_output = agent.step(
        image,
        depth,
        camera_pose,
        instruction,
        intrinsic=args.camera_intrinsic,
        look_down=look_down,
    )
    if dual_sys_output.output_action is not None and dual_sys_output.output_action == [5]:
        look_down = True
        dual_sys_output = agent.step(
            image,
            depth,
            camera_pose,
            instruction,
            intrinsic=args.camera_intrinsic,
            look_down=look_down,
        )

    json_output = {}
    if dual_sys_output.output_action is not None:
        json_output['discrete_action'] = dual_sys_output.output_action
    else:
        json_output['trajectory'] = dual_sys_output.output_trajectory.tolist()
        if dual_sys_output.output_pixel is not None:
            json_output['pixel_goal'] = dual_sys_output.output_pixel

    t1 = time.time()
    generate_time = t1 - t0
    print(f"dual sys step {generate_time}")
    print(f"json_output {json_output}")
    log_server_state(json_output)
    log_server_event(f'[RAW_RESPONSE] {json.dumps(json_output, ensure_ascii=False)}')
    return jsonify(json_output)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", type=str, default="cuda:0")
    parser.add_argument("--model_path", type=str, default="checkpoints/InternVLA-N1")
    parser.add_argument("--resize_w", type=int, default=384)
    parser.add_argument("--resize_h", type=int, default=384)
    parser.add_argument("--num_history", type=int, default=8)
    parser.add_argument("--plan_step_gap", type=int, default=4)
    args = parser.parse_args()

    args.camera_intrinsic = np.array(
        [
            [489.2552490234375, 0.0, 317.91510009765625, 0.0],
            [0.0, 489.2552490234375, 216.17910766601562, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    agent = InternVLAN1AsyncAgent(args)
    agent.step(
        np.zeros((480, 640, 3), dtype=np.uint8),
        np.zeros((480, 640), dtype=np.float32),
        np.eye(4),
        "hello",
        intrinsic=args.camera_intrinsic,
    )
    agent.reset()

    app.run(host='0.0.0.0', port=5801)
