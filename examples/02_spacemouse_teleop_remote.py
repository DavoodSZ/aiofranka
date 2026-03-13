"""
Same as 02_spacemouse_teleop.py but using FrankaRemoteController (sync API).

Requires: aiofranka start --ip 172.16.0.2  (in another terminal)
"""

import time
import numpy as np
import aiofranka 
from aiofranka import FrankaRemoteController
from aiofranka.utils.spacemouse import SpaceMouse
import threading
import os
import sys
import tty
import termios
import select
import cv2
import argparse
from concurrent.futures import ThreadPoolExecutor, wait
from collections import deque

from pycaas import PycaasClient
import aprilcube

CONTROL_FREQ = 50

# 5x5 grid of (KP, KD) configurations
KP_VALUES = [10.0, 50.0, 100.0, 250.0, 500.0]
KD_VALUES = [10.0, 20.0, 30.0, 40.0, 50.0]
KP_KD_GRID = [(kp, kd) for kp in KP_VALUES for kd in KD_VALUES]

# let's remove the four corners to focus on the more reasonable gain ranges
KP_KD_GRID = [pair for pair in KP_KD_GRID if pair not in [(10.0, 10.0), (10.0, 50.0), (500.0, 10.0), (500.0, 50.0)]]

# Boundary conditions: (KP=10, KD=10) -> 6.0,  (KP=500, KD=10) -> 0.2
_KP_LO, _KP_HI = 10.0, 500.0
_SCALE_LO, _SCALE_HI = 0.6, 0.016  # per-unit-KD values


def runtime_scale(KP, KD, n=1.5):
    """Compute RUNTIME_SCALE with tunable KP falloff exponent n.

    n=1  ≈ original 1/KP formula
    n>1  → more aggressive falloff at intermediate KP
    """
    A = (_SCALE_LO - _SCALE_HI) / (_KP_LO ** (-n) - _KP_HI ** (-n))
    B = _SCALE_HI - A * _KP_HI ** (-n)
    return KD * (A * KP ** (-n) + B)


# Episode control events
success_event = threading.Event()
failure_event = threading.Event()


def get_next_episode_number(data_dir):
    if not os.path.exists(data_dir):
        return 1
    max_num = 0
    for name in os.listdir(data_dir):
        if name.startswith("episode_") and os.path.isdir(os.path.join(data_dir, name)):
            try:
                num = int(name.split("_")[1])
                max_num = max(max_num, num)
            except (IndexError, ValueError):
                continue
    return max_num + 1


TARGET_PER_TYPE = 100


def count_episodes(data_dir):
    """Return {(kp, kd): {"type_RG": n, "type_GR": n}} by scanning existing directories."""
    counts = {}
    for kp, kd in KP_KD_GRID:
        config_path = os.path.join(data_dir, f"K{kp:.0f}_D{kd:.0f}_with_states")
        rg, gr = 0, 0
        if os.path.exists(config_path):
            for name in os.listdir(config_path):
                ep_path = os.path.join(config_path, name)
                if name.startswith("episode_") and os.path.isdir(ep_path):
                    if os.path.exists(os.path.join(ep_path, "type_RG")):
                        rg += 1
                    elif os.path.exists(os.path.join(ep_path, "type_GR")):
                        gr += 1
        counts[(kp, kd)] = {"type_RG": rg, "type_GR": gr}
    return counts


def sample_next_config(data_dir, current_type):
    """Pick the (KP, KD) most behind for current_type. Returns None if all done."""
    counts = count_episodes(data_dir)
    candidates = [
        (kp, kd, tc[current_type])
        for (kp, kd), tc in counts.items()
        if tc[current_type] < TARGET_PER_TYPE
    ]
    if not candidates:
        return None
    min_count = min(c[2] for c in candidates)
    best = [(kp, kd) for kp, kd, n in candidates if n == min_count]
    return best[np.random.choice(len(best))]


_pct_history = deque(maxlen=10)  # stores (time, pct) samples

def print_progress(data_dir):
    """Print a colored progress bar with summary."""
    counts = count_episodes(data_dir)
    total = sum(tc["type_RG"] + tc["type_GR"] for tc in counts.values())
    target = len(KP_KD_GRID) * TARGET_PER_TYPE * 2
    pct = 100.0 * total / target if target else 0
    done_slots = sum(
        (1 if tc["type_RG"] >= TARGET_PER_TYPE else 0) +
        (1 if tc["type_GR"] >= TARGET_PER_TYPE else 0)
        for tc in counts.values()
    )
    total_slots = len(KP_KD_GRID) * 2
    # Visual progress bar (30 chars wide)
    bar_len = 30
    filled = int(bar_len * pct / 100)
    bar = "\u2588" * filled + "\u2591" * (bar_len - filled)
    # ANSI: bold cyan
    CYAN = "\033[1;36m"
    RESET = "\033[0m"
    # Track rate using rolling window: seconds per 1% point
    now = time.time()
    _pct_history.append((now, pct))
    timing_str = ""
    if len(_pct_history) >= 2:
        t0, p0 = _pct_history[0]
        delta_pct = pct - p0
        delta_t = now - t0
        if delta_pct > 0:
            s = delta_t / delta_pct
            eta_sec = s * (100.0 - pct)
            if s < 60:
                rate_str = f"{s:.1f}s/%"
            else:
                rate_str = f"{s/60:.1f}m/%"
            if eta_sec < 3600:
                eta_str = f"{eta_sec/60:.0f}m"
            else:
                eta_str = f"{eta_sec/3600:.1f}h"
            timing_str = f", {rate_str}, ETA {eta_str}"
    print(f"{CYAN}[Progress] [{bar}] {pct:.1f}%  {total}/{target} eps, {done_slots}/{total_slots} slots done{timing_str}{RESET}")


def encode_grid_mp4(frame_lists, is_bgr_list, labels, filepath, fps=CONTROL_FREQ):
    """Encode multiple frame lists into a 2x2 grid MP4.

    Each pane is downscaled 2x so the grid is the same size as one camera stream.
    """
    n = max(len(fl) for fl in frame_lists)
    if n == 0:
        return
    orig_h, orig_w = None, None
    for fl in frame_lists:
        if fl:
            orig_h, orig_w = fl[0].shape[:2]
            break
    if orig_h is None:
        return
    cell_h, cell_w = orig_h // 2, orig_w // 2

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(filepath, fourcc, fps, (cell_w * 2, cell_h * 2))
    grid = np.zeros((cell_h * 2, cell_w * 2, 3), dtype=np.uint8)
    black = np.zeros((cell_h, cell_w, 3), dtype=np.uint8)
    positions = [(0, 0), (0, cell_w), (cell_h, 0), (cell_h, cell_w)]

    label_frame_idx = 0

    for i in range(n):
        for j, fl in enumerate(frame_lists):
            if not fl:
                f = black
            elif i < len(fl):
                f = fl[i]
            else:
                f = fl[-1]
            f = cv2.resize(f, (cell_w, cell_h), interpolation=cv2.INTER_NEAREST)
            if f.ndim == 2:
                f = cv2.cvtColor(f, cv2.COLOR_GRAY2BGR)
            elif not is_bgr_list[j]:
                f = cv2.cvtColor(f, cv2.COLOR_RGB2BGR)
            if i == label_frame_idx and labels[j]:
                cv2.putText(f, labels[j], (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            r, c = positions[j]
            grid[r:r + cell_h, c:c + cell_w] = f
        writer.write(grid)
    writer.release()


def encode_frames_to_mp4(frames, filepath, fps=CONTROL_FREQ, is_bgr=False):
    """Encode frames to MP4. Assumes RGB unless is_bgr=True."""
    if len(frames) == 0:
        return
    h, w = frames[0].shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(filepath, fourcc, fps, (w, h))
    for frame in frames:
        bgr = frame if is_bgr else cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        writer.write(bgr)
    writer.release()


def save_depth_frames(frames, filepath):
    """Save raw uint16 depth frames as compressed npz."""
    if len(frames) == 0:
        return
    np.savez_compressed(filepath, depth=np.stack(frames))


def keyboard_listener():
    """Background thread for non-blocking SSH keyboard input.

    Controls:
        r - Save current episode as SUCCESS, reset
        t - Save current episode as FAILURE, reset
    """
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key.lower() == "r":
                    print("\n[Keyboard] SUCCESS - saving episode and resetting")
                    success_event.set()
                elif key.lower() == "t":
                    print("\n[Keyboard] FAILURE - saving episode and resetting")
                    failure_event.set()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main(args):

    # Connect peripherals
    pycaas_client = PycaasClient()
    status = pycaas_client.status()
    stream_ids = status.get("active_streams", [])
    print(f"[Init] pycaas streams: {stream_ids}")

    # Cube pose detector
    K = pycaas_client.get_intrinsics_matrix("rs_242522070762")
    ext = np.loadtxt("assets/extrinsic.json")
    cube_det = aprilcube.detector(
        "assets/5x3x1_30_cube",
        intrinsic_cfg=K,
        extrinsic=ext,
        enable_filter=True,
        fast=True,
    )
    color_stream = next((s for s in stream_ids if "color" in s), stream_ids[0] if stream_ids else None)
    cube_det.start_async()
    print(f"[Init] Cube detector ready (async), using stream: {color_stream}")

    
    aiofranka.unlock()

    controller = FrankaRemoteController("172.16.0.2")
    controller.start()

    controller.move()
    time.sleep(1.0)

    # Save initial joint positions for resetting between episodes
    initial_qpos = controller.state["qpos"].copy()

    # Data directory
    os.makedirs(args.data_dir, exist_ok=True)

    # Keyboard listener
    keyboard_thread = threading.Thread(target=keyboard_listener, daemon=True)
    keyboard_thread.start()
    print("[Init] Keyboard: 'r'=SUCCESS, 't'=FAILURE")

    spacemouse = SpaceMouse(yaw_only=True)
    print("[Init] Spacemouse connected.")

    executor = ThreadPoolExecutor()
    pending_encode_futures = []

    # Initial task type from CLI flag
    current_type = "type_GR" if args.GR else "type_RG"

    try:
        while True:
            # --- Pick the most under-collected config ---
            print_progress(args.data_dir)
            result = sample_next_config(args.data_dir, current_type)
            if result is None:
                print(f"[Done] All configs have {TARGET_PER_TYPE} episodes for {current_type}.")
                break
            KP, KD = result
            RUNTIME_SCALE = runtime_scale(KP, KD, n=args.scale_exp)
            T_SCALE = 0.05 * RUNTIME_SCALE

            # Per-config data directory: data_dir/kp_{kp}_kd_{kd}/episode_XXX
            config_dir = os.path.join(args.data_dir, f"K{KP:.0f}_D{KD:.0f}_with_states")
            os.makedirs(config_dir, exist_ok=True)
            episode_num = get_next_episode_number(config_dir)

            # Switch to OSC with sampled gains
            controller.switch("osc")
            controller.ee_kp = np.array([KP] * 3 + [500.0] * 3)
            controller.ee_kd = np.array([KD] * 3 + [30.0] * 3)
            controller.set_freq(CONTROL_FREQ)

            # --- Start new episode ---
            print(f"\n[Episode {episode_num:03d}] [{current_type}] KP={KP}, KD={KD}, RUNTIME_SCALE={RUNTIME_SCALE:.2f}")
            print(f"  Config dir: {config_dir}")
            print(f"  Recording... ('r'=success, 't'=failure)")

            timestamps = []
            qpos_log = []
            qvel_log = []
            ee_log = []
            ee_desired_log = []
            last_torque_log = []
            cube_T_log = []
            last_cube_T = np.eye(4)
            cube_viz_frames = []
            camera_frames = {sid: [] for sid in stream_ids}
            camera_frame_indices = {sid: [] for sid in stream_ids}

            success_event.clear()
            failure_event.clear()

            # --- Inner control loop (50Hz) ---
            loop_count = 0
            recording = False
            DEADZONE = 0.01

            while True:
                if success_event.is_set() or failure_event.is_set():
                    is_success = success_event.is_set()
                    break

                # 1. Read spacemouse + compute target EE
                translation_raw, _, _ = spacemouse.read()

                # Start recording on first non-trivial spacemouse input
                if not recording and np.linalg.norm(translation_raw) > DEADZONE:
                    recording = True
                    print("  [Recording started]")

                # 3. Record camera frames (only after recording starts)
                if recording:
                    for sid in stream_ids:
                        if "depth" in sid:
                            continue
                        try:
                            frame = pycaas_client.get_frame(sid)
                            if frame is not None:
                                camera_frames[sid].append(frame)
                                camera_frame_indices[sid].append(loop_count)
                        except Exception:
                            pass

                # 3b. Submit frame to async cube detector ASAP
                det_sid = "rs_242522070762_color"
                try:
                    det_frame = pycaas_client.get_frame(det_sid)
                    if det_frame is not None:
                        cube_det.submit_frame(det_frame)
                except Exception:
                    pass

                # Read state (no state_lock needed in server mode)
                state_snap = controller.state
                current_pose = state_snap["ee"].copy()
                current_qpos = state_snap["qpos"].copy()
                current_qvel = state_snap["qvel"].copy()
                current_torque = state_snap["last_torque"].copy()
                desired_pose = controller.ee_desired.copy()

                translation_delta = np.clip(
                    translation_raw * T_SCALE, -T_SCALE, T_SCALE
                )

                target_ee = current_pose.copy()
                target_ee[:3, 3] += translation_delta
                target_ee[:3, :3] = desired_pose[:3, :3]

                # 2. Record robot state (only after recording starts)
                if recording:
                    t = time.time()
                    timestamps.append(t)
                    qpos_log.append(current_qpos)
                    qvel_log.append(current_qvel)
                    ee_log.append(current_pose)
                    ee_desired_log.append(target_ee.copy())
                    last_torque_log.append(current_torque)

                # 3c. Read latest cube detection result
                result = cube_det.get_latest()
                if recording:
                    if result is not None and result["success"] and result["T"] is not None:
                        cube_T_log.append(result["T"].copy())
                        last_cube_T = result["T"].copy()
                    else:
                        cube_T_log.append(last_cube_T.copy())
                if recording and result is not None and "debug_viz" in result:
                    cube_viz_frames.append(result["debug_viz"])

                # 4. Send command (set() sleeps for remainder of 20ms)
                controller.set("ee_desired", target_ee)

                loop_count += 1

            # --- Episode ended ---
            n_steps = len(timestamps)
            label = "SUCCESS" if is_success else "FAILURE"
            print(f"[Episode {episode_num:03d}] {label} [{current_type}] — {n_steps} steps recorded")

            if not args.practice:
                ep_dir = os.path.join(config_dir, f"episode_{episode_num:03d}")
                os.makedirs(ep_dir, exist_ok=True)
                open(os.path.join(ep_dir, "success" if is_success else "failure"), "w").close()
                open(os.path.join(ep_dir, current_type), "w").close()

                # Wait for any prior encoding to finish
                if pending_encode_futures:
                    pending = [f for f in pending_encode_futures if not f.done()]
                    if pending:
                        print(f"  Waiting for {len(pending)} prior encode(s) to finish...")
                        wait(pending)
                    pending_encode_futures.clear()

                # Build and save npz
                if n_steps > 0:
                    save_dict = {
                        "timestamps": np.array(timestamps),
                        "qpos": np.stack(qpos_log),
                        "qvel": np.stack(qvel_log),
                        "ee": np.stack(ee_log),
                        "ee_desired": np.stack(ee_desired_log),
                        "last_torque": np.stack(last_torque_log),
                        "cube_T": np.stack(cube_T_log),
                        "success": np.array(is_success),
                        "control_freq": np.array(CONTROL_FREQ),
                        "ee_kp": np.array([KP] * 3 + [500.0] * 3),
                        "ee_kd": np.array([KD] * 3 + [30.0] * 3),
                    }
                    for sid in stream_ids:
                        save_dict[f"camera_indices_{sid}"] = np.array(camera_frame_indices[sid], dtype=np.int64)

                    npz_path = os.path.join(ep_dir, "state.npz")
                    np.savez(npz_path, **save_dict)
                    print(f"  Saved {npz_path}")

                # Fire off frame encoding in background threads
                for sid in stream_ids:
                    if "depth" in sid:
                        continue
                    frames = camera_frames[sid]
                    if len(frames) == 0:
                        continue
                    out_path = os.path.join(ep_dir, f"{sid}.mp4")
                    print(f"  Encoding {len(frames)} frames -> {out_path} (background)")
                    pending_encode_futures.append(
                        executor.submit(encode_frames_to_mp4, frames, out_path, CONTROL_FREQ)
                    )

                if cube_viz_frames:
                    viz_path = os.path.join(ep_dir, "cube_debug.mp4")
                    print(f"  Encoding {len(cube_viz_frames)} cube debug frames -> {viz_path} (background)")
                    pending_encode_futures.append(
                        executor.submit(encode_frames_to_mp4, cube_viz_frames, viz_path, CONTROL_FREQ)
                    )

                color_sids = [s for s in stream_ids if "color" in s]
                grid_lists = []
                grid_bgr_flags = []
                grid_labels = []
                for sid in color_sids[:3]:
                    grid_lists.append(camera_frames.get(sid, []))
                    grid_bgr_flags.append(False)
                    grid_labels.append(sid.replace("_color", "").replace("rs_", ""))
                grid_lists.append(cube_viz_frames)
                grid_bgr_flags.append(True)
                grid_labels.append("cube_det")
                while len(grid_lists) < 4:
                    grid_lists.append([])
                    grid_bgr_flags.append(True)
                    grid_labels.append("")
                grid_path = os.path.join(ep_dir, "grid.mp4")
                print(f"  Encoding 2x2 grid -> {grid_path} (background)")
                pending_encode_futures.append(
                    executor.submit(encode_grid_mp4, grid_lists[:4], grid_bgr_flags[:4], grid_labels[:4], grid_path, CONTROL_FREQ)
                )

            # Reset robot while encoding runs in parallel
            base = np.array([1, 1, 1, 1, 0.6, 0.6, 0.6])
            controller.switch("impedance")
            controller.kp = base * 80
            controller.kd = base * 4
            controller.set_freq(CONTROL_FREQ)
            print("  Resetting to initial pose...")
            controller.move(initial_qpos)
            time.sleep(1.0)

            # Wait for encodes to finish before next episode
            if not args.practice and pending_encode_futures:
                pending = [f for f in pending_encode_futures if not f.done()]
                if pending:
                    print(f"  Waiting for {len(pending)} encode(s) to finish...")
                    wait(pending)
                pending_encode_futures.clear()

            print(f"\a[Episode {episode_num:03d}] Reset complete, starting next episode.")

            # Determine next task type
            if is_success:
                current_type = "type_GR" if current_type == "type_RG" else "type_RG"
            else:
                current_type = ask_task_type()

    finally:
        cube_det.stop_async()
        # controller.stop()
        executor.shutdown(wait=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir", type=str, default="./data")
    parser.add_argument("--GR", action="store_true", help="Start with type_GR instead of type_RG")
    parser.add_argument("--scale-exp", type=float, default=1.0,
                        help="KP falloff exponent for RUNTIME_SCALE (1=~original, higher=more aggressive)")
    parser.add_argument("--practice", action="store_true", help="Run in practice mode (no saving, just keyboard control)")
    args = parser.parse_args()
    main(args)
