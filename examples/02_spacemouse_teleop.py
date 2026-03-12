import time
import asyncio
import aiofranka
import numpy as np
from aiofranka.robot import RobotInterface
from aiofranka import FrankaController
from aiofranka.utils.spacemouse import SpaceMouse
import threading
import os
import sys
import tty
import termios
import select
import cv2
import argparse

from pycaas import PycaasClient
from aiofranka.gripper import GripperController
import aprilcube

CONTROL_FREQ = 50

# Episode control events (bridged from keyboard thread to async loop)
success_event = asyncio.Event()
failure_event = asyncio.Event()


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


def _ask_task_type_blocking():
    """Blocking input for task type selection (run in executor)."""
    while True:
        choice = input("[Task Type] Next episode type — (1) RG  (2) GR: ").strip()
        if choice == "1":
            return "type_RG"
        elif choice == "2":
            return "type_GR"
        print("  Invalid input, enter 1 or 2.")


async def ask_task_type():
    """Ask the user for task type without blocking the event loop."""
    loop = asyncio.get_event_loop()
    return await loop.run_in_executor(None, _ask_task_type_blocking)


def encode_grid_mp4(frame_lists, is_bgr_list, labels, filepath, fps=CONTROL_FREQ):
    """Encode multiple frame lists into a 2x2 grid MP4.

    Each pane is downscaled 2x so the grid is the same size as one camera stream.
    """
    n = max(len(fl) for fl in frame_lists)
    if n == 0:
        return
    # Get original size, then halve for each cell
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

    # Burn label into first frame of grid only
    label_frame_idx = 0

    for i in range(n):
        for j, fl in enumerate(frame_lists):
            if not fl:
                f = black
            elif i < len(fl):
                f = fl[i]
            else:
                f = fl[-1]
            # Downscale + color convert in one resize call
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
                    success_event._loop.call_soon_threadsafe(success_event.set)
                elif key.lower() == "t":
                    print("\n[Keyboard] FAILURE - saving episode and resetting")
                    failure_event._loop.call_soon_threadsafe(failure_event.set)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


async def main(args):

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
    # Pick a color stream for detection
    color_stream = next((s for s in stream_ids if "color" in s), stream_ids[0] if stream_ids else None)
    cube_det.start_async()
    print(f"[Init] Cube detector ready (async), using stream: {color_stream}")
    
    aiofranka.set_configuration(mass = 1.0, com=[0, 0, 0.057])

    robot = RobotInterface("172.16.0.2")
    controller = FrankaController(robot)

    # Close gripper and keep it closed
    # gripper = GripperController()
    # await gripper.start()
    # gripper.close()

    await controller.start()
    await controller.move()
    await asyncio.sleep(1.0)


    # Save initial joint positions for resetting between episodes
    initial_qpos = controller.state["qpos"].copy()



    # KP = 10.0
    # KD = 10.0
    # RUNTIME_SCALE = 6.0

    # KP = 10.0
    # KD = 20.0
    # RUNTIME_SCALE = 12.0

    # KP = 10.0
    # KD = 30.0
    # RUNTIME_SCALE = 18.0

    KP = 500.0
    KD = 10.0
    RUNTIME_SCALE = 0.2

    # KP = 500.0
    # KD = 50.0
    # RUNTIME_SCALE = 0.8

    # KP = 10.0
    # KD = 50.0
    # RUNTIME_SCALE = 30.0

    

    # Switch to OSC mode
    controller.switch("osc")
    controller.ee_kp = np.array([KP] * 3 + [500.0] * 3)
    controller.ee_kd = np.array([KD] * 3 + [30.0] * 3)
    controller.set_freq(CONTROL_FREQ)

    # Translation scale: base * RUNTIME_SCALE
    T_SCALE = 0.05 * RUNTIME_SCALE
    # Data directory
    os.makedirs(args.data_dir, exist_ok=True)
    episode_num = get_next_episode_number(args.data_dir)

    # Keyboard listener
    loop = asyncio.get_event_loop()
    success_event._loop = loop
    failure_event._loop = loop
    keyboard_thread = threading.Thread(target=keyboard_listener, daemon=True)
    keyboard_thread.start()
    print("[Init] Keyboard: 'r'=SUCCESS, 't'=FAILURE")

    spacemouse = SpaceMouse(yaw_only=True)
    print("[Init] Spacemouse connected.")
    print(f"[Init] KP={KP}, KD={KD}, RUNTIME_SCALE={RUNTIME_SCALE}")

    pending_encode_tasks = []  # background MP4 encoding futures

    # Initial task type from CLI flag
    current_type = "type_GR" if args.GR else "type_RG"

    try:
        while True:
            # --- Start new episode ---
            print(f"\n[Episode {episode_num:03d}] [{current_type}] Recording... ('r'=success, 't'=failure)")

            timestamps = []
            qpos_log = []
            qvel_log = []
            ee_log = []
            ee_desired_log = []
            last_torque_log = []
            cube_T_log = []
            last_cube_T = np.eye(4)  # fallback until first detection
            cube_viz_frames = []
            camera_frames = {sid: [] for sid in stream_ids}
            camera_frame_indices = {sid: [] for sid in stream_ids}  # maps each frame to its timestep index

            success_event.clear()
            failure_event.clear()

            # --- Inner control loop (50Hz) ---
            timing_log = {"loop": [], "spacemouse": [], "record": [], "camera": [], "set": []}
            loop_count = 0
            recording = False  # Only start recording on first spacemouse input
            DEADZONE = 0.01  # Min translation magnitude to trigger recording

            while True:
                t_loop_start = time.perf_counter()

                if success_event.is_set() or failure_event.is_set():
                    is_success = success_event.is_set()
                    break

                # 1. Read spacemouse + compute target EE
                t0 = time.perf_counter()
                translation_raw, _, _ = spacemouse.read()

                # Start recording on first non-trivial spacemouse input
                if not recording and np.linalg.norm(translation_raw) > DEADZONE:
                    recording = True
                    print(f"  [Recording started]")

                # 3. Record camera frames (only after recording starts)
                t0 = time.perf_counter()
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
                t_camera = time.perf_counter() - t0

                # 3b. Submit frame to async cube detector ASAP (always, even before recording)
                det_sid = "rs_242522070762_color"
                try:
                    det_frame = pycaas_client.get_frame(det_sid)
                    if det_frame is not None:
                        cube_det.submit_frame(det_frame)
                except Exception:
                    pass


                # No rotation — keep initial orientation
                with controller.state_lock:
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
                t_spacemouse = time.perf_counter() - t0

                # 2. Record robot state (only after recording starts)
                t0 = time.perf_counter()
                if recording:
                    t = time.time()
                    timestamps.append(t)
                    qpos_log.append(current_qpos)
                    qvel_log.append(current_qvel)
                    ee_log.append(current_pose)
                    ee_desired_log.append(target_ee.copy())
                    last_torque_log.append(current_torque)
                t_record = time.perf_counter() - t0


                # 3c. Read latest cube detection result (only after recording starts)
                result = cube_det.get_latest()
                if recording:
                    if result is not None and result["success"] and result["T"] is not None:
                        cube_T_log.append(result["T"].copy())
                        last_cube_T = result["T"].copy()
                    else:
                        cube_T_log.append(last_cube_T.copy())
                if recording and result is not None and "debug_viz" in result:
                    cube_viz_frames.append(result["debug_viz"])

                # 4. Send command LAST (set() sleeps for remainder of 20ms)
                t0 = time.perf_counter()
                await controller.set("ee_desired", target_ee)
                t_set = time.perf_counter() - t0


            # --- Episode ended ---
            ep_dir = os.path.join(args.data_dir, f"episode_{episode_num:03d}")
            os.makedirs(ep_dir, exist_ok=True)
            open(os.path.join(ep_dir, "success" if is_success else "failure"), "w").close()
            open(os.path.join(ep_dir, current_type), "w").close()
            n_steps = len(timestamps)
            label = "SUCCESS" if is_success else "FAILURE"
            print(f"[Episode {episode_num:03d}] {label} [{current_type}] — {n_steps} steps recorded")

            # Wait for any prior encoding to finish before reusing executor
            if pending_encode_tasks:
                pending = [t for t in pending_encode_tasks if not t.done()]
                if pending:
                    print(f"  Waiting for {len(pending)} prior encode(s) to finish...")
                    await asyncio.gather(*pending)
                pending_encode_tasks.clear()

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
                    # Controller config (needed to reproduce behavior)
                    "control_freq": np.array(CONTROL_FREQ),
                    "ee_kp": np.array([KP] * 3 + [500.0] * 3),
                    "ee_kd": np.array([KD] * 3 + [30.0] * 3),
                }
                # Camera frame-to-timestep index mapping
                for sid in stream_ids:
                    save_dict[f"camera_indices_{sid}"] = np.array(camera_frame_indices[sid], dtype=np.int64)

                npz_path = os.path.join(ep_dir, "state.npz")
                np.savez(npz_path, **save_dict)
                print(f"  Saved {npz_path}")

            # Fire off frame encoding in background threads BEFORE reset
            # so they run in parallel with the robot move
            for sid in stream_ids:
                if "depth" in sid:
                    continue
                frames = camera_frames[sid]
                if len(frames) == 0:
                    continue
                out_path = os.path.join(ep_dir, f"{sid}.mp4")
                print(f"  Encoding {len(frames)} frames -> {out_path} (background)")
                pending_encode_tasks.append(
                    loop.run_in_executor(
                        None, encode_frames_to_mp4, frames, out_path, CONTROL_FREQ
                    )
                )

            if cube_viz_frames:
                viz_path = os.path.join(ep_dir, "cube_debug.mp4")
                print(f"  Encoding {len(cube_viz_frames)} cube debug frames -> {viz_path} (background)")
                pending_encode_tasks.append(
                    loop.run_in_executor(
                        None, encode_frames_to_mp4, cube_viz_frames, viz_path, CONTROL_FREQ
                    )
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
            pending_encode_tasks.append(
                loop.run_in_executor(
                    None, encode_grid_mp4, grid_lists[:4], grid_bgr_flags[:4], grid_labels[:4], grid_path, CONTROL_FREQ
                )
            )

            # Reset robot while encoding runs in parallel
            base = np.array([1, 1, 1, 1, 0.6, 0.6, 0.6])
            controller.switch("impedance")
            with controller.state_lock:
                controller.kp = base * 80
                controller.kd = base * 4
            controller.set_freq(CONTROL_FREQ)
            print("  Resetting to initial pose...")
            await controller.move(initial_qpos)
            await asyncio.sleep(1.0)

            # Wait for encodes to finish before switching back to OSC
            if pending_encode_tasks:
                pending = [t for t in pending_encode_tasks if not t.done()]
                if pending:
                    print(f"  Waiting for {len(pending)} encode(s) to finish...")
                    await asyncio.gather(*pending)
                pending_encode_tasks.clear()

            print(f"\a[Episode {episode_num:03d}] Reset complete, starting next episode.")

            # Switch back to OSC for next episode
            controller.switch("osc")
            controller.ee_kp = np.array([KP] * 3 + [500.0] * 3)
            controller.ee_kd = np.array([KD] * 3 + [30.0] * 3)
            controller.set_freq(CONTROL_FREQ)

            # Determine next task type
            if is_success:
                # Alternate on success
                current_type = "type_GR" if current_type == "type_RG" else "type_RG"
            else:
                # Re-ask on failure
                current_type = await ask_task_type()

            episode_num += 1

    finally:
        cube_det.stop_async()
        await controller.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir", type=str, default="./data")
    parser.add_argument("--GR", action="store_true", help="Start with type_GR instead of type_RG")
    args = parser.parse_args()
    asyncio.run(main(args))
