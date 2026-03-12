"""
Same as 01_collect_ref_traj.py but using FrankaRemoteController (sync API).

Requires: aiofranka start --ip 172.16.0.2  (in another terminal)
"""

import numpy as np
from aiofranka import FrankaRemoteController
import time
import os


def main():
    controller = FrankaRemoteController("172.16.0.2")
    controller.start()

    base = np.array([1, 1, 1, 1, 0.6, 0.6, 0.6])

    kps = [16, 32, 64, 128, 256, 512]
    kds = [1, 2, 4, 8, 12, 16, 24]

    kp_kd_pairs = [(kp, kd) for kp in kps for kd in kds]

    for kp, kd in kp_kd_pairs:

        controller.kp = base * 80
        controller.kd = base * 4
        print("Moving to initial position...")
        controller.move([0, 0, 0.0, -1.57079, 0, 1.57079, -0.7853])

        print(f"Testing with kp={kp}, kd={kd}")

        time.sleep(2.0)

        controller.switch("impedance")
        controller.set_freq(50)

        controller.kp = base * kp
        controller.kd = base * kd

        logs = {
            'qpos': [],
            'qvel': [],
            'qdes': [],
            'ctrl': [],
        }

        for cnt in range(200):
            state = controller.state
            logs['qpos'].append(state['qpos'].copy())
            logs['qvel'].append(state['qvel'].copy())
            logs['ctrl'].append(state['last_torque'].copy())
            logs['qdes'].append(controller.q_desired.copy())

            delta = np.sin(cnt / 50.0 * np.pi) * 0.1
            q_desired = delta + controller.initial_qpos
            controller.set("q_desired", q_desired)

        time.sleep(1.0)

        for key in logs:
            logs[key] = np.stack(logs[key])

        os.makedirs("./examples/sysid_left_correct_gravcomp/", exist_ok=True)
        np.savez(f"./examples/sysid_left_correct_gravcomp/sysid_K{int(kp)}_D{int(kd)}.npz", **logs)


if __name__ == "__main__":
    main()
