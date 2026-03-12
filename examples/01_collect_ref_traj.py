"""
This script collects reference trajectory data under different impedance control gains.
It moves the Franka robot arm in a sinusoidal pattern while logging joint positions, velocities,
desired positions, and control torques. The collected data is saved to a .npz file for further analysis.

When called with --X and --Y flags, it prints the tracking error as a parseable line:
    SYSID_ERROR=<float>
"""


import asyncio
import numpy as np
import argparse
import aiofranka
from aiofranka.robot import RobotInterface
from aiofranka import FrankaController
import time
import os
import matplotlib.pyplot as plt


async def main():

    parser = argparse.ArgumentParser()
    # X=0.234461, Y=0.016865, Z=-0.032987, W=-0.008128
    parser.add_argument("--X", type=float, default=0.234461, help="Mass offset")
    parser.add_argument("--Y", type=float, default=0.016865, help="COM z-offset")
    parser.add_argument("--Z", type=float, default=-0.032987, help="COM y-offset")
    parser.add_argument("--W", type=float, default=-0.008128, help="COM x-offset")
    parser.add_argument('--dir', type=str, default='sysid_left_cmaes_gravcomp_v6', help='Directory to save logs')
    args = parser.parse_args()

    X = args.X
    Y = args.Y
    Z = args.Z  # between -0.1 and 0.1
    W = args.W  # also between -0.1 and 0.1

    aiofranka.set_configuration(mass = 1.0 + X, com=[W, Z, Y])

    robot = RobotInterface("172.16.0.2")
    controller = FrankaController(robot)

    await controller.start()


    base = np.array([1, 1, 1, 1, 0.6, 0.6, 0.6])

    kps = [ 16, 32, 64, 128, 160, 256, 512 ]
    kds = [ 1, 2, 4, 8, 12, 16, 24 ]


    kp_kd_pairs = [ (kp, kd) for kp in kps for kd in kds ]


    for kp, kd in kp_kd_pairs:


        with controller.state_lock:
            controller.kp = base * 80
            controller.kd = base * 4
            print("Moving to initial position...")
        await controller.move([0, 0, 0.0, -1.57079, 0, 1.57079, -0.7853])


        print(f"Testing with kp={kp}, kd={kd}")

        await asyncio.sleep(1.0)

        # run the controller test
        controller.switch("impedance")
        controller.set_freq(50)

        with controller.state_lock:
            controller.kp = base * kp
            controller.kd = base * kd

        logs = {
            'qpos': [],
            'qvel': [],
            'qdes': [],
            'ctrl': [],
        }


        for cnt in range(200):

            logs['qpos'].append(controller.robot.data.qpos.copy())
            logs['qvel'].append(controller.robot.data.qvel.copy())
            logs['ctrl'].append(controller.robot.data.ctrl.copy())
            logs['qdes'].append(controller.q_desired.copy())

            delta = np.sin(cnt / 50.0 * np.pi) * 0.1
            init = controller.initial_qpos
            await controller.set("q_desired", delta + init)

        # await asyncio.sleep(1.0)

        for key in logs.keys():
            logs[key] = np.stack(logs[key])

        # # load up a reference trajectory from "sysid_left_more/sysid_K16_D2.npz"
        # ref_traj = np.load(f"./examples/sysid_left_more/sysid_K{int(kp)}_D{int(kd)}.npz")
        # ref_qpos = ref_traj['qpos']
        # ref_qdes = ref_traj['qdes']

        # ref_error = ref_qdes - ref_qpos
        # my_error = logs['qdes'] - logs['qpos']

        # error = np.linalg.norm(ref_error - my_error)
        # print(f"SYSID_ERROR={error}")

        os.makedirs(f"./examples/{args.dir}/", exist_ok=True)
        np.savez(f"./examples/{args.dir}/sysid_K{int(kp)}_D{int(kd)}.npz", **logs)

if __name__ == "__main__":
    asyncio.run(main())
