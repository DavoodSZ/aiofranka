"""
Robotiq Gripper Toggle (Sync / Remote API)

Same as 04_gripper.py but using GripperRemoteController (no async/await needed).

Press SPACE to toggle the gripper open/closed.
Press Q or Ctrl+C to quit.

Works over SSH — uses raw terminal input (no GUI needed).

Requirements:
    pip install "aiofranka[robotiq]"

Usage:
    python examples/04_gripper_remote.py
    python examples/04_gripper_remote.py --port /dev/ttyUSB1
"""

import sys
import tty
import termios
import argparse
from aiofranka import GripperRemoteController


def main():
    parser = argparse.ArgumentParser(description="Toggle gripper with spacebar (sync API)")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB1")
    parser.add_argument("--speed", type=int, default=255)
    parser.add_argument("--force", type=int, default=255)
    args = parser.parse_args()

    gripper = GripperRemoteController(args.port)
    gripper.start()

    gripper.speed = args.speed
    gripper.force = args.force

    # Start open
    is_closed = False
    gripper.q_desired = 0
    gripper.wait_until_reached(timeout=3.0)

    # Save terminal settings and switch to raw mode
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setraw(fd)

    print("Gripper ready. SPACE=toggle, Q=quit\r")
    print(f"  State: OPEN (qpos={gripper.qpos})\r")

    try:
        while True:
            key = sys.stdin.read(1)

            if key == " ":
                is_closed = not is_closed
                if is_closed:
                    gripper.q_desired = 255
                    label = "CLOSED"
                    print(gripper.qpos, "CLOSED")
                else:
                    gripper.q_desired = 0
                    label = "OPEN"
                    print(gripper.qpos, "OPEN")
                print(f"  -> {label} (q_desired={gripper.q_desired})\r")

            elif key in ("q", "Q", "\x03"):  # q or Ctrl+C
                break

    finally:
        # Restore terminal settings
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print("\nShutting down...")
        gripper.q_desired = 0
        import time
        time.sleep(0.5)
        gripper.stop()


if __name__ == "__main__":
    main()
