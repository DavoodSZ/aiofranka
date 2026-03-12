"""
aiofranka: Asyncio-based Franka Robot Control

A high-level Python library for controlling Franka Emika robots using asyncio.
Combines pylibfranka for real-time control with MuJoCo for kinematics/dynamics.

Main Components:
    RobotInterface: Low-level robot interface (real or simulation)
    FrankaController: High-level asyncio controller with multiple modes
    FrankaLockUnlock: Client for robot authentication and brake control

Quick Example:
    >>> import asyncio
    >>> from aiofranka import RobotInterface, FrankaController
    >>> 
    >>> async def main():
    ...     robot = RobotInterface("172.16.0.2")
    ...     controller = FrankaController(robot)
    ...     await controller.start()
    ...     await controller.move()  # Move to home
    ...     await controller.stop()
    >>> 
    >>> asyncio.run(main())

For detailed documentation, see README.md and USAGE_GUIDE.md
"""

from aiofranka.controller import FrankaController
from aiofranka.robot import RobotInterface
from aiofranka.async_utils import asyncify, async_input, CudaInferenceThread, mpify
from aiofranka.remote import FrankaRemoteController
from aiofranka.server import start, stop, lock, unlock, set_configuration

# Optional gripper support - only import if dependencies are available
try:
    from aiofranka.gripper import GripperController, RobotiqGripperInterface, create_gripper
    _HAS_ROBOTIQ = True
except ImportError:
    _HAS_ROBOTIQ = False

__version__ = "0.1.0"
__all__ = ["RobotInterface", "FrankaController", "FrankaRemoteController", "asyncify", "async_input", "CudaInferenceThread", "mpify", "start", "stop", "lock", "unlock", "set_configuration"]

if _HAS_ROBOTIQ:
    __all__.extend(["GripperController", "RobotiqGripperInterface", "create_gripper"])