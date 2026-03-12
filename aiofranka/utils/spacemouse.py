import pyspacemouse
import numpy as np


class SpaceMouse:
    """Wrapper around pyspacemouse that drains the HID buffer on each read.

    Without draining, stale HID reports queue up in the kernel buffer and
    cause the robot to keep moving after the spacemouse is released.

    Args:
        translation_scale: Multiplier for translation deltas (m per axis unit).
        translation_clip: Max absolute translation delta per read (m).
        rotation_scale: Multiplier for rotation deltas (degrees per axis unit).
        rotation_clip: Max absolute rotation delta per read (degrees).
    """

    def __init__(
        self,
        translation_scale: float = 0.006,
        translation_clip: float = 0.006,
        rotation_scale: float = 0.8,
        rotation_clip: float = 0.8,
        yaw_only: bool = False,
    ):
        self.translation_scale = translation_scale
        self.translation_clip = translation_clip
        self.rotation_scale = rotation_scale
        self.rotation_clip = rotation_clip
        self.yaw_only = yaw_only
        self._device = pyspacemouse.open().__enter__()

    def read(self):
        """Read the latest spacemouse state, draining any buffered HID reports.

        Returns:
            (translation_raw, rotation_raw, buttons):
                translation_raw: np.ndarray (3,) raw normalized axes [-1, 1].
                rotation_raw: np.ndarray (3,) raw euler angles (degrees),
                    respecting yaw_only setting.
                buttons: list[int] button states from the latest event.
        """
        event = self._device.read()
        while self._device._device.read(self._device._info.bytes_to_read):
            event = self._device.read()

        translation_raw = np.array([event.x, event.y, event.z])

        if self.yaw_only:
            rotation_raw = np.array([0, 0, -event.yaw])
        else:
            rotation_raw = np.array([-event.pitch, event.roll, -event.yaw])

        return translation_raw, rotation_raw, event.buttons
