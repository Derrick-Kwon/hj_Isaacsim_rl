# SPDX-License-Identifier: BSD-3-Clause
# SenseGlove → (5 fingertip xyz) → select 4 tips (T, I, M, R) → geort model → Shadow Hand Lite joint positions (16-DOF)
# DeviceBase-compatible teleop device for Isaac Lab demo recording.
# NOTE: All output post-processing is removed (no remap, no offset/scale, no low-pass).

from __future__ import annotations

import os
import time
import threading
import weakref
from typing import Callable, Optional, Sequence, Tuple, List

import numpy as np
import torch

# Omniverse for keyboard-style callbacks (same pattern as Se3Keyboard)
import carb
import omni

# Isaac Lab device base
from isaaclab.devices.device_base import DeviceBase
# from ..device_base import DeviceBase

# ROS 2
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64MultiArray

# Your trained retargeting model
import geort


class _TipSubNode(Node):
    """ROS2 node that subscribes to fingertip positions.

    Expected message:
      - Topic publishes Float64MultiArray with flattened xyz per tip.
      - Order is: Thumb, Index, Middle, Ring, Little (T, I, M, R, L).
      - If K tips exist, data length is 3 * K (xyz per tip).
    """

    def __init__(self, node_name: str, topic: str, queue_size: int = 10):
        super().__init__(node_name)
        self._lock = threading.Lock()
        self._latest = None  # np.ndarray of shape (K, 3)
        self.create_subscription(Float64MultiArray, topic, self._cb, queue_size)

    def _cb(self, msg: Float64MultiArray):
        arr = np.asarray(msg.data, dtype=np.float32)
        if arr.size != 15:
            return
        
        offset_z = 40  # mm 단위 z-offset for aligning orientation between glove and robot hand
        offset_y = -6 
        offset_x = 20 # mm 단위 x-offset

        pts = arr.reshape(5, 3)
        pts[:, 0] += offset_x
        pts[:, 1] += offset_y
        pts[:, 2] += offset_z

        pts = pts / 1000.0
        # pts[:,2] -= 0.045
        R = np.array([
            [0, 1, 0],
            [0, 0, 1],
            [1, 0, 0]
        ], dtype=np.float32)
        pts = pts.dot(R.T)
        # Validate length: must be divisible by 3 (x,y,z per tip)
        pts = pts.reshape(5, 3)
        # with self._lock:
        self._latest = pts

    def read_latest(self) -> Optional[np.ndarray]:
        with self._lock:
            return None if self._latest is None else self._latest.copy()


class SenseGloveHandDevice(DeviceBase):
    """Teleop device that converts 5-tip SenseGlove xyz to 4-tip input for a geort model.

    Pipeline:
      (1) Subscribe fingertip xyz (T, I, M, R, L) from ROS2 topic as Float64MultiArray.
      (2) Select 4 tips (default: T, I, M, R = [0,1,2,3]) to form a (4,3) float32 array.
      (3) Run geort model: (4,3) → joint positions (expected 16-DOF for 4-finger Shadow Hand Lite).
      (4) Return (joints,) to be used directly as environment actions.

    Args:
        tip_topic: ROS2 topic publishing flattened xyz, order T, I, M, R, L.
        tip_indices: Exactly 4 indices to feed the model (default [0,1,2,3] → T, I, M, R).
        model_checkpoint_tag: geort checkpoint tag (e.g., "best_model").
        model_epoch: epoch index to load (-1 for latest).
        expected_out_dof: sanity check for output joints (default 16).
        ros_domain_id: set ROS_DOMAIN_ID if needed.
        device_override: force "cpu" or "cuda" if needed.
    """

    def __init__(
        self,
        tip_topic: str | None = None,
        tip_indices: Sequence[int] | None = None,
        model_checkpoint_tag: str | None = None,
        model_epoch: int | None = None,
        expected_out_dof: int | None = None,
        ros_domain_id: Optional[str | int] = None,
        device_override: Optional[str] = None,
    ):
        # --- Resolve configuration here (single source of truth) ---
        tip_topic = tip_topic or os.getenv("SG_TIP_TOPIC", "/senseglove/rh/fingertip_positions")

        if tip_indices is None:
            env_idx = os.getenv("SG_TIP_IDX")
            tip_indices = [int(x) for x in env_idx.split(",")] if env_idx else [0, 1, 2, 3]

        model_checkpoint_tag = model_checkpoint_tag or os.getenv("SG_MODEL_TAG", "best_model")
        model_epoch = model_epoch if model_epoch is not None else int(os.getenv("SG_MODEL_EPOCH", "-1"))
        expected_out_dof = expected_out_dof if expected_out_dof is not None else int(os.getenv("SG_OUT_DOF", "16"))

        # Keyboard callback infra (same as Se3Keyboard.add_callback)
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._additional_callbacks = {}
        self._keyboard_sub = self._input.subscribe_to_keyboard_events(
            self._keyboard,
            lambda event, *args, obj=weakref.proxy(self): obj._on_keyboard_event(event, *args),
        )

        # ROS2 init
        self._rclpy_inited_here = False
        if not rclpy.ok():
            if ros_domain_id is not None:
                os.environ["ROS_DOMAIN_ID"] = str(ros_domain_id)
            rclpy.init()
            self._rclpy_inited_here = True

        self._node = _TipSubNode("sg_retarget_device", tip_topic)
        # self._executor = MultiThreadedExecutor(num_threads=1)
        # self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

        # Select 4 tips out of 5 (default: T,I,M,R = [0,1,2,3], ignore Little)
        self._tip_indices: List[int] = list(tip_indices)
        if len(self._tip_indices) != 4:
            raise ValueError("tip_indices must contain exactly 4 indices (e.g., [0,1,2,3] for T,I,M,R).")

        # geort model
        self._torch_device = (
            torch.device(device_override)
            if device_override is not None
            else torch.device("cuda" if torch.cuda.is_available() else "cpu")
        )
        self._model = geort.load_model(model_checkpoint_tag, epoch=model_epoch)
        if hasattr(self._model, "to"):
            try:
                self._model.to(self._torch_device)
            except Exception:
                pass
        if hasattr(self._model, "eval"):
            self._model.eval()

        # Output checks
        self._expected_out_dof = int(expected_out_dof)

    # ------- DeviceBase API -------

    def __str__(self) -> str:
        return (
            "SenseGloveHandDevice (retargeted joints for 4-finger Shadow Hand Lite)\n"
            f"\tTip topic: Float64MultiArray (flattened xyz)\n"
            f"\tTip indices used (T,I,M,R default): {self._tip_indices}\n"
            f"\tExpected output DOF: {self._expected_out_dof}"
        )

    def reset(self):
        # Nothing to reset (no filters)
        return

    def add_callback(self, key: str, func: Callable):
        self._additional_callbacks[key] = func

    def advance(self) -> Tuple[np.ndarray]:
        """Return (joint_positions,) once per call."""
        tips = self._node.read_latest()
        if tips is None:
            # No data yet → return empty; pre_process will guard
            return (np.zeros(0, dtype=np.float32),)

        # Guard: require enough tips; we select 4 of them (T,I,M,R by default)
        if tips.shape[0] < max(self._tip_indices) + 1:
            return (np.zeros(0, dtype=np.float32),)

        # Build (4,3) float32 array for the geort model
        try:
            pts4 = tips[self._tip_indices, :].astype(np.float32, copy=False)
            # print(pts4)
        except Exception:
            return (np.zeros(0, dtype=np.float32),)

        # geort.forward expects NumPy array, not torch.Tensor
        qpos = self._model.forward(pts4)  # expected shape (16,) for Shadow Hand Lite
        # Ensure 1D float32 numpy vector
        if isinstance(qpos, torch.Tensor):
            qpos = qpos.detach().cpu().numpy()
        qpos = np.asarray(qpos, dtype=np.float32).reshape(-1)

        # Optional consistency check (strict)
        if self._expected_out_dof > 0 and qpos.size != self._expected_out_dof:
            return (np.zeros(0, dtype=np.float32),)

        return (qpos,)

    # ------- internals / lifecycle -------

    def __del__(self):
        try:
            self._input.unsubscribe_from_keyboard_events(self._keyboard, self._keyboard_sub)
        except Exception:
            pass
        # try:
        #     self._executor.shutdown(timeout_sec=0.2)
        # except Exception:
        #     pass
        try:
            self._node.destroy_node()
        except Exception:
            pass
        try:
            if self._rclpy_inited_here and rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    def _spin(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self._node,timeout_sec=0.05)
                time.sleep(0.001)
        except Exception:
            pass

    def _on_keyboard_event(self, event, *args, **kwargs):
        # Optional: bind "R", "S", "T" from record_demos.py for reset/pause/resume
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name in self._additional_callbacks:
                try:
                    self._additional_callbacks[event.input.name]()
                except Exception:
                    pass
        return True
