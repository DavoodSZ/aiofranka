#!/usr/bin/env python3
"""
Minimal Real Robot Script for Relative Joint Position Actions

Observation space (21 dims):
  - joint_position_error (7): target_joints - current_joints
  - joint_vel (7): current velocity
  - last_action (7): previous action

Action space:
  - Relative: target = current_pos + clamp(action, -1, 1) * scale

Usage:
    # With sampled target (from training distribution)
    python play_real_rel.py \
        --checkpoint path/to/best_agent.pt \
        --action_scale 0.0125

    # With explicit target
    python play_real_rel.py \
        --checkpoint path/to/best_agent.pt \
        --action_scale 0.0125 \
        --target_joints 0.0 -0.569 0.0 -2.810 0.0 3.037 0.741
"""

import argparse
import numpy as np
import time
import torch

# Real robot imports
import asyncio
from aiofranka import RobotInterface, FrankaController

# ============================================================================
# FR3 Constants
# ============================================================================

FR3_DEFAULT_JOINT_POS = np.array([0.0, -0.569, 0.0, -2.810, 0.0, 3.037, 0.741], dtype=np.float32)
FR3_JOINT_POS_MIN = np.array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159], dtype=np.float32)
FR3_JOINT_POS_MAX = np.array([2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159], dtype=np.float32)

# Safe limits (used for training distribution)
FR3_JOINT_POS_MIN_SAFE = np.array([-2.3476, -1.5454, -2.4937, -2.7714, -2.51, 0.7773, -2.7045], dtype=np.float32)
FR3_JOINT_POS_MAX_SAFE = np.array([2.3476, 1.5454, 2.4937, -0.4225, 2.51, 4.2841, 2.7045], dtype=np.float32)
FR3_DEFAULT_JOINT_POS_SAFE = (FR3_JOINT_POS_MIN_SAFE + FR3_JOINT_POS_MAX_SAFE) / 2.0

# Training samples ±0.5 rad around safe default
TRAINING_SAMPLING_RANGE = 0.5


def sample_target_from_training_distribution() -> np.ndarray:
    """Sample target joints from the same distribution used in training."""
    r = np.random.uniform(-1.0, 1.0, size=7)
    target = FR3_DEFAULT_JOINT_POS_SAFE + r * TRAINING_SAMPLING_RANGE
    target = np.clip(target, FR3_JOINT_POS_MIN_SAFE, FR3_JOINT_POS_MAX_SAFE)
    return target.astype(np.float32)

# ============================================================================
# USER MUST IMPLEMENT THESE FUNCTIONS
# ============================================================================

def read_joint_positions() -> np.ndarray:
    """Read current joint positions from robot. Return shape (7,)."""
    raise NotImplementedError("Implement read_joint_positions()")

def read_joint_velocities() -> np.ndarray:
    """Read current joint velocities from robot. Return shape (7,)."""
    raise NotImplementedError("Implement read_joint_velocities()")

def apply_action(action: np.ndarray) -> None:
    """Send joint position command to robot. Input shape (7,)."""
    raise NotImplementedError("Implement apply_action()")

# ============================================================================
# Core Functions
# ============================================================================

def load_policy(checkpoint_path: str, device: str = "cpu"):
    """
    Load policy from skrl checkpoint.

    Returns policy state dict and state preprocessor (running mean/var).
    """
    checkpoint = torch.load(checkpoint_path, map_location=device)

    policy_state = checkpoint["policy"]
    state_preprocessor = checkpoint.get("state_preprocessor", None)

    return policy_state, state_preprocessor, device


def build_observation(joint_pos: np.ndarray, joint_vel: np.ndarray,
                      target_joints: np.ndarray, last_action: np.ndarray) -> np.ndarray:
    """
    Build observation vector (21 dims).

    Order: [joint_position_error(7), joint_vel(7), last_action(7)]

    This matches the observation space in rel_cat_joint_reach_env_cfg.py
    """
    # joint_position_error = goal - current (direction to move)
    joint_position_error = target_joints - joint_pos

    obs = np.concatenate([
        joint_position_error,  # 7
        joint_vel,             # 7
        last_action,           # 7
    ])
    return obs.astype(np.float32)


def normalize_observation(obs: np.ndarray, running_mean: np.ndarray,
                          running_var: np.ndarray, epsilon: float = 1e-8) -> np.ndarray:
    """Apply running standard scaler normalization (same as training)."""
    normalized = (obs - running_mean) / np.sqrt(running_var + epsilon)
    return normalized.astype(np.float32)  # Ensure float32 for torch


def process_action(raw_action: np.ndarray, current_joint_pos: np.ndarray,
                   action_scale: float) -> np.ndarray:
    """
    Process raw policy output to joint position command.

    Relative action: target = current_pos + clamp(raw, -1, 1) * scale
    """
    # Clamp raw action to [-1, 1] (same as training)
    clamped = np.clip(raw_action, -1.0, 1.0)

    # Compute target position
    target = current_joint_pos + clamped * action_scale

    # Safety: clip to joint limits
    target = np.clip(target, FR3_JOINT_POS_MIN, FR3_JOINT_POS_MAX)

    return target


def run_policy(policy_state, state_preprocessor, device: str,
               target_joints: np.ndarray, action_scale: float,
               control_freq: float = 30.0):
    """Main control loop."""

    # Build policy network (matches skrl: shared 2x64 MLP)
    obs_dim = 21
    action_dim = 7

    # skrl uses "net_container" for shared layers, then policy_layer for output
    policy = torch.nn.Sequential(
        torch.nn.Linear(obs_dim, 64),
        torch.nn.ELU(),
        torch.nn.Linear(64, 64),
        torch.nn.ELU(),
        torch.nn.Linear(64, action_dim),  # policy_layer outputs mean action
    ).to(device)

    # Map skrl checkpoint keys to our sequential model
    policy[0].weight.data = policy_state["net_container.0.weight"]
    policy[0].bias.data = policy_state["net_container.0.bias"]
    policy[2].weight.data = policy_state["net_container.2.weight"]
    policy[2].bias.data = policy_state["net_container.2.bias"]
    policy[4].weight.data = policy_state["policy_layer.weight"]
    policy[4].bias.data = policy_state["policy_layer.bias"]
    policy.eval()

    # Get normalization parameters
    if state_preprocessor is not None:
        running_mean = state_preprocessor["running_mean"].cpu().numpy()
        running_var = state_preprocessor["running_variance"].cpu().numpy()
        print(f"Using observation normalization (mean shape: {running_mean.shape})")
    else:
        running_mean = np.zeros(obs_dim)
        running_var = np.ones(obs_dim)
        print("WARNING: No state preprocessor found, using identity normalization")

    # State
    last_action = np.zeros(7, dtype=np.float32)
    control_dt = 1.0 / control_freq
    step = 0

    print(f"\n{'='*60}")
    print(f"Starting control loop at {control_freq} Hz")
    print(f"Action scale: {action_scale}")
    print(f"Target joints: {target_joints}")
    print(f"Press Ctrl+C to stop")
    print(f"{'='*60}\n")

    try:
        while True:
            t0 = time.time()

            # 1. Read robot state
            joint_pos = read_joint_positions()
            joint_vel = read_joint_velocities()

            # 2. Build observation
            obs = build_observation(joint_pos, joint_vel, target_joints, last_action)

            # 3. Normalize observation (same as training)
            obs_normalized = normalize_observation(obs, running_mean, running_var)
            obs_tensor = torch.from_numpy(obs_normalized).unsqueeze(0).to(device)

            # 4. Get action from policy
            with torch.no_grad():
                raw_action = policy(obs_tensor)[0].cpu().numpy()

            # 5. Process action (relative)
            action = process_action(raw_action, joint_pos, action_scale)

            # 6. Apply to robot
            apply_action(action)

            # 7. Update state
            last_action = action.copy()
            step += 1

            # 8. Print status
            if step % 30 == 0:
                error = np.linalg.norm(joint_pos - target_joints)
                print(f"Step {step:4d} | Error: {error:.4f} rad | "
                      f"Raw: [{raw_action[0]:+.3f}, {raw_action[1]:+.3f}, ...] | "
                      f"Pos: [{joint_pos[0]:+.3f}, {joint_pos[1]:+.3f}, ...]")

            # 9. Sleep to maintain frequency
            elapsed = time.time() - t0
            if elapsed < control_dt:
                time.sleep(control_dt - elapsed)

    except KeyboardInterrupt:
        print(f"\n\nStopped after {step} steps")


# ============================================================================
# Main
# ============================================================================

async def main(args):
    # load robot
    robot = RobotInterface("172.16.0.2") 
    controller = FrankaController(robot)
    
    await controller.start()

    # move robot to initial pose
    await controller.move(FR3_DEFAULT_JOINT_POS_SAFE)

    # Sample target from training distribution if not provided
    if args.target_joints is None:
        target_joints = sample_target_from_training_distribution()
        print(f"Sampled target from training distribution: {target_joints}")
    else:
        target_joints = np.array(args.target_joints, dtype=np.float32)

    print(f"Loading checkpoint: {args.checkpoint}")
    policy_state, state_preprocessor, device = load_policy(args.checkpoint, args.device)

    print(f"Running with:")
    print(f"  action_scale: {args.action_scale}")
    print(f"  target_joints: {target_joints}")
    print(f"  control_freq: {args.control_freq} Hz")

    run_policy(policy_state, state_preprocessor, device, target_joints,
               args.action_scale, args.control_freq)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Minimal relative action robot control")
    parser.add_argument("--checkpoint", type=str, required=True, help="Path to checkpoint")
    parser.add_argument("--action_scale", type=float, required=True, help="Action scale")
    parser.add_argument("--target_joints", type=float, nargs=7, default=None,
                        help="Target joint positions (7 values)")
    parser.add_argument("--control_freq", type=float, default=30.0, help="Control frequency Hz")
    parser.add_argument("--device", type=str, default="cpu", help="Device (cpu/cuda)")
    args = parser.parse_args()

    asyncio.run(main(args))
