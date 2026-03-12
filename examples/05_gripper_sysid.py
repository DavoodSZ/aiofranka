"""
Gripper Response Curve Collection

Collects gripper response data under different speed settings by exciting with:
1. Sine wave (2 seconds, starting from middle position)
2. Square wave to 0 (1 second)
3. Square wave to 255 (1 second)

The collected data is saved to a .npz file and plotted to a .png file.

Requirements:
    pip install "aiofranka[robotiq]"

Usage:
    python examples/11_gripper_sysid.py --speed 128
    python examples/11_gripper_sysid.py --speed 64 --output_dir ./gripper_sysid/
"""

import asyncio
import argparse
import os
import numpy as np
import matplotlib.pyplot as plt
from aiofranka import GripperController


async def collect_response(gripper: GripperController, speed: int, freq: float = 50.0):
    """
    Collect gripper response data.
    
    Args:
        gripper: Initialized GripperController
        speed: Gripper speed setting (1-255)
        freq: Data collection frequency in Hz
        
    Returns:
        dict: Collected data with qpos, q_desired, time arrays
    """
    dt = 1.0 / freq
    
    logs = {
        'qpos': [],
        'q_desired': [],
        'time': [],
        'speed': speed,
    }
    
    t = 0.0
    
    # --- Phase 1: Move to middle position ---
    print("Moving to middle position (128)...")
    gripper.speed = 255  # Fast move to start
    gripper.q_desired = 128
    gripper.set_freq(freq)  # Ensure we get updates at the desired frequency
    await gripper.wait_until_reached(tolerance=5, timeout=3.0)
    await asyncio.sleep(0.5)
    
    # Set test speed
    gripper.speed = speed
    print(f"Starting data collection at speed={speed}...")
    
    # --- Phase 2: Sine wave (2 seconds) ---
    print("  Phase 1/3: Sine wave excitation (2s)...")
    sine_duration = 2.0
    sine_steps = int(sine_duration * freq)
    
    for i in range(sine_steps):
        # Sine wave: amplitude 60, centered at 128, frequency ~0.5Hz
        target = int(128 + 60 * np.sin(i / freq * np.pi))  # 1 full cycle in 2 seconds
        # gripper.q_desired = target
        await gripper.set("q_desired", target)  # Use async set to ensure timing
        
        logs['time'].append(t)
        logs['qpos'].append(gripper.qpos)
        logs['q_desired'].append(target)
        
        # await asyncio.sleep(dt)
        t += dt
    
    # --- Phase 3: Square wave to 0 (1 second) ---
    print("  Phase 2/3: Square wave to 0 (1s)...")
    square_duration = 2.0
    square_steps = int(square_duration * freq)
    
    for i in range(square_steps):
        await gripper.set("q_desired", 0)
        
        logs['time'].append(t)
        logs['qpos'].append(gripper.qpos)
        logs['q_desired'].append(0)
        t += dt
    
    # --- Phase 4: Square wave to 255 (1 second) ---
    print("  Phase 3/3: Square wave to 255 (1s)...")
    
    await gripper.set("q_desired", 200)
    for i in range(square_steps):
        await gripper.set("q_desired", 200)
        logs['time'].append(t)
        logs['qpos'].append(gripper.qpos)
        logs['q_desired'].append(200)
        # await asyncio.sleep(dt)
        t += dt
    
    # Convert to numpy arrays
    for key in ['qpos', 'q_desired', 'time']:
        logs[key] = np.array(logs[key])
    
    return logs


def plot_response(logs: dict, output_path: str):
    """
    Plot gripper response and save to file.
    
    Args:
        logs: Collected data dictionary
        output_path: Path to save the plot
    """
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    
    time = logs['time']
    qpos = logs['qpos']
    q_desired = logs['q_desired']
    speed = logs['speed']
    
    # Plot 1: Position tracking
    ax1 = axes[0]
    ax1.plot(time, q_desired, 'r--', label='q_desired (target)', linewidth=2, alpha=0.8)
    ax1.plot(time, qpos, 'b-', label='qpos (actual)', linewidth=1.5)
    ax1.set_ylabel('Position (0-255)')
    ax1.set_title(f'Gripper Response Curve (speed={speed})')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim(-10, 265)
    
    # Add phase annotations
    ax1.axvspan(0, 2, alpha=0.1, color='green', label='Sine')
    ax1.axvspan(2, 4, alpha=0.1, color='blue', label='Square→0')
    ax1.axvspan(4, 6, alpha=0.1, color='red', label='Square→255')
    ax1.text(1.0, 250, 'Sine Wave', ha='center', fontsize=10, color='green')
    ax1.text(3.0, 250, '→0', ha='center', fontsize=10, color='blue')
    ax1.text(5.0, 250, '→255', ha='center', fontsize=10, color='red')
    
    # Plot 2: Tracking error
    ax2 = axes[1]
    error = qpos - q_desired
    ax2.plot(time, error, 'k-', linewidth=1)
    ax2.fill_between(time, error, 0, alpha=0.3)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Error (qpos - q_desired)')
    ax2.set_title('Tracking Error')
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=0, color='r', linestyle='--', alpha=0.5)
    
    # Add statistics
    rmse = np.sqrt(np.mean(error**2))
    max_error = np.max(np.abs(error))
    ax2.text(0.02, 0.95, f'RMSE: {rmse:.1f}\nMax Error: {max_error:.1f}', 
             transform=ax2.transAxes, fontsize=10, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Plot saved to: {output_path}")


async def main():
    parser = argparse.ArgumentParser(description='Collect gripper response curves')
    parser.add_argument('--speed', type=int, default=128,
                        help='Gripper speed (1-255, default: 128)')
    parser.add_argument('--port', type=str, default='/dev/ttyUSB1',
                        help='Serial port for gripper (default: /dev/ttyUSB1)')
    parser.add_argument('--output_dir', type=str, default='./examples/gripper_sysid/',
                        help='Output directory for saved files')
    parser.add_argument('--freq', type=float, default=50.0,
                        help='Data collection frequency in Hz (default: 50)')
    args = parser.parse_args()
    
    # Validate speed
    speed = max(1, min(255, args.speed))
    
    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)
    
    print("=" * 50)
    print("Gripper Response Curve Collection")
    print("=" * 50)
    print(f"  Speed: {speed}")
    print(f"  Port: {args.port}")
    print(f"  Frequency: {args.freq} Hz")
    print(f"  Output: {args.output_dir}")
    print("=" * 50)
    
    # Initialize gripper
    gripper = GripperController(args.port, loop_rate=200, read_every_n = 1)
    await gripper.start()
    
    try:
        # Collect response data
        logs = await collect_response(gripper, speed, args.freq)
        
        # Save data
        npz_path = os.path.join(args.output_dir, f'gripper_response_speed{speed}.npz')
        np.savez(npz_path, **logs)
        print(f"Data saved to: {npz_path}")
        
        # Plot and save
        png_path = os.path.join(args.output_dir, f'gripper_response_speed{speed}.png')
        plot_response(logs, png_path)
        
        # Print summary
        print("\n" + "=" * 50)
        print("Summary:")
        print("=" * 50)
        error = logs['qpos'] - logs['q_desired']
        print(f"  Total samples: {len(logs['time'])}")
        print(f"  Duration: {logs['time'][-1]:.2f}s")
        print(f"  RMSE: {np.sqrt(np.mean(error**2)):.2f}")
        print(f"  Max error: {np.max(np.abs(error)):.2f}")
        print("=" * 50)
        
    finally:
        # Return to open and stop
        gripper.q_desired = 0
        await asyncio.sleep(1.0)
        await gripper.stop()


if __name__ == "__main__":
    asyncio.run(main())
