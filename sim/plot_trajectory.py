#!/usr/bin/env python3
"""
LunarAscent - Trajectory Visualization
6-DOF launch vehicle flight profile plotter
Reads trajectory.csv from main.cpp simulation output

Author: LunarAscent Team
License: MIT
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
import os
import sys


def load_trajectory(filename: str = "trajectory.csv") -> dict:
    if not os.path.exists(filename):
        print(f"[ERROR] File not found: {filename}")
        print("Run the C++ simulation first: ./lunar_ascent")
        sys.exit(1)

    data = np.loadtxt(filename, delimiter=',', skiprows=1)

    if data.shape[1] < 12:
        print(f"[ERROR] Expected 12 columns, got {data.shape[1]}")
        sys.exit(1)

    return {
        'time_s': data[:, 0],
        'altitude_km': data[:, 1],
        'velocity_m_s': data[:, 2],
        'mach': data[:, 3],
        'q_kpa': data[:, 4],
        'accel_g': data[:, 5],
        'mass_kg': data[:, 6],
        'thrust_N': data[:, 7],
        'pitch_deg': data[:, 8],
        'throttle_pct': data[:, 9],
        'stage': data[:, 10],
        'phase': data[:, 11],
    }


def find_events(data: dict) -> dict:
    events = {}

    max_q_idx = np.argmax(data['q_kpa'])
    events['max_q'] = {
        'time': data['time_s'][max_q_idx],
        'value': data['q_kpa'][max_q_idx],
        'label': f"Max-Q: {data['q_kpa'][max_q_idx]:.1f} kPa"
    }

    stage_changes = np.where(np.diff(data['stage']) > 0)[0]
    for i, idx in enumerate(stage_changes):
        stage_num = int(data['stage'][idx])
        events[f'stage_{stage_num+1}_start'] = {
            'time': data['time_s'][idx],
            'label': f"Stage {stage_num+2} Ignition"
        }
        events[f'stage_{stage_num}_end'] = {
            'time': data['time_s'][idx],
            'label': f"Stage {stage_num+1} Cutoff (MECO)"
        }

    max_accel_idx = np.argmax(data['accel_g'])
    events['max_accel'] = {
        'time': data['time_s'][max_accel_idx],
        'value': data['accel_g'][max_accel_idx],
        'label': f"Max Accel: {data['accel_g'][max_accel_idx]:.1f}g"
    }

    throttle_drops = np.where(np.diff(data['throttle_pct']) < -10)[0]
    if len(throttle_drops) > 0:
        events['throttle_drop'] = {
            'time': data['time_s'][throttle_drops[0]],
            'label': 'Throttle Down'
        }

    return events


def plot_trajectory(data: dict, events: dict, save: bool = True):
    fig, axes = plt.subplots(3, 2, figsize=(16, 12))
    fig.suptitle("LunarAscent - Launch Vehicle Trajectory Profile",
                 fontsize=18, fontweight='bold', color='white')

    bg_color = '#0a0a1a'
    fig.patch.set_facecolor(bg_color)
    for ax in axes.flat:
        ax.set_facecolor('#0d0d2a')
        ax.tick_params(colors='white')
        ax.xaxis.label.set_color('white')
        ax.yaxis.label.set_color('white')
        ax.title.set_color('white')
        for spine in ax.spines.values():
            spine.set_color('#333366')

    ax1 = axes[0, 0]
    ax1.plot(data['time_s'], data['altitude_km'], '#00ff88', linewidth=2)
    ax1.set_title("Altitude vs Time", fontsize=13, fontweight='bold')
    ax1.set_ylabel("Altitude (km)")
    ax1.set_xlabel("Time (s)")
    ax1.grid(True, alpha=0.2, color='white')
    ax1.fill_between(data['time_s'], 0, data['altitude_km'], alpha=0.1, color='#00ff88')

    for key, event in events.items():
        if event['time'] < data['time_s'][-1]:
            ax1.axvline(x=event['time'], color='#ff8800', linestyle='--',
                       linewidth=1, alpha=0.6)
            ax1.text(event['time'], data['altitude_km'].max() * (0.75 - 0.05 * list(events.keys()).index(key)),
                    event['label'], color='#ff8800', fontsize=8, rotation=90,
                    verticalalignment='center')

    ax2 = axes[0, 1]
    ax2.plot(data['time_s'], data['velocity_m_s'], '#ff4488', linewidth=2)
    ax2.set_title("Velocity vs Time", fontsize=13, fontweight='bold')
    ax2.set_ylabel("Velocity (m/s)")
    ax2.set_xlabel("Time (s)")
    ax2.grid(True, alpha=0.2, color='white')
    ax2.fill_between(data['time_s'], 0, data['velocity_m_s'], alpha=0.1, color='#ff4488')

    ax3 = axes[1, 0]
    ax3.plot(data['time_s'], data['accel_g'], '#ffaa00', linewidth=2, label='Acceleration')
    ax3.plot(data['time_s'], data['q_kpa'] / 10.0, '#4488ff', linewidth=2, label='Q/10 (kPa)')
    ax3.set_title("Acceleration & Dynamic Pressure", fontsize=13, fontweight='bold')
    ax3.set_ylabel("Acceleration (g)")
    ax3.set_xlabel("Time (s)")
    ax3.legend(loc='upper left', facecolor='#0d0d2a', edgecolor='#333366',
              labelcolor='white')
    ax3.grid(True, alpha=0.2, color='white')

    ax4 = axes[1, 1]
    ax4.plot(data['time_s'], data['mach'], '#ff44ff', linewidth=2)
    ax4.axhline(y=1.0, color='red', linestyle='--', linewidth=1, alpha=0.5, label='Mach 1')
    ax4.set_title("Mach Number", fontsize=13, fontweight='bold')
    ax4.set_ylabel("Mach")
    ax4.set_xlabel("Time (s)")
    ax4.legend(loc='upper left', facecolor='#0d0d2a', edgecolor='#333366',
              labelcolor='white')
    ax4.grid(True, alpha=0.2, color='white')

    ax5 = axes[2, 0]
    ax5.plot(data['time_s'], data['pitch_deg'], '#00ccff', linewidth=2, label='Pitch')
    ax5.plot(data['time_s'], data['throttle_pct'], '#ff8800', linewidth=2, label='Throttle %')
    ax5.set_title("Pitch Angle & Throttle", fontsize=13, fontweight='bold')
    ax5.set_ylabel("Pitch (deg) / Throttle (%)")
    ax5.set_xlabel("Time (s)")
    ax5.legend(loc='upper right', facecolor='#0d0d2a', edgecolor='#333366',
              labelcolor='white')
    ax5.grid(True, alpha=0.2, color='white')
    ax5.set_ylim(0, 110)

    ax6 = axes[2, 1]
    ax6.plot(data['time_s'], data['mass_kg'], '#ffff44', linewidth=2)
    ax6.fill_between(data['time_s'], data['mass_kg'].min() * 0.9, data['mass_kg'],
                     alpha=0.3, color='#ffff44')
    ax6.set_title("Vehicle Mass", fontsize=13, fontweight='bold')
    ax6.set_ylabel("Mass (kg)")
    ax6.set_xlabel("Time (s)")
    ax6.grid(True, alpha=0.2, color='white')

    flight_phases = ['PRE-LAUNCH', 'VERT ASC', 'PITCH OVER', 'GRAV TURN',
                     'MAX-Q', 'OPEN LOOP', 'CLSD LOOP', 'ORBIT INJ', 'COAST', 'TLI', 'TERM', 'DONE']
    phase_str = " → ".join(flight_phases[:8])
    fig.text(0.5, 0.01, f"Flight Profile: {phase_str}",
             ha='center', fontsize=10, color='#888888', fontfamily='monospace')

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])

    if save:
        output_file = "trajectory_profile.png"
        plt.savefig(output_file, dpi=200, bbox_inches='tight', facecolor=bg_color)
        print(f"[Plot] Saved trajectory profile to: {output_file}")

    plt.show()


def main():
    print("=" * 60)
    print("  LunarAscent - Trajectory Visualizer")
    print("=" * 60)

    csv_file = "trajectory.csv"
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]

    print(f"\nLoading: {csv_file}")
    data = load_trajectory(csv_file)
    print(f"Loaded {len(data['time_s'])} data points")
    print(f"Time range: {data['time_s'][0]:.1f}s - {data['time_s'][-1]:.1f}s")
    print(f"Max altitude: {data['altitude_km'].max():.1f} km")
    print(f"Max velocity: {data['velocity_m_s'].max():.1f} m/s")
    print(f"Max Mach: {data['mach'].max():.2f}")
    print(f"Max Q: {data['q_kpa'].max():.1f} kPa")
    print(f"Max acceleration: {data['accel_g'].max():.1f}g")
    print(f"Final mass: {data['mass_kg'][-1]:.0f} kg")

    events = find_events(data)
    print(f"\nDetected {len(events)} mission events:")
    for key, event in events.items():
        print(f"  T+{event['time']:.1f}s: {event['label']}")

    print("\nGenerating plots...")
    plot_trajectory(data, events, save=True)

    print("Done.")


if __name__ == "__main__":
    main()