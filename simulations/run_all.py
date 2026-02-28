#!/usr/bin/env python3
"""
一鍵執行所有模擬
==================
依序執行 4 個模擬腳本，驗證 ArduPlane EKF3 和 PID 調參文件的核心概念。

使用方式：
    python3 simulations/run_all.py
"""

import subprocess
import sys
import os
import time


def run_simulation(script_name, description):
    """執行單一模擬腳本"""
    print(f"\n{'='*70}")
    print(f"  {description}")
    print(f"  Script: {script_name}")
    print(f"{'='*70}")

    script_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), script_name)

    start = time.time()
    result = subprocess.run(
        [sys.executable, script_path],
        capture_output=True, text=True, timeout=120
    )
    elapsed = time.time() - start

    if result.returncode == 0:
        print(result.stdout)
        print(f"  [COMPLETED in {elapsed:.1f}s]")
        return True
    else:
        print(f"  [FAILED]")
        print(f"  STDOUT: {result.stdout[:500]}")
        print(f"  STDERR: {result.stderr[:500]}")
        return False


def main():
    print("""
╔══════════════════════════════════════════════════════════════════════╗
║  ArduPlane EKF3 & PID Tuning - Simulation Verification Suite       ║
║  Based on: ArduPlane_EKF3_Fusion_and_PID_Tuning.md                ║
║  Mode: Software-only (no hardware required)                        ║
╚══════════════════════════════════════════════════════════════════════╝
    """)

    simulations = [
        ("sim1_ekf_state_estimation.py",
         "Sim 1: EKF State Estimation (Sections 1.1-1.5, 1.9-1.10)"),
        ("sim2_pid_tuning.py",
         "Sim 2: PID Tuning Strategy (Sections 2.1-2.5, 2.9.2)"),
        ("sim3_sensor_fusion.py",
         "Sim 3: Multi-Sensor Fusion & Multi-Lane (Sections 1.2, 1.5-1.8)"),
        ("sim4_tecs_navigation.py",
         "Sim 4: TECS & L1 Navigation (Sections 2.7-2.8)"),
    ]

    results = []
    total_start = time.time()

    for script, desc in simulations:
        success = run_simulation(script, desc)
        results.append((desc, success))

    total_time = time.time() - total_start

    # 最終摘要
    print(f"\n{'='*70}")
    print("FINAL SUMMARY")
    print(f"{'='*70}")

    all_pass = True
    for desc, success in results:
        status = "PASS" if success else "FAIL"
        icon = "[OK]" if success else "[!!]"
        print(f"  {icon} {status}: {desc}")
        if not success:
            all_pass = False

    print(f"\n  Total time: {total_time:.1f}s")

    if all_pass:
        print("\n  ALL SIMULATIONS PASSED!")
        print("  The document's core concepts have been verified through simulation.")
    else:
        print("\n  Some simulations failed. Check the output above for details.")

    # 列出產生的圖片
    output_dir = os.path.dirname(os.path.abspath(__file__))
    pngs = sorted([f for f in os.listdir(output_dir) if f.startswith('output_') and f.endswith('.png')])
    if pngs:
        print(f"\n  Generated {len(pngs)} visualization files:")
        for f in pngs:
            print(f"    - simulations/{f}")

    return 0 if all_pass else 1


if __name__ == '__main__':
    sys.exit(main())
