#!/usr/bin/env python3
"""
Example layer: MIT position control demo for reBot B601-RS.
示例层：reBot B601-RS MIT 位置控制演示。

Features / 功能：
1. Fill in the 6 joint target angles manually in TARGET_ANGLES.
   在 TARGET_ANGLES 列表中手动填写 6 个关节目标角度。
2. All joints default to 20 deg/s.
   六个关节默认速度统一为 20 度/秒。
3. Press Esc, Ctrl+C, or let the program end to slowly return to zero.
   按 Esc、Ctrl+C 或程序结束时缓慢回到零点。

Run / 运行：
    python3 examples/mit_position_control.py
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

# Allow running this file directly from the project root.
# 允许从项目根目录直接运行本文件。
sys.path.insert(
    0,
    str(Path(__file__).resolve().parent.parent),
)

from rebot import ReBotRSMITController, load_config


# =============================================================================
# Demo parameters / 演示参数
# =============================================================================

# Controller config file path; None means the default config/rebotarm_rs.yaml.
# 控制器配置文件路径；None 表示使用默认的 config/rebotarm_rs.yaml。
CONFIG_PATH = None

# Target angles for J1-J6, in degrees. / 目标角度，顺序为 J1-J6，单位：度。
TARGET_ANGLES = [
    50.0,  # J1
    0.0,   # J2
    0.0,   # J3
    0.0,   # J4
    0.0,   # J5
    0.0,   # J6
]

# Max motion speed of each joint, in deg/s.
# 六个关节的最大运动速度，单位：度/秒。
JOINT_SPEEDS_DEG_S = [
    20.0,  # J1
    20.0,  # J2
    20.0,  # J3
    20.0,  # J4
    20.0,  # J5
    20.0,  # J6
]


def main() -> None:
    if CONFIG_PATH is None:
        # Without a config it loads config/rebotarm_rs.yaml automatically.
        # 不传配置时自动加载 config/rebotarm_rs.yaml。
        arm = ReBotRSMITController()
    else:
        arm = ReBotRSMITController(
            load_config(CONFIG_PATH)
        )

    try:
        arm.start(
            enable_esc=True,
            install_signal_handlers=True,
        )

        # Set all six joints to 20 deg/s. / 六个关节统一设置为 20°/s。
        arm.set_max_speeds(JOINT_SPEEDS_DEG_S)

        print(
            "[Speed / 速度] "
            f"Speed: {JOINT_SPEEDS_DEG_S} deg/s / "
            f"速度: {JOINT_SPEEDS_DEG_S} 度/秒"
        )

        print(
            "[Target / 目标] "
            f"Target: {TARGET_ANGLES} deg / "
            f"目标: {TARGET_ANGLES} 度"
        )

        # This line must run, otherwise the arm never updates its target.
        # 这一行必须执行，机械臂才会更新目标。
        arm.set_joint_angles(TARGET_ANGLES)

        while not arm.is_stopped:
            target_angles = arm.get_target_angles()
            command_angles = arm.get_command_angles()

            temperature_text = [
                "--"
                if value is None
                else round(value, 1)
                for value in arm.last_temperatures
            ]

            print(
                "\r"
                f"Target/目标: {[round(x, 2) for x in target_angles]}  |  "
                f"Sent/发送: {[round(x, 2) for x in command_angles]}  |  "
                f"MOS temp/MOS温度: {temperature_text}",
                end="",
                flush=True,
            )

            time.sleep(0.10)

    except Exception as error:
        print(
            f"\n[Program error / 程序错误] Error: {error} / 错误: {error}"
        )

    finally:
        if not arm.is_stopped:
            arm.stop(
                return_to_zero=True,
                wait=True,
            )
        else:
            arm.wait_until_stopped()


if __name__ == "__main__":
    main()
