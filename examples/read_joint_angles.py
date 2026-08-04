#!/usr/bin/env python3
"""
Tutorial 5: Read actual mechanical joint angles from the motors.
教程 5：读取电机实际机械位置。

Run / 运行：
    python3 examples/read_joint_angles.py

Expected motion / 预期动作：
    1. 启动后立刻打印「启动后实际角度」：应与当时机械臂真实姿态接近。
       First print shows actual angles right after start — close to real pose.
    2. J1 以约 15°/s 向 +20° 运动约 5 s（或直至被中断）。
       J1 moves toward +20° at ~15°/s for about 5 s (unless interrupted).
    3. 运动中终端显示发送角度逐渐变化；5 s 后打印「运动后实际角度」。
       Sent angles update on screen; after 5 s, print actual angles again.
    4. 运动后 J1 的实际角度应接近 20°（与发送/目标接近，允许小误差）。
       After motion, J1 actual angle should be near 20° (small error OK).
    5. 最后缓慢回零并失能。
       Finally slow return-to-zero and disable.
"""

from __future__ import annotations

import time

from _bootstrap import setup_project_path

setup_project_path()

from rebot import ReBotRSMITController


def print_expected_motion() -> None:
    print(
        "[Expected / 预期] Print actual angles before/after J1 moves to +20°; "
        "actual should track sent (~20°); then return-to-zero / "
        "运动前后各打印一次实际角度；J1 运动后实际值应接近 20°；随后回零"
    )


def main() -> None:
    print_expected_motion()

    arm = ReBotRSMITController()

    try:
        arm.start(enable_esc=True, install_signal_handlers=True)
        arm.set_max_speeds([15.0] * 7)

        actual = arm.read_joint_angles()
        print(
            f"启动后实际角度 / actual after start: {actual} deg / 度"
        )

        arm.set_joint_angles([20.0, 0, 0, 0, 0, 0, 0])

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not arm.is_stopped:
            sent = arm.get_command_angles()
            print(
                f"\r发送角度 / sent: "
                f"{[round(x, 2) for x in sent]}",
                end="",
                flush=True,
            )
            time.sleep(0.1)

        print()
        actual = arm.read_joint_angles()
        print(
            f"运动后实际角度 / actual after motion: {actual} deg / 度"
        )

    finally:
        if not arm.is_stopped:
            arm.stop(return_to_zero=True, wait=True)
        else:
            arm.wait_until_stopped()


if __name__ == "__main__":
    main()
