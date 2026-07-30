#!/usr/bin/env python3
"""
接口层：reBot B601-RS MIT 控制器。

功能：
1. MIT 模式持续发送位置指令
2. 各关节独立的速度限制
3. 实时监控 RobStride MOS 温度
4. 温度报警、超温回零、紧急失能三级保护
5. 按 Esc、Ctrl+C 或调用 arm.stop() 时缓慢回到零点
6. 回零过程中再次按 Ctrl+C，立即中止回零并失能

所有硬件与安全参数来自配置层（rebot/config.py 读取 config/*.yaml）。

角度单位：
    外部目标角度：度
    MotorBridge MIT 指令：弧度
"""

from __future__ import annotations

import math
import signal
import threading
import time
from typing import Sequence

from motorbridge import Controller, Mode

try:
    from pynput import keyboard
except Exception:
    keyboard = None

from .config import (
    DEFAULT_CONFIG_PATH,
    ControllerConfig,
    load_config,
)


class ReBotRSMITController:
    """reBot B601-RS MIT 控制器。"""

    def __init__(
        self,
        config: ControllerConfig | None = None,
    ) -> None:
        if config is None:
            config = load_config()

            print(
                "[配置] 已加载配置文件："
                f"{DEFAULT_CONFIG_PATH}"
            )

        self.config = config

        self.channel = self.config.channel
        self.control_hz = float(self.config.control_hz)
        self.telemetry_hz = float(
            self.config.telemetry_hz
        )

        self.temp_alarm_c = float(
            self.config.temperatures.alarm_c
        )
        self.temp_return_zero_c = float(
            self.config.temperatures.return_zero_c
        )
        self.temp_disconnect_c = float(
            self.config.temperatures.disconnect_c
        )

        self.motor_count = len(self.config.motors)

        self.controller: Controller | None = None
        self.motors = []

        # 用户设置的最终目标，单位：弧度。
        self.target_positions = [
            0.0 for _ in self.config.motors
        ]

        # 经过速度限制后实际发送给电机的目标，单位：弧度。
        self.command_positions = [
            0.0 for _ in self.config.motors
        ]

        self.max_speeds_rad_s = [
            math.radians(20.0)
            for _ in self.config.motors
        ]

        self.target_lock = threading.RLock()
        self.io_lock = threading.RLock()
        self.shutdown_lock = threading.Lock()

        self.worker_stop_event = threading.Event()
        self.abort_return_event = threading.Event()
        self.shutdown_done_event = threading.Event()

        self.control_thread: threading.Thread | None = None
        self.telemetry_thread: threading.Thread | None = None
        self.shutdown_thread: threading.Thread | None = None

        self.keyboard_listener = None

        self.started = False
        self.shutdown_started = False

        self.return_to_zero_requested = True
        self.thermal_return_requested = False
        self.emergency_disconnect_requested = False

        self.shutdown_reason = ""
        self.signal_count = 0
        self.last_error: Exception | None = None

        self.last_temperatures: list[float | None] = [
            None for _ in self.config.motors
        ]

    # -------------------------------------------------------------------------
    # 连接与启动
    # -------------------------------------------------------------------------

    def connect(self) -> None:
        """连接 CAN、注册电机、切换 MIT 模式并使能。"""

        if self.controller is not None:
            return

        print(f"[连接] 打开 CAN 接口：{self.channel}")

        self.controller = Controller(self.channel)

        try:
            for motor_config in self.config.motors:
                motor = self.controller.add_robstride_motor(
                    motor_config.motor_id,
                    self.config.host_id,
                    motor_config.model,
                )

                self.motors.append(motor)

                print(
                    f"[连接] 电机 {motor_config.motor_id} "
                    f"({motor_config.model}) 已注册"
                )

            # 启动前读取实际位置，避免使能后突然跳动。
            current_positions = self._read_positions_rad()

            with self.target_lock:
                self.target_positions[:] = current_positions
                self.command_positions[:] = current_positions

            print(
                "[连接] 当前角度："
                f"{[round(math.degrees(x), 2) for x in current_positions]}"
            )

            for index, motor in enumerate(self.motors):
                motor_id = self.config.motors[index].motor_id

                print(
                    f"[连接] 电机 {motor_id} "
                    "切换到 MIT 模式"
                )

                with self.io_lock:
                    motor.ensure_mode(
                        Mode.MIT,
                        1000,
                    )

                time.sleep(0.05)

            # 所有电机只使能一次。
            with self.io_lock:
                self.controller.enable_all()

            time.sleep(0.30)

            print("[连接] 所有电机已使能")

        except Exception:
            self._disable_and_close()
            raise

    def start(
        self,
        *,
        enable_esc: bool = True,
        install_signal_handlers: bool = True,
    ) -> None:
        """启动 MIT 控制线程和温度监控线程。"""

        if self.started:
            return

        self.connect()

        self.worker_stop_event.clear()
        self.abort_return_event.clear()
        self.shutdown_done_event.clear()

        self.shutdown_started = False
        self.signal_count = 0
        self.last_error = None
        self.started = True

        self.control_thread = threading.Thread(
            target=self._control_loop,
            name="rebot-rs-mit-control",
            daemon=True,
        )
        self.control_thread.start()

        self.telemetry_thread = threading.Thread(
            target=self._temperature_loop,
            name="rebot-rs-temperature",
            daemon=True,
        )
        self.telemetry_thread.start()

        if enable_esc:
            self._start_keyboard_listener()

        if (
            install_signal_handlers
            and threading.current_thread()
            is threading.main_thread()
        ):
            signal.signal(
                signal.SIGINT,
                self._signal_handler,
            )
            signal.signal(
                signal.SIGTERM,
                self._signal_handler,
            )

        print(
            f"[启动] MIT 频率："
            f"{self.control_hz:.0f} Hz"
        )
        print(
            f"[启动] 温度刷新："
            f"{self.telemetry_hz:.1f} Hz"
        )
        print(
            "[启动] 温度阈值："
            f"报警={self.temp_alarm_c:.1f}°C，"
            f"回零={self.temp_return_zero_c:.1f}°C，"
            f"断开={self.temp_disconnect_c:.1f}°C"
        )
        print(
            "[启动] Esc / Ctrl+C / arm.stop() "
            "会缓慢回零后失能"
        )

    # -------------------------------------------------------------------------
    # MIT 控制
    # -------------------------------------------------------------------------

    def _control_loop(self) -> None:
        """持续生成带速度限制的目标并发送 MIT 指令。"""

        period = 1.0 / self.control_hz
        next_tick = time.perf_counter()

        while not self.worker_stop_event.is_set():
            try:
                with self.target_lock:
                    for index in range(self.motor_count):
                        target = self.target_positions[index]
                        command = self.command_positions[index]

                        error = target - command

                        max_step = (
                            self.max_speeds_rad_s[index]
                            * period
                        )

                        step = max(
                            -max_step,
                            min(max_step, error),
                        )

                        self.command_positions[index] += step

                    commands = self.command_positions.copy()

                self._send_mit_positions(commands)

            except Exception as error:
                self.last_error = error

                print(
                    f"\n[控制错误] {error}"
                )

                # 通信异常时不再尝试回零。
                self.request_stop(
                    reason=f"控制通信异常：{error}",
                    return_to_zero=False,
                    emergency=True,
                    wait=False,
                )
                return

            next_tick += period
            sleep_time = next_tick - time.perf_counter()

            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_tick = time.perf_counter()

    def _send_mit_positions(
        self,
        positions_rad: Sequence[float],
    ) -> None:
        """向全部电机发送一组 MIT 位置指令。"""

        with self.io_lock:
            for index, motor in enumerate(self.motors):
                motor_config = self.config.motors[index]

                motor.send_mit(
                    float(positions_rad[index]),
                    0.0,
                    float(motor_config.kp),
                    float(motor_config.kd),
                    0.0,
                )

    # -------------------------------------------------------------------------
    # 外部控制接口
    # -------------------------------------------------------------------------

    def set_joint_angles(
        self,
        angles_deg: Sequence[float],
    ) -> None:
        """
        设置全部关节目标角度。

        示例：
            arm.set_joint_angles(
                [50, 0, 0, 0, 0, 0]
            )
        """

        if self.shutdown_started:
            return

        if len(angles_deg) != self.motor_count:
            raise ValueError(
                f"必须提供 {self.motor_count} 个角度，"
                f"当前收到 {len(angles_deg)} 个"
            )

        positions = []

        for index, angle in enumerate(angles_deg):
            value = float(angle)

            if not math.isfinite(value):
                raise ValueError(
                    f"关节 {index + 1} 的角度无效"
                )

            positions.append(math.radians(value))

        with self.target_lock:
            self.target_positions[:] = positions

    def set_joint_angle(
        self,
        joint_id: int,
        angle_deg: float,
    ) -> None:
        """只修改一个关节的目标角度。"""

        if self.shutdown_started:
            return

        if not 1 <= joint_id <= self.motor_count:
            raise ValueError(
                f"joint_id 必须为 1-{self.motor_count}"
            )

        value = float(angle_deg)

        if not math.isfinite(value):
            raise ValueError("angle_deg 无效")

        with self.target_lock:
            self.target_positions[joint_id - 1] = (
                math.radians(value)
            )

    def set_max_speeds(
        self,
        speeds_deg_s: Sequence[float],
    ) -> None:
        """设置各关节的最大运动速度，单位：度/秒。"""

        if len(speeds_deg_s) != self.motor_count:
            raise ValueError(
                f"必须提供 {self.motor_count} 个速度"
            )

        speeds_rad_s = []

        for index, speed in enumerate(speeds_deg_s):
            value = float(speed)

            if not math.isfinite(value) or value <= 0:
                raise ValueError(
                    f"关节 {index + 1} 的速度必须大于 0"
                )

            speeds_rad_s.append(
                math.radians(value)
            )

        with self.target_lock:
            self.max_speeds_rad_s[:] = speeds_rad_s

    def get_target_angles(self) -> list[float]:
        """返回最终目标角度，单位：度。"""

        with self.target_lock:
            values = self.target_positions.copy()

        return [
            math.degrees(value)
            for value in values
        ]

    def get_command_angles(self) -> list[float]:
        """返回当前实际发送的平滑目标角度，单位：度。"""

        with self.target_lock:
            values = self.command_positions.copy()

        return [
            math.degrees(value)
            for value in values
        ]

    def read_joint_angles(self) -> list[float]:
        """读取一次实际机械位置，单位：度。"""

        return [
            math.degrees(value)
            for value in self._read_positions_rad()
        ]

    # -------------------------------------------------------------------------
    # 位置与温度读取
    # -------------------------------------------------------------------------

    def _read_positions_rad(self) -> list[float]:
        """读取 RobStride mechPos 参数 0x7019。"""

        positions = []

        with self.io_lock:
            for index, motor in enumerate(self.motors):
                try:
                    position = motor.robstride_get_param_f32(
                        0x7019,
                        timeout_ms=500,
                    )

                    positions.append(float(position))

                except Exception as error:
                    motor_id = (
                        self.config.motors[index].motor_id
                    )

                    raise RuntimeError(
                        f"电机 {motor_id} 位置读取失败："
                        f"{error}"
                    ) from error

        return positions

    @staticmethod
    def _safe_float(value) -> float | None:
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None

        if not math.isfinite(number):
            return None

        return number

    def _read_temperatures_once(
        self,
    ) -> list[float | None]:
        """请求新反馈并读取每个电机的 MOS 温度。"""

        temperatures: list[float | None] = []

        with self.io_lock:
            for motor in self.motors:
                state = None

                try:
                    motor.request_feedback()
                except Exception:
                    pass

                try:
                    if self.controller is not None:
                        self.controller.poll_feedback_once()
                except Exception:
                    pass

                try:
                    state = motor.get_state()
                except Exception:
                    state = None

                if state is None:
                    temperatures.append(None)
                    continue

                temperatures.append(
                    self._safe_float(
                        getattr(state, "t_mos", None)
                    )
                )

        self.last_temperatures = temperatures
        return temperatures

    def _temperature_loop(self) -> None:
        """执行三级温度保护。"""

        period = 1.0 / self.telemetry_hz

        while not self.worker_stop_event.wait(period):
            try:
                temperatures = self._read_temperatures_once()

            except Exception as error:
                print(
                    f"\n[温度] 反馈读取失败：{error}"
                )
                continue

            valid_temperatures = [
                (index, temp)
                for index, temp in enumerate(temperatures)
                if temp is not None
            ]

            if not valid_temperatures:
                continue

            hottest_index, hottest_temp = max(
                valid_temperatures,
                key=lambda item: item[1],
            )

            hottest_motor_id = self.config.motors[
                hottest_index
            ].motor_id

            if hottest_temp >= self.temp_disconnect_c:
                print(
                    "\n[紧急断开] "
                    f"电机 {hottest_motor_id} MOS="
                    f"{hottest_temp:.1f}°C，"
                    "立即失能，不再回零"
                )

                self.request_stop(
                    reason=(
                        f"电机 {hottest_motor_id} "
                        "达到紧急断开温度"
                    ),
                    return_to_zero=False,
                    emergency=True,
                    wait=False,
                )
                return

            if hottest_temp >= self.temp_return_zero_c:
                print(
                    "\n[高温回零] "
                    f"电机 {hottest_motor_id} MOS="
                    f"{hottest_temp:.1f}°C，"
                    "停止运动并缓慢回零"
                )

                self.request_stop(
                    reason=(
                        f"电机 {hottest_motor_id} "
                        "达到回零温度"
                    ),
                    return_to_zero=True,
                    thermal=True,
                    wait=False,
                )
                return

            if hottest_temp >= self.temp_alarm_c:
                print(
                    "\n[温度报警] "
                    f"电机 {hottest_motor_id} MOS="
                    f"{hottest_temp:.1f}°C，"
                    "机械臂继续运行"
                )

    # -------------------------------------------------------------------------
    # Esc 和 Ctrl+C
    # -------------------------------------------------------------------------

    def _start_keyboard_listener(self) -> None:
        if keyboard is None:
            print(
                "[键盘] 未安装或无法使用 pynput，"
                "Esc 功能关闭；Ctrl+C 和 stop() 仍然有效"
            )
            return

        try:
            self.keyboard_listener = keyboard.Listener(
                on_press=self._on_key_press
            )
            self.keyboard_listener.start()

        except Exception as error:
            self.keyboard_listener = None

            print(
                f"[键盘] Esc 监听启动失败：{error}"
            )

    def _on_key_press(self, key) -> bool | None:
        if keyboard is not None and key == keyboard.Key.esc:
            print(
                "\n[退出] 检测到 Esc，"
                "正在缓慢回到零点"
            )

            self.request_stop(
                reason="用户按下 Esc",
                return_to_zero=True,
                wait=False,
            )
            return False

        return None

    def _signal_handler(self, signum, _frame) -> None:
        self.signal_count += 1

        if self.signal_count == 1:
            print(
                f"\n[退出] 收到信号 {signum}，"
                "正在缓慢回到零点"
            )

            self.request_stop(
                reason=f"收到信号 {signum}",
                return_to_zero=True,
                wait=False,
            )

        else:
            print(
                "\n[退出] 第二次收到 Ctrl+C，"
                "立即中止回零并失能"
            )

            self.abort_return_event.set()
            self.worker_stop_event.set()

    # -------------------------------------------------------------------------
    # 安全停止与回零
    # -------------------------------------------------------------------------

    @staticmethod
    def _smoothstep(alpha: float) -> float:
        alpha = max(0.0, min(1.0, alpha))
        return alpha * alpha * (3.0 - 2.0 * alpha)

    def request_stop(
        self,
        *,
        reason: str = "调用 stop()",
        return_to_zero: bool = True,
        thermal: bool = False,
        emergency: bool = False,
        wait: bool = False,
    ) -> None:
        """请求停止。第一次调用会创建安全停止线程。"""

        with self.shutdown_lock:
            if not self.shutdown_started:
                self.shutdown_started = True
                self.shutdown_reason = reason

                self.return_to_zero_requested = bool(
                    return_to_zero
                )
                self.thermal_return_requested = bool(
                    thermal
                )
                self.emergency_disconnect_requested = bool(
                    emergency
                )

                self.shutdown_thread = threading.Thread(
                    target=self._shutdown_worker,
                    name="rebot-rs-safe-shutdown",
                    daemon=True,
                )
                self.shutdown_thread.start()

        if (
            wait
            and self.shutdown_thread is not None
            and threading.current_thread()
            is not self.shutdown_thread
        ):
            self.shutdown_done_event.wait()

    def stop(
        self,
        *,
        return_to_zero: bool = True,
        wait: bool = True,
    ) -> None:
        """
        外部程序调用的停止接口。

        默认：
            缓慢回零 -> 失能 -> 关闭 CAN
        """

        self.request_stop(
            reason="程序调用 arm.stop()",
            return_to_zero=return_to_zero,
            wait=wait,
        )

    def _shutdown_worker(self) -> None:
        print(
            f"\n[停止] 原因：{self.shutdown_reason}"
        )

        # 先停止正常 MIT 控制和温度线程。
        self.worker_stop_event.set()

        current_thread = threading.current_thread()

        for thread in (
            self.control_thread,
            self.telemetry_thread,
        ):
            if (
                thread is not None
                and thread is not current_thread
                and thread.is_alive()
            ):
                thread.join(timeout=3.0)

        if self.keyboard_listener is not None:
            try:
                self.keyboard_listener.stop()
            except Exception:
                pass

        try:
            if (
                self.return_to_zero_requested
                and not self.emergency_disconnect_requested
                and self.controller is not None
                and self.motors
            ):
                self._safe_return_to_zero()

        except Exception as error:
            print(
                f"\n[回零错误] {error}"
            )

        finally:
            self._disable_and_close()

            self.started = False
            self.shutdown_done_event.set()

            print("[停止] 安全停止流程完成")

    def _safe_return_to_zero(self) -> None:
        """使用 smoothstep 轨迹缓慢返回全部关节零点。"""

        try:
            start_positions = self._read_positions_rad()

        except Exception as error:
            print(
                "[回零] 无法读取实际角度，"
                f"改用当前发送角度：{error}"
            )

            with self.target_lock:
                start_positions = (
                    self.command_positions.copy()
                )

        target_positions = [
            0.0 for _ in self.config.motors
        ]

        return_zero_config = self.config.return_zero

        speed_deg_s = (
            return_zero_config.thermal_max_speed_deg_s
            if self.thermal_return_requested
            else return_zero_config.max_speed_deg_s
        )

        # smoothstep 的最大斜率为 1.5。
        required_times = [
            (
                1.5
                * abs(math.degrees(position))
                / speed_deg_s
            )
            for position in start_positions
        ]

        duration_s = max(
            return_zero_config.min_time_s,
            *required_times,
        )

        steps = max(
            2,
            int(round(duration_s * self.control_hz)),
        )

        print(
            "[回零] 正在缓慢回零："
            f"预计 {duration_s:.1f} 秒，"
            f"峰值速度不超过 {speed_deg_s:.1f}°/s"
        )
        print(
            "[回零] 再次按 Ctrl+C 可立即中止回零"
        )

        period = 1.0 / self.control_hz
        next_tick = time.perf_counter()
        last_thermal_check = -float("inf")

        completed = True

        for step_index in range(1, steps + 1):
            if self.abort_return_event.is_set():
                print(
                    "\n[回零] 用户中止回零"
                )
                completed = False
                break

            now = time.monotonic()

            # 高温回零过程中继续检查紧急断开阈值。
            if (
                self.thermal_return_requested
                and now - last_thermal_check
                >= 1.0 / self.telemetry_hz
            ):
                last_thermal_check = now

                temperatures = (
                    self._read_temperatures_once()
                )

                valid_temperatures = [
                    (index, temp)
                    for index, temp
                    in enumerate(temperatures)
                    if temp is not None
                ]

                if valid_temperatures:
                    hottest_index, hottest_temp = max(
                        valid_temperatures,
                        key=lambda item: item[1],
                    )

                    if (
                        hottest_temp
                        >= self.temp_disconnect_c
                    ):
                        motor_id = self.config.motors[
                            hottest_index
                        ].motor_id

                        print(
                            "\n[紧急断开] 回零过程中 "
                            f"电机 {motor_id} MOS="
                            f"{hottest_temp:.1f}°C，"
                            "立即中止回零"
                        )
                        completed = False
                        break

            alpha = self._smoothstep(
                step_index / steps
            )

            commands = [
                start
                + alpha * (target - start)
                for start, target in zip(
                    start_positions,
                    target_positions,
                )
            ]

            self._send_mit_positions(commands)

            with self.target_lock:
                self.command_positions[:] = commands
                self.target_positions[:] = commands

            next_tick += period
            sleep_time = next_tick - time.perf_counter()

            if sleep_time > 0:
                time.sleep(sleep_time)

        if completed:
            self._send_mit_positions(target_positions)

            with self.target_lock:
                self.command_positions[:] = target_positions
                self.target_positions[:] = target_positions

            time.sleep(
                self.config.return_zero.settle_time_s
            )

            print(
                "[回零] 已发送零点并完成保持，"
                "准备失能"
            )

    # -------------------------------------------------------------------------
    # 资源清理
    # -------------------------------------------------------------------------

    def _disable_and_close(self) -> None:
        """失能电机并关闭 MotorBridge 资源。"""

        if self.controller is not None:
            try:
                with self.io_lock:
                    self.controller.disable_all()

                print("[失能] 所有电机已失能")

            except Exception as error:
                print(
                    f"[失能] disable_all 失败：{error}"
                )

        for motor in self.motors:
            try:
                motor.close()
            except Exception:
                pass

        self.motors.clear()

        if self.controller is not None:
            try:
                self.controller.shutdown()
            except Exception:
                pass

            try:
                self.controller.close()
            except Exception:
                pass

        self.controller = None

    @property
    def is_stopped(self) -> bool:
        return self.shutdown_done_event.is_set()

    def wait_until_stopped(
        self,
        timeout: float | None = None,
    ) -> bool:
        return self.shutdown_done_event.wait(timeout)
