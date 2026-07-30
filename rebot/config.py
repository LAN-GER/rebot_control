#!/usr/bin/env python3
"""
配置层：reBot B601-RS 的硬件、控制与安全配置。

配置文件为 YAML，默认位于项目根目录的 config/rebotarm_rs.yaml：

    from rebot import load_config

    config = load_config()                       # config/rebotarm_rs.yaml
    config = load_config("config/my_arm.yaml")   # 指定其他配置文件

控制器（rebot.controller）不硬编码任何数值。
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from pathlib import Path

# 默认配置文件路径：项目根目录/config/rebotarm_rs.yaml。
DEFAULT_CONFIG_PATH = (
    Path(__file__).resolve().parent.parent
    / "config"
    / "rebotarm_rs.yaml"
)


@dataclass(frozen=True)
class MotorConfig:
    """单个电机的配置。"""

    motor_id: int
    model: str
    kp: float
    kd: float


@dataclass(frozen=True)
class TemperatureThresholds:
    """三级温度保护阈值，单位：°C。"""

    alarm_c: float = 80.0
    return_zero_c: float = 100.0
    disconnect_c: float = 140.0

    def __post_init__(self) -> None:
        if not (
            self.alarm_c
            < self.return_zero_c
            < self.disconnect_c
        ):
            raise ValueError(
                "温度阈值必须满足："
                "报警 < 回零 < 立即断开"
            )


@dataclass(frozen=True)
class ReturnZeroConfig:
    """安全回零参数。"""

    # 普通 Esc/Ctrl+C/stop() 回零峰值速度，单位：度/秒。
    max_speed_deg_s: float = 15.0

    # 高温触发后的回零峰值速度，单位：度/秒。
    thermal_max_speed_deg_s: float = 8.0

    # 最短回零时间，单位：秒。
    min_time_s: float = 3.0

    # 到达零点后保持时间，单位：秒。
    settle_time_s: float = 0.30


# reBot B601-RS 默认电机配置：
# J1-J3: RS06
# J4-J6: RS00
#
# MIT 参数参考 reBotArm_control_py 的 RS 配置。
DEFAULT_MOTORS: tuple[MotorConfig, ...] = (
    MotorConfig(motor_id=1, model="rs-06", kp=50.0, kd=3.0),
    MotorConfig(motor_id=2, model="rs-06", kp=150.0, kd=10.0),
    MotorConfig(motor_id=3, model="rs-06", kp=150.0, kd=10.0),
    MotorConfig(motor_id=4, model="rs-00", kp=50.0, kd=5.0),
    MotorConfig(motor_id=5, model="rs-00", kp=50.0, kd=4.0),
    MotorConfig(motor_id=6, model="rs-00", kp=50.0, kd=4.0),
)


@dataclass(frozen=True)
class ControllerConfig:
    """控制器完整配置。"""

    # CAN 接口与主机 ID。
    channel: str = "can0"
    host_id: int = 0xFD

    # MIT 指令发送频率，不是机械臂运动速度。
    control_hz: float = 200.0

    # 温度读取频率。
    telemetry_hz: float = 2.0

    motors: tuple[MotorConfig, ...] = DEFAULT_MOTORS

    temperatures: TemperatureThresholds = field(
        default_factory=TemperatureThresholds
    )

    return_zero: ReturnZeroConfig = field(
        default_factory=ReturnZeroConfig
    )

    def __post_init__(self) -> None:
        if self.control_hz <= 0:
            raise ValueError("control_hz 必须大于 0")

        if self.telemetry_hz <= 0:
            raise ValueError("telemetry_hz 必须大于 0")

        if not self.motors:
            raise ValueError("motors 不能为空")

    @classmethod
    def from_yaml(
        cls,
        path: str | Path,
    ) -> "ControllerConfig":
        """从 YAML 文件加载配置，未填写的项使用默认值。"""

        try:
            import yaml
        except ImportError as error:
            raise ImportError(
                "读取 YAML 配置需要 PyYAML："
                "pip install pyyaml"
            ) from error

        path = Path(path)

        if not path.is_file():
            raise FileNotFoundError(
                f"配置文件不存在：{path}"
            )

        data = yaml.safe_load(
            path.read_text(encoding="utf-8")
        )

        if data is None:
            data = {}

        if not isinstance(data, dict):
            raise ValueError(
                f"配置文件格式错误：{path}，"
                "顶层必须是键值对"
            )

        can = _section(data, "can", path)
        control = _section(data, "control", path)

        # replace() 遇到未知键会抛 TypeError，可发现配置笔误。
        try:
            temperatures = replace(
                TemperatureThresholds(),
                **_section(data, "temperatures", path),
            )
            return_zero = replace(
                ReturnZeroConfig(),
                **_section(data, "return_zero", path),
            )
        except TypeError as error:
            raise ValueError(
                f"配置文件 {path} 含有未知配置项：{error}"
            ) from error

        motors_data = data.get("motors")

        if motors_data is None:
            motors = DEFAULT_MOTORS
        else:
            if not isinstance(motors_data, list):
                raise ValueError(
                    f"配置文件 {path} 中 motors "
                    "必须是列表"
                )

            motors = tuple(
                MotorConfig(
                    motor_id=int(item["motor_id"]),
                    model=str(item["model"]),
                    kp=float(item["kp"]),
                    kd=float(item["kd"]),
                )
                for item in motors_data
            )

        return cls(
            channel=str(
                can.get("channel", cls.channel)
            ),
            host_id=int(
                can.get("host_id", cls.host_id)
            ),
            control_hz=float(
                control.get(
                    "control_hz",
                    cls.control_hz,
                )
            ),
            telemetry_hz=float(
                control.get(
                    "telemetry_hz",
                    cls.telemetry_hz,
                )
            ),
            motors=motors,
            temperatures=temperatures,
            return_zero=return_zero,
        )


def _section(
    data: dict,
    name: str,
    path: Path,
) -> dict:
    """取出一个配置分组，缺省返回空 dict。"""

    section = data.get(name)

    if section is None:
        return {}

    if not isinstance(section, dict):
        raise ValueError(
            f"配置文件 {path} 中 {name} "
            "必须是键值对"
        )

    return section


def load_config(
    path: str | Path | None = None,
) -> ControllerConfig:
    """
    加载控制器配置。

    不传 path 时读取默认配置文件 config/rebotarm_rs.yaml。
    """

    if path is None:
        path = DEFAULT_CONFIG_PATH

    return ControllerConfig.from_yaml(path)
