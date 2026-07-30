# rebot_control

reBot B601-RS 六轴机械臂 MIT 位置控制程序（Python），基于 MotorBridge 通过 CAN 总线控制 RobStride 电机。

## 硬件配置

- 机械臂：reBot B601-RS（6 关节）
- 电机：J1-J3 为 RS06，J4-J6 为 RS00
- 通信：CAN 总线（默认 `can0`，主机 ID `0xFD`）

## 代码结构

```
rebot_control/
├── config/
│   └── rebotarm_rs.yaml            # 配置文件：CAN / 电机 / 温度阈值 / 回零参数
├── rebot/                      # 核心包
│   ├── __init__.py             # 对外导出公共 API
│   ├── config.py               # 配置层：读取 YAML，dataclass 定义与校验
│   └── controller.py           # 接口层：ReBotRSMITController（纯控制逻辑）
└── examples/
    └── mit_position_control.py # 示例层：目标角度、速度等演示参数 + main()
```

分层说明：

- **配置文件**（`config/rebotarm_rs.yaml`）：所有可调参数的单一来源，改参数只需编辑 YAML，不碰代码。
- **配置层**（`rebot/config.py`）：`load_config()` 读取 YAML 并构造 `ControllerConfig` 等 dataclass，做合法性校验（阈值顺序、频率、未知键笔误等）。
- **接口层**（`rebot/controller.py`）：`ReBotRSMITController`，只依赖 `ControllerConfig`，不含任何硬编码数值和演示参数。
- **示例层**（`examples/mit_position_control.py`）：具体的目标角度、速度和运行流程，演示如何调用接口层。

## 功能特性

- MIT 模式持续发送位置指令（默认 200 Hz）
- 各关节独立的速度限制（默认统一 20°/s，控制循环内做速度平滑）
- 实时监控各电机 MOS 温度（默认 2 Hz）
- 三级温度保护：
  - ≥ 80°C：温度报警，继续运行
  - ≥ 100°C：停止运动，缓慢回零后失能
  - ≥ 140°C：立即紧急失能，不再回零
- 安全退出：按 Esc、Ctrl+C 或调用 `arm.stop()`，机械臂以 smoothstep 轨迹缓慢回零后失能
- 回零过程中再次按 Ctrl+C，立即中止回零并失能
- 启动时先读取实际位置作为初始指令，避免使能后突然跳动

## 依赖安装

```bash
pip install motorbridge pynput pyyaml
```

`pynput` 用于 Esc 键监听，未安装时仅 Esc 功能失效，Ctrl+C 和 `stop()` 仍然有效。

## CAN 接口准备

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
```

RobStride 电机默认波特率为 1 Mbps，请按实际配置调整。

## 使用方法

1. 编辑 `config/rebotarm_rs.yaml` 调整 CAN 接口、温度阈值、电机参数等。
2. 修改 `examples/mit_position_control.py` 顶部的 `TARGET_ANGLES` 和 `JOINT_SPEEDS_DEG_S`。
3. 运行：

```bash
python3 examples/mit_position_control.py
```

### 配置文件（`config/rebotarm_rs.yaml`）

| 配置 | 说明 | 默认值 |
|---|---|---|
| `can.channel` | CAN 接口名 | `can0` |
| `can.host_id` | 主机 ID | `0xFD` |
| `control.control_hz` | MIT 指令发送频率（非运动速度） | 200 Hz |
| `control.telemetry_hz` | 温度读取频率 | 2 Hz |
| `temperatures.alarm_c` | 温度报警阈值 | 80°C |
| `temperatures.return_zero_c` | 高温回零阈值 | 100°C |
| `temperatures.disconnect_c` | 紧急失能阈值 | 140°C |
| `return_zero.max_speed_deg_s` | 普通回零峰值速度 | 15°/s |
| `return_zero.thermal_max_speed_deg_s` | 高温触发的回零峰值速度 | 8°/s |
| `return_zero.min_time_s` | 最短回零时间 | 3.0 s |
| `motors` | 各电机 ID / 型号 / MIT `kp` / `kd` 列表 | 见 YAML 注释 |

YAML 中未填写的项自动使用代码内默认值；填了不存在的键会直接报错，防止笔误。

## 作为库调用

```python
from rebot import ReBotRSMITController, load_config

# 默认读取 config/rebotarm_rs.yaml，也可指定其他 YAML
arm = ReBotRSMITController(load_config("config/rebotarm_rs.yaml"))

arm.start(enable_esc=True)

arm.set_max_speeds([20.0] * 6)            # 度/秒
arm.set_joint_angles([50, 0, 0, 0, 0, 0]) # J1-J6 目标角度（度）
arm.set_joint_angle(1, 30.0)              # 只修改单个关节

print(arm.read_joint_angles())            # 读取实际机械位置（度）

arm.stop()  # 缓慢回零 -> 失能 -> 关闭 CAN
```

不传任何参数时 `ReBotRSMITController()` 会自动加载 `config/rebotarm_rs.yaml`。

## 单位约定

- 外部接口（目标角度、速度）：度、度/秒
- MotorBridge MIT 指令内部：弧度、弧度/秒

## 注意事项

- 运行前请确认机械臂周围无遮挡，首次使用建议先用小角度、低速度测试。
- 通信异常时程序不会尝试回零，而是立即紧急失能。
- `control_hz` 只是指令发送频率，机械臂实际运动速度由 `set_max_speeds` 决定。
