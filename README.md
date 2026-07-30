# rebot_control

**[中文文档](README_zh.md)**

MIT position control for the reBot B601-RS 6-axis robot arm (Python). It drives RobStride motors over a CAN bus via MotorBridge.

## Hardware

- Robot arm: reBot B601-RS (6 joints)
- Motors: J1-J3 are RS06, J4-J6 are RS00
- Communication: CAN bus (default channel `can0`, host ID `0xFD`)

## Project Layout

```
rebot_control/
├── config/
│   └── rebotarm_rs.yaml        # Config file: CAN / motors / temperature thresholds / return-to-zero
├── rebot/                      # Core package
│   ├── __init__.py             # Public API exports
│   ├── config.py               # Config layer: YAML loading, dataclasses and validation
│   └── controller.py           # Interface layer: ReBotRSMITController (pure control logic)
└── examples/
    └── mit_position_control.py # Example layer: demo target angles, speeds and main()
```

Layer overview:

- **Config file** (`config/rebotarm_rs.yaml`): single source of truth for all tunable parameters — edit the YAML, never the code.
- **Config layer** (`rebot/config.py`): `load_config()` reads the YAML and builds validated dataclasses such as `ControllerConfig` (threshold ordering, frequencies, typo'd keys, etc.).
- **Interface layer** (`rebot/controller.py`): `ReBotRSMITController`, depends only on `ControllerConfig`; no hard-coded values or demo parameters.
- **Example layer** (`examples/mit_position_control.py`): concrete target angles, speeds and the run loop, showing how to use the interface layer.

## Features

- Continuous MIT position commands (200 Hz by default)
- Per-joint speed limits (20°/s for all joints by default, smoothed inside the control loop)
- Real-time MOS temperature monitoring of every motor (2 Hz by default)
- Three-level temperature protection:
  - ≥ 80°C: warning, keeps running
  - ≥ 100°C: stops motion, slowly returns to zero, then disables
  - ≥ 140°C: immediate emergency disable, no return-to-zero
- Safe exit: pressing Esc, Ctrl+C, or calling `arm.stop()` returns the arm to zero along a smoothstep trajectory before disabling
- Pressing Ctrl+C again during return-to-zero aborts it and disables immediately
- Reads actual positions before enabling to avoid sudden jumps at startup

## Installation

```bash
pip install motorbridge pynput pyyaml
```

`pynput` is used for Esc-key listening. Without it only the Esc shortcut is disabled; Ctrl+C and `stop()` still work.

## CAN Interface Setup

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
```

RobStride motors use 1 Mbps by default — adjust to your actual configuration.

## Usage

1. Edit `config/rebotarm_rs.yaml` to adjust the CAN interface, temperature thresholds, motor parameters, etc.
2. Edit `TARGET_ANGLES` and `JOINT_SPEEDS_DEG_S` at the top of `examples/mit_position_control.py`.
3. Run:

```bash
python3 examples/mit_position_control.py
```

### Config File (`config/rebotarm_rs.yaml`)

| Key | Description | Default |
|---|---|---|
| `can.channel` | CAN interface name | `can0` |
| `can.host_id` | Host ID | `0xFD` |
| `control.control_hz` | MIT command rate (not the motion speed) | 200 Hz |
| `control.telemetry_hz` | Temperature polling rate | 2 Hz |
| `temperatures.alarm_c` | Temperature warning threshold | 80°C |
| `temperatures.return_zero_c` | High-temperature return-to-zero threshold | 100°C |
| `temperatures.disconnect_c` | Emergency disable threshold | 140°C |
| `return_zero.max_speed_deg_s` | Peak speed of a normal return-to-zero | 15°/s |
| `return_zero.thermal_max_speed_deg_s` | Peak speed of a thermal return-to-zero | 8°/s |
| `return_zero.min_time_s` | Minimum return-to-zero duration | 3.0 s |
| `motors` | Per-motor ID / model / MIT `kp` / `kd` list | See YAML comments |

Keys omitted from the YAML fall back to the in-code defaults; unknown keys raise an error to catch typos.

## Using as a Library

```python
from rebot import ReBotRSMITController, load_config

# Reads config/rebotarm_rs.yaml by default; any other YAML can be given
arm = ReBotRSMITController(load_config("config/rebotarm_rs.yaml"))

arm.start(enable_esc=True)

arm.set_max_speeds([20.0] * 6)            # deg/s
arm.set_joint_angles([50, 0, 0, 0, 0, 0]) # J1-J6 target angles (deg)
arm.set_joint_angle(1, 30.0)              # change a single joint

print(arm.read_joint_angles())            # read actual mechanical positions (deg)

arm.stop()  # slow return-to-zero -> disable -> close CAN
```

Calling `ReBotRSMITController()` without arguments automatically loads `config/rebotarm_rs.yaml`.

## Units

- External interfaces (target angles, speeds): degrees, deg/s
- MotorBridge MIT commands internally: radians, rad/s

## Notes

- Make sure the area around the arm is clear before running. For the first run, use small angles and low speeds.
- On a communication error the program does not attempt to return to zero — it disables immediately.
- `control_hz` is only the command rate; the actual motion speed is set by `set_max_speeds`.
