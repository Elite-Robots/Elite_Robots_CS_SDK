# Message Format

This document describes the message format, and the length of each message is fixed.

## 1. ReverseInterface
The size of each piece of data is 4 bytes.

### 1.1 Messages Sent to the Control Script
Message length: 8 int data.

| Name | Description |
| ---- | ---- |
| timeout | Timeout (in milliseconds) |
| Data 1 | Data such as joint angles and velocities<sup>*1</sup> (transmitted after being amplified by 1,000,000 times) |
| Data 2 | Data such as joint angles and velocities (transmitted after being amplified by 1,000,000 times) |
| Data 3 | Data such as joint angles and velocities (transmitted after being amplified by 1,000,000 times) |
| Data 4 | Data such as joint angles and velocities (transmitted after being amplified by 1,000,000 times) |
| Data 5 | Data such as joint angles and velocities (transmitted after being amplified by 1,000,000 times) |
| Data 6 | Data such as joint angles and velocities (transmitted after being amplified by 1,000,000 times) |
| Mode | <sup>*2</sup>Control mode (determines the motion method such as movej or servoj) |

> *1: Since the script reads integers, while data like joint angles are floating-point numbers, they need to be amplified by 1,000,000 times for transmission. When the script reads these data, it will divide them by 1,000,000.
> *2: Corresponds to the `ControlMode` enumeration in the code `ControlMode.hpp`.

For the `writeTrajectoryControlAction()` interface, the message format is as follows:

| Name | Description |
| ---- | ---- |
| timeout | Timeout (in milliseconds) |
| Action | Start, cancel or noop |
| Number of Points | The number of points |
| Reserved | Reserved |
| Reserved | Reserved |
| Reserved | Reserved |
| Reserved | Reserved |
| Mode | The value is `ControlMode::MODE_TRAJECTORY` |



## 2. ScriptCommandInterface

### 2.1 Messages Sent to the Control Script
Message length: 26 int data.

| Name | Description |
| ---- | ---- |
| command | <sup>*1</sup>Instruction |
| Instruction Data 1 - 25 | <sup>*2</sup>Data required by the instruction |

> *1: Corresponds to the `ScriptCommandInterface::Cmd` enumeration in the code `ScriptCommandInterface.hpp`.
> *2: Depending on the instruction, the number of useful data varies. Unused data can be set to 0.

Common command payloads:

| Command | Value | Data |
| ---- | ---- | ---- |
| `ZERO_FTSENSOR` | 0 | No payload |
| `SET_PAYLOAD` | 1 | Data 1: mass, Data 2-4: center of gravity, amplified by 1,000,000 |
| `SET_TOOL_VOLTAGE` | 2 | Data 1: tool voltage, amplified by 1,000,000 |
| `START_FORCE_MODE` | 3 | Data 1-6: task frame, Data 7-12: selection vector, Data 13-18: wrench, Data 19: force mode, Data 20-25: limits |
| `END_FORCE_MODE` | 4 | No payload |
| `SET_COLLISION_DETECT_ENABLED` | 9 | Data 1: 1 means enabled, 0 means disabled |
| `SET_COLLISION_SENSITIVITY` | 10 | Data 1: sensitivity ratio in percent, range [10, 100] |
| `SET_MOUNTING_PLANE` | 11 | Data 1: Z-axis rotation angle in radians, Data 2: mounting plane tilt angle in radians, amplified by 1,000,000 |

## 3. TrajectoryInterface

### 3.1 Messages Sent to the Control Script
Message length: 21 int data.

| Name | Description |
| ---- | ---- |
| x or Joint 1 | The x coordinate value of the target point, or the angle value of joint 1<sup>*1</sup> (transmitted after being amplified by 1,000,000 times) |
| y or Joint 2 | The y coordinate value of the target point, or the angle value of joint 2 (transmitted after being amplified by 1,000,000 times) |
| z or Joint 3 | The z coordinate value of the target point, or the angle value of joint 3 (transmitted after being amplified by 1,000,000 times) |
| rx or Joint 4 | The rx coordinate value of the target point, or the angle value of joint 4 (transmitted after being amplified by 1,000,000 times) |
| ry or Joint 5 | The ry coordinate value of the target point, or the angle value of joint 5 (transmitted after being amplified by 1,000,000 times) |
| rz or Joint 6 | The rz coordinate value of the target point, or the angle value of joint 6 (transmitted after being amplified by 1,000,000 times) |
| 7 - 18 | Reserved |
| Time | The time to reach the target point |
| Blend Radius | (transmitted after being amplified by 1,000,000 times) |
| Trajectory Type | <sup>*1</sup>Trajectory type |

> *1: Since the script reads integers, while data like joint angles are floating-point numbers, they need to be amplified by 1,000,000 times for transmission. When the script reads these data, it will divide them by 1,000,000.
> *2: Corresponds to the `TrajectoryMotionType` enumeration in the code `TrajectoryInterface.hpp`.

### 3.2 Messages Returned by the Control Script
Message length: 1 int data.

| Name | Description |
| ---- | ---- |
| Result | <sup>*1</sup>Operation result |
> *2: Corresponds to the `TrajectoryMotionResult` enumeration in the code `DataType.hpp`. 
