# EliteDriverConfig

## Definition

```cpp
class EliteDriverConfig {
   public:
    // IP-address under which the robot is reachable.
    std::string robot_ip;

    // EliRobot template script file that should be used to generate scripts that can be run.
    std::string script_file_path;

    // Local IP-address that the reverse_port and trajectory_port will bound.
    std::string local_ip = "";

    // If the driver should be started in headless mode.
    bool headless_mode = false;

    // The driver will offer an interface to receive the program's script on this port.
    // If the robot cannot connect to this port, `External Control` will stop immediately.
    int script_sender_port = 50002;

    // Port that will be opened by the driver to allow direct communication between the driver and the robot controller.
    int reverse_port = 50001;

    // Port used for sending trajectory points to the robot in case of trajectory forwarding.
    int trajectory_port = 50003;

    // Port used for forwarding script commands to the robot. The script commands will be executed locally on the robot.
    int script_command_port = 50004;

    // The duration of servoj motion.
    float servoj_time = 0.008;

    // Time [S], range [0.03,0.2] smoothens the trajectory with this lookahead time
    float servoj_lookahead_time = 0.1;

    // Servo gain.
    int servoj_gain = 300;

    // Acceleration [rad/s^2]. The acceleration of stopj motion.
    float stopj_acc = 8;

    // Maximum duration [S] for constant-velocity extrapolation when no new servoj point arrives.
    float servoj_extrapolate_max_time = 0.08;

    // Deceleration duration [S] used to linearly ramp extrapolation speed down to zero.
    float servoj_decelerate_time = 0.01;

    // Joint velocity threshold [rad/s] used to decide whether joints are stable enough to lock hold position.
    float servoj_hold_velocity_threshold = 0.05;

    // Stable duration [S] required before locking hold position after extrapolation speed reaches zero.
    float servoj_hold_stable_time = 0.04;

    // SDK-managed user frames. The pose is expressed in the base frame.
    std::vector<UserFrame> user_frames;

    // Number of user frame slots generated in the robot script.
    int32_t max_user_frame_count = MAX_USER_FRAME_COUNT;

    EliteDriverConfig() = default;
    ~EliteDriverConfig() = default;
};
```

## Description

This class serves as the configuration input when constructing the `EliteDriver` class, providing parameters such as the robot's IP address and control script path.

## Parameters

- `robot_ip`
    - Type: `std::string`
    - Description: Robot IP address.

- `script_file_path`
    - Type: `std::string`
    - Description: Template for the robot control script. An executable script will be generated based on this template.

- `local_ip`
    - Type: `std::string`
    - Description: Local IP address. When left as default (empty string), `EliteDriver` will attempt to automatically detect the local IP.
    - Note: If the robot cannot connect to your local TCP server via the `socket_open()` script command, you need to manually set this parameter or check your network.

- `headless_mode`:
    - Type: `bool`
    - Description: Whether to run in headless mode。 After using this mode, there is no need to use the 'External Control' plugin. If true, the constructor will send a control script to the robot's primary port once.

- `script_sender_port`
    - Type: `int`
    - Description: Port used for sending control scripts. If this port cannot be connected, the `External Control` plugin will stop immediately.

- `reverse_port`
    - Type: `int`
    - Description: Reverse communication port. This port primarily sends control modes and partial control data.

- `trajectory_port`
    - Type: `int`
    - Description: Port for sending trajectory points.

- `script_command_port`
    - Type: `int`
    - Description: Port for sending script commands.

- `servoj_time`
    - Type: `float`
    - Description: Time interval for `servoj()` command.

- `servoj_lookahead_time`
    - Type: `float`
    - Description: Lookahead time for `servoj()` command, range [0.03, 0.2] seconds.

- `servoj_gain`
    - Type: `int`
    - Description: Servo gain.

- `stopj_acc`
    - Type: `float`
    - Description: Acceleration for stopping motion (rad/s²).

- `servoj_extrapolate_max_time`
    - Type: `float`
    - Description: Maximum duration [S] for constant-velocity extrapolation when no new `servoj()` point arrives.

- `servoj_decelerate_time`
    - Type: `float`
    - Description: Deceleration duration [S] used to linearly ramp extrapolation speed down to zero.

- `servoj_hold_velocity_threshold`
    - Type: `float`
    - Description: Joint velocity threshold [rad/s] used to determine when joints are stable enough to lock a hold position.

- `servoj_hold_stable_time`
    - Type: `float`
    - Description: Stable duration [S] required before locking the hold position after extrapolation speed has converged to zero.

- `user_frames`
    - Type: `std::vector<UserFrame>`
    - Description: Initial list of user frames managed by the SDK at startup. Each `UserFrame` contains `id`, `name`, `pose`, and `valid` fields.
    - `id`: User frame id in the range `[0, max_user_frame_count)`.
    - `name`: User frame name. It is stored by the SDK and is not currently used for robot-side coordinate conversion.
    - `pose`: User frame pose relative to the base frame, formatted as `[x, y, z, rx, ry, rz]`; position is in meters and orientation is in radians.
    - `valid`: Whether the frame is valid. Invalid frames are not used for motion or pose conversion.

- `max_user_frame_count`
    - Type: `int32_t`
    - Description: Number of user-frame slots generated in the robot control script. The valid range is `1~16`, and the default is `16`. User frame ids must be less than this value.
    - Note: `-1` is reserved for the base frame and is not an index in `user_frames`.

## Tuning Profiles (Network Jitter)

Note: These profiles are tuning guidance based on network quality, not mandatory defaults. Time parameters are in seconds, and velocity threshold is in rad/s.

| Profile | Typical scenario | `servoj_extrapolate_max_time` | `servoj_decelerate_time` | `servoj_hold_velocity_threshold` | `servoj_hold_stable_time` |
| --- | --- | --- | --- | --- | --- |
| Conservative | Noticeable jitter or occasional packet loss, prioritize anti-runaway behavior | 0.04 | 0.03 | 0.03 | 0.05 |
| Balanced | Typical industrial network, balance tracking and stability | 0.08 | 0.02 | 0.05 | 0.04 |
| Aggressive | Stable network, prioritize tracking feel | 0.12 | 0.01 | 0.08 | 0.03 |

Example:

```cpp
EliteDriverConfig config;

// Balanced profile
config.servoj_extrapolate_max_time = 0.08;
config.servoj_decelerate_time = 0.02;
config.servoj_hold_velocity_threshold = 0.05;
config.servoj_hold_stable_time = 0.04;
```

How to choose a profile:

- Move to a more conservative profile: obvious overshoot, unstable settling, or residual motion after prolonged missing points.
- Move to a more aggressive profile: network is stable and tracking lag is noticeable; better responsiveness is preferred.
- Keep current profile: no runaway risk, no obvious jitter, and tracking error is acceptable.

Quick troubleshooting map:

| Symptom | Tune first | Suggested direction |
| --- | --- | --- |
| A small "surge" after prolonged missing points | `servoj_decelerate_time` | Increase (smoother convergence) |
| Stops quickly but braking feels too hard | `servoj_decelerate_time` | Increase moderately |
| Noticeable lag / sticky tracking | `servoj_extrapolate_max_time` | Increase (longer constant-velocity window) |
| Jitter before settling; hold lock comes too late | `servoj_hold_velocity_threshold` | Increase moderately |
| Hold locks too early and suppresses fine tracking | `servoj_hold_stable_time` | Increase |
| Hold lock takes too long; drifting feeling persists | `servoj_hold_stable_time` | Decrease |

Suggested safety bounds:

| Parameter | Recommended range | Notes |
| --- | --- | --- |
| `servoj_extrapolate_max_time` | 0.02 ~ 0.20 | Too small can make tracking feel sticky; too large may amplify extrapolated drift during communication gaps. |
| `servoj_decelerate_time` | 0.005 ~ 0.08 | Too small can feel harsh; too large can delay settling. |
| `servoj_hold_velocity_threshold` | 0.02 ~ 0.12 | Too small may prevent hold lock; too large may lock too early. |
| `servoj_hold_stable_time` | 0.01 ~ 0.10 | Too small can mis-detect stability; too large delays hold lock. |

Note: These are recommended engineering bounds, not hard limits. Adjust one parameter at a time, with 10% to 20% step changes.
