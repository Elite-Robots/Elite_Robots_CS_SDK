# EliteDriverConfig

## 定义

``` cpp
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

## 描述

这个类是用于输入给`EliteDriver`类构造时的配置，用于告知`EliteDriver`类机器人的IP地址、控制脚本路径等参数。

## 参数

- `robot_ip`
    - 类型：`std::string`
    - 描述：机器人IP。

- `script_file_path`
    - 类型：`std::string`
    - 描述：机器人控制脚本的模板，依据此模板会生成一个可以被运行的实际脚本。

- `local_ip`
    - 类型：`std::string`
    - 描述：本地IP，当其为默认值“空字符串”时，`EliteDriver`会尝试自动获取本机IP。
    - 注意：如果机器人无法通过脚本指令`socket_open()`连接到你本地的TCP服务器，那么就需要手动设置这个参数，或者检查网络。

- `headless_mode`：
    - 类型：`bool`
    - 描述：是否以无界面模式运行，使用此模式后，无需使用`External Control`插件。如果此参数为true，那么在构造函数中，将会向机器人的 primary 端口发送一次控制脚本。

- `script_sender_port`
    - 类型：`int`
    - 描述：用于发送控制脚本的端口。如果无法连接此端口，`External Control`插件将会停止运行。

- reverse_port
    - 类型：`int`
    - 描述：反向通信端口。此端口主要发送控制模式以及部分控制数据。

- trajectory_port
    - 类型：`int`
    - 描述：发送轨迹点的端口。

- script_command_port
    - 类型：`int`
    - 描述：发送脚本命令的端口。

- servoj_time
    - 类型：`float`
    - 描述：`servoj()`指令的时间间隔。


- servoj_lookahead_time
    - 类型：`float`
    - 描述：`servoj()`指令瞻时间，范围 [0.03, 0.2] 秒。

- servoj_gain
    - 类型：`int`
    - 描述：伺服增益。

- stopj_acc
    - 类型：`float`
    - 描述：停止运动的加速度 (rad/s²)。

- servoj_extrapolate_max_time
    - 类型：`float`
    - 描述：当没有及时接收到新的 `servoj()` 点时，保持恒速外推的最大时长 [S]。

- servoj_decelerate_time
    - 类型：`float`
    - 描述：将外推速度线性收敛到 0 的减速时间 [S]。

- servoj_hold_velocity_threshold
    - 类型：`float`
    - 描述：用于判定关节是否足够稳定以锁定保持点的关节速度阈值 [rad/s]。

- servoj_hold_stable_time
    - 类型：`float`
    - 描述：外推速度收敛到 0 后，锁定保持点所需的稳定持续时间 [S]。

- `user_frames`
    - 类型：`std::vector<UserFrame>`
    - 描述：SDK 启动时管理的初始用户坐标系列表。每个 `UserFrame` 包含 `id`、`name`、`pose` 和 `valid` 字段。
    - `id`：用户坐标系编号，范围为 `[0, max_user_frame_count)`。
    - `name`：用户坐标系名称，由 SDK 保存，当前不参与机器人端的坐标变换。
    - `pose`：用户坐标系相对于基座坐标系的位姿 `[x,y,z,rx,ry,rz]`，位置单位为 m，姿态单位为 rad。
    - `valid`：坐标系是否有效。无效坐标系不会被用于运动和坐标转换。

- `max_user_frame_count`
    - 类型：`int32_t`
    - 描述：机器人控制脚本生成的用户坐标系槽位数量，取值范围为 `1~16`，默认值为 `16`。用户坐标系编号必须小于该值。
    - 注意：`-1` 保留表示基座坐标系，不属于 `user_frames` 的编号范围。

## 调参档位（网络抖动）

说明：以下档位是基于网络质量的调参建议，不是强制默认值。单位中，时间参数为秒，速度阈值为 rad/s。

| 档位 | 适用场景 | `servoj_extrapolate_max_time` | `servoj_decelerate_time` | `servoj_hold_velocity_threshold` | `servoj_hold_stable_time` |
| --- | --- | --- | --- | --- | --- |
| 保守 | 抖动明显、偶发丢包，优先防飞车 | 0.04 | 0.03 | 0.03 | 0.05 |
| 平衡 | 一般工业网络，兼顾跟随与稳定 | 0.08 | 0.02 | 0.05 | 0.04 |
| 激进 | 网络稳定，优先跟随手感 | 0.12 | 0.01 | 0.08 | 0.03 |

示例：

```cpp
EliteDriverConfig config;

// 平衡档
config.servoj_extrapolate_max_time = 0.08;
config.servoj_decelerate_time = 0.02;
config.servoj_hold_velocity_threshold = 0.05;
config.servoj_hold_stable_time = 0.04;
```

选档建议：

- 升到更保守档位：出现明显超调、停不稳、长时间无新点后仍有持续运动趋势。
- 升到更激进档位：网络稳定且轨迹跟随有明显滞后，希望提升跟随手感。
- 当前档位可保留：无飞车风险、无明显抖动，且轨迹跟随误差可接受。

快速排障映射：

| 现象 | 优先调整参数 | 调整方向 |
| --- | --- | --- |
| 长时间无新点后仍有“冲一下”的感觉 | `servoj_decelerate_time` | 增大（更平缓收敛） |
| 很快停住但感觉“刹车太硬” | `servoj_decelerate_time` | 适度增大 |
| 明显滞后、跟随发粘 | `servoj_extrapolate_max_time` | 增大（延长恒速外推窗口） |
| 停住前抖动，迟迟不锁定保持点 | `servoj_hold_velocity_threshold` | 适度增大 |
| 过早锁定保持点，细小跟随被抑制 | `servoj_hold_stable_time` | 增大 |
| 长时间不锁定，保持点漂移感明显 | `servoj_hold_stable_time` | 减小 |

安全边界建议值：

| 参数 | 建议范围 | 说明 |
| --- | --- | --- |
| `servoj_extrapolate_max_time` | 0.02 ~ 0.20 | 过小会导致跟随发粘，过大可能放大网络中断期间的位移外推。 |
| `servoj_decelerate_time` | 0.005 ~ 0.08 | 过小制动偏硬，过大则停稳变慢。 |
| `servoj_hold_velocity_threshold` | 0.02 ~ 0.12 | 过小难以进入锁定，过大可能过早锁定。 |
| `servoj_hold_stable_time` | 0.01 ~ 0.10 | 过小会误判稳定，过大会延迟锁定保持点。 |

注：以上为推荐工程范围，不是硬限制。建议单次只调整一个参数，并以 10%~20% 的步进逐步验证。

