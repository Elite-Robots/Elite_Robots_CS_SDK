# EliteDriver 类

## 简介

EliteDriver 是用于与机器人进行数据交互的主要类。它负责建立所有必要的套接字连接，并处理与机器人的数据交换。EliteDriver 会向机器人发送控制脚本，机器人在运行控制脚本后，会和 EliteDriver 建立通讯，接收运动数据，并且必要时会发送运动结果。

## 头文件
```cpp
#include <Elite/EliteDriver.hpp>
```

## 构造与析构函数

### ***构造函数***
```cpp
EliteDriver(const EliteDriverConfig& config)
```
- ***功能***

    创建 EliteDriver 对象，并初始化与机器人通信的必要连接。  
    以下情况此函数会抛出异常：  
    1. TCP server 创建失败，通常是因为端口被占用导致的。
    2. 连接机器人的primary port失败。

- ***参数***
    - config：配置，参考[配置](./EliteDriverConfig.cn.md)

### ***构造函数***(此函数已废弃)
```cpp
EliteDriver::EliteDriver(
    const std::string& robot_ip, 
    const std::string& local_ip, 
    const std::string& script_file,
    bool headless_mode = false, 
    int script_sender_port = 50002, 
    int reverse_port = 50001,
    int trajectory_port = 50003, 
    int script_command_port = 50004, 
    float servoj_time = 0.008,
    float servoj_lookhead_time = 0.1, 
    int servoj_gain = 300, 
    float stopj_acc = 8.0);
```
- ***功能***

    创建 EliteDriver 对象，并初始化与机器人通信的必要连接。

- ***参数***
    - robot_ip：机器人 IP 地址。
    - local_ip：本机 IP 地址。
    - script_file：控制脚本模板文件。
    - headless_mode：是否以无界面模式运行，使用此模式后，无需使用`External Control`插件。如果此参数为true，那么在构造函数中，将会向机器人的 primary 端口发送一次控制脚本。
    - script_sender_port：用于发送控制脚本的端口。如果无法连接此端口，`External Control`插件将会停止运行。
    - reverse_port：反向通信端口。
    - trajectory_port：发送轨迹点的端口。
    - script_command_port：发送脚本命令的端口。
    - servoj_time：伺服运动的时间参数。
    - servoj_lookhead_time：伺服运动前瞻时间，范围 [0.03, 0.2] 秒。
    - servoj_gain：伺服增益。
    - stopj_acc：停止运动的加速度 (rad/s²)。

---

### ***析构函数***
```cpp
~EliteDriver::EliteDriver()
```
- ***功能***

    释放资源，析构时会关闭socket。

---

## 运动控制

### ***控制关节位置***
```cpp
bool writeServoj(const vector6d_t& pos, int timeout_ms, bool cartesian = false)
```
- ***功能***
    向机器人发送伺服运动的指令。

- ***参数***
    - pos：目标点位

    - timeout_ms：设置机器人读取下一条指令的超时时间，小于等于0时会无限等待。

    - cartesian：如果发送的点是笛卡尔的，则为true，如果是基于关节的，则为false。

- ***返回值***：指令发送成功返回 true，失败返回 false。

    当 `cartesian` 为 `true` 且未使用带 `user_frame_id` 的重载时，目标位姿使用当前活动坐标系；默认活动坐标系为基座坐标系。

---

### ***控制末端速度***
```cpp
bool writeSpeedl(const vector6d_t& vel, int timeout_ms)
```
- ***功能***
    向机器人发送线速度控制指令。

- ***参数***
    - vel：线速度 [x, y, z, rx, ry, rz]。

    - timeout_ms：设置机器人读取下一条指令的超时时间，小于等于0时会无限等待。

- ***返回值***：指令发送成功返回 true，失败返回 false。

    该重载使用当前活动坐标系；默认活动坐标系为基座坐标系。

### ***在指定用户坐标系下控制伺服位姿或关节位置***
```cpp
bool writeServoj(const vector6d_t& pos, int timeout_ms, bool cartesian, int32_t user_frame_id)
```
- ***功能***

    向机器人发送伺服运动指令。当 `cartesian` 为 `true` 时，`pos` 按指定用户坐标系解释，机器人端会将目标位姿转换到基座坐标系后执行；当 `cartesian` 为 `false` 时，`pos` 为关节角，用户坐标系参数不参与运动计算。

- ***参数***
    - pos：目标位姿，格式为 `[x,y,z,rx,ry,rz]`。位置单位为 m，姿态单位为 rad。
    - timeout_ms：设置机器人读取下一条指令的超时时间，小于等于 0 时会无限等待。
    - cartesian：必须为 `true` 才使用用户坐标系；为 `false` 时 `pos` 是关节角，`user_frame_id` 被忽略。
    - user_frame_id：用户坐标系编号。`-1` 表示基座坐标系，非负值表示 SDK 管理的用户坐标系。

- ***返回值***：指令发送成功返回 true，失败返回 false。

### ***在指定用户坐标系下控制末端速度***
```cpp
bool writeSpeedl(const vector6d_t& vel, int timeout_ms, int32_t user_frame_id)
```

- ***功能***

    按指定用户坐标系解释 TCP 线速度和角速度。SDK 会根据用户坐标系相对于基座的旋转，将速度投影到基座坐标系后发送给机器人。

- ***参数***
    - vel：速度 `[vx,vy,vz,wx,wy,wz]`。线速度单位为 m/s，角速度单位为 rad/s。
    - timeout_ms：设置机器人读取下一条指令的超时时间。小于等于 0 时会无限等待。
    - user_frame_id：`-1` 表示基座坐标系，非负值表示 SDK 管理的用户坐标系。

- ***返回值***：指令发送成功返回 true，失败返回 false。

---

### ***使机器人空闲***
```cpp
bool writeIdle(int timeout_ms)
```
- ***功能***

    发送空闲指令，如果机器人正在运动会使机器人停止运动。

- ***参数***
    - timeout_ms：设置机器人读取下一条指令的超时时间，小于等于0时会无限等待。

- ***返回值***：指令发送成功返回 true，失败返回 false。

---

### ***Freedrive***
```cpp
bool writeFreedrive(FreedriveAction action, int timeout_ms)
```
- ***功能***

    发送Freedrive模式的指令，如：开启Freedrive，停止Freedrive。

- ***参数***
    - action：Freedrive动作，有：开启（START）、停止(END)、空操作(NOOP)
    - timeout_ms：设置机器人读取下一条指令的超时时间，小于等于0时会无限等待。

- ***注意***：写入`START`动作之后，需要在超时时间内写入下一条指令，可以写入`NOOP`。

---

## 用户坐标系变换

### ***设置用户坐标系***
```cpp
bool setUserFrame(int32_t frame_id, const vector6d_t& pose)
```
- ***功能***

    新增或更新一个 SDK 管理的用户坐标系。用户坐标系位姿保存到 SDK，并通过 `script_command_socket` 同步到 External Control 脚本。

- ***参数***
    - frame_id：用户坐标系编号，范围为 `[0, max_user_frame_count)`。
    - pose：用户坐标系相对于基座坐标系的位姿 `[x,y,z,rx,ry,rz]`，位置单位为 m，姿态单位为 rad。

- ***返回值***：SDK 成功写入同步 socket 返回 true，否则返回 false。

### ***设置用户坐标系（对象重载）***
```cpp
bool setUserFrame(const UserFrame& frame)
```
- ***功能***

    使用 `UserFrame` 对象新增或更新一个 SDK 管理的用户坐标系。当前同步使用对象中的 `id` 和 `pose` 字段。

- ***参数***
    - frame：用户坐标系对象。`id` 为坐标系编号，`pose` 为相对于基座坐标系的位姿；此重载实际使用 `id` 和 `pose`，`name`、`valid` 不会通过该调用更新。

- ***返回值***：SDK 成功写入同步 socket 返回 true，否则返回 false。

### ***获取指定用户坐标系***
```cpp
bool getUserFrame(int32_t frame_id, UserFrame& frame) const
```
- ***功能***

    查询 SDK 当前保存的指定用户坐标系。该接口读取的是 SDK 缓存，不是示教器中的用户坐标系表。

- ***参数***
    - frame_id：要查询的用户坐标系编号。
    - frame：输出用户坐标系对象。

- ***返回值***：找到有效坐标系返回 true，否则返回 false。

### ***获取全部用户坐标系***
```cpp
std::vector<UserFrame> getUserFrames() const
```
- ***功能***

    返回 SDK 当前保存的全部用户坐标系列表。返回值是 SDK 内部缓存的副本，不是示教器中的用户坐标系表。

- ***返回值***：SDK 当前保存的用户坐标系列表。未配置用户坐标系时返回空列表。

### ***设置当前活动用户坐标系***
```cpp
bool setActiveUserFrame(int32_t user_frame_id)
```
- ***功能***

    设置 SDK 当前活动的默认坐标系。默认值为 `-1`，表示基座坐标系。

- ***参数***
    - user_frame_id：`-1` 表示基座坐标系；非负值表示用户坐标系编号，且该坐标系必须已配置并有效。

- ***返回值***：设置成功返回 true；编号超出配置范围或坐标系不存在、无效时返回 false。

- ***说明***

    以下接口即使不带 `user_frame_id` 的笛卡尔接口会默认使用当前活动坐标系：

    - `writeServoj(const vector6d_t&, int, bool)`
    - `writeSpeedl(const vector6d_t&, int)`
    - `writeTrajectoryPoint(const vector6d_t&, float, float, bool)`
    - `writeTrajectoryPoint(const vector6d_t&, float, bool, float, float)`

    对于 `writeTrajectoryPoint()`，只有 `cartesian == true` 时才使用当前活动坐标系；当 `cartesian == false` 时，`positions` 表示关节角，不受活动坐标系影响。`writeSpeedj()` 也不受该设置影响。

### ***获取当前活动用户坐标系***
```cpp
int32_t getActiveUserFrame() const
```
- ***功能***

    获取 SDK 当前活动的默认坐标系编号。

- ***返回值***：返回当前活动坐标系编号；`-1` 表示基座坐标系。

---

## 轨迹运动

### ***设置轨迹运动结果回调***
```cpp
void setTrajectoryResultCallback(std::function<void(TrajectoryMotionResult)> cb)
```
- ***功能***

    注册轨迹完成时的回调函数。
    控制机器人的一种方式是将路点一次性发给机器人，当执行完成时，这里注册的回调函数将被触发。

- ***参数***
    - cb：执行完成时的回调函数

---

### ***按时间写入轨迹路点***
```cpp
bool writeTrajectoryPoint(const vector6d_t& positions, float time, float blend_radius, bool cartesian)
```
- ***功能***

    向轨迹 socket 写入一个轨迹路点，并按 `time` 规划运动。

- ***参数***
    - positions：关节或笛卡尔路点。笛卡尔位姿格式为 `[x,y,z,rx,ry,rz]`，位置单位为 m，姿态单位为 rad。
    - time：到达该路点的时间。
    - blend_radius：两个路点之间的转接半径。
    - cartesian：笛卡尔路点为 `true`，关节路点为 `false`。

- ***说明***

    - 当 `time == 0` 时，机器人使用控制器默认的 `movej` / `movel` 参数。
    - 当 `cartesian == true` 时，路点使用当前活动用户坐标系解释；当前活动坐标系为基座坐标系时，等同于基座坐标系。
    - 当 `cartesian == false` 时，`positions` 表示关节角，不受当前活动用户坐标系影响。

- ***返回值***：路点发送成功返回 true，失败返回 false。

### ***按时间写入指定用户坐标系下的轨迹路点***
```cpp
bool writeTrajectoryPoint(const vector6d_t& positions, float time, float blend_radius, bool cartesin, int32_t user_frame_id)

- ***功能***

    向轨迹 socket 写入一个轨迹路点，并按 `time` 规划运动。笛卡尔路点使用指定用户坐标系解释，机器人执行前会将目标位姿转换到基座坐标系。

- ***参数***
    - positions：关节或笛卡尔路点。笛卡尔位姿格式为 `[x,y,z,rx,ry,rz]`，位置单位为 m，姿态单位为 rad。
    - time：到达该路点的时间。
    - blend_radius：两个路点之间的转接半径。
    - cartesian：笛卡尔路点为 `true`，关节路点为 `false`。
    - user_frame_id：`-1` 表示基座坐标系，非负值表示 SDK 管理的用户坐标系。

- ***说明***

    - 当 `time == 0` 时，机器人使用控制器默认的 `movej` / `movel` 参数。
    - 仅当 `cartesian == true` 时使用 `user_frame_id`；当 `cartesian == false` 时，`positions` 表示关节角，`user_frame_id` 不生效。

- ***返回值***：路点发送成功返回 true，失败返回 false。

### ***按速度和加速度写入轨迹路点***
```cpp
bool writeTrajectoryPoint(const vector6d_t& positions, float blend_radius, bool cartesian, float speed, float acceleration)
```
- ***功能***

    向轨迹 socket 写入一个轨迹路点，内部将 `time` 固定为 0，并按给定速度和加速度规划运动。

- ***参数***
    - positions：关节或笛卡尔路点。笛卡尔位姿格式为 `[x,y,z,rx,ry,rz]`，位置单位为 m，姿态单位为 rad。
    - blend_radius：两个路点之间的转接半径。
    - cartesian：笛卡尔路点为 `true`，关节路点为 `false`。
    - speed：关节轨迹时为 `movej` 的关节速度，笛卡尔轨迹时为 `movel` 的工具速度。
    - acceleration：关节轨迹时为 `movej` 的关节加速度，笛卡尔轨迹时为 `movel` 的工具加速度。

- ***说明***

    - 当 `cartesian == true` 时，路点使用当前活动用户坐标系解释；当前活动坐标系为基座坐标系时，等同于基座坐标系。
    - 当 `cartesian == false` 时，`positions` 表示关节角，不受当前活动用户坐标系影响。

- ***返回值***：路点发送成功返回 true，失败返回 false。

### ***按速度和加速度写入指定用户坐标系下的轨迹路点***
```cpp
bool writeTrajectoryPoint(const vector6d_t& positions, float blend_radius, bool cartesian,  float speed, float acceleration, int32_t user_frame_id)
```
- ***功能***

    向轨迹 socket 写入一个轨迹路点，内部将 `time` 固定为 0，并按给定速度和加速度规划运动。笛卡尔路点使用指定用户坐标系解释，机器人执行前会将目标位姿转换到基座坐标系。

- ***参数***
    - positions：关节或笛卡尔路点。笛卡尔位姿格式为 `[x,y,z,rx,ry,rz]`，位置单位为 m，姿态单位为 rad。
    - blend_radius：两个路点之间的转接半径。
    - cartesian：笛卡尔路点为 `true`，关节路点为 `false`。
    - speed：关节轨迹时为 `movej` 的关节速度，笛卡尔轨迹时为 `movel` 的工具速度。
    - acceleration：关节轨迹时为 `movej` 的关节加速度，笛卡尔轨迹时为 `movel` 的工具加速度。
    - user_frame_id：`-1` 表示基座坐标系，非负值表示 SDK 管理的用户坐标系。

- ***说明***

    - 仅当 `cartesian == true` 时使用 `user_frame_id`；当 `cartesian == false` 时，`positions` 表示关节角，`user_frame_id` 不生效。

- ***返回值***：路点发送成功返回 true，失败返回 false。

### ***轨迹控制动作***
```cpp
bool writeTrajectoryControlAction(TrajectoryControlAction action, const int point_number, int timeout_ms)
```
- ***功能***

    发送轨迹控制指令。

- ***参数***
    - action：轨迹控制的动作。

    - point_number：路点的数量。

    - timeout_ms：设置机器人读取下一条指令的超时时间，小于等于0时会无限等待。

- ***返回值***：指令发送成功返回 true，失败返回 false。

- ***注意***：写入`START`动作之后，需要在超时时间内写入下一条指令，可以写入`NOOP`。

---

## 机器人配置
### ***力传感器去皮***
```cpp
bool zeroFTSensor()
```
- ***功能***

    将力/力矩传感器测量的施加在工具 TCP 上的力/力矩值清零（去皮），所述力/力矩值为 get_tcp_force(True) 脚本指令获取的施加在工具 TCP 上的力/力矩矢量，该矢量已进行负载补偿等处理。
    该指令执行后，当前的力/力矩测量值会被作为力/力矩参考值保存，后续所有的力/力矩测量值都会减去该力/力矩参考值（去皮）。
    请注意，上述力/力矩参考值会在该指令执行时更新，在控制器重启后将重置为 0。

- ***返回值***：指令发送成功返回 true，失败返回 false。

---

### ***设置末端负载***
```cpp
bool setPayload(double mass, const vector3d_t& cog)
```
- ***功能***

    该命令用于设置机器人载荷的质量、重心和转动惯量。

- ***参数***
    - mass：负载质量

    - cog：有效载荷的重心坐标（相对于法兰框架）。

- ***返回值***：指令发送成功返回 true，失败返回 false。

---

### ***设置工具电压***
```cpp
bool setToolVoltage(const ToolVoltage& vol)
```
- ***功能***

    设置工具电压
    
- ***参数***
    - vol：工具电压

- ***返回值***：指令发送成功返回 true，失败返回 false。

---

### ***开启力控模式***
```cpp
bool startForceMode(const vector6d_t& reference_frame, const vector6int32_t& selection_vector, const vector6d_t& wrench, const ForceMode& mode, const vector6d_t& limits)
```
- ***功能***

    开启力控模式。

- ***参数***
    - reference_frame：定义力控参考坐标系的位姿矢量，该位姿矢量是相对于基座坐标系定义的。格式为 [X,Y,Z,Rx,Ry,Rz]，其中 X、Y、Z 表示位置，单位为 m；Rx、Ry、Rz 表示位姿，单位为 rad。Rx、Ry、Rz 采用标准 RPY 角定义。selection_vector：由 0 和 1 组成的六维矢量，用于定义力控坐标系中的力控轴，1表示力控轴，0 表示非力控轴。
    - selection_vector：由 0 和 1 组成的六维矢量，用于定义力控坐标系中的力控轴，1表示力控轴，0 表示非力控轴。
    - wrench：机器人施加于环境的目标力/力矩。机器人将沿/绕力控轴调整位姿以便达到指定的目标力/力矩。格式为 [Fx,Fy,Fz,Mx,My,Mz]，其中 Fx、Fy、Fz 表示沿力控轴方向施加的力，单位为 N；Mx、My、Mz 表示绕力控轴方向施加的力矩，单位为 Nm。该值对非力控轴无效。由于关节安全限制，实际施加的力/力矩可能低于设置的目标力/力矩。在单独的线程中使用 get_tcp_force 脚本指令可读取实际施加于环境的力/力矩。
    - mode：力控模式参数，integer 型数据，范围为 0 到 3，用于定义力控模式，即：力控坐标系如何定义或者如何由力控参考坐标系变换获得。
        - 0：固定模式。力控坐标系为力控参考坐标系。
        - 1：点模式。力控坐标系的 Y 轴由机器人 TCP 原点指向力控参考坐标系的原点。
        - 2：运动模式。力控坐标系的 X 轴为 TCP 移动方向矢量在力控参考坐标系的 X-Y 平面内的投影。
    - limits：速度限制参数，六维矢量，float 型数据。格式为\[Vx,Vy,Vz,ωx,ωy,ωz\]，其中 Vx、Vy、Vz 表示沿该轴允许的最大 TCP 速度，单位为m/s；ωx、ωy、ωz 表示绕该轴允许的最大 TCP 速度，单位为 rad/s。该速度限制参数对非力控轴无效，非力控轴仍执行该轴上的原始轨迹。

- ***返回值***：指令发送成功返回 true，失败返回 false。

---

### ***关闭力控模式***
```cpp
bool endForceMode()
```
- ***功能***

    关闭力控模式
    
- ***返回值***：指令发送成功返回 true，失败返回 false。

---

### ***打开或关闭碰撞检测***
```cpp
bool setCollisionDetectEnabled(bool enable)
```
- ***功能***

    通过外部控制脚本命令 socket 打开或关闭碰撞检测。

- ***参数***
    - enable：true 表示打开碰撞检测，false 表示关闭碰撞检测。

- ***返回值***：指令发送成功返回 true，失败返回 false。

---

### ***设置碰撞检测灵敏度***
```cpp
bool setCollisionSensitivity(int32_t ratio)
```
- ***功能***

    设置碰撞检测灵敏度。

- ***参数***
    - ratio：碰撞检测灵敏度，单位为百分比，有效范围为 [10, 100]。

- ***返回值***：指令发送成功返回 true，失败返回 false。

---

### ***设置机器人安装平面***
```cpp
bool setMountingPlane(double z_rotation, double tilt = 0.0)
```
- ***功能***

    通过动态调整重力方向设置机器人安装平面。参数含义对应示教器安装平面的显示值。

- ***参数***
    - z_rotation：绕机器人基坐标系 Z 轴的旋转角度，单位为 rad。
    - tilt：安装平面倾斜角度，单位为 rad。

- ***注意***：重力向量按 `[g * sin(tilt) * sin(z_rotation), -g * sin(tilt) * cos(z_rotation), g * cos(tilt)]` 计算，其中 `g = 9.82`。

- ***返回值***：指令发送成功返回 true，失败返回 false。

---

## 其余

### ***停止外部控制***
```cpp
bool stopControl(int wait_ms = 10000)
```
- ***功能***

    发送停止指令到机器人，机器人将退出控制脚本，并且将停止接收来自PC的指令。

- ***参数***
    - wait_ms: 阻塞等待机器人断开连接的时间（毫秒）。范围：> 5ms。

- ***返回值***

    指令发送成功返回 true，失败返回 false。以下情况会返回false：
    - 已经与机器人断开连接。
    - 等待时间内未与机器人断开连接。

---

### ***是否与机器人连接***
```cpp
bool isRobotConnected()
```
- ***功能***

        是否和机器人连接上

- ***返回值***：已连接返回 true，未连接返回 false。

---

### 发送脚本
```cpp
bool sendScript(const std::string& script)
```
- ***功能***

    向机器人的30001端口发送可执行脚本

- ***参数***
    - script：待发送的脚本。

- ***返回值***：发送成功返回 true，失败返回 false。

---

### ***发送控制脚本***
```cpp
bool sendExternalControlScript()
```
- ***功能***
    向机器人发送外部控制脚本。可用于建立或恢复与机器人的控制。

- ***返回值***：发送成功返回 true，失败返回 false。

---

### ***获取机器人Primary端口的数据包***
```cpp
bool getPrimaryPackage(std::shared_ptr<PrimaryPackage> pkg, int timeout_ms)
```
- ***功能***
    获取机器人30001的数据包

- ***参数***
    - pkg：待获取的数据包（参考[PrimaryPort](./PrimaryPort.cn.md)）
    
    - timeout_ms:获取超时时间。

- ***返回值***：获取成功返回 true，失败返回 false。

---

### ***重新连接机器人Primary端口***
```cpp
bool primaryReconnect()
```

- ***功能***
    重新建立连接到机器人的30001端口。

- ***返回值***：成功返回 true，失败返回 false。

---

### ***获取机器人Primary端口接口***
```cpp
PrimaryPortInterface& primaryPort()
```

- ***功能***
    获取 `EliteDriver` 内部持有并已连接的 `PrimaryPortInterface` 实例，避免用户再次实例化并重复连接 30001 端口。

- ***返回值***：PrimaryPortInterface 引用。

- ***注意***
    返回对象的生命周期由 `EliteDriver` 管理，请勿在 `EliteDriver` 析构后继续使用该引用。

---

### ***注册机器人异常回调***
```cpp
void registerRobotExceptionCallback(std::function<void(RobotExceptionSharedPtr)> cb)
```

- ***功能***
    注册机器人异常回调函数。当从机器人的 primary 端口接收到异常报文时，将调用该回调函数。回调函数接收一个 RobotExceptionSharedPtr 类型的参数，表示发生的异常信息。

- ***参数***
    - registerRobotExceptionCallback: 回调函数，用于处理接收到的机器人异常。参数为机器人异常的共享指针(参考：[RobotException](./RobotException.cn.md))。
    
---

### ***启用工具RS485通讯***
```cpp
SerialCommunicationSharedPtr startToolRs485(const SerialConfig& config,  const std::string& ssh_password, int tcp_port = 54321)
```

- ***功能***
    启用工具RS485通讯。此接口会通过ssh登录机器人控制柜操作系统，并在机器人控制器上启动一个 socat 进程，将工具RS485串口的数据转发到指定的 TCP/IP 端口。

- ***参数***
    - config：串口配置。详情可查看：[串口通讯](./SerialCommunication.cn.md)
    - ssh_password：机器人控制柜操作系统ssh登录密码。
    - tcp_port：TCP 端口。

- ***返回值***：一个可以操作串口的对象。详情可查看：[串口通讯](./SerialCommunication.cn.md)
- ***注意***：如果要使用此功能，建议安装 libssh ，如果在非Linux系统下使用，则必须安装 libssh 库。
---

### ***停止工具RS485通讯***
```cpp
bool endToolRs485(SerialCommunicationSharedPtr com, const std::string& ssh_password)
```

- ***功能***
    停止工具RS485通讯。

- ***参数***
    - com：操作串口的对象。
    - ssh_password：机器人控制柜操作系统ssh登录密码。

- ***返回值***：成功停止工具RS485通讯。
- ***注意***：如果要使用此功能，建议安装 libssh ，如果在非Linux系统下使用，则必须安装 libssh 库。

---

### ***启用主板RS485通讯***
```cpp
SerialCommunicationSharedPtr startBoardRs485(const SerialConfig& config,  const std::string& ssh_password, int tcp_port = 54321)
```

- ***功能***
    启用主板RS485通讯。此接口会通过ssh登录机器人控制柜操作系统，并在机器人控制器上启动一个 socat 进程，将工具RS485串口的数据转发到指定的 TCP/IP 端口。

- ***参数***
    - config：串口配置。详情可查看：[串口通讯](./SerialCommunication.cn.md)
    - ssh_password：机器人控制柜操作系统ssh登录密码。
    - tcp_port：TCP 端口。

- ***返回值***：一个可以操作串口的对象。详情可查看：[串口通讯](./SerialCommunication.cn.md)
- ***注意***：如果要使用此功能，建议安装 libssh ，如果在非Linux系统下使用，则必须安装 libssh 库。
---

### ***停止工具RS485通讯***
```cpp
bool endBoardRs485(SerialCommunicationSharedPtr com, const std::string& ssh_password)
```

- ***功能***
    停止主板RS485通讯。

- ***参数***
    - com：操作串口的对象。
    - ssh_password：机器人控制柜操作系统ssh登录密码。

- ***返回值***：成功停止工具RS485通讯。
- ***注意***：如果要使用此功能，建议安装 libssh ，如果在非Linux系统下使用，则必须安装 libssh 库。

---
