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

### ***写入轨迹路点***
```cpp
bool writeTrajectoryPoint(const vector6d_t& positions, float time, float blend_radius, bool cartesian)
```
- ***功能***

    向专门的socket写入轨迹路点。

- ***参数***
    - positions：路点
    
    - time：到达路点的时间
    
    - blend_radius：两个路点的转接半径

    - cartesian：如果发送的点是笛卡尔的，则为true，如果是基于关节的，则为false

- ***返回值***：指令发送成功返回 true，失败返回 false。

---

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



