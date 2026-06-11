# Primary Port

## 简介

SDK中提供了与机器人30001端口连接、发送脚本的接口，以及解析30001端口数据的框架。SDK中只有部分数据包的解析，如果要解析其他数据包，需要手动编写解析代码。

# PrimaryPortInterface 类

## 简介

此接口提供了与机器人primary port接口通讯的方法。

## PrimaryPortInterface 头文件
```cpp
#include <Elite/PrimaryPortInterface.hpp>
```

## PrimaryPortInterface 类的构造函数

### ***构造函数***
```cpp
PrimaryPortInterface::PrimaryPortInterface()
```
- ***功能***

    初始化数据。注意，此构造函数不会连接机器人。

---

## 通讯

### ***连接***
```cpp
bool connect(const std::string& ip, int port = PRIMARY_PORT)
```

- ***功能***

    连接到机器人的30001端口（默认）。

- ***参数***
    - ip：机器人端口

    - timeout_ms：设置机器人读取下一条指令的超时时间，小于等于0时会无限等待。

- ***返回值***：成功返回 true，失败返回 false。

- ***注意***
    1. 如果重复调用此函数而没有调用disconnect，那么会断开原来的连接。
    2. 重复调用此函数的频率最好不要超过2HZ。

---

### ***断开连接***
```cpp
void disconnect()
```
- ***功能***

    断开与机器人的连接

- ***注意***

    建议在调用此函数之后进行500ms左右延迟再调用connect。

---

### ***发送脚本***
```cpp
bool sendScript(const std::string& script)
```
- ***功能***

    向机器人发送可执行脚本

- ***参数***
    - script：待发送的脚本。

- ***返回值***：发送成功返回 true，失败返回 false。

---

### ***机器人上电***
```cpp
bool powerOn()
```
- ***功能***

    通过 30001 端口发送普通脚本 `power on`，并通过 30001 Primary 状态报文确认机器人模式。

- ***返回值***：发送成功且 `RobotModeData.robot_mode` 变为 `IDLE` 或 `RUNNING` 返回 true；发送失败或等待目标状态超时返回 false。

- ***注意***

    如果当前机器人模式已经是 `IDLE` 或 `RUNNING`，此接口会直接返回 true。

---

### ***机器人下电***
```cpp
bool powerOff()
```
- ***功能***

    通过 30001 端口发送普通脚本 `power off`，并通过 30001 Primary 状态报文确认机器人模式。

- ***返回值***：发送成功且 `RobotModeData.robot_mode` 变为 `POWER_OFF` 返回 true；发送失败或等待目标状态超时返回 false。

---

### ***释放抱闸***
```cpp
bool brakeRelease()
```
- ***功能***

    通过 30001 端口发送普通脚本 `set robotmode run`，释放机器人抱闸。

- ***返回值***：发送成功且 `RobotModeData.robot_mode` 变为 `RUNNING` 返回 true；前置状态不满足、发送失败或等待目标状态超时返回 false。

- ***注意***

    此接口会先检查 30001 状态报文：机器人模式需要为 `IDLE`，安全模式需要为 `NORMAL`、`REDUCED` 或 `RECOVERY`。如果当前机器人模式已经是 `RUNNING`，会直接返回 true。

---

### ***暂停任务***
```cpp
bool pauseProgram()
```
- ***功能***

    通过 30001 端口发送普通脚本 `pause task`，暂停当前正在运行的任务。

- ***返回值***：发送成功且任务状态变为 `PAUSED` 返回 true；当前任务未运行、发送失败或等待目标状态超时返回 false。

- ***注意***

    此接口根据 `RobotModeData.is_task_running` 和 `RobotModeData.is_task_paused` 判断任务状态。只有当前任务状态为 `PLAYING` 时才会发送暂停命令；如果任务已经是 `PAUSED`，会直接返回 true。

---

### ***停止任务***
```cpp
bool stopProgram()
```
- ***功能***

    通过 30001 端口发送普通脚本 `stop task`，停止当前任务。

- ***返回值***：发送成功且任务状态变为 `STOPPED` 返回 true；发送失败或等待目标状态超时返回 false。

- ***注意***

    此接口根据 `RobotModeData.is_task_running` 和 `RobotModeData.is_task_paused` 判断任务状态。如果当前任务已经是 `STOPPED`，会直接返回 true。

---

### ***解除保护停***
```cpp
bool unlockProtectiveStop()
```
- ***功能***

    通过 30001 端口发送普通脚本 `set unlock protective stop`，解除机器人保护停状态。

- ***返回值***：发送成功且 `RobotModeData.is_robot_protective_stopped` 变为 false 返回 true；发送失败或等待目标状态超时返回 false。

---

### ***重启安全系统***
```cpp
bool safetySystemRestart()
```
- ***功能***

    通过 30001 端口发送普通脚本 `restart safetyboard`，重启安全板。

- ***返回值***：发送成功且 `MasterBoardData.safety_mode` 变为 `NORMAL` 返回 true；发送失败或等待目标状态超时返回 false。

---

### ***设置速度比例***
```cpp
bool setSpeedScaling(int scaling)
```
- ***功能***

    通过 30001 端口发送普通脚本 `set speed <scaling / 100.0>`，设置机器人目标速度比例。

- ***参数***
    - scaling：目标速度百分比。

- ***返回值***：发送成功且 `RobotModeData.target_speed_fraction` 回读为目标值返回 true；发送失败或等待目标状态超时返回 false。

- ***注意***

    30001 状态报文解析仅作为这些控制接口的内部确认机制，不作为 `PrimaryPortInterface` 的公开状态查询 API。外部业务如果需要主动读取机器人状态，仍建议使用 RTSI 接口。

---

### 获取数据包
```cpp
bool getPackage(std::shared_ptr<PrimaryPackage> pkg, int timeout_ms)
```
- ***功能***

    获取机器人数据包并解析

- ***参数***
    - pkg：待获取的数据包

    - timeout_ms：等待超时时间。

- ***返回值***：获取成功返回 true，失败返回 false。

---

### 获取本地的IP地址
```cpp
std::string getLocalIP()
```
- ***功能***

    获取本机IP地址。

- ***返回值***：本机IP地址。如果为空，与机器人30001的连接则存在问题。

---

### ***注册机器人异常回调***
```cpp
void registerRobotExceptionCallback(std::function<void(RobotExceptionSharedPtr)> cb)
```

- ***功能***
    注册机器人异常回调函数。当从机器人的 primary 端口接收到异常报文时，将调用该回调函数。回调函数接收一个 RobotExceptionSharedPtr 类型的参数，表示发生的异常信息。

- ***参数***
    - registerRobotExceptionCallback: 回调函数，用于处理接收到的机器人异常。参数为机器人异常的共享指针(参考：[RobotException](./RobotException.cn.md))。


# PrimaryPackage 类

## 简介

此类主要用于被继承，以此来获取Primary端口数据。

SDK中没有提供全部的数据包解析，如果需要其他的数据包，可以参考`RobotConfPackage.hpp`中`KinematicsInfo`的实现方式来编写自己需要的数据包解析。报文的格式参考Elite官方文档“CS_用户手册_机器人状态报文.xlsx”。

## PrimaryPackage 头文件
```cpp
#include <Elite/PrimaryPackage.hpp>
```

## PrimaryPackage 类的构造函数
### ***构造函数***
```cpp
PrimaryPackage::PrimaryPackage(int type)
```
- ***功能***

    初始化数据

- ***参数***
    - type：数据包类型（参考Elite官方文档：CS_用户手册_机器人状态报文.xlsx）

---

## 纯虚函数

### 解析报文
```cpp
void parser(int len, const std::vector<uint8_t>::const_iterator& iter)
```
- ***功能***

     由子类来完成具体实现，解析Primary端口机器人状态报文的子报文。当子类实例作为参数传入`PrimaryPortInterface::getPackage()`中会被调用。
    
- ***参数***
    - len：子报文的长度。

    - iter：子报文在整个报文中的位置。

---

## 其余

### ***获取报文类型***
```cpp
int getType()
```
- ***功能***

    获取数据包的类型。

- ***返回值***：数据包类型。

---

## SDK内部使用
### 等待更新
```cpp
bool waitUpdate(int timeout_ms)
```
- ***功能***

    等待数据包数据更新。在getPackage（）函数中调用。

- ***参数***
    - timeout_ms：超时时间
    
- ***返回值***：未超时返回 true，超时返回 false。

---

# PrimaryStatePackage 类

## 简介

SDK 提供了部分 30001 状态子报文解析类，用于解析控制接口内部确认所需的机器人状态。此类解析流程与自定义 `PrimaryPackage` 一致：继承 `PrimaryPackage`、实现 `parser()`，并通过 `PrimaryPortInterface::getPackage()` 获取对应子报文。

## PrimaryStatePackage 头文件

```cpp
#include <Elite/PrimaryStatePackage.hpp>
```

## RobotModeDataPackage 类

### ***功能***

解析 `ROBOT_STATE_PACKAGE_TYPE_ROBOT_MODE_DATA = 0` 子报文。

### ***主要字段***

- `robot_mode`：机器人模式。
- `target_speed_fraction`：目标速度缩放比。
- `speed_scaling`：机器人程序运行速度缩放比。
- `is_robot_power_on`：机器人是否上电。
- `is_robot_protective_stopped`：机器人是否处于保护停止。
- `is_task_running`：任务是否运行。
- `is_task_paused`：任务是否暂停。

### ***使用方式***

```cpp
auto package = std::make_shared<RobotModeDataPackage>();
if (primary.getPackage(package, 500)) {
    RobotModeData data = package->data();
}
```

## MasterBoardDataPackage 类

### ***功能***

解析 `ROBOT_STATE_PACKAGE_TYPE_MASTERBOARD_DATA = 3` 子报文中的安全状态字段。

### ***主要字段***

- `safety_mode`：安全模式。
- `is_robot_in_reduced_mode`：机器人是否处于缩减模式。
- `operational_mode_selector_input`：模式选择器输入状态。
- `threeposition_enabling_device_input`：三位开关输入状态。
- `internal_use`：内部使用字段。

### ***使用方式***

```cpp
auto package = std::make_shared<MasterBoardDataPackage>();
if (primary.getPackage(package, 500)) {
    MasterBoardData data = package->data();
}
```

---

# KinematicsInfo 类

## 简介

机器人配置数据中运动学的数据包解析。PrimaryPackage 是此接口的父类，除了 PrimaryPackage 类中的方法以外，此接口主要内容是机器人的DH参数。

## KinematicsInfo 头文件

```cpp
#include <Elite/RobotConfPackage.hpp>
```

## DH参数

- `vector6d_t dh_a_`

- `vector6d_t dh_d_`

- `vector6d_t dh_alpha_`
