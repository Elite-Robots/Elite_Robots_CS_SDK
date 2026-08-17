# DashboardClient 类

## 简介

DashboardClient类提供了与机器人仪表盘服务器的交互接口，用于执行各种机器人控制命令和状态查询。

## 头文件
```cpp
#include <Elite/DashboardClient.hpp>
```

## 接口

### 连接服务器
```cpp
bool connect(const std::string& ip, int port = 29999)
```
- ***功能***

    连接仪表盘服务器

- ***参数***

    - ip：仪表盘服务器IP地址
    - port：仪表盘服务器端口(默认29999)

- ***返回值***：连接成功返回true，失败返回false

---

### 断开连接
```cpp
void disconnect()
```
- ***功能***

    断开与仪表盘服务器的连接

---

### 抱闸释放
```cpp
bool brakeRelease()
```
- ***功能***

    释放机器人抱闸。

- ***返回值***：操作成功返回true，失败返回false

---

### 关闭安全弹窗
```cpp
bool closeSafetyDialog()
```
- ***功能***

    关闭安全消息弹窗

- ***返回值***：操作成功返回true，失败返回false

---

### 连接检查
```cpp
bool echo()
```
- ***功能***

    检查与仪表盘shell服务器的连接状态

- ***返回值***：机器人响应成功返回true

---

### 获取帮助信息
```cpp
std::string help(const std::string& cmd)
```
- ***功能***

    获取指定命令的帮助信息

- ***参数***

    - cmd：需要帮助的命令

- ***返回值***：命令的帮助字符串

---

### 添加日志
```cpp
bool log(const std::string& message)
```
- ***功能***

    添加日志消息(消息中的'\n'或'\r'会被替换为"\\n"和"\\r")

- ***参数***

    - message：日志内容

- ***返回值***：操作成功返回true，失败返回false

---

### 弹出/关闭消息框
```cpp
bool popup(const std::string& arg, const std::string& message = "")
```
- ***功能***

    弹出或关闭消息框

- ***参数***

    - arg：
        - "-c": 关闭消息框
        - "-s": 弹出消息框
    - message：消息内容(可选)

- ***返回值***：操作成功返回true，失败返回false

---

### 退出仪表盘
```cpp
void quit()
```
- ***功能***

    退出仪表盘并断开连接

---

### 重启机器人
```cpp
void reboot()
```
- ***功能***

    重启机器人并断开连接

---

### 获取机器人类型
```cpp
std::string robot()
```
- ***功能***

    获取机器人类型

- ***返回值***：机器人类型字符串

---

### 机器人上电
```cpp
bool powerOn()
```
- ***功能***

    机器人上电

- ***返回值***：操作成功返回true，失败返回false

---

### 机器人断电
```cpp
bool powerOff()
```
- ***功能***

    机器人断电

- ***返回值***：操作成功返回true，失败返回false

---

### 关闭机器人
```cpp
void shutdown()
```
- ***功能***

    关闭机器人并断开连接

---

### 获取远程控制模式状态
```cpp
bool remoteControl()
```
- ***功能***

    获取远程控制模式是否已开启

- ***返回值***：远程控制模式已开启返回 true，否则返回 false

---

### 设置远程控制模式状态
```cpp
bool remoteControl(bool enable)
```
- ***功能***

    打开或关闭远程控制模式

- ***参数***
    - enable：true 表示打开远程控制模式，false 表示关闭远程控制模式

- ***返回值***：操作成功返回 true，失败返回 false

---

### 获取当前控制模式
```cpp
RemoteControlMode remoteControlMode()
```
- ***功能***

    获取当前控制模式

- ***返回值***：返回 `RemoteControlMode::REMOTE`、`RemoteControlMode::LOCAL`、`RemoteControlMode::NONE` 或 `RemoteControlMode::UNKNOWN`

---

### 获取速度比例
```cpp
int speedScaling()
```
- ***功能***

    获取机器人速度比例百分比

- ***返回值***：速度比例百分比

---

### 获取机器人模式
```cpp
RobotMode robotMode()
```
- ***功能***

    获取机器人当前模式

- ***返回值***：RobotMode枚举值

---

### 获取安全模式
```cpp
SafetyMode safetyMode()
```
- ***功能***

    获取安全模式

- ***返回值***：SafetyMode枚举值

---

### 重启安全系统
```cpp
bool safetySystemRestart()
```
- ***功能***

    重启安全系统

- ***返回值***：操作成功返回true，失败返回false

---

### 获取任务状态
```cpp
TaskStatus runningStatus()
```
- ***功能***

    获取任务运行状态

- ***返回值***：TaskStatus枚举值

---

### 解除保护性停止
```cpp
bool unlockProtectiveStop()
```
- ***功能***

    解除机器人保护性停止

- ***返回值***：操作成功返回true，失败返回false

---

### 获取命令用法
```cpp
std::string usage(const std::string& cmd)
```
- ***功能***

    查询仪表盘shell命令的用法

- ***参数***

    - cmd：要查询的命令

- ***返回值***：命令用法字符串

---

### 获取仪表盘版本
```cpp
std::string version()
```
- ***功能***

    获取仪表盘版本信息

- ***返回值***：版本信息字符串

---

### 加载机器人配置
```cpp
bool loadConfiguration(const std::string& path)
```
- ***功能***

    加载机器人配置文件

- ***参数***

    - path：配置文件路径

- ***返回值***：加载成功返回true，失败返回false

---

### 获取配置路径
```cpp
std::string configurationPath()
```
- ***功能***

    获取当前配置文件路径

- ***返回值***：配置文件路径字符串

---

### 检查配置修改
```cpp
bool isConfigurationModify()
```
- ***功能***

    检查配置是否被修改

- ***返回值***：已修改返回true

---

### 运行程序
```cpp
bool playProgram()
```
- ***功能***

    运行程序

- ***返回值***：操作成功返回true，失败返回false

---

### 暂停程序
```cpp
bool pauseProgram()
```
- ***功能***

    暂停程序

- ***返回值***：操作成功返回true，失败返回false

---

### 停止程序
```cpp
bool stopProgram()
```
- ***功能***

    停止程序

- ***返回值***：操作成功返回true，失败返回false

---

### 设置速度比例
```cpp
bool setSpeedScaling(int scaling)
```
- ***功能***

    设置速度比例

- ***参数***

    - scaling：要设置的速度比例

- ***返回值***：设置成功返回true，失败返回false

---

### 获取任务路径
```cpp
std::string getTaskPath()
```
- ***功能***

    获取当前任务路径

- ***返回值***：任务相对路径字符串

---

### 加载任务
```cpp
bool loadTask(const std::string& path)
```
- ***功能***

    加载任务

- ***参数***

    - path：任务路径

- ***返回值***：加载成功返回true，失败返回false

---

### 获取任务状态
```cpp
TaskStatus getTaskStatus()
```
- ***功能***

    获取任务状态

- ***返回值***：TaskStatus枚举值

---

### 检查任务运行状态
```cpp
bool taskIsRunning()
```
- ***功能***

    检查任务是否正在运行

- ***返回值***：正在运行返回true

---

### 检查任务保存状态
```cpp
bool isTaskSaved()
```
- ***功能***

    检查任务是否已保存

- ***返回值***：已保存返回true

---

### 发送并接收命令
```cpp
std::string sendAndReceive(const std::string& cmd)
```
- ***功能***

    发送仪表盘命令并接收响应

- ***参数***

    - cmd：要发送的仪表盘命令

- ***返回值***：命令响应字符串

--- 
