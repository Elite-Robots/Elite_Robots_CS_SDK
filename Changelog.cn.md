# Changelog for Elite Robots CS SDK

## [Unrelease]

### Added
- 新增一个带速度规划的“servoj”示例。
- `RtsiIOInterface` 
  - 新增字符串列表构造函数。
  - 新增判断是否连接的接口:`isConnected()`。
  - 新增判断是否开始数据同步的接口`isStarted()`。
  - 新增接口 `getActualTCPPose()`、`getActualTCPVelocity()`、`getActualTCPForce()`（修正原拼写错误）。
- `DashboardClient`
  - 新增获取机器人类型接口：`robotType()`
  - 新增获取机器人序列号接口：`robotSerialNumber()`
  - 新增获取机器人ID接口：`robotID()`
- 默认的日志句柄增加时间戳信息。
- 新增串口通讯相关接口。
- 添加了一个启动docker仿真的脚本。

### Changed
- 调整 `external_control.script` 中 “trajectory_socket” 的“timeout”值。
- `RtsiIOInterface` 允许输入空路径以及空列表。

### Fixed
- 修复 `external_control.script` 中 `extrapolate()`函数计算的步长为固定的steptime的问题。
- 修复解析30001机器人错误异常报文时，遗漏了“float”类型的错误。
- 修复解析30001机器人错误异常报文时，错误等级不正确的问题。
- 修复收到30001不完整的报文后，出现的内存问题。
- 修复`TcpServer`中静态资源析构顺序提前的问题。
- 修复`EliteDriver::registerRobotExceptionCallback()`接口没有实现的问题。
- 修复析构时会崩溃的问题。
- 修复`EliteDriver::startForceMode()`不生效的问题。

### Deprecated
- 弃用 `DashboardClient::robot()` 未来版本将移除，请改用 `DashboardClient::robotType()`
- 由于拼写错误，以下接口已弃用，将在未来版本移除，请使用对应的新接口：
  - `RtsiIOInterface::getAcutalTCPPose()` -> `RtsiIOInterface::getActualTCPPose()`
  - `RtsiIOInterface::getAcutalTCPVelocity()` -> `RtsiIOInterface::getActualTCPVelocity()`
  - `RtsiIOInterface::getAcutalTCPForce()` -> `RtsiIOInterface::getActualTCPForce()`

### Removed
- 移除`EliteDriver::writeServoj()`的`queue_mode`参数

## [v1.2.0] - 2025-08-14

### Added
- `ControllerLog::downloadSystemLog()`：下载机器人系统日志的接口。
- `UPGRADE::upgradeControlSoftware()`：远程升级机器人控制软件的接口。
- `EliteDriver`：
  - `writeFreedrive()`：新增 Freedrive 控制接口。
  - `registerRobotExceptionCallback()`： 新增机器人异常回调注册接口
  - `EliteDriver()`: 构造函数新增`stopj_acc`参数（停止运动的加速度）。
  - `writeServoj()`: 新增`cartesian`和`queue_mode`参数。
- `EliteDriverConfig`：新增构造配置类，支持更灵活的初始化。
  - 通过 `EliteDriver(const EliteDriverConfig& config)` 构造时，不再强制要求本地 IP。
  - 新增`servoj_queue_pre_recv_size`与`servoj_queue_pre_recv_timeout`参数。
- `PrimaryPortInterface`：
  - `getLocalIP()`：新增获取本地 IP 的接口。
  - `registerRobotExceptionCallback()`： 新增机器人异常回调注册接口
- 新增机器人异常接口：
  - `RobotException`：基类。
  - `RobotError`：机器人错误。
  - `RobotRuntimeException`：机器人脚本运行时错误。
- `DashboardClient`
  - 新增`sendAndReceive()`接口。
- 新增完整的 API 文档（Markdown）。
- 新增编译向导文档。

### Changed
- 重构 `TcpServer` 的多端口监听逻辑，统一在单线程中处理以下端口：
  - Reverse port
  - Trajectory port
  - Script sender port
  - Script command port
- 优化 `PrimaryPortInterface`：
  - 改用同步方式接收和解析数据。
  - 支持断线后自动重连。
- 优化 RTSI 模块性能：
  - 减少 `RtsiClientInterface` 的 Socket 数据拷贝次数。
  - 简化 `RtsiIOInterface.hpp` 的后台线程循环逻辑。
  - `RtsiIOInterface::getRecipeValue()` 与 `RtsiIOInterface::setInputRecipeValue()`接口从private变为public，并且添加了常用的显式实例化声明。
  - `RtsiIOInterface`改为保护继承。
- 调整了项目Readme的结构。
- 增强`external_control.script`的线程安全性
- 使用`boost::program_options`来解析示例程序的输入参数。


### Fixed
- 修复 `EliteDriver` 析构时可能因悬垂指针导致的崩溃问题。
- 修复 `DashboardClient::setSpeedScaling()` 接口设置新值返回失败的问题。

### Deprecated
- `EliteDriver` 的旧构造函数已废弃，未来版本将移除，请改用 `EliteDriverConfig`。

### Removed
- 移除`EliteException::Code::SOCKET_OPT_CANCEL` 

## [v1.1.0] - 2024-10-30
### Initial Release
- 首次公开版本发布。