[中文](./README.cn.md)
# Elite Robots CS SDK

This SDK is a C++ library for Elibot Robots' CS series robotic arms. With this library, developers can implement C++-based drivers to leverage the versatility of Elibot CS series robotic arms for building external applications.

## Requirements
- ***CS Controller*** (robot control software)  
    - Require **≥ 2.15.0**.  
    - If your robot's control software version is lower than these, an upgrade is recommended.
- boost version >= (recommend)1.74
- cmake version >= 3.22.1

## Build & Install
If your system is Ubuntu20.04, Ubuntu22.04 or Ubuntu24.04, you can run the following command to install elite-cs-series-sdk:
```bash
sudo add-apt-repository ppa:elite-robots/cs-robot-series-sdk
sudo apt update
sudo apt install elite-cs-series-sdk
```

If compilation and installation are required, please refer to the [Compilation Guide](./doc/BuildGuide/BuildGuide.en.md). 

## User guide
[English guide](./doc/UserGuide/en/UserGuide.en.md)

## Architecture
[Code architecture](./doc/Architecture/Arch.en.md)

## API document
[API](./doc/API/en/API.en.md)

## Compatible Operating Systems
Tested on the following system platforms:

 * Ubuntu 22.04 (Jammy Jellyfish)
 * Ubuntu 16.04 (Xenial Xerus)
 * Windows 11

## Compiler
Currently compiled with the following compilers:

 * gcc 11.4.0
 * gcc 5.5.0
 * msvc 19.40.33808


以下是我从 `feature/force_mode` 分支读取到的新脚本功能总结，可直接整理进脚本手册。

**核心接口**

```python
zero_ftsensor()
force_mode(task_frame, selection_vector, wrench, type, limits)
end_force_mode()
force_mode_set_damping(damping=0.005)
force_mode_set_gain_scaling(gain_scaling=1.0)
```

**force_mode()**

```python
force_mode(task_frame, selection_vector, wrench, type, limits)
```

参数说明：

- `task_frame`：
  - 可以是位姿列表 `[x, y, z, rx, ry, rz]`
  - 也可以是字符串 `"tcp"` 或 `"TCP"`
- `selection_vector`：
  - 长度 6，只允许 `0` 或 `1`
  - `[x, y, z, rx, ry, rz]`
  - `1` 表示该轴进入力控/柔顺控制
  - `0` 表示该轴保持刚性跟踪
- `wrench`：
  - 长度 6，目标力/力矩
  - `[Fx, Fy, Fz, Tx, Ty, Tz]`
  - 平移单位：N
  - 旋转单位：Nm
- `type`：
  - `1`：point 模式
  - `2`：fixed 模式
  - `3`：motion 模式
- `limits`：
  - 长度 6
  - 对 `selection_vector=1` 的轴，表示该轴最大柔顺速度限制
    - 平移轴单位：m/s
    - 旋转轴单位：rad/s
  - 对 `selection_vector=0` 的轴，表示该轴最大允许偏移限制
    - 平移轴单位：m
    - 旋转轴单位：rad
  - 所有值必须 `>= 0`

**坐标模式**

`type=1`，point 模式：

- `task_frame` 必须是位姿列表。
- 控制坐标系会随 TCP 位置变化。
- 其 Y 轴指向 `TCP -> task_frame 原点` 方向。
- 常用于围绕某个空间点的力控/柔顺跟踪。

`type=2`，fixed 模式：

- `task_frame` 可以是位姿列表，也可以是 `"tcp"`。
- 如果是位姿列表，则使用固定任务坐标系。
- 如果是 `"tcp"`，则任务坐标系实时等于当前 TCP 坐标系。
- 这是实现“始终沿 TCP Z 方向力控”的推荐方式。

`type=3`，motion 模式：

- `task_frame` 必须是位姿列表。
- X 轴根据实际运动方向动态更新。
- Z 轴参考 `task_frame` 的 Z 方向。
- 该模式不允许 `selection_vector[0] = 1`，也就是不支持 X 方向力控选择。

**约束规则**

- `type` 只接受 `1/2/3`。
- `selection_vector` 只接受 `0/1`。
- `limits` 必须全为非负数。
- `task_frame="tcp"` 只支持 `type=2`。
- 没有力传感器时，相关函数会报错。

**常用示例**

沿 TCP Z 方向施加/保持力：

```python
zero_ftsensor()
force_mode("tcp",
           [0, 0, 1, 0, 0, 0],
           [0.0, 0.0, -5.0, 0.0, 0.0, 0.0],
           2,
           [0.0, 0.0, 0.15, 0.0, 0.0, 0.0])
sleep(10.0)
end_force_mode()
stopl(5.0)
```

固定基坐标 Z 方向力控：

```python
zero_ftsensor()
force_mode([0, 0, 0, 0, 0, 0],
           [0, 0, 1, 0, 0, 0],
           [0.0, 0.0, -5.0, 0.0, 0.0, 0.0],
           2,
           [0.0, 0.0, 0.15, 0.0, 0.0, 0.0])
sleep(10.0)
end_force_mode()
```

**手感调节**

```python
force_mode_set_damping(0.005)
force_mode_set_gain_scaling(1.0)
```

- `force_mode_set_damping(damping)`
  - 范围：`0.0 ~ 1.0`
  - 越大越稳、越钝；越小越灵敏。
- `force_mode_set_gain_scaling(gain_scaling)`
  - 范围：`0.0 ~ 2.0`
  - 越大对力误差响应越强，越跟手，但过大可能抖动。

建议客户先使用默认值；如果感觉拖动偏重，可以先把 `gain_scaling` 调到 `1.2~1.5`。

**研发诊断接口**

```python
force_mode_set_developer_logging(enable)
force_mode_get_developer_logging()
force_mode_debug_enable(enable, decimation=5, capacity=2000)
force_mode_debug_clear()
force_mode_debug_snapshot()
force_mode_debug_fetch(clear=True)
force_mode_debug_dump_csv(path=None, clear=True)
```

说明：

- 研发日志默认关闭。
- `force_mode_set_developer_logging(True)` 后才会记录调试数据。
- `force_mode_debug_enable(True, decimation, capacity)` 开启内存缓存。
- `force_mode_debug_dump_csv()` 可导出 CSV。
- 关闭 developer logging 时，不应产生 force mode CSV。