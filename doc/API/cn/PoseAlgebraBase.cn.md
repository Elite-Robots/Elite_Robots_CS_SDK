# PoseAlgebraBase 模块

## 简介

`PoseAlgebraBase` 模块定义了位姿代数（Pose Algebra）插件的统一接口。
该接口支持基础位姿运算（求逆、组合、加减、变换矩阵与位姿向量转换、距离计算）以及世界坐标系与本地坐标系之间的相互转换。

SDK 当前内置了两个实现：
- `ELITE::ElitePoseAlgebra`
- `ELITE::EigenPoseAlgebra`

---

## 头文件

```cpp
// 位姿代数接口
#include <Elite/PoseAlgebraBase.hpp>

// 插件加载（用于动态加载插件）
#include <Elite/ClassLoader.hpp>
```

---

# 一、PoseAlgebraError 枚举

```cpp
enum class PoseAlgebraError {
    SUCCESS = 0,
    INVALID_INPUT,
    SINGULAR_MATRIX,
    INVALID_ROTATION_MATRIX,
    NUMERICAL_ERROR,
    UNSUPPORTED_OPERATION,
    INTERNAL_ERROR,
};
```

## 说明

位姿代数操作的错误码。

| 枚举值 | 说明 |
| :--- | :--- |
| `SUCCESS` | 操作成功。 |
| `INVALID_INPUT` | 输入无效（如 NaN/Inf 或数据格式错误）。 |
| `SINGULAR_MATRIX` | 矩阵求逆失败，矩阵是奇异的。 |
| `INVALID_ROTATION_MATRIX` | 旋转部分不是有效的正交旋转矩阵。 |
| `NUMERICAL_ERROR` | 数值不稳定、溢出或精度损失。 |
| `UNSUPPORTED_OPERATION` | 当前实现不支持该操作。 |
| `INTERNAL_ERROR` | 内部异常。 |

---

# 二、PoseMatrix 结构体

```cpp
struct PoseMatrix {
    std::array<std::array<double, 4>, 4> data;
};
```

## 说明

4x4 齐次变换矩阵。矩阵使用标准刚体变换布局：
$$
\begin{bmatrix}
R_{3 \times 3} & t_{3 \times 1} \\
0_{1 \times 3} & 1
\end{bmatrix}
$$
存储方式为行优先（row-major）。

---

# 三、PoseDistance 结构体

```cpp
struct PoseDistance {
    double linear_distance;
    double angular_distance;
};
```

## 说明

两个位姿之间的距离分量。

### 成员

| 成员 | 类型 | 说明 |
|------|------|------|
| `linear_distance` | `double` | 线性距离（米）。 |
| `angular_distance` | `double` | 角度距离（弧度）。 |

---

# 四、PoseAlgebraResult 结构体

```cpp
struct PoseAlgebraResult {
    PoseAlgebraError error;
    std::string message;
};
```

## 说明

详细的操作状态。

### 成员

| 成员 | 类型 | 说明 |
|------|------|------|
| `error` | `PoseAlgebraError` | 操作状态码。 |
| `message` | `std::string` | 详细错误消息或诊断信息。 |

---

# 五、PoseAlgebraBase 类

```cpp
class PoseAlgebraBase
```

## 说明

位姿代数插件的抽象基类。所有位姿代数插件必须继承此类。支持 `PoseMatrix` 结构与 `vector6d_t` 结构的操作。

---

## 常量

```cpp
static constexpr double ZERO_TOLERANCE = 1e-6;
```

验证与旋转检查使用的数值容差。

---

## 构造函数

```cpp
PoseAlgebraBase() = default;
```

---

## 析构函数

```cpp
virtual ~PoseAlgebraBase() = default;
```

---

## 静态方法

### setSuccess

```cpp
static void setSuccess(PoseAlgebraResult& result);
```

**说明：** 将操作结果标记为成功并清空错误信息。

### setError

```cpp
static void setError(
    PoseAlgebraResult& result,
    PoseAlgebraError error,
    const std::string& message
);
```

**说明：** 设置错误码与详细的错误信息。

### clamp

```cpp
static double clamp(double value, double low, double high);
```

**说明：** 将数值限制在 `[low, high]` 范围内。

### safeAcos

```cpp
static double safeAcos(double cos_theta);
```

**说明：** 安全的余弦反函数。将输入限制在 `[-1.0, 1.0]` 之间并计算反余弦。

### validateVectorFinite

```cpp
static bool validateVectorFinite(
    const vector6d_t& pose,
    PoseAlgebraResult& result,
    const std::string& name
);
```

**说明：** 验证 6D 向量所有元素是否为有限值。若出现 NaN 或 Inf，在 `result` 中返回错误详情并返回 `false`。

### validateMatrixFinite

```cpp
static bool validateMatrixFinite(
    const PoseMatrix& pose,
    PoseAlgebraResult& result,
    const std::string& name
);
```

**说明：** 验证 4x4 位姿矩阵所有元素是否为有限值。

### determinant3x3

```cpp
static double determinant3x3(const PoseMatrix& pose);
```

**说明：** 计算位姿矩阵旋转部分 (左上角 3x3 子矩阵) 的行列式。

---

## 虚函数接口

所有核心代数操作在其失败时，都会通过 `result` 输出详细的错误信息；操作成功则返回 `true`。为满足多种数据结构要求，部分方法重载了不同的签名格式（如 `PoseMatrix` 与 `vector6d_t`）。

---

### inverse

```cpp
virtual bool inverse(const PoseMatrix& pose, PoseMatrix& inverse_pose, PoseAlgebraResult& result) const = 0;
virtual bool inverse(const vector6d_t& pose, vector6d_t& inverse_pose, PoseAlgebraResult& result) const = 0;
```

#### 说明

计算位姿的逆变换。

#### 参数
- `pose` : 输入待求逆的位姿。
- `inverse_pose` : 输出位姿，保存求逆后的结果。
- `result` : 详细操作结果状态。

---

### multiply

```cpp
virtual bool multiply(const PoseMatrix& left_pose, const PoseMatrix& right_pose, PoseMatrix& out_pose, PoseAlgebraResult& result) const = 0;
virtual bool multiply(const vector6d_t& left_pose, const vector6d_t& right_pose, vector6d_t& out_pose, PoseAlgebraResult& result) const = 0;
```

#### 说明

计算位姿合成（乘法）：$out\_pose = left\_pose  \times  right\_pose$

#### 参数
- `left_pose` : 左侧操作的位姿。
- `right_pose` : 右侧操作的位姿。
- `out_pose` : 输出位姿，合成后的结果。
- `result` : 详细操作结果状态。

---

### add / subtract

```cpp
virtual bool add(const PoseMatrix& left_pose, const PoseMatrix& right_pose, PoseMatrix& out_pose, PoseAlgebraResult& result) const = 0;
virtual bool add(const vector6d_t& left_pose, const vector6d_t& right_pose, vector6d_t& out_pose, PoseAlgebraResult& result) const = 0;

virtual bool subtract(const PoseMatrix& left_pose, const PoseMatrix& right_pose, PoseMatrix& out_pose, PoseAlgebraResult& result) const = 0;
virtual bool subtract(const vector6d_t& left_pose, const vector6d_t& right_pose, vector6d_t& out_pose, PoseAlgebraResult& result) const = 0;
```

#### 说明

对位姿进行分量元素的逐项相加或相减。

#### 参数
- `left_pose` : 第一个输入操作数。
- `right_pose` : 第二个输入操作数。
- `out_pose` : 结果。
- `result` : 详细操作结果状态。

---

### vectorToMatrix / matrixToVector

```cpp
virtual bool vectorToMatrix(const vector6d_t& pose_vector, PoseMatrix& pose_matrix, PoseAlgebraResult& result) const = 0;
virtual bool matrixToVector(const PoseMatrix& pose_matrix, vector6d_t& pose_vector, PoseAlgebraResult& result) const = 0;
```

#### 说明

实现 6D 位姿向量 `[x, y, z, roll, pitch, yaw]` 与 4x4 齐次变换矩阵的互相转换。其中 translation 对应 xyz（单位米），rotation 对应 rpy（单位弧度）。

#### 参数（以 vectorToMatrix 为例）
- `pose_vector` : 输入转换用的 6D 向量位姿。
- `pose_matrix` : 输出计算出的对应的 4x4 位姿矩阵。
- `result` : 详细操作结果状态。

---

### distance

```cpp
virtual bool distance(const PoseMatrix& pose_a, const PoseMatrix& pose_b, PoseDistance& out_distance, PoseAlgebraResult& result) const = 0;
virtual bool distance(const vector6d_t& pose_a, const vector6d_t& pose_b, PoseDistance& out_distance, PoseAlgebraResult& result) const = 0;
```

#### 说明

计算两点之间的平移距离与旋转距离。平移距离通常使用欧氏距离，旋转距离基于方位差异的差角计算。

#### 参数
- `pose_a` : 包含起点信息的位姿。
- `pose_b` : 包含终点信息的位姿。
- `out_distance` : 输出的两点间的平移距离（米）和旋转角度（弧度）。
- `result` : 详细操作结果状态。

---

### worldToLocal / localToWorld

```cpp
virtual bool worldToLocal(const PoseMatrix& world_ref_pose, const PoseMatrix& world_pose, PoseMatrix& local_pose, PoseAlgebraResult& result) const = 0;
virtual bool worldToLocal(const vector6d_t& world_ref_pose, const vector6d_t& world_pose, vector6d_t& local_pose, PoseAlgebraResult& result) const = 0;

virtual bool localToWorld(const PoseMatrix& world_ref_pose, const PoseMatrix& local_pose, PoseMatrix& world_pose, PoseAlgebraResult& result) const = 0;
virtual bool localToWorld(const vector6d_t& world_ref_pose, const vector6d_t& local_pose, vector6d_t& world_pose, PoseAlgebraResult& result) const = 0;
```

#### 说明

用于世界坐标系（World）与本地坐标系（Local）之间的相互转换：
- `worldToLocal`: $	ext{local\_pose} = 	ext{inverse}(world\_ref\_pose) 	imes 	ext{world\_pose}$
- `localToWorld`: $	ext{world\_pose} = 	ext{world\_ref\_pose} 	imes 	ext{local\_pose}$

#### 参数
- `world_ref_pose` : 本地参考系在世界坐标下的表示。
- `world_pose` / `local_pose` : 待转换或转换后结果表示的位姿。
- `result` : 详细操作结果状态。

---

## 使用示例

### 动态加载位姿代数插件

```cpp
#include <Elite/ClassLoader.hpp>
#include <Elite/PoseAlgebraBase.hpp>

// 初始化插件加载器，路径为插件库所在目录
ELITE::ClassLoader loader("path/to/plugins");

// 加载支持的位姿代数插件（例如 ElitePoseAlgebra）
auto pose_alg = loader.createInstance<ELITE::PoseAlgebraBase>("ELITE::ElitePoseAlgebra");

if (pose_alg) {
    ELITE::vector6d_t p1 = {0.5, 0.0, 0.5, 0, 0, 0};
    ELITE::vector6d_t p2 = {0.1, 0.0, 0.0, 0, 0, 0};
    ELITE::vector6d_t out;
    ELITE::PoseAlgebraResult result;

    if (pose_alg->multiply(p1, p2, out, result)) {
        // 输出合成后的位姿
    } else {
        // 错误处理：result.message
    }
}
```
