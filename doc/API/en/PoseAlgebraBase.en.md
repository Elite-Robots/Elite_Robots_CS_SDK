# PoseAlgebraBase Module

## Overview

The `PoseAlgebraBase` module defines a unified interface for pose algebra plugins.
This interface supports basic pose operations (inverse, composition, addition/subtraction, conversion between transformation matrices and pose vectors, distance computation) and transformations between world and local coordinate frames.

The SDK currently includes two built-in implementations:

* `ELITE::ElitePoseAlgebra`
* `ELITE::EigenPoseAlgebra`

---

## Header Files

```cpp
// Pose algebra interface
#include <Elite/PoseAlgebraBase.hpp>

// Plugin loader (for dynamic plugin loading)
#include <Elite/ClassLoader.hpp>
```

---

# 1. PoseAlgebraError Enum

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

## Description

Error codes for pose algebra operations.

| Enum Value                | Description                                               |
| :------------------------ | :-------------------------------------------------------- |
| `SUCCESS`                 | Operation succeeded.                                      |
| `INVALID_INPUT`           | Invalid input (e.g., NaN/Inf or incorrect data format).   |
| `SINGULAR_MATRIX`         | Failed to invert matrix; matrix is singular.              |
| `INVALID_ROTATION_MATRIX` | Rotation part is not a valid orthogonal rotation matrix.  |
| `NUMERICAL_ERROR`         | Numerical instability, overflow, or precision loss.       |
| `UNSUPPORTED_OPERATION`   | Operation is not supported by the current implementation. |
| `INTERNAL_ERROR`          | Internal exception occurred.                              |

---

# 2. PoseMatrix Struct

```cpp
struct PoseMatrix {
    std::array<std::array<double, 4>, 4> data;
};
```

## Description

4×4 homogeneous transformation matrix. The matrix follows the standard rigid body transformation layout:

$$
\begin{bmatrix}
R_{3 \times 3} & t_{3 \times 1} \\
0_{1 \times 3} & 1
\end{bmatrix}
$$

Stored in row-major order.

---

# 3. PoseDistance Struct

```cpp
struct PoseDistance {
    double linear_distance;
    double angular_distance;
};
```

## Description

Distance components between two poses.

### Members

| Member             | Type     | Description                 |
| ------------------ | -------- | --------------------------- |
| `linear_distance`  | `double` | Linear distance (meters).   |
| `angular_distance` | `double` | Angular distance (radians). |

---

# 4. PoseAlgebraResult Struct

```cpp
struct PoseAlgebraResult {
    PoseAlgebraError error;
    std::string message;
};
```

## Description

Detailed operation status.

### Members

| Member    | Type               | Description                                |
| --------- | ------------------ | ------------------------------------------ |
| `error`   | `PoseAlgebraError` | Operation status code.                     |
| `message` | `std::string`      | Detailed error message or diagnostic info. |

---

# 5. PoseAlgebraBase Class

```cpp
class PoseAlgebraBase
```

## Description

Abstract base class for pose algebra plugins. All pose algebra plugins must inherit from this class. Supports operations on `PoseMatrix` and `vector6d_t` structures.

---

## Constants

```cpp
static constexpr double ZERO_TOLERANCE = 1e-6;
```

Numerical tolerance for validation and rotation checks.

---

## Constructor

```cpp
PoseAlgebraBase() = default;
```

---

## Destructor

```cpp
virtual ~PoseAlgebraBase() = default;
```

---

## Static Methods

### setSuccess

```cpp
static void setSuccess(PoseAlgebraResult& result);
```

**Description:** Marks the operation as successful and clears any error messages.

### setError

```cpp
static void setError(
    PoseAlgebraResult& result,
    PoseAlgebraError error,
    const std::string& message
);
```

**Description:** Sets the error code and detailed error message.

### clamp

```cpp
static double clamp(double value, double low, double high);
```

**Description:** Restricts a value within the `[low, high]` range.

### safeAcos

```cpp
static double safeAcos(double cos_theta);
```

**Description:** Safe arccos function. Clamps the input to `[-1.0, 1.0]` before computing arccos.

### validateVectorFinite

```cpp
static bool validateVectorFinite(
    const vector6d_t& pose,
    PoseAlgebraResult& result,
    const std::string& name
);
```

**Description:** Checks that all elements of a 6D vector are finite. Returns `false` and sets error details in `result` if NaN or Inf are detected.

### validateMatrixFinite

```cpp
static bool validateMatrixFinite(
    const PoseMatrix& pose,
    PoseAlgebraResult& result,
    const std::string& name
);
```

**Description:** Checks that all elements of a 4×4 pose matrix are finite.

### determinant3x3

```cpp
static double determinant3x3(const PoseMatrix& pose);
```

**Description:** Computes the determinant of the rotation part (top-left 3×3 submatrix) of the pose matrix.

---

## Virtual Interface

All core algebra operations output detailed error information through `result` upon failure; return `true` if successful. Some methods are overloaded to support multiple data structures (`PoseMatrix` and `vector6d_t`).

---

### inverse

```cpp
virtual bool inverse(const PoseMatrix& pose, PoseMatrix& inverse_pose, PoseAlgebraResult& result) const = 0;
virtual bool inverse(const vector6d_t& pose, vector6d_t& inverse_pose, PoseAlgebraResult& result) const = 0;
```

#### Description

Computes the inverse of a pose.

#### Parameters

* `pose`: Input pose to invert.
* `inverse_pose`: Output pose storing the inverse result.
* `result`: Detailed operation status.

---

### multiply

```cpp
virtual bool multiply(const PoseMatrix& left_pose, const PoseMatrix& right_pose, PoseMatrix& out_pose, PoseAlgebraResult& result) const = 0;
virtual bool multiply(const vector6d_t& left_pose, const vector6d_t& right_pose, vector6d_t& out_pose, PoseAlgebraResult& result) const = 0;
```

#### Description

Pose composition (multiplication):
$$
out_pose = left_pose \times right_pose
$$

#### Parameters

* `left_pose`: Left-hand pose operand.
* `right_pose`: Right-hand pose operand.
* `out_pose`: Output pose, result of composition.
* `result`: Detailed operation status.

---

### add / subtract

```cpp
virtual bool add(const PoseMatrix& left_pose, const PoseMatrix& right_pose, PoseMatrix& out_pose, PoseAlgebraResult& result) const = 0;
virtual bool add(const vector6d_t& left_pose, const vector6d_t& right_pose, vector6d_t& out_pose, PoseAlgebraResult& result) const = 0;

virtual bool subtract(const PoseMatrix& left_pose, const PoseMatrix& right_pose, PoseMatrix& out_pose, PoseAlgebraResult& result) const = 0;
virtual bool subtract(const vector6d_t& left_pose, const vector6d_t& right_pose, vector6d_t& out_pose, PoseAlgebraResult& result) const = 0;
```

#### Description

Element-wise addition or subtraction of pose components.

#### Parameters

* `left_pose`: First input operand.
* `right_pose`: Second input operand.
* `out_pose`: Resulting pose.
* `result`: Detailed operation status.

---

### vectorToMatrix / matrixToVector

```cpp
virtual bool vectorToMatrix(const vector6d_t& pose_vector, PoseMatrix& pose_matrix, PoseAlgebraResult& result) const = 0;
virtual bool matrixToVector(const PoseMatrix& pose_matrix, vector6d_t& pose_vector, PoseAlgebraResult& result) const = 0;
```

#### Description

Converts between a 6D pose vector `[x, y, z, roll, pitch, yaw]` and a 4×4 homogeneous transformation matrix. Translation corresponds to xyz (meters), rotation corresponds to rpy (radians).

#### Parameters (example for vectorToMatrix)

* `pose_vector`: Input 6D vector pose.
* `pose_matrix`: Output 4×4 pose matrix.
* `result`: Detailed operation status.

---

### distance

```cpp
virtual bool distance(const PoseMatrix& pose_a, const PoseMatrix& pose_b, PoseDistance& out_distance, PoseAlgebraResult& result) const = 0;
virtual bool distance(const vector6d_t& pose_a, const vector6d_t& pose_b, PoseDistance& out_distance, PoseAlgebraResult& result) const = 0;
```

#### Description

Computes translation and rotation distances between two poses. Translation usually uses Euclidean distance; rotation uses angular difference.

#### Parameters

* `pose_a`: Starting pose.
* `pose_b`: Ending pose.
* `out_distance`: Output translation distance (meters) and rotation angle (radians).
* `result`: Detailed operation status.

---

### worldToLocal / localToWorld

```cpp
virtual bool worldToLocal(const PoseMatrix& world_ref_pose, const PoseMatrix& world_pose, PoseMatrix& local_pose,
                          PoseAlgebraResult& result) const = 0;
virtual bool worldToLocal(const vector6d_t& world_ref_pose, const vector6d_t& world_pose, vector6d_t& local_pose, PoseAlgebraResult& result) const = 0;

virtual bool localToWorld(const PoseMatrix& world_ref_pose, const PoseMatrix& local_pose, PoseMatrix& world_pose, PoseAlgebraResult& result) const = 0;
virtual bool localToWorld(const vector6d_t& world_ref_pose, const vector6d_t& local_pose, vector6d_t& world_pose, PoseAlgebraResult& result) const = 0;
```

#### Description

Transforms between world and local coordinate frames:  

- `worldToLocal`:  
$$
local\_pose = inverse(world\_ref\_pose) \times world\_pose
$$  

- `localToWorld`:  
$$
world\_pose = world\_ref\_pose \times local\_pose
$$  

#### Parameters

- `world_ref_pose`: Local reference frame expressed in world coordinates.  
- `world_pose` / `local_pose`: Pose to convert or resulting pose.  
- `result`: Detailed operation status.  

---

## Usage Example

### Dynamically Load a Pose Algebra Plugin

```cpp
#include <Elite/ClassLoader.hpp>
#include <Elite/PoseAlgebraBase.hpp>

// Initialize plugin loader, specifying plugin library path
ELITE::ClassLoader loader("path/to/plugins");

// Load a supported pose algebra plugin (e.g., ElitePoseAlgebra)
auto pose_alg = loader.createInstance<ELITE::PoseAlgebraBase>("ELITE::ElitePoseAlgebra");

if (pose_alg) {
    ELITE::vector6d_t p1 = {0.5, 0.0, 0.5, 0, 0, 0};
    ELITE::vector6d_t p2 = {0.1, 0.0, 0.0, 0, 0, 0};
    ELITE::vector6d_t out;
    ELITE::PoseAlgebraResult result;

    if (pose_alg->multiply(p1, p2, out, result)) {
        // Output the composed pose
    } else {
        // Error handling: result.message
    }
}
```