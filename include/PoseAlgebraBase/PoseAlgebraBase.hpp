// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.

/**
 * @file PoseAlgebraBase.hpp
 * @brief Core pose algebra interface for matrix/vector operations and frame conversion.
 */
#ifndef __ELITE__POSE_ALGEBRA_BASE_HPP__
#define __ELITE__POSE_ALGEBRA_BASE_HPP__

#include <Elite/DataType.hpp>
#include <Elite/EliteOptions.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>

namespace ELITE {

/**
 * @brief 4x4 homogeneous pose matrix.
 *
 * The matrix uses the standard rigid transform layout:
 * [ R(3x3) t(3x1) ]
 * [ 0 0 0   1    ]
 */
struct PoseMatrix {
    /// Raw matrix storage in row-major indexing [row][col].
    std::array<std::array<double, 4>, 4> data{
        {{1.0, 0.0, 0.0, 0.0}, {0.0, 1.0, 0.0, 0.0}, {0.0, 0.0, 1.0, 0.0}, {0.0, 0.0, 0.0, 1.0}}};
};

/**
 * @brief Linear and angular distance components between two poses.
 */
struct PoseDistance {
    /// Translation distance in meters.
    double linear_distance{0.0};

    /// Orientation distance in radians.
    double angular_distance{0.0};
};

/**
 * @brief Error codes for pose algebra operations.
 */
enum class PoseAlgebraError {
    SUCCESS = 0,              // Operation completed successfully.
    INVALID_INPUT,            // Input is invalid, e.g. NaN/Inf or malformed pose data.
    SINGULAR_MATRIX,          // Matrix inversion failed because matrix is singular.
    INVALID_ROTATION_MATRIX,  // Rotation part is not a valid orthonormal rotation matrix.
    NUMERICAL_ERROR,          // Numeric instability, overflow/underflow, or precision breakdown occurred.
    UNSUPPORTED_OPERATION,    // Requested operation is not supported by this implementation.
    INTERNAL_ERROR,           // Unexpected internal failure.
};

/**
 * @brief Detailed status for pose algebra operations.
 */
struct PoseAlgebraResult {
    /// Operation status code.
    PoseAlgebraError error{PoseAlgebraError::SUCCESS};

    /// Human-readable details for failures or diagnostics.
    std::string message;
};

/**
 * @brief Abstract base interface for pose algebra plugins.
 *
 * Supported pose representations:
 * - PoseMatrix: 4x4 homogeneous transform
 * - vector6d_t: [x, y, z, roll, pitch, yaw]
 *
 * Concrete implementations should report detailed status through PoseAlgebraResult.
 */
class PoseAlgebraBase {
   public:
    /// Default constructor.
    PoseAlgebraBase() = default;

    /// Virtual destructor for safe polymorphic use.
    virtual ~PoseAlgebraBase() = default;

    /// Numerical tolerance used by validation and rotation checks.
    static constexpr double ZERO_TOLERANCE = 1e-6;

    /**
     * @brief Mark an operation as successful.
     *
     * @param result Result object to update.
     */
    static void setSuccess(PoseAlgebraResult& result) {
        result.error = PoseAlgebraError::SUCCESS;
        result.message.clear();
    }

    /**
     * @brief Mark an operation as failed with details.
     *
     * @param result Result object to update.
     * @param error Error code to set.
     * @param message Human-readable failure detail.
     */
    static void setError(PoseAlgebraResult& result, PoseAlgebraError error, const std::string& message) {
        result.error = error;
        result.message = message;
    }

    /**
     * @brief Clamp a value between a lower and upper bound.
     * 
     * @param value The value to be clamped.
     * @param low The lower bound.
     * @param high The upper bound.
     * @return double The clamped value.
     */
    static double clamp(double value, double low, double high) { return std::max(low, std::min(value, high)); }

    /**
     * @brief Safely compute the arc cosine of a value, clamping it to the valid range [-1, 1].
     * 
     * @param cos_theta The cosine value.
     * @return double The arc cosine of the clamped value.
     */
    static double safeAcos(double cos_theta) {
        double safe_value = clamp(cos_theta, -1.0, 1.0);
        if (safe_value > 1.0) {
            safe_value = 1.0;
        } else if (safe_value < -1.0) {
            safe_value = -1.0;
        }
        return std::acos(safe_value);
    }

    /**
     * @brief Validate that all elements of a 6D vector are finite.
     * 
     * @param pose The 6D vector to be validated.
     * @param result Detailed result of the validation, including error code and message.
     * @param name Name of the vector for error reporting.
     * @return true if all elements are finite, false otherwise.
     */
    static bool validateVectorFinite(const vector6d_t& pose, PoseAlgebraResult& result, const std::string& name) {
        for (size_t i = 0; i < pose.size(); ++i) {
            if (!std::isfinite(pose[i])) {
                setError(result, PoseAlgebraError::INVALID_INPUT,
                         name + " contains non-finite value at index " + std::to_string(i));
                return false;
            }
        }
        return true;
    }

    /**
     * @brief Validate that all elements of a pose matrix are finite.
     * 
     * @param pose The pose matrix to be validated.
     * @param result Detailed result of the validation, including error code and message.
     * @param name Name of the matrix for error reporting.
     * @return true if all elements are finite, false otherwise.
     */
    static bool validateMatrixFinite(const PoseMatrix& pose, PoseAlgebraResult& result, const std::string& name) {
        for (size_t i = 0; i < pose.data.size(); ++i) {
            for (size_t j = 0; j < pose.data[i].size(); ++j) {
                if (!std::isfinite(pose.data[i][j])) {
                    setError(result, PoseAlgebraError::INVALID_INPUT,
                             name + " contains non-finite value at [" + std::to_string(i) + "][" + std::to_string(j) + "]");
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * @brief Compute the determinant of a 3x3 pose matrix.
     *
     * @param pose The 3x3 pose matrix.
     * @return double The determinant of the matrix.
     */
    static double determinant3x3(const PoseMatrix& pose) {
        const double r00 = pose.data[0][0];
        const double r01 = pose.data[0][1];
        const double r02 = pose.data[0][2];
        const double r10 = pose.data[1][0];
        const double r11 = pose.data[1][1];
        const double r12 = pose.data[1][2];
        const double r20 = pose.data[2][0];
        const double r21 = pose.data[2][1];
        const double r22 = pose.data[2][2];

        return r00 * (r11 * r22 - r12 * r21) - r01 * (r10 * r22 - r12 * r20) + r02 * (r10 * r21 - r11 * r20);
    }

    /**
     * @brief Inverse a pose matrix.
     *
     * @param pose Input pose matrix to be inverted.
     * @param inverse_pose Output inverted pose matrix.
     * @param result Detailed result of the operation, including error code and message.
        * @return true if the operation is successful, false otherwise. In case of failure, the `result` parameter will contain the
        * error details.
     */
    ELITE_EXPORT virtual bool inverse(const PoseMatrix& pose, PoseMatrix& inverse_pose, PoseAlgebraResult& result) const = 0;

    /**
     * @brief Inverse a 6D pose vector.
     *
     * @param pose Input 6D pose vector (xyz + rpy) to be inverted.
     * @param inverse_pose Output inverted 6D pose vector.
     * @param result Detailed result of the operation, including error code and message.
     * @return true if the operation is successful, false otherwise. In case of failure, the `result` parameter will contain the
     * error details.
     */
    ELITE_EXPORT virtual bool inverse(const vector6d_t& pose, vector6d_t& inverse_pose, PoseAlgebraResult& result) const = 0;

    /**
     * @brief Compose two pose matrices: out_pose = left_pose * right_pose.
     *
     * @param left_pose The left operand pose matrix.
     * @param right_pose The right operand pose matrix.
     * @param out_pose The output pose matrix resulting from the composition.
     * @param result Detailed result of the operation, including error code and message.
     * @return true if the operation is successful, false otherwise. In case of failure, the `result` parameter will contain the
     * error details.
     */
    ELITE_EXPORT virtual bool multiply(const PoseMatrix& left_pose, const PoseMatrix& right_pose, PoseMatrix& out_pose,
                                       PoseAlgebraResult& result) const = 0;

    /**
     * @brief Compose two 6D pose vectors: out_pose = left_pose * right_pose.
     *
     * @param left_pose The left operand 6D pose vector (xyz + rpy).
     * @param right_pose The right operand 6D pose vector (xyz + rpy).
     * @param out_pose The output 6D pose vector resulting from the composition.
     * @param result Detailed result of the operation, including error code and message.
     * @return true if the operation is successful, false otherwise. In case of failure, the `result` parameter will contain the
     * error details.
     */
    ELITE_EXPORT virtual bool multiply(const vector6d_t& left_pose, const vector6d_t& right_pose, vector6d_t& out_pose,
                                       PoseAlgebraResult& result) const = 0;

    /**
     * @brief Add two pose matrices element-wise.
     *
     * @param left_pose The first operand pose matrix.
     * @param right_pose The second operand pose matrix.
     * @param out_pose The output pose matrix resulting from the element-wise addition.
     * @param result Detailed result of the operation, including error code and message.
     * @return true if the operation is successful, false otherwise. In case of failure, the `result` parameter will contain the
     * error details.
     */
    ELITE_EXPORT virtual bool add(const PoseMatrix& left_pose, const PoseMatrix& right_pose, PoseMatrix& out_pose,
                                  PoseAlgebraResult& result) const = 0;

    /**
     * @brief Add two 6D pose vectors element-wise.
     *
     * @param left_pose The first operand 6D pose vector (xyz + rpy).
     * @param right_pose The second operand 6D pose vector (xyz + rpy).
     * @param out_pose The output 6D pose vector resulting from the element-wise addition.
     * @param result Detailed result of the operation, including error code and message.
     * @return true if the operation is successful, false otherwise. In case of failure, the `result` parameter will contain the
     * error details.
     */
    ELITE_EXPORT virtual bool add(const vector6d_t& left_pose, const vector6d_t& right_pose, vector6d_t& out_pose,
                                  PoseAlgebraResult& result) const = 0;

    /**
     * @brief Subtract two pose matrices element-wise.
     *
     * @param left_pose The first operand pose matrix.
     * @param right_pose The second operand pose matrix.
     * @param out_pose The output pose matrix resulting from the element-wise subtraction.
     * @param result Detailed result of the operation, including error code and message.
     * @return true if the operation is successful, false otherwise. In case of failure, the `result` parameter will contain the
     * error details.
     */
    ELITE_EXPORT virtual bool subtract(const PoseMatrix& left_pose, const PoseMatrix& right_pose, PoseMatrix& out_pose,
                                       PoseAlgebraResult& result) const = 0;

    /**
     * @brief Subtract two 6D pose vectors element-wise.
     *
     * @param left_pose The first operand 6D pose vector (xyz + rpy).
     * @param right_pose The second operand 6D pose vector (xyz + rpy).
     * @param out_pose The output 6D pose vector resulting from the element-wise subtraction.
     * @param result Detailed result of the operation, including error code and message.
     * @return true if the operation is successful, false otherwise. In case of failure, the `result` parameter will contain the
     * error details.
     */
    ELITE_EXPORT virtual bool subtract(const vector6d_t& left_pose, const vector6d_t& right_pose, vector6d_t& out_pose,
                                       PoseAlgebraResult& result) const = 0;

    /**
     * @brief Convert a 6D pose vector (xyz + rpy) to a 4x4 pose matrix.
     *
     * @param pose_vector The input 6D pose vector, where the first three elements represent the translation (x, y, z) in meters,
     * and the last three elements represent the rotation (roll, pitch, yaw) in radians.
     * @param pose_matrix The output 4x4 pose matrix representing the same pose as the input vector. The upper-left 3x3 submatrix
     * represents the rotation, and the rightmost column represents the translation.
     * @param result Detailed result of the operation, including error code and message. If the input vector is invalid (e.g.,
     * contains NaN or Inf values), the result will indicate an error with the appropriate error code and message.
     * @return true if the operation is successful, false otherwise. In case of failure, the `result` parameter will contain the
     * error details.
     */
    ELITE_EXPORT virtual bool vectorToMatrix(const vector6d_t& pose_vector, PoseMatrix& pose_matrix,
                                             PoseAlgebraResult& result) const = 0;

    /**
     * @brief Convert a 4x4 pose matrix to a 6D pose vector (xyz + rpy).
     *
     * @param pose_matrix The input 4x4 pose matrix, where the upper-left 3x3 submatrix represents the rotation, and the rightmost
     * column represents the translation.
     * @param pose_vector The output 6D pose vector, where the first three elements represent the translation (x, y, z) in meters,
     * and the last three elements represent the rotation (roll, pitch, yaw) in radians.
     * @param result Detailed result of the operation, including error code and message. If the input matrix is invalid (e.g., not a
     * proper homogeneous transformation matrix, contains NaN or Inf values, or has an invalid rotation submatrix), the result will
     * indicate an error with the appropriate error code and message.
     * @return true if the operation is successful, false otherwise. In case of failure, the `result` parameter will contain the
     * error details.
     */
    ELITE_EXPORT virtual bool matrixToVector(const PoseMatrix& pose_matrix, vector6d_t& pose_vector,
                                             PoseAlgebraResult& result) const = 0;

    /**
     * @brief Calculate linear and angular distances between two pose matrices.
     *
     * @param pose_a The first input pose matrix.
     * @param pose_b The second input pose matrix.
     * @param out_distance The output structure containing the calculated linear and angular distances between the two poses. The
     * linear distance is typically calculated as the Euclidean distance between the translation components of the two poses, while
     * the angular distance is calculated based on the difference in orientation (e.g., using the angle of the relative rotation).
     * @param result Detailed result of the operation, including error code and message. If either input pose is invalid (e.g.,
     * contains NaN or Inf values, or has an invalid rotation submatrix), the result will indicate an error with the appropriate
     * error code and message.
     * @return true if the operation is successful, false otherwise. In case of failure, the `result` parameter will contain the
     * error details.
     */
    ELITE_EXPORT virtual bool distance(const PoseMatrix& pose_a, const PoseMatrix& pose_b, PoseDistance& out_distance,
                                       PoseAlgebraResult& result) const = 0;

    /**
     * @brief Calculate linear and angular distances between two 6D pose vectors.
     *
     * @param pose_a The first input 6D pose vector (xyz + rpy).
     * @param pose_b The second input 6D pose vector (xyz + rpy).
     * @param out_distance The output structure containing the calculated linear and angular distances between the two poses. The
     * linear distance is typically calculated as the Euclidean distance between the translation components of the two poses, while
     * the angular distance is calculated based on the difference in orientation (e.g., using the angle of the relative rotation).
     * @param result Detailed result of the operation, including error code and message. If either input pose is invalid (e.g.,
     * contains NaN or Inf values), the result will indicate an error with the appropriate error code and message.
     * @return true if the operation is successful, false otherwise. In case of failure, the `result` parameter will contain the
     * error details.
     */
    ELITE_EXPORT virtual bool distance(const vector6d_t& pose_a, const vector6d_t& pose_b, PoseDistance& out_distance,
                                       PoseAlgebraResult& result) const = 0;

    /**
     * @brief Convert a pose from world coordinates to local coordinates.
     *
     * @param world_ref_pose The local reference frame pose expressed in world coordinates.
     * @param world_pose The target pose expressed in world coordinates.
     * @param local_pose The output target pose expressed in the local reference frame.
     * @param result Detailed result of the operation, including error code and message.
     * @return true if the operation is successful, false otherwise. In case of failure, the `result` parameter will contain the
     * error details.
     */
    ELITE_EXPORT virtual bool worldToLocal(const PoseMatrix& world_ref_pose, const PoseMatrix& world_pose,
                                           PoseMatrix& local_pose, PoseAlgebraResult& result) const = 0;

    /**
     * @brief Convert a 6D pose from world coordinates to local coordinates.
     *
     * @param world_ref_pose The local reference frame pose (xyz + rpy) expressed in world coordinates.
     * @param world_pose The target pose (xyz + rpy) expressed in world coordinates.
     * @param local_pose The output target pose (xyz + rpy) expressed in the local reference frame.
     * @param result Detailed result of the operation, including error code and message.
     * @return true if the operation is successful, false otherwise. In case of failure, the `result` parameter will contain the
     * error details.
     */
    ELITE_EXPORT virtual bool worldToLocal(const vector6d_t& world_ref_pose, const vector6d_t& world_pose,
                                           vector6d_t& local_pose, PoseAlgebraResult& result) const = 0;

    /**
     * @brief Convert a pose from local coordinates to world coordinates.
     *
     * @param world_ref_pose The local reference frame pose expressed in world coordinates.
     * @param local_pose The target pose expressed in the local reference frame.
     * @param world_pose The output target pose expressed in world coordinates.
     * @param result Detailed result of the operation, including error code and message.
     * @return true if the operation is successful, false otherwise. In case of failure, the `result` parameter will contain the
     * error details.
     */
    ELITE_EXPORT virtual bool localToWorld(const PoseMatrix& world_ref_pose, const PoseMatrix& local_pose,
                                           PoseMatrix& world_pose, PoseAlgebraResult& result) const = 0;

    /**
     * @brief Convert a 6D pose from local coordinates to world coordinates.
     *
     * @param world_ref_pose The local reference frame pose (xyz + rpy) expressed in world coordinates.
     * @param local_pose The target pose (xyz + rpy) expressed in the local reference frame.
     * @param world_pose The output target pose (xyz + rpy) expressed in world coordinates.
     * @param result Detailed result of the operation, including error code and message.
     * @return true if the operation is successful, false otherwise. In case of failure, the `result` parameter will contain the
     * error details.
     */
    ELITE_EXPORT virtual bool localToWorld(const vector6d_t& world_ref_pose, const vector6d_t& local_pose,
                                           vector6d_t& world_pose, PoseAlgebraResult& result) const = 0;
};

using PoseAlgebraBaseSharedPtr = std::shared_ptr<PoseAlgebraBase>;

}  // namespace ELITE

#endif  // __ELITE__POSE_ALGEBRA_BASE_HPP__