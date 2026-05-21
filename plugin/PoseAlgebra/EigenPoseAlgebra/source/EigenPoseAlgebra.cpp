#include "EigenPoseAlgebra.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <string>

namespace {

Eigen::Matrix4d toEigenMatrix(const ELITE::PoseMatrix& pose) {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            matrix(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) = pose.data[i][j];
        }
    }
    return matrix;
}

ELITE::PoseMatrix toPoseMatrix(const Eigen::Matrix4d& matrix) {
    ELITE::PoseMatrix pose;
    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            pose.data[i][j] = matrix(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j));
        }
    }
    return pose;
}

bool validateRotation(const Eigen::Matrix3d& rotation, ELITE::PoseAlgebraResult& result, const std::string& name) {
    const Eigen::Matrix3d should_be_identity = rotation.transpose() * rotation;
    const double orthonormal_error = (should_be_identity - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff();
    if (orthonormal_error > ELITE::PoseAlgebraBase::ZERO_TOLERANCE) {
        ELITE::PoseAlgebraBase::setError(result, ELITE::PoseAlgebraError::INVALID_ROTATION_MATRIX,
                                         name + " contains a non-orthonormal rotation matrix");
        return false;
    }

    const double determinant = rotation.determinant();
    if (std::fabs(determinant) < ELITE::PoseAlgebraBase::ZERO_TOLERANCE) {
        ELITE::PoseAlgebraBase::setError(result, ELITE::PoseAlgebraError::SINGULAR_MATRIX,
                                         name + " rotation matrix is singular and cannot be inverted");
        return false;
    }

    if (std::fabs(determinant - 1.0) > ELITE::PoseAlgebraBase::ZERO_TOLERANCE) {
        ELITE::PoseAlgebraBase::setError(result, ELITE::PoseAlgebraError::INVALID_ROTATION_MATRIX,
                                         name + " rotation determinant is not close to +1");
        return false;
    }

    return true;
}

bool validatePoseMatrix(const ELITE::PoseMatrix& pose, ELITE::PoseAlgebraResult& result, const std::string& name) {
    if (!ELITE::PoseAlgebraBase::validateMatrixFinite(pose, result, name)) {
        return false;
    }

    const Eigen::Matrix4d matrix = toEigenMatrix(pose);
    if (std::fabs(matrix(3, 0)) > ELITE::PoseAlgebraBase::ZERO_TOLERANCE || std::fabs(matrix(3, 1)) > ELITE::PoseAlgebraBase::ZERO_TOLERANCE ||
        std::fabs(matrix(3, 2)) > ELITE::PoseAlgebraBase::ZERO_TOLERANCE || std::fabs(matrix(3, 3) - 1.0) > ELITE::PoseAlgebraBase::ZERO_TOLERANCE) {
        ELITE::PoseAlgebraBase::setError(result, ELITE::PoseAlgebraError::INVALID_INPUT, name + " is not a valid homogeneous pose matrix");
        return false;
    }

    return validateRotation(matrix.block<3, 3>(0, 0), result, name);
}

bool vectorToIsometry(const ELITE::vector6d_t& pose_vector, Eigen::Isometry3d& pose_iso, ELITE::PoseAlgebraResult& result,
                      const std::string& name) {
    if (!ELITE::PoseAlgebraBase::validateVectorFinite(pose_vector, result, name)) {
        return false;
    }

    const Eigen::AngleAxisd rot_z(pose_vector[5], Eigen::Vector3d::UnitZ());
    const Eigen::AngleAxisd rot_y(pose_vector[4], Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd rot_x(pose_vector[3], Eigen::Vector3d::UnitX());

    pose_iso = Eigen::Isometry3d::Identity();
    pose_iso.linear() = (rot_z * rot_y * rot_x).toRotationMatrix();
    pose_iso.translation() = Eigen::Vector3d(pose_vector[0], pose_vector[1], pose_vector[2]);

    if (!std::isfinite(pose_iso.matrix().sum())) {
        ELITE::PoseAlgebraBase::setError(result, ELITE::PoseAlgebraError::NUMERICAL_ERROR,
                                         name + " produced non-finite values when converting to matrix");
        return false;
    }

    return true;
}

ELITE::vector6d_t matrixToVectorUnchecked(const Eigen::Matrix4d& matrix) {
    ELITE::vector6d_t pose_vector{};
    pose_vector[0] = matrix(0, 3);
    pose_vector[1] = matrix(1, 3);
    pose_vector[2] = matrix(2, 3);

    const Eigen::Vector3d zyx = matrix.block<3, 3>(0, 0).eulerAngles(2, 1, 0);
    pose_vector[3] = zyx[2];
    pose_vector[4] = zyx[1];
    pose_vector[5] = zyx[0];

    return pose_vector;
}

bool computeDistance(const Eigen::Isometry3d& pose_a, const Eigen::Isometry3d& pose_b, ELITE::PoseDistance& out_distance) {
    out_distance.linear_distance = (pose_a.translation() - pose_b.translation()).norm();

    const Eigen::Matrix3d relative_rotation = pose_a.linear().transpose() * pose_b.linear();
    const double trace = relative_rotation.trace();
    const double cos_theta = ELITE::PoseAlgebraBase::clamp((trace - 1.0) * 0.5, -1.0, 1.0);
    out_distance.angular_distance = ELITE::PoseAlgebraBase::safeAcos(cos_theta);

    return std::isfinite(out_distance.linear_distance) && std::isfinite(out_distance.angular_distance);
}

}  // namespace

namespace ELITE {

bool EigenPoseAlgebra::inverse(const PoseMatrix& pose, PoseMatrix& inverse_pose, PoseAlgebraResult& result) const {
    if (!validatePoseMatrix(pose, result, "pose")) {
        return false;
    }

    const Eigen::Matrix4d matrix = toEigenMatrix(pose);
    Eigen::Isometry3d pose_iso = Eigen::Isometry3d::Identity();
    pose_iso.linear() = matrix.block<3, 3>(0, 0);
    pose_iso.translation() = matrix.block<3, 1>(0, 3);

    const Eigen::Isometry3d inverse_iso = pose_iso.inverse();
    inverse_pose = toPoseMatrix(inverse_iso.matrix());

    if (!validatePoseMatrix(inverse_pose, result, "inverse pose")) {
        return false;
    }

    PoseAlgebraBase::setSuccess(result);
    return true;
}

bool EigenPoseAlgebra::inverse(const vector6d_t& pose, vector6d_t& inverse_pose, PoseAlgebraResult& result) const {
    Eigen::Isometry3d pose_iso = Eigen::Isometry3d::Identity();
    if (!vectorToIsometry(pose, pose_iso, result, "pose")) {
        return false;
    }

    inverse_pose = matrixToVectorUnchecked(pose_iso.inverse().matrix());
    if (!PoseAlgebraBase::validateVectorFinite(inverse_pose, result, "inverse pose")) {
        PoseAlgebraBase::setError(result, PoseAlgebraError::NUMERICAL_ERROR, "inverse pose vector contains non-finite values");
        return false;
    }

    PoseAlgebraBase::setSuccess(result);
    return true;
}

bool EigenPoseAlgebra::multiply(const PoseMatrix& left_pose, const PoseMatrix& right_pose, PoseMatrix& out_pose,
                                PoseAlgebraResult& result) const {
    if (!validatePoseMatrix(left_pose, result, "left pose")) {
        return false;
    }
    if (!validatePoseMatrix(right_pose, result, "right pose")) {
        return false;
    }

    const Eigen::Matrix4d out_matrix = toEigenMatrix(left_pose) * toEigenMatrix(right_pose);
    out_pose = toPoseMatrix(out_matrix);

    if (!validatePoseMatrix(out_pose, result, "output pose")) {
        return false;
    }

    PoseAlgebraBase::setSuccess(result);
    return true;
}

bool EigenPoseAlgebra::multiply(const vector6d_t& left_pose, const vector6d_t& right_pose, vector6d_t& out_pose,
                                PoseAlgebraResult& result) const {
    Eigen::Isometry3d left_iso = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d right_iso = Eigen::Isometry3d::Identity();
    if (!vectorToIsometry(left_pose, left_iso, result, "left pose")) {
        return false;
    }
    if (!vectorToIsometry(right_pose, right_iso, result, "right pose")) {
        return false;
    }

    out_pose = matrixToVectorUnchecked((left_iso * right_iso).matrix());
    if (!PoseAlgebraBase::validateVectorFinite(out_pose, result, "output pose")) {
        PoseAlgebraBase::setError(result, PoseAlgebraError::NUMERICAL_ERROR, "output pose vector contains non-finite values");
        return false;
    }

    PoseAlgebraBase::setSuccess(result);
    return true;
}

bool EigenPoseAlgebra::add(const PoseMatrix& left_pose, const PoseMatrix& right_pose, PoseMatrix& out_pose,
                           PoseAlgebraResult& result) const {
    if (!PoseAlgebraBase::validateMatrixFinite(left_pose, result, "left pose")) {
        return false;
    }
    if (!PoseAlgebraBase::validateMatrixFinite(right_pose, result, "right pose")) {
        return false;
    }

    const Eigen::Matrix4d out_matrix = toEigenMatrix(left_pose) + toEigenMatrix(right_pose);
    out_pose = toPoseMatrix(out_matrix);

    if (!PoseAlgebraBase::validateMatrixFinite(out_pose, result, "output pose")) {
        PoseAlgebraBase::setError(result, PoseAlgebraError::NUMERICAL_ERROR, "output pose matrix contains non-finite values");
        return false;
    }

    PoseAlgebraBase::setSuccess(result);
    return true;
}

bool EigenPoseAlgebra::add(const vector6d_t& left_pose, const vector6d_t& right_pose, vector6d_t& out_pose,
                           PoseAlgebraResult& result) const {
    if (!PoseAlgebraBase::validateVectorFinite(left_pose, result, "left pose")) {
        return false;
    }
    if (!PoseAlgebraBase::validateVectorFinite(right_pose, result, "right pose")) {
        return false;
    }

    for (size_t i = 0; i < out_pose.size(); ++i) {
        out_pose[i] = left_pose[i] + right_pose[i];
    }

    if (!PoseAlgebraBase::validateVectorFinite(out_pose, result, "output pose")) {
        PoseAlgebraBase::setError(result, PoseAlgebraError::NUMERICAL_ERROR, "output pose vector contains non-finite values");
        return false;
    }

    PoseAlgebraBase::setSuccess(result);
    return true;
}

bool EigenPoseAlgebra::subtract(const PoseMatrix& left_pose, const PoseMatrix& right_pose, PoseMatrix& out_pose,
                                PoseAlgebraResult& result) const {
    if (!PoseAlgebraBase::validateMatrixFinite(left_pose, result, "left pose")) {
        return false;
    }
    if (!PoseAlgebraBase::validateMatrixFinite(right_pose, result, "right pose")) {
        return false;
    }

    const Eigen::Matrix4d out_matrix = toEigenMatrix(left_pose) - toEigenMatrix(right_pose);
    out_pose = toPoseMatrix(out_matrix);

    if (!PoseAlgebraBase::validateMatrixFinite(out_pose, result, "output pose")) {
        PoseAlgebraBase::setError(result, PoseAlgebraError::NUMERICAL_ERROR, "output pose matrix contains non-finite values");
        return false;
    }

    PoseAlgebraBase::setSuccess(result);
    return true;
}

bool EigenPoseAlgebra::subtract(const vector6d_t& left_pose, const vector6d_t& right_pose, vector6d_t& out_pose,
                                PoseAlgebraResult& result) const {
    if (!PoseAlgebraBase::validateVectorFinite(left_pose, result, "left pose")) {
        return false;
    }
    if (!PoseAlgebraBase::validateVectorFinite(right_pose, result, "right pose")) {
        return false;
    }

    for (size_t i = 0; i < out_pose.size(); ++i) {
        out_pose[i] = left_pose[i] - right_pose[i];
    }

    if (!PoseAlgebraBase::validateVectorFinite(out_pose, result, "output pose")) {
        PoseAlgebraBase::setError(result, PoseAlgebraError::NUMERICAL_ERROR, "output pose vector contains non-finite values");
        return false;
    }

    PoseAlgebraBase::setSuccess(result);
    return true;
}

bool EigenPoseAlgebra::vectorToMatrix(const vector6d_t& pose_vector, PoseMatrix& pose_matrix,
                                      PoseAlgebraResult& result) const {
    Eigen::Isometry3d pose_iso = Eigen::Isometry3d::Identity();
    if (!vectorToIsometry(pose_vector, pose_iso, result, "pose vector")) {
        return false;
    }

    pose_matrix = toPoseMatrix(pose_iso.matrix());
    if (!validatePoseMatrix(pose_matrix, result, "pose matrix")) {
        PoseAlgebraBase::setError(result, PoseAlgebraError::NUMERICAL_ERROR, "failed to construct a valid pose matrix from pose vector");
        return false;
    }

    PoseAlgebraBase::setSuccess(result);
    return true;
}

bool EigenPoseAlgebra::matrixToVector(const PoseMatrix& pose_matrix, vector6d_t& pose_vector,
                                      PoseAlgebraResult& result) const {
    if (!validatePoseMatrix(pose_matrix, result, "pose matrix")) {
        return false;
    }

    pose_vector = matrixToVectorUnchecked(toEigenMatrix(pose_matrix));
    if (!PoseAlgebraBase::validateVectorFinite(pose_vector, result, "pose vector")) {
        PoseAlgebraBase::setError(result, PoseAlgebraError::NUMERICAL_ERROR, "failed to extract a valid pose vector from pose matrix");
        return false;
    }

    PoseAlgebraBase::setSuccess(result);
    return true;
}

bool EigenPoseAlgebra::distance(const PoseMatrix& pose_a, const PoseMatrix& pose_b, PoseDistance& out_distance,
                                PoseAlgebraResult& result) const {
    if (!validatePoseMatrix(pose_a, result, "pose_a")) {
        return false;
    }
    if (!validatePoseMatrix(pose_b, result, "pose_b")) {
        return false;
    }

    Eigen::Isometry3d pose_a_iso = Eigen::Isometry3d::Identity();
    pose_a_iso.linear() = toEigenMatrix(pose_a).block<3, 3>(0, 0);
    pose_a_iso.translation() = toEigenMatrix(pose_a).block<3, 1>(0, 3);

    Eigen::Isometry3d pose_b_iso = Eigen::Isometry3d::Identity();
    pose_b_iso.linear() = toEigenMatrix(pose_b).block<3, 3>(0, 0);
    pose_b_iso.translation() = toEigenMatrix(pose_b).block<3, 1>(0, 3);

    if (!computeDistance(pose_a_iso, pose_b_iso, out_distance)) {
        PoseAlgebraBase::setError(result, PoseAlgebraError::NUMERICAL_ERROR, "failed to compute pose distance due to numerical error");
        return false;
    }

    PoseAlgebraBase::setSuccess(result);
    return true;
}

bool EigenPoseAlgebra::distance(const vector6d_t& pose_a, const vector6d_t& pose_b, PoseDistance& out_distance,
                                PoseAlgebraResult& result) const {
    Eigen::Isometry3d pose_a_iso = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d pose_b_iso = Eigen::Isometry3d::Identity();

    if (!vectorToIsometry(pose_a, pose_a_iso, result, "pose_a")) {
        return false;
    }
    if (!vectorToIsometry(pose_b, pose_b_iso, result, "pose_b")) {
        return false;
    }

    if (!computeDistance(pose_a_iso, pose_b_iso, out_distance)) {
        PoseAlgebraBase::setError(result, PoseAlgebraError::NUMERICAL_ERROR, "failed to compute pose distance due to numerical error");
        return false;
    }

    PoseAlgebraBase::setSuccess(result);
    return true;
}

bool EigenPoseAlgebra::worldToLocal(const PoseMatrix& world_ref_pose, const PoseMatrix& world_pose, PoseMatrix& local_pose,
                                    PoseAlgebraResult& result) const {
    PoseMatrix inverse_world_ref_pose;
    if (!inverse(world_ref_pose, inverse_world_ref_pose, result)) {
        return false;
    }

    return multiply(inverse_world_ref_pose, world_pose, local_pose, result);
}

bool EigenPoseAlgebra::worldToLocal(const vector6d_t& world_ref_pose, const vector6d_t& world_pose, vector6d_t& local_pose,
                                    PoseAlgebraResult& result) const {
    PoseMatrix world_ref_pose_matrix;
    PoseMatrix world_pose_matrix;
    PoseMatrix local_pose_matrix;

    if (!vectorToMatrix(world_ref_pose, world_ref_pose_matrix, result)) {
        return false;
    }
    if (!vectorToMatrix(world_pose, world_pose_matrix, result)) {
        return false;
    }
    if (!worldToLocal(world_ref_pose_matrix, world_pose_matrix, local_pose_matrix, result)) {
        return false;
    }

    return matrixToVector(local_pose_matrix, local_pose, result);
}

bool EigenPoseAlgebra::localToWorld(const PoseMatrix& world_ref_pose, const PoseMatrix& local_pose, PoseMatrix& world_pose,
                                    PoseAlgebraResult& result) const {
    return multiply(world_ref_pose, local_pose, world_pose, result);
}

bool EigenPoseAlgebra::localToWorld(const vector6d_t& world_ref_pose, const vector6d_t& local_pose, vector6d_t& world_pose,
                                    PoseAlgebraResult& result) const {
    PoseMatrix world_ref_pose_matrix;
    PoseMatrix local_pose_matrix;
    PoseMatrix world_pose_matrix;

    if (!vectorToMatrix(world_ref_pose, world_ref_pose_matrix, result)) {
        return false;
    }
    if (!vectorToMatrix(local_pose, local_pose_matrix, result)) {
        return false;
    }
    if (!localToWorld(world_ref_pose_matrix, local_pose_matrix, world_pose_matrix, result)) {
        return false;
    }

    return matrixToVector(world_pose_matrix, world_pose, result);
}

}  // namespace ELITE

#include <Elite/ClassRegisterMacro.hpp>
ELITE_CLASS_LOADER_REGISTER_CLASS(ELITE::EigenPoseAlgebra, ELITE::PoseAlgebraBase);
