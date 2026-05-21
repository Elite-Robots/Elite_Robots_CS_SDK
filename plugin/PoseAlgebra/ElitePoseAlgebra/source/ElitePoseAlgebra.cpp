#include "ElitePoseAlgebra.hpp"

#include <algorithm>
#include <cmath>

namespace {

bool validatePoseMatrix(const ELITE::PoseMatrix& pose, ELITE::PoseAlgebraResult& result, const std::string& name) {
    if (!ELITE::PoseAlgebraBase::validateMatrixFinite(pose, result, name)) {
        return false;
    }

    if (std::fabs(pose.data[3][0]) > ELITE::PoseAlgebraBase::ZERO_TOLERANCE || std::fabs(pose.data[3][1]) > ELITE::PoseAlgebraBase::ZERO_TOLERANCE ||
        std::fabs(pose.data[3][2]) > ELITE::PoseAlgebraBase::ZERO_TOLERANCE || std::fabs(pose.data[3][3] - 1.0) > ELITE::PoseAlgebraBase::ZERO_TOLERANCE) {
        ELITE::PoseAlgebraBase::setError(result, ELITE::PoseAlgebraError::INVALID_INPUT, name + " is not a valid homogeneous pose matrix");
        return false;
    }

    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            double dot = 0.0;
            for (size_t k = 0; k < 3; ++k) {
                dot += pose.data[k][i] * pose.data[k][j];
            }

            const double expected = (i == j) ? 1.0 : 0.0;
            if (std::fabs(dot - expected) > ELITE::PoseAlgebraBase::ZERO_TOLERANCE) {
                ELITE::PoseAlgebraBase::setError(result, ELITE::PoseAlgebraError::INVALID_ROTATION_MATRIX,
                                                 name + " contains a non-orthonormal rotation matrix");
                return false;
            }
        }
    }

    const double det = ELITE::PoseAlgebraBase::determinant3x3(pose);
    if (std::fabs(det) < ELITE::PoseAlgebraBase::ZERO_TOLERANCE) {
        ELITE::PoseAlgebraBase::setError(result, ELITE::PoseAlgebraError::SINGULAR_MATRIX,
                                         name + " rotation matrix is singular and cannot be inverted");
        return false;
    }

    if (std::fabs(det - 1.0) > ELITE::PoseAlgebraBase::ZERO_TOLERANCE) {
        ELITE::PoseAlgebraBase::setError(result, ELITE::PoseAlgebraError::INVALID_ROTATION_MATRIX,
                                         name + " rotation determinant is not close to +1");
        return false;
    }

    return true;
}

void fillRotationFromRpy(double roll, double pitch, double yaw, ELITE::PoseMatrix& pose_matrix) {
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);

    pose_matrix.data[0][0] = cy * cp;
    pose_matrix.data[0][1] = cy * sp * sr - sy * cr;
    pose_matrix.data[0][2] = cy * sp * cr + sy * sr;

    pose_matrix.data[1][0] = sy * cp;
    pose_matrix.data[1][1] = sy * sp * sr + cy * cr;
    pose_matrix.data[1][2] = sy * sp * cr - cy * sr;

    pose_matrix.data[2][0] = -sp;
    pose_matrix.data[2][1] = cp * sr;
    pose_matrix.data[2][2] = cp * cr;
}

ELITE::PoseMatrix toPoseMatrixUnchecked(const ELITE::vector6d_t& pose_vector) {
    ELITE::PoseMatrix pose_matrix;
    fillRotationFromRpy(pose_vector[3], pose_vector[4], pose_vector[5], pose_matrix);

    pose_matrix.data[0][3] = pose_vector[0];
    pose_matrix.data[1][3] = pose_vector[1];
    pose_matrix.data[2][3] = pose_vector[2];

    pose_matrix.data[3][0] = 0.0;
    pose_matrix.data[3][1] = 0.0;
    pose_matrix.data[3][2] = 0.0;
    pose_matrix.data[3][3] = 1.0;

    return pose_matrix;
}

ELITE::vector6d_t toPoseVectorUnchecked(const ELITE::PoseMatrix& pose_matrix) {
    ELITE::vector6d_t pose_vector{};

    pose_vector[0] = pose_matrix.data[0][3];
    pose_vector[1] = pose_matrix.data[1][3];
    pose_vector[2] = pose_matrix.data[2][3];

    const double nx = pose_matrix.data[0][0];
    const double ny = pose_matrix.data[1][0];
    const double nz = pose_matrix.data[2][0];
    const double ox = pose_matrix.data[0][1];
    const double oy = pose_matrix.data[1][1];
    const double oz = pose_matrix.data[2][1];
    const double az = pose_matrix.data[2][2];

    const double pitch = std::atan2(-nz, std::sqrt(nx * nx + ny * ny));
    const double cp = std::cos(pitch);

    double roll = 0.0;
    double yaw = 0.0;

    if (std::fabs(cp) > ELITE::PoseAlgebraBase::ZERO_TOLERANCE) {
        yaw = std::atan2(ny, nx);
        roll = std::atan2(oz, az);
    } else {
        yaw = 0.0;
        roll = (pitch > 0.0) ? std::atan2(ox, oy) : -std::atan2(ox, oy);
    }

    pose_vector[3] = roll;
    pose_vector[4] = pitch;
    pose_vector[5] = yaw;

    return pose_vector;
}

ELITE::PoseMatrix multiplyMatrixUnchecked(const ELITE::PoseMatrix& left_pose, const ELITE::PoseMatrix& right_pose) {
    ELITE::PoseMatrix out_pose;
    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            double value = 0.0;
            for (size_t k = 0; k < 4; ++k) {
                value += left_pose.data[i][k] * right_pose.data[k][j];
            }
            out_pose.data[i][j] = value;
        }
    }
    return out_pose;
}

ELITE::PoseMatrix inverseMatrixUnchecked(const ELITE::PoseMatrix& pose) {
    ELITE::PoseMatrix inverse_pose;

    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            inverse_pose.data[i][j] = pose.data[j][i];
        }
    }

    const double tx = pose.data[0][3];
    const double ty = pose.data[1][3];
    const double tz = pose.data[2][3];

    inverse_pose.data[0][3] = -(inverse_pose.data[0][0] * tx + inverse_pose.data[0][1] * ty + inverse_pose.data[0][2] * tz);
    inverse_pose.data[1][3] = -(inverse_pose.data[1][0] * tx + inverse_pose.data[1][1] * ty + inverse_pose.data[1][2] * tz);
    inverse_pose.data[2][3] = -(inverse_pose.data[2][0] * tx + inverse_pose.data[2][1] * ty + inverse_pose.data[2][2] * tz);

    inverse_pose.data[3][0] = 0.0;
    inverse_pose.data[3][1] = 0.0;
    inverse_pose.data[3][2] = 0.0;
    inverse_pose.data[3][3] = 1.0;

    return inverse_pose;
}

bool computeDistanceUnchecked(const ELITE::PoseMatrix& pose_a, const ELITE::PoseMatrix& pose_b,
                            ELITE::PoseDistance& out_distance) {
    const double dx = pose_a.data[0][3] - pose_b.data[0][3];
    const double dy = pose_a.data[1][3] - pose_b.data[1][3];
    const double dz = pose_a.data[2][3] - pose_b.data[2][3];
    out_distance.linear_distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    double rel_trace = 0.0;
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            double value = 0.0;
            for (size_t k = 0; k < 3; ++k) {
                value += pose_a.data[k][i] * pose_b.data[k][j];
            }
            if (i == j) {
                rel_trace += value;
            }
        }
    }

    const double cos_theta = ELITE::PoseAlgebraBase::clamp((rel_trace - 1.0) * 0.5, -1.0, 1.0);
    out_distance.angular_distance = ELITE::PoseAlgebraBase::safeAcos(cos_theta);

    return std::isfinite(out_distance.linear_distance) && std::isfinite(out_distance.angular_distance);
}

}  // namespace

namespace ELITE {

bool ElitePoseAlgebra::inverse(const PoseMatrix& pose, PoseMatrix& inverse_pose, PoseAlgebraResult& result) const {
    if (!validatePoseMatrix(pose, result, "pose")) {
        return false;
    }

    inverse_pose = inverseMatrixUnchecked(pose);
    if (!validatePoseMatrix(inverse_pose, result, "inverse pose")) {
        return false;
    }

    PoseAlgebraBase::setSuccess(result);
    return true;
}

bool ElitePoseAlgebra::inverse(const vector6d_t& pose, vector6d_t& inverse_pose, PoseAlgebraResult& result) const {
    if (!PoseAlgebraBase::validateVectorFinite(pose, result, "pose")) {
        return false;
    }

    const PoseMatrix pose_matrix = toPoseMatrixUnchecked(pose);
    const PoseMatrix inverse_pose_matrix = inverseMatrixUnchecked(pose_matrix);
    inverse_pose = toPoseVectorUnchecked(inverse_pose_matrix);

    if (!PoseAlgebraBase::validateVectorFinite(inverse_pose, result, "inverse pose")) {
        PoseAlgebraBase::setError(result, PoseAlgebraError::NUMERICAL_ERROR, "inverse pose vector contains non-finite values");
        return false;
    }

    PoseAlgebraBase::setSuccess(result);
    return true;
}

bool ElitePoseAlgebra::multiply(const PoseMatrix& left_pose, const PoseMatrix& right_pose, PoseMatrix& out_pose,
                             PoseAlgebraResult& result) const {
    if (!validatePoseMatrix(left_pose, result, "left pose")) {
        return false;
    }
    if (!validatePoseMatrix(right_pose, result, "right pose")) {
        return false;
    }

    out_pose = multiplyMatrixUnchecked(left_pose, right_pose);
    if (!validatePoseMatrix(out_pose, result, "output pose")) {
        return false;
    }

    PoseAlgebraBase::setSuccess(result);
    return true;
}

bool ElitePoseAlgebra::multiply(const vector6d_t& left_pose, const vector6d_t& right_pose, vector6d_t& out_pose,
                             PoseAlgebraResult& result) const {
    if (!PoseAlgebraBase::validateVectorFinite(left_pose, result, "left pose")) {
        return false;
    }
    if (!PoseAlgebraBase::validateVectorFinite(right_pose, result, "right pose")) {
        return false;
    }

    const PoseMatrix left_pose_matrix = toPoseMatrixUnchecked(left_pose);
    const PoseMatrix right_pose_matrix = toPoseMatrixUnchecked(right_pose);
    const PoseMatrix out_pose_matrix = multiplyMatrixUnchecked(left_pose_matrix, right_pose_matrix);
    out_pose = toPoseVectorUnchecked(out_pose_matrix);

    if (!PoseAlgebraBase::validateVectorFinite(out_pose, result, "output pose")) {
        PoseAlgebraBase::setError(result, PoseAlgebraError::NUMERICAL_ERROR, "output pose vector contains non-finite values");
        return false;
    }

    PoseAlgebraBase::setSuccess(result);
    return true;
}

bool ElitePoseAlgebra::add(const PoseMatrix& left_pose, const PoseMatrix& right_pose, PoseMatrix& out_pose,
                        PoseAlgebraResult& result) const {
    if (!PoseAlgebraBase::validateMatrixFinite(left_pose, result, "left pose")) {
        return false;
    }
    if (!PoseAlgebraBase::validateMatrixFinite(right_pose, result, "right pose")) {
        return false;
    }

    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            out_pose.data[i][j] = left_pose.data[i][j] + right_pose.data[i][j];
        }
    }

    if (!PoseAlgebraBase::validateMatrixFinite(out_pose, result, "output pose")) {
        PoseAlgebraBase::setError(result, PoseAlgebraError::NUMERICAL_ERROR, "output pose matrix contains non-finite values");
        return false;
    }

    PoseAlgebraBase::setSuccess(result);
    return true;
}

bool ElitePoseAlgebra::add(const vector6d_t& left_pose, const vector6d_t& right_pose, vector6d_t& out_pose,
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

bool ElitePoseAlgebra::subtract(const PoseMatrix& left_pose, const PoseMatrix& right_pose, PoseMatrix& out_pose,
                             PoseAlgebraResult& result) const {
    if (!PoseAlgebraBase::validateMatrixFinite(left_pose, result, "left pose")) {
        return false;
    }
    if (!PoseAlgebraBase::validateMatrixFinite(right_pose, result, "right pose")) {
        return false;
    }

    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            out_pose.data[i][j] = left_pose.data[i][j] - right_pose.data[i][j];
        }
    }

    if (!PoseAlgebraBase::validateMatrixFinite(out_pose, result, "output pose")) {
        PoseAlgebraBase::setError(result, PoseAlgebraError::NUMERICAL_ERROR, "output pose matrix contains non-finite values");
        return false;
    }

    PoseAlgebraBase::setSuccess(result);
    return true;
}

bool ElitePoseAlgebra::subtract(const vector6d_t& left_pose, const vector6d_t& right_pose, vector6d_t& out_pose,
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

bool ElitePoseAlgebra::vectorToMatrix(const vector6d_t& pose_vector, PoseMatrix& pose_matrix, PoseAlgebraResult& result) const {
    if (!PoseAlgebraBase::validateVectorFinite(pose_vector, result, "pose vector")) {
        return false;
    }

    pose_matrix = toPoseMatrixUnchecked(pose_vector);

    if (!validatePoseMatrix(pose_matrix, result, "pose matrix")) {
        PoseAlgebraBase::setError(result, PoseAlgebraError::NUMERICAL_ERROR, "failed to construct a valid pose matrix from pose vector");
        return false;
    }

    PoseAlgebraBase::setSuccess(result);
    return true;
}

bool ElitePoseAlgebra::matrixToVector(const PoseMatrix& pose_matrix, vector6d_t& pose_vector, PoseAlgebraResult& result) const {
    if (!validatePoseMatrix(pose_matrix, result, "pose matrix")) {
        return false;
    }

    pose_vector = toPoseVectorUnchecked(pose_matrix);
    if (!PoseAlgebraBase::validateVectorFinite(pose_vector, result, "pose vector")) {
        PoseAlgebraBase::setError(result, PoseAlgebraError::NUMERICAL_ERROR, "failed to extract a valid pose vector from pose matrix");
        return false;
    }

    PoseAlgebraBase::setSuccess(result);
    return true;
}

bool ElitePoseAlgebra::distance(const PoseMatrix& pose_a, const PoseMatrix& pose_b, PoseDistance& out_distance,
                             PoseAlgebraResult& result) const {
    if (!validatePoseMatrix(pose_a, result, "pose_a")) {
        return false;
    }
    if (!validatePoseMatrix(pose_b, result, "pose_b")) {
        return false;
    }

    if (!computeDistanceUnchecked(pose_a, pose_b, out_distance)) {
        PoseAlgebraBase::setError(result, PoseAlgebraError::NUMERICAL_ERROR, "failed to compute pose distance due to numerical error");
        return false;
    }

    PoseAlgebraBase::setSuccess(result);
    return true;
}

bool ElitePoseAlgebra::distance(const vector6d_t& pose_a, const vector6d_t& pose_b, PoseDistance& out_distance,
                             PoseAlgebraResult& result) const {
    if (!PoseAlgebraBase::validateVectorFinite(pose_a, result, "pose_a")) {
        return false;
    }
    if (!PoseAlgebraBase::validateVectorFinite(pose_b, result, "pose_b")) {
        return false;
    }

    const PoseMatrix pose_a_matrix = toPoseMatrixUnchecked(pose_a);
    const PoseMatrix pose_b_matrix = toPoseMatrixUnchecked(pose_b);

    if (!computeDistanceUnchecked(pose_a_matrix, pose_b_matrix, out_distance)) {
        PoseAlgebraBase::setError(result, PoseAlgebraError::NUMERICAL_ERROR, "failed to compute pose distance due to numerical error");
        return false;
    }

    PoseAlgebraBase::setSuccess(result);
    return true;
}

bool ElitePoseAlgebra::worldToLocal(const PoseMatrix& world_ref_pose, const PoseMatrix& world_pose, PoseMatrix& local_pose,
                                 PoseAlgebraResult& result) const {
    PoseMatrix inverse_world_ref_pose;
    if (!inverse(world_ref_pose, inverse_world_ref_pose, result)) {
        return false;
    }

    return multiply(inverse_world_ref_pose, world_pose, local_pose, result);
}

bool ElitePoseAlgebra::worldToLocal(const vector6d_t& world_ref_pose, const vector6d_t& world_pose, vector6d_t& local_pose,
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

bool ElitePoseAlgebra::localToWorld(const PoseMatrix& world_ref_pose, const PoseMatrix& local_pose, PoseMatrix& world_pose,
                                 PoseAlgebraResult& result) const {
    return multiply(world_ref_pose, local_pose, world_pose, result);
}

bool ElitePoseAlgebra::localToWorld(const vector6d_t& world_ref_pose, const vector6d_t& local_pose, vector6d_t& world_pose,
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
ELITE_CLASS_LOADER_REGISTER_CLASS(ELITE::ElitePoseAlgebra, ELITE::PoseAlgebraBase);
