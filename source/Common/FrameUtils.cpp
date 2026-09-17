// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
#include "FrameUtils.hpp"

#include <algorithm>
#include <cmath>

namespace ELITE {
namespace {

struct Matrix4 {
    double v[4][4];
};

Matrix4 poseToMatrix(const vector6d_t& pose) {
    const double cx = std::cos(pose[3]);
    const double sx = std::sin(pose[3]);
    const double cy = std::cos(pose[4]);
    const double sy = std::sin(pose[4]);
    const double cz = std::cos(pose[5]);
    const double sz = std::sin(pose[5]);
    return Matrix4{{{cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx, pose[0]},
                    {sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx, pose[1]},
                    {-sy, cy * sx, cy * cx, pose[2]},
                    {0, 0, 0, 1}}};
}

Matrix4 multiplyMatrix(const Matrix4& left, const Matrix4& right) {
    Matrix4 result{};
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            for (int k = 0; k < 4; ++k) {
                result.v[row][col] += left.v[row][k] * right.v[k][col];
            }
        }
    }
    return result;
}

Matrix4 inversePoseMatrix(const Matrix4& pose) {
    Matrix4 result{{{pose.v[0][0], pose.v[1][0], pose.v[2][0], 0},
                    {pose.v[0][1], pose.v[1][1], pose.v[2][1], 0},
                    {pose.v[0][2], pose.v[1][2], pose.v[2][2], 0},
                    {0, 0, 0, 1}}};
    for (int row = 0; row < 3; ++row) {
        result.v[row][3] = -(result.v[row][0] * pose.v[0][3] + result.v[row][1] * pose.v[1][3] +
                             result.v[row][2] * pose.v[2][3]);
    }
    return result;
}

vector6d_t matrixToPose(const Matrix4& matrix) {
    const double pitch = std::asin(std::max(-1.0, std::min(1.0, -matrix.v[2][0])));
    const double cos_pitch = std::cos(pitch);
    double roll = 0.0;
    double yaw = 0.0;
    if (std::fabs(cos_pitch) > 1e-9) {
        roll = std::atan2(matrix.v[2][1], matrix.v[2][2]);
        yaw = std::atan2(matrix.v[1][0], matrix.v[0][0]);
    } else {
        yaw = std::atan2(-matrix.v[0][1], matrix.v[1][1]);
    }
    return vector6d_t{{matrix.v[0][3], matrix.v[1][3], matrix.v[2][3], roll, pitch, yaw}};
}

}  // namespace

vector6d_t poseToBaseFrame(const vector6d_t& pose, const vector6d_t& frame_pose) {
    return matrixToPose(multiplyMatrix(poseToMatrix(frame_pose), poseToMatrix(pose)));
}

vector6d_t poseFromBaseFrame(const vector6d_t& base_pose, const vector6d_t& frame_pose) {
    return matrixToPose(multiplyMatrix(inversePoseMatrix(poseToMatrix(frame_pose)), poseToMatrix(base_pose)));
}

vector6d_t rotateVector(const vector6d_t& frame_pose, const vector6d_t& vector) {
    const double rx = frame_pose[3];
    const double ry = frame_pose[4];
    const double rz = frame_pose[5];
    const double cx = std::cos(rx);
    const double sx = std::sin(rx);
    const double cy = std::cos(ry);
    const double sy = std::sin(ry);
    const double cz = std::cos(rz);
    const double sz = std::sin(rz);

    const double r00 = cz * cy;
    const double r01 = cz * sy * sx - sz * cx;
    const double r02 = cz * sy * cx + sz * sx;
    const double r10 = sz * cy;
    const double r11 = sz * sy * sx + cz * cx;
    const double r12 = sz * sy * cx - cz * sx;
    const double r20 = -sy;
    const double r21 = cy * sx;
    const double r22 = cy * cx;

    return vector6d_t{{r00 * vector[0] + r01 * vector[1] + r02 * vector[2],
                       r10 * vector[0] + r11 * vector[1] + r12 * vector[2],
                       r20 * vector[0] + r21 * vector[1] + r22 * vector[2],
                       r00 * vector[3] + r01 * vector[4] + r02 * vector[5],
                       r10 * vector[3] + r11 * vector[4] + r12 * vector[5],
                       r20 * vector[3] + r21 * vector[4] + r22 * vector[5]}};
}

}  // namespace ELITE
