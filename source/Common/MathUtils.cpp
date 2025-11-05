#include "MathUtils.hpp"
#include "DataType.hpp"
#include <cmath>

namespace ELITE {

PoseMatrix PoseMatrix::operator*(const PoseMatrix& other) const {
    PoseMatrix result;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            result.data[i][j] = 0.0;
            for (int k = 0; k < 4; ++k) {
                result.data[i][j] += data[i][k] * other.data[k][j];
            }
        }
    }
    return result;
}

PoseMatrix& PoseMatrix::operator*=(const PoseMatrix& other) {
    *this = *this * other;
    return *this;
}

PoseMatrix PoseMatrix::inverse() const {
    PoseMatrix inv{};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            inv.data[i][j] = data[j][i];
        }
    }
    inv.data[0][3] = -data[0][3] * data[0][0] - data[1][3] * data[1][0] - data[2][3] * data[2][0];
    inv.data[1][3] = -data[0][3] * data[0][1] - data[1][3] * data[1][1] - data[2][3] * data[2][1];
    inv.data[2][3] = -data[0][3] * data[0][2] - data[1][3] * data[1][2] - data[2][3] * data[2][2];
    inv.data[3][0] = 0.0;
    inv.data[3][1] = 0.0;
    inv.data[3][2] = 0.0;
    inv.data[3][3] = 1.0;
    return inv;
}

namespace MATH_UTILS 
{


PoseMatrix vector2Matrix(const vector6d_t& vt) {
    PoseMatrix matrix{};
    double s[3] = {0};
    double c[3] = {0};
    c[0] = std::cos(vt[3]);
    c[1] = std::cos(vt[4]);
    c[2] = std::cos(vt[5]);
    s[0] = std::sin(vt[3]);
    s[1] = std::sin(vt[4]);
    s[2] = std::sin(vt[5]);

    matrix.data[0][0] = c[2] * c[1];
    matrix.data[0][1] = c[2] * s[1] * s[0] - s[2] * c[0];
    matrix.data[0][2] = c[2] * s[1] * c[0] + s[2] * s[0];
    matrix.data[0][3] = vt[0];

    matrix.data[1][0] = s[2] * c[1];
    matrix.data[1][1] = s[2] * s[1] * s[0] + c[2] * c[0];
    matrix.data[1][2] = s[2] * s[1] * c[0] - c[2] * s[0];
    matrix.data[1][3] = vt[1];

    matrix.data[2][0] = -s[1];
    matrix.data[2][1] = c[1] * s[0];
    matrix.data[2][2] = c[1] * c[0];
    matrix.data[2][3] = vt[2];

    matrix.data[3][0] = 0;
    matrix.data[3][1] = 0;
    matrix.data[3][2] = 0;
    matrix.data[3][3] = 1;
    return matrix;
}

vector6d_t matrix2Vector(const PoseMatrix& pm) {
    double nx = 0.0, ny = 0.0, nz = 0.0, ox = 0.0, oy = 0.0, oz = 0.0, az = 0.0, px = 0.0, py = 0.0, pz = 0.0;
    double r_y = 0.0;
    double r_x = 0.0;
    double r_z = 0.0;

    nx = pm.data[0][0];
    ny = pm.data[1][0];
    nz = pm.data[2][0];

    ox = pm.data[0][1];
    oy = pm.data[1][1];
    oz = pm.data[2][1];

    az = pm.data[2][2];

    px = pm.data[0][3];
    py = pm.data[1][3];
    pz = pm.data[2][3];
    r_y = atan2(-nz, sqrt(nx * nx + ny * ny));  // beta
    if (isZero(cos(r_y)) == 0) {
        r_z = atan2(ny, nx);  // alfa
        r_x = atan2(oz, az);  // gama
    } else {
        if (r_y > 0) {
            r_z = 0;
            r_x = atan2(ox, oy);
        } else {
            r_z = 0;
            r_x = -atan2(ox, oy);
        }
    }
    vector6d_t rpy{};
    rpy[0] = px;
    rpy[1] = py;
    rpy[2] = pz;
    rpy[3] = r_x;
    rpy[4] = r_y;
    rpy[5] = r_z;
    return rpy;
}


double poseDistance(const vector6d_t& pose1, const vector6d_t& pose2) {
    double dx = pose1[0] - pose2[0];
    double dy = pose1[1] - pose2[1];
    double dz = pose1[2] - pose2[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

PoseMatrix transLocalFrameToWorld(const PoseMatrix& wp, const PoseMatrix& lp) {
    return wp * lp;
}

vector6d_t transLocalFrameToWorld(const vector6d_t& wp, const vector6d_t& lp) {
    auto wp_mat = vector2Matrix(wp);
    auto lp_mat = vector2Matrix(lp);
    auto result_mat = wp_mat * lp_mat;
    return matrix2Vector(result_mat);
}

PoseMatrix transWorldFrameToLocal(const PoseMatrix& wp, const PoseMatrix& lp) {
    return wp.inverse() * lp;
}

vector6d_t transWorldFrameToLocal(const vector6d_t& wp, const vector6d_t& lp) {
    auto wp_mat = vector2Matrix(wp);
    auto lp_mat = vector2Matrix(lp);
    auto result_mat = wp_mat.inverse() * lp_mat;
    return matrix2Vector(result_mat);
}

} // namespace MATH_UTILS
} // namespace ELITE
