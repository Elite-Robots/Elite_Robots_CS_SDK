#ifndef __ELITE__MATH_UTILS_HPP__
#define __ELITE__MATH_UTILS_HPP__

#include <Elite/DataType.hpp>
#include <cmath>

namespace ELITE
{

namespace MATH_UTILS {

/**
 * @brief Check if a double value is zero within a minimum threshold
 * 
 * @param value The double value to check
 * @param minVal The minimum threshold to consider as zero (default is 1.0e-9)
 * @return true if the value is considered zero
 * @return false if the value is not zero
 */
inline bool isZero(double value, double minVal = 1.0e-9) {
    return std::fabs(value) < minVal;
}

/**
 * @brief Convert a 6D vector (position + RPY) to a 4x4 pose matrix
 * 
 * @param vt 6D vector (position + RPY)
 * @return PoseMatrix 4x4 pose matrix
 */
PoseMatrix vector2Matrix(const vector6d_t& vt);

/**
 * @brief Convert a 4x4 pose matrix to a 6D vector (position + RPY)
 * 
 * @param pm The 4x4 pose matrix 
 * @return vector6d_t The 6D vector (position + RPY)
 */
vector6d_t matrix2Vector(const PoseMatrix& pm);

/**
 * @brief Calculate the Euclidean distance between the positions of two poses
 * 
 * @param pose1 pose 1
 * @param pose2 pose 2
 * @return double The Euclidean distance between the positions of the two poses
 */
double poseDistance(const vector6d_t& pose1, const vector6d_t& pose2);

/**
 * @brief Transform a local frame pose to a world frame pose
 * 
 *     T_world_target = T_world_local * T_local_target
 * 
 * @param wp Local coordinate system values in the world coordinate system.
 * @param lp Values of the target coordinate system in the local coordinate system.
 * @return PoseMatrix/vector6d_t Transformed pose in the world coordinate system. 
 */
PoseMatrix transLocalFrameToWorld(const PoseMatrix& wp, const PoseMatrix& lp);
vector6d_t transLocalFrameToWorld(const vector6d_t& wp, const vector6d_t& lp);

/**
 * @brief Transform a world frame pose to a local frame pose
 * 
 *     T_local_target = T_world_local_inv * T_world_target
 * 
 * @param wp Local coordinate system values in the world coordinate system.
 * @param lp Values of the target coordinate system in the world coordinate system.
 * @return PoseMatrix/vector6d_t Transformed pose in the local coordinate system. 
 */
PoseMatrix transWorldFrameToLocal(const PoseMatrix& wp, const PoseMatrix& lp);
vector6d_t transWorldFrameToLocal(const vector6d_t& wp, const vector6d_t& lp);

} // namespace MATH_UTILS
} // namespace ELITE

#endif  // __ELITE__MATH_UTILS_HPP__