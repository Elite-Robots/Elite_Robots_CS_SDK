// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// FrameUtils.hpp
// Provides pose and vector transformations between robot coordinate frames.
#ifndef __ELITE_FRAME_UTILS_HPP__
#define __ELITE_FRAME_UTILS_HPP__

#include <Elite/DataType.hpp>

namespace ELITE {

/**
 * @brief Transform a pose expressed in a frame into the base frame.
 * @param pose Pose expressed in the frame.
 * @param frame_pose Pose of the frame expressed in the base frame.
 * @return Pose expressed in the base frame.
 */
ELITE_EXPORT vector6d_t poseToBaseFrame(const vector6d_t& pose, const vector6d_t& frame_pose);

/**
 * @brief Transform a pose expressed in the base frame into a frame.
 * @param base_pose Pose expressed in the base frame.
 * @param frame_pose Pose of the frame expressed in the base frame.
 * @return Pose expressed in the frame.
 */
ELITE_EXPORT vector6d_t poseFromBaseFrame(const vector6d_t& base_pose, const vector6d_t& frame_pose);

/**
 * @brief Rotate the linear and angular parts of a six-dimensional vector.
 * @param frame_pose Pose of the frame expressed in the base frame.
 * @param vector Vector expressed in the frame.
 * @return Vector expressed in the base frame.
 */
ELITE_EXPORT vector6d_t rotateVector(const vector6d_t& frame_pose, const vector6d_t& vector);

}  // namespace ELITE

#endif
