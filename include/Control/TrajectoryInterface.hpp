// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// TrajectoryInterface.hpp
// Provides the TrajectoryInterface class for robot trajectory control.
#ifndef __TRAJECTORY_INTERFACE_HPP__
#define __TRAJECTORY_INTERFACE_HPP__

#include <functional>
#include <memory>
#include "DataType.hpp"
#include "ReversePort.hpp"
#include "TcpServer.hpp"

namespace ELITE {

enum class TrajectoryMotionType : int {
    JOINT = 0,      // movej
    CARTESIAN = 1,  // movel
    SPLINE = 2      // spline
};

class TrajectoryInterface : public ReversePort {
   public:
    static const int TRAJECTORY_MESSAGE_LEN = 21;
    static const int TRAJECTORY_FEEDBACK_LEN = 10;

    TrajectoryInterface() = delete;

    /**
     * @brief Construct a new Trajectory Interface object include TcpServer
     *
     * @param port Port the Server is started on
     * @param resource TCP resource shared pointer
     */
    TrajectoryInterface(int port, std::shared_ptr<TcpServer::StaticResource> resource);

    ~TrajectoryInterface();

    /**
     * @brief Register a callback for the robot-based trajectory execution completion.
     *
     *  One mode of robot control is to forward a complete trajectory to the robot for execution.
     *  When the execution is done, the callback function registered here will be triggered.
     *
     * @param cb Callback function that will be triggered in the event of finishing
     */
    void setMotionResultCallback(std::function<void(TrajectoryMotionResult)> cb) { motion_result_func_ = cb; }

    /**
     * @brief Register a callback for robot-based trajectory feedback frames.
     *
     * @param cb Callback function that will be triggered when the robot reports trajectory progress.
     */
    void setMotionFeedbackCallback(std::function<void(const TrajectoryMotionFeedback&)> cb) { motion_feedback_func_ = cb; }

    /**
     * @brief Writes a trajectory point onto the dedicated socket.
     *
     * @param positions Desired joint or cartesian positions
     * @param time Time for the robot to reach this point
     * @param blend_radius The radius to be used for blending between control points
     * @param cartesian True, if the point sent is cartesian, false if joint-based
     * @return true
     * @return false
     */
    bool writeTrajectoryPoint(const vector6d_t& positions, float time, float blend_radius, bool cartesian);

    /**
     * @brief Writes a trajectory point onto the dedicated socket.
     *
     * @param positions Desired joint or cartesian positions
     * @param blend_radius The radius to be used for blending between control points
     * @param cartesian True, if the point sent is cartesian, false if joint-based
     * @param speed Joint speed for movej or TCP speed for movel
     * @param acceleration Joint acceleration for movej or TCP acceleration for movel
     * @return true
     * @return false
     */
    bool writeTrajectoryPoint(const vector6d_t& positions, float blend_radius, bool cartesian, float speed, float acceleration);

   private:
    bool writeTrajectoryPoint(const vector6d_t& positions, float time, float blend_radius, bool cartesian, float speed,
                              float acceleration);

    std::function<void(TrajectoryMotionResult)> motion_result_func_;
    std::function<void(const TrajectoryMotionFeedback&)> motion_feedback_func_;
};

}  // namespace ELITE

#endif
