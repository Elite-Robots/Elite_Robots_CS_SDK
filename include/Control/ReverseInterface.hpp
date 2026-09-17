// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// ReverseInterface.hpp
// Provides the ReverseInterface class for robot communication.
#ifndef __REVERSE_INTERFACE_HPP__
#define __REVERSE_INTERFACE_HPP__

#include "ControlMode.hpp"
#include "DataType.hpp"
#include "ReversePort.hpp"
#include "TcpServer.hpp"

#include <boost/asio.hpp>
#include <cstdint>
#include <mutex>

namespace ELITE {

/**
 * @brief The ReverseInterface class handles communication to the robot.
 * It starts a server and waits for the robot to connect via its EliteRobot program.
 *
 *
 */
class ReverseInterface : public ReversePort {
   public:
    // timeout + six values + mode + user frame id.
    static const int REVERSE_DATA_SIZE = 9;

    ReverseInterface() = delete;

    /**
     * @brief Construct a new Reverse Interface object include TcpServer
     *
     * @param port Port the Server is started
     * @param resource TCP resource shared pointer
     */
    ReverseInterface(int port, std::shared_ptr<TcpServer::StaticResource> resource);

    /**
     * @brief Destroy the Reverse Interface object. Will disconnect robot connection.
     *
     */
    ~ReverseInterface();

    /**
     * @brief Writes needed information to the robot to be read by the EliteRobot program.
     *
     * @param pos
     * @param mode
     * @param timeout_ms
     * @return true
     * @return false
     */
    bool writeJointCommand(const vector6d_t& pos, ControlMode mode, int timeout_ms);
    bool writeJointCommand(const vector6d_t* pos, ControlMode mode, int timeout_ms);
    bool writeJointCommand(const vector6d_t& pos, ControlMode mode, int timeout_ms, int32_t user_frame_id);
    bool writeJointCommand(const vector6d_t* pos, ControlMode mode, int timeout_ms, int32_t user_frame_id);

    /**
     * @brief Writes needed information to the robot to be read by the EliteRobot program.
     *
     * @param action Trajectory action assigned to this command. See documentation of TrajectoryControlAction for details on
     * possible values.
     * @param point_number The number of points of the trajectory to be executed
     * @param timeout_ms The read timeout configuration for the reverse socket running in the external control script on the robot.
     * @return true success
     * @return false fail
     */
    bool writeTrajectoryControlAction(TrajectoryControlAction action, const int point_number, int timeout_ms);

    /**
     * @brief Writes a freedrive mode control command to the robot
     *
     * @param action Freedrive mode action assigned to this command.
     * @param timeout_ms The read timeout configuration for the reverse socket running in the external control script on the robot.
     * @return true success
     * @return false fail
     */
    bool writeFreedrive(FreedriveAction action, int timeout_ms);

    /**
     * @brief Finish external control script.
     *
     * @return true success
     * @return false fail
     */
    bool stopControl();
};

}  // namespace ELITE

#endif
