// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// ScriptCommandInterface.hpp
// Provides the ScriptCommandInterface class for robot script commands.
#ifndef __SCRIPT_COMMAND_INTERFACE_HPP__
#define __SCRIPT_COMMAND_INTERFACE_HPP__

#include "DataType.hpp"
#include "ReversePort.hpp"
#include "SerialCommunication.hpp"
#include "TcpServer.hpp"

#include <boost/asio.hpp>
#include <cstdint>
#include <memory>

namespace ELITE {

class ScriptCommandInterface : public ReversePort {
   private:
    enum class Cmd {
        ZERO_FTSENSOR = 0,
        SET_PAYLOAD = 1,
        SET_TOOL_VOLTAGE = 2,
        START_FORCE_MODE = 3,
        END_FORCE_MODE = 4,
        START_TOOL_COMMUNICATION = 5,
        END_TOOL_COMMUNICATION = 6,
        START_BOARD_RS485 = 7,
        END_BOARD_RS485 = 8,
        SET_COLLISION_DETECT_ENABLED = 9,
        SET_COLLISION_SENSITIVITY = 10,
        SET_MOUNTING_PLANE = 11,
    };

    enum class SerialResult {
        START = 1,
        END = 2,
    };

   public:
    static constexpr int SCRIPT_COMMAND_DATA_SIZE = 26;

    ScriptCommandInterface() = delete;

    /**
     * @brief Construct a new Script Command Interface object
     *
     * @param port Server port
     * @param resource TCP resource shared pointer
     */
    ScriptCommandInterface(int port, std::shared_ptr<TcpServer::StaticResource> resource);

    ~ScriptCommandInterface();

    /**
     * @brief zero the force/torque applied to the TCP measured by the sensor(tare sensor).
     *
     * @return true
     * @return false
     */
    bool zeroFTSensor();

    /**
     * @brief This command is used to set the mass,
     * center of gravity and moment of inertia of the robot payload
     *
     * @param mass The mass of the payload
     * @param cog The coordinates of the center of gravity of the payload (relative to the flange frame).
     * @return true success
     * @return false fail
     */
    bool setPayload(double mass, const vector3d_t& cog);

    /**
     * @brief Set the tool voltage
     *
     * @param vol Tool voltage
     * @return true success
     * @return false fail
     */
    bool setToolVoltage(const ToolVoltage& vol);

    /**
     * @brief This command is used to enable force control mode and the robot will be controlled in the force control mode.
     *
     * @param reference_frame A pose vector that defines the force reference frame relative to the base frame.
     * The format is [X,Y,Z,Rx,Ry,Rz], where X, Y, and Z represent position with the unit of m, Rx, Ry, and RZ
     * represent pose with the unit of rad which is defined by standard Euler angles.
     * @param selection_vector a 6-dimensional vector consisting of 0 and 1 that defines the compliant axis in the force frame.
     * 1 represents the axis is compliant and 0 represents the axis is non compliant.
     * @param wrench The force/torque applied to the environment by the robot.
     * The robot moves/rotates along the compliant axis to adjust its pose to achieve the target force/torque.
     * The format is [Fx,Fy,Fz,Mx,My,Mz], where Fx, Fy, and Fz represent the force applied along the
     * compliant axis with the unit of N, Mx, My, and Mz represent the torque applied about the
     * compliant axis with the unit of Nm. This value is invalid for the non-compliant axis. Due to the
     * safety restrictions of joints, the actual applied force/torque is lower than the set one. In the
     * separate thread, the command get_tcp_force may be used to read the actual force/torque applied to the environment.
     * @param mode The parameter for force control mode
     * @param limits The parameter for the speed limit. The format is [Vx,Vy,Vz,ωx,ωy,ωz],
     * where Vx, Vy, and Vz represent the maximum speed for TCP along
     * the compliant axis with the unit of m/s, ωx, ωy, and ωz represent the maximum speed for TCP
     * about the compliant axis with the unit of rad/s. This parameter is invalid for the non-compliant
     * axis whose trajectory will be as set before.
     * @return true success
     * @return false fail
     */
    bool startForceMode(const vector6d_t& task_frame, const vector6int32_t& selection_vector, const vector6d_t& wrench,
                        const ForceMode& mode, const vector6d_t& limits);

    /**
     * @brief This command is used to disable the force control mode. It also will be performed when the procedure ends.
     *
     * @return true success
     * @return false fail
     */
    bool endForceMode();

    /**
     * @brief Enable or disable collision detection.
     *
     * @param enable true to enable, false to disable
     * @return true success
     * @return false fail
     */
    bool setCollisionDetectEnabled(bool enable);

    /**
     * @brief Set collision detection sensitivity.
     *
     * @param ratio Sensitivity ratio in percent. Valid range is [10, 100].
     * @return true success
     * @return false fail
     */
    bool setCollisionSensitivity(int32_t ratio);

    /**
     * @brief Set robot mounting plane by dynamically adjusting the gravity direction.
     *
     * @param z_rotation Rotation angle around the robot base Z axis, in radians.
     * @param tilt Mounting plane tilt angle, in radians.
     * @return true success
     * @return false fail
     */
    bool setMountingPlane(double z_rotation, double tilt = 0.0);
};

}  // namespace ELITE

#endif
