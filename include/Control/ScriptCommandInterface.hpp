#ifndef __SCRIPT_COMMAND_INTERFACE_HPP__
#define __SCRIPT_COMMAND_INTERFACE_HPP__

#include <boost/asio.hpp>
#include <memory>
#include <vector>
#include "DataType.hpp"
#include "ReversePort.hpp"
#include "TcpServer.hpp"
#include <iostream>

namespace ELITE {

class ScriptCommandInterface : public ReversePort {
   private:
    enum class Cmd {
        ZERO_FTSENSOR = 0,
        SET_PAYLOAD = 1,
        SET_TOOL_VOLTAGE = 2,
        START_FORCE_MODE = 3,
        END_FORCE_MODE = 4,
        INVERSE_KINEMATICS = 5,
        FORWARD_KINEMATICS = 6
    };

   public:
    static constexpr int SCRIPT_COMMAND_DATA_SIZE = 26;

    std::mutex data_mutex_;
    std::vector<uint8_t> recv_buffer_;
    std::vector<int32_t> last_data_;
    bool data_received_ = false;
    ScriptCommandInterface() = delete;

    /**
     * @brief Construct a new Script Command Interface object
     *
     * @param port Server port
     */
    ScriptCommandInterface(int port);

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
     * @brief This command calculates the inverse kinematics solution based on the given pose, near joint configuration, and TCP
     * (Tool Center Point).
     *
     * The function computes the joint angles required to achieve a specified pose in Cartesian space.
     * If the `near_joint` and `tcp` parameters are not provided, the function will use default values.
     *
     * @param pose The target pose in Cartesian space [x, y, z, Rx, Ry, Rz], where x, y, z are in meters, and Rx, Ry, Rz are in
     * radians.
     * @param near_joint near_joint: The joint angle configuration closest to the desired pose.
     * @param tcp (Optional) The Tool Center Point (TCP) configuration. If not provided, the default TCP is used.
     *
     * @return A shared pointer to a `vector6d_t` containing the joint angles in radians that correspond to the target pose.
     *         Returns nullptr if no valid solution is found.
     */

    std::shared_ptr<vector6d_t> getInverseKinematics(const vector6d_t& pose, const vector6d_t& near_joint, const vector6d_t& tcp);


    /**
     * @brief This command calculates the forward kinematics solution based on the given joint angles and TCP (Tool Center Point).
     *
     * The function computes the pose in Cartesian space corresponding to a specified set of joint angles.
     * If the `tcp` parameter is not provided, the function will use a default value.
     *
     * @param joint The joint angles [J1, J2, J3, J4, J5, J6] in radians.
     * @param tcp (Optional) The Tool Center Point (TCP) configuration. If not provided, the default TCP is used.
     *
     * @return A shared pointer to a `vector6d_t` containing the pose [x, y, z, Rx, Ry, Rz], where x, y, z are in meters,
     *         and Rx, Ry, Rz are in radians. Returns nullptr if no valid solution is found.
     */
    std::shared_ptr<vector6d_t> getForwardKinematics(const vector6d_t& joint, const vector6d_t& tcp);

    void handReceive(const uint8_t* data, int data_length);
};

}  // namespace ELITE

#endif