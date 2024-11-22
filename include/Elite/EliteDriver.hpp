/**
 * @file EliteDriver.hpp
 * @author yanxiaojia
 * @brief This file include main interface.
 * @date 2024-08-21
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef __ELITE_DRIVER_HPP__
#define __ELITE_DRIVER_HPP__

#include <Elite/EliteOptions.hpp>
#include <Elite/DataType.hpp>
#include <Elite/PrimaryPackage.hpp>

#include <memory>
#include <string>
#include <functional>

namespace ELITE
{

/**
 * @brief This is the main class for interfacing the driver.
 *  It sets up all the necessary socket connections and handles the data exchange with the robot.
 */
class EliteDriver
{
private:
    class Impl;
    Impl* impl_;
public:
    EliteDriver() = delete;

    /**
     * @brief Construct a new Elite Driver object
     * 
     * @param robot_ip IP-address under which the robot is reachable.
     * @param local_ip Local IP-address that the reverse_port and trajectory_port will bound.
     * @param script_file EliRobot template script file that should be used to generate scripts that can be run.
     * @param headless_mode If the driver should be started in headless mode.
     * @param script_sender_port The driver will offer an interface to receive the program's script on this port. 
     *                           If the robot cannot connect to this port, `External Control` will stop immediately.
     * @param reverse_port Port that will be opened by the driver to allow direct communication between the driver and the robot controller.
     * @param trajectory_port Port used for sending trajectory points to the robot in case of trajectory forwarding.
     * @param script_command_port Port used for forwarding script commands to the robot. The script commands will be
     * executed locally on the robot.
     * @param servoj_time The duration of servoj motion.
     * @param servoj_lookhead_time Time [S], range [0.03,0.2] smoothens the trajectory with this lookahead time
     * @param servoj_gain Not use in current EliRobot&EliServer version(2.11.0).
     */
    ELITE_EXPORT EliteDriver(const std::string& robot_ip, const std::string& local_ip, const std::string& script_file,
                bool headless_mode = false, int script_sender_port = 50002, int reverse_port = 50001,
                int trajectory_port = 50003, int script_command_port = 50004, float servoj_time = 0.008,
                float servoj_lookhead_time = 0.08, int servoj_gain = 300);

    /**
     * @brief Destroy the Elite Driver object
     * 
     */
    ELITE_EXPORT ~EliteDriver();

    /**
     * @brief Write servoj() points to robot
     * 
     * @param pos points
     * @param timeout_ms The read timeout configuration for the reverse socket running in the external control script on the robot.
     * @return true 
     * @return false 
     */
    ELITE_EXPORT bool writeServoj(const vector6d_t& pos, int timeout_ms);

    /**
     * @brief Write speedl() velocity to robot
     * 
     * @param vel line velocity ([x, y, z, rx, ry, rz])
     * @param timeout_ms The read timeout configuration for the reverse socket running in the external control script on the robot.
     * @return true 
     * @return false 
     */
    ELITE_EXPORT bool writeSpeedl(const vector6d_t& vel, int timeout_ms);

    /**
     * @brief Write speedj() velocity to robot
     * 
     * @param vel joint velocity
     * @param timeout_ms The read timeout configuration for the reverse socket running in the external control script on the robot.
     * @return true 
     * @return false 
     */
    ELITE_EXPORT bool writeSpeedj(const vector6d_t& vel, int timeout_ms);

    /**
     * @brief Register a callback for the robot-based trajectory execution completion.
     * 
     *  One mode of robot control is to forward a complete trajectory to the robot for execution.
     *  When the execution is done, the callback function registered here will be triggered.
     * 
     * @param cb Callback function that will be triggered in the event of finishing
     */
    ELITE_EXPORT void setTrajectoryResultCallback(std::function<void(TrajectoryMotionResult)> cb);

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
    ELITE_EXPORT bool writeTrajectoryPoint(const vector6d_t& positions, float time, float blend_radius, bool cartesian);

    /**
     * @brief Writes a control message in trajectory forward mode.
     * 
     * @param action The action to be taken, such as starting a new trajectory
     * @param point_number The number of points of a new trajectory to be sent
     * @param timeout_ms The read timeout configuration for the reverse socket running in the external control script on the robot.
     * @return true 
     * @return false 
     */
    ELITE_EXPORT bool writeTrajectoryControlAction(TrajectoryControlAction action, const int point_number, int timeout_ms);

    /**
     * @brief Write a idle signal only.
     * 
     *  When robot recv idle signal, robot will stop motion.
     * 
     * @param timeout_ms The read timeout configuration for the reverse socket running in the external control script on the robot.
     * @return true 
     * @return false 
     */
    ELITE_EXPORT bool writeIdle(int timeout_ms);

    /**
     * @brief Sends a stop command to the socket interface which will signal the program running on 
     * the robot to no longer listen for commands sent from the remote pc.
     * 
     * @return true 
     * @return false 
     */
    ELITE_EXPORT bool stopControl();

    /**
     * @brief Print generated EliRobot script from template
     * 
     */
    [[deprecated("Print script is deprecated, instead use ExternalControl plugin or send script to robot.")]]
    ELITE_EXPORT void printRobotScript();

    /**
     * @brief Is robot connect to server.
     * 
     * @return true connected
     * @return false don't
     */
    ELITE_EXPORT bool isRobotConnected();

    /**
     * @brief zero the force/torque applied to the TCP measured by the sensor(tare sensor).
     * 
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool zeroFTSensor();
    
    /**
     * @brief This command is used to set the mass, 
     * center of gravity and moment of inertia of the robot payload
     * 
     * @param mass The mass of the payload
     * @param cog The coordinates of the center of gravity of the payload (relative to the flange frame).
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool setPayload(double mass, const vector3d_t& cog);
    
    /**
     * @brief Set the tool voltage
     * 
     * @param vol Tool voltage
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool setToolVoltage(const ToolVoltage& vol);
    
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
    ELITE_EXPORT bool startForceMode(const vector6d_t& reference_frame, const vector6int32_t& selection_vector,
                        const vector6d_t& wrench, const ForceMode& mode, const vector6d_t& limits);
    
    /**
     * @brief This command is used to disable the force control mode. It also will be performed when the procedure ends. 
     * 
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool endForceMode();

    /**
     * @brief Send a custom script.
     * 
     * @param script Custom script
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool sendScript(const std::string& script);

    /**
     * @brief Send external control script
     * 
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool sendExternalControlScript();
    
    /**
     * @brief Get primary port sub-package
     * 
     * @param pkg sub-package
     * @param timeout_ms timeout
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool getPrimaryPackage(std::shared_ptr<PrimaryPackage> pkg, int timeout_ms);

    /**
     * @brief Reconnect robot primary interface
     * 
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool primaryReconnect();
};



} // namespace name






#endif