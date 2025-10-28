/**
 * @file EliteDriver.hpp
 * @author yanxiaojia
 * @brief This file include main interface.
 * @date 2024-08-21
 *
 * @copyright Copyright (c) 2024+
 *
 */
#ifndef __ELITE_DRIVER_HPP__
#define __ELITE_DRIVER_HPP__

#include <Elite/DataType.hpp>
#include <Elite/EliteOptions.hpp>
#include <Elite/PrimaryPackage.hpp>
#include <Elite/PrimaryPortInterface.hpp>
#include <functional>
#include <memory>
#include <string>
#include "kinematics.hpp"

namespace ELITE {

class EliteDriverConfig {
   public:
    // IP-address under which the robot is reachable.
    std::string robot_ip;

    // EliRobot template script file that should be used to generate scripts that can be run.
    std::string script_file_path;

    // Local IP-address that the reverse_port and trajectory_port will bound.
    std::string local_ip = "";

    // If the driver should be started in headless mode.
    bool headless_mode = false;

    // The driver will offer an interface to receive the program's script on this port.
    // If the robot cannot connect to this port, `External Control` will stop immediately.
    int script_sender_port = 50002;

    // Port that will be opened by the driver to allow direct communication between the driver and the robot controller.
    int reverse_port = 50001;

    // Port used for sending trajectory points to the robot in case of trajectory forwarding.
    int trajectory_port = 50003;

    // Port used for forwarding script commands to the robot. The script commands will be executed locally on the robot.
    int script_command_port = 50004;

    // The duration of servoj motion.
    float servoj_time = 0.008;

    // Time [S], range [0.03,0.2] smoothens the trajectory with this lookahead time
    float servoj_lookahead_time = 0.1;

    // Servo gain.
    int servoj_gain = 300;

    // Acceleration [rad/s^2]. The acceleration of stopj motion.
    float stopj_acc = 8;

    // When using the `writeServoj()` and the `queue_mode` parameter is true, the timeout duration for the queue waiting for. (For
    // detailed descriptions of the queue mode, please refer to the description of this interface in the API documentation.)
    int servoj_queue_pre_recv_size = 10;

    // When using the `writeServoj()` and the `queue_mode` parameter is true, the timeout duration for the queue waiting for
    // pre-stored points. If the value is less than or equal to 0, the timeout duration will be calculated based on
    // `servoj_queue_pre_recv_size * servoj_time`.(For detailed descriptions of the queue mode, please refer to the description of
    // this interface in the API documentation.)
    float servoj_queue_pre_recv_timeout = -1;

    EliteDriverConfig() = default;
    ~EliteDriverConfig() = default;
};

/**
 * @brief This is the main class for interfacing the driver.
 *  It sets up all the necessary socket connections and handles the data exchange with the robot.
 */
class EliteDriver {
   private:
    class Impl;
    std::unique_ptr<Impl> impl_;
    void init(const EliteDriverConfig& config);

   public:
    EliteDriver() = delete;

    /**
     * @brief Construct a new Elite Driver object
     *
     * @param config Configuration class for the EliteDriver. See it's code annotation for details.
     */
    ELITE_EXPORT EliteDriver(const EliteDriverConfig& config);

    /**
     * @brief Construct a new Elite Driver object
     *
     * @param robot_ip IP-address under which the robot is reachable.
     * @param local_ip Local IP-address that the reverse_port and trajectory_port will bound.
     * @param script_file EliRobot template script file that should be used to generate scripts that can be run.
     * @param headless_mode If the driver should be started in headless mode.
     * @param script_sender_port The driver will offer an interface to receive the program's script on this port.
     *                           If the robot cannot connect to this port, `External Control` will stop immediately.
     * @param reverse_port Port that will be opened by the driver to allow direct communication between the driver and the robot
     * controller.
     * @param trajectory_port Port used for sending trajectory points to the robot in case of trajectory forwarding.
     * @param script_command_port Port used for forwarding script commands to the robot. The script commands will be
     * executed locally on the robot.
     * @param servoj_time The duration of servoj motion.
     * @param servoj_lookahead_time Time [S], range [0.03,0.2] smoothens the trajectory with this lookahead time
     * @param servoj_gain servo gain.
     * @param stopj_acc acceleration [rad/s^2]. The acceleration of stopj motion.
     */
    [[deprecated(
        "Construct a EliteDriver object with an argument list is deprecated. Please use"
        "EliteDriver(const EliteDriverConfig& config) instead. This function will be removed in June 2027.")]] ELITE_EXPORT
    EliteDriver(const std::string& robot_ip, const std::string& local_ip, const std::string& script_file,
                bool headless_mode = false, int script_sender_port = 50002, int reverse_port = 50001, int trajectory_port = 50003,
                int script_command_port = 50004, float servoj_time = 0.008, float servoj_lookahead_time = 0.1,
                int servoj_gain = 300, float stopj_acc = 8.0);

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
     * @param cartesian True if the point sent is cartesian, false if joint-based
     * @param queue_mode True if use queue mode, false if normal mode. (For detailed descriptions of the queue mode, please refer to
     * the description of this interface in the API documentation.)
     * @return true Joint angles sent successfully.
     * @return false Fail to send joint angles.
     */
    ELITE_EXPORT bool writeServoj(const vector6d_t& pos, int timeout_ms, bool cartesian = false, bool queue_mode = false);

    /**
     * @brief Write speedl() velocity to robot
     *
     * @param vel line velocity ([x, y, z, rx, ry, rz])
     * @param timeout_ms The read timeout configuration for the reverse socket running in the external control script on the robot.
     * @return true Linear velocity sent successfully.
     * @return false Fail to send linear velocity.
     */
    ELITE_EXPORT bool writeSpeedl(const vector6d_t& vel, int timeout_ms);

    /**
     * @brief Write speedj() velocity to robot
     *
     * @param vel joint velocity
     * @param timeout_ms The read timeout configuration for the reverse socket running in the external control script on the robot.
     * @return true Joint velocity sent successfully.
     * @return false Fail to send joint velocity.
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
     * @return true Trajectory point sent successfully.
     * @return false Fail to send trajectory point.
     */
    ELITE_EXPORT bool writeTrajectoryPoint(const vector6d_t& positions, float time, float blend_radius, bool cartesian);

    /**
     * @brief Writes a control message in trajectory forward mode.
     *
     * @param action The action to be taken, such as starting a new trajectory
     * @param point_number The number of points of a new trajectory to be sent
     * @param timeout_ms The read timeout configuration for the reverse socket running in the external control script on the robot.
     * @return true Trajectory action sent successfully.
     * @return false Fail to send trajectory action.
     */
    ELITE_EXPORT bool writeTrajectoryControlAction(TrajectoryControlAction action, const int point_number, int timeout_ms);

    /**
     * @brief Write a idle signal only.
     *
     *  When robot recv idle signal, robot will stop motion.
     *
     * @param timeout_ms The read timeout configuration for the reverse socket running in the external control script on the robot.
     * @return true Idle signal sent successfully.
     * @return false Fail to send idle signal.
     */
    ELITE_EXPORT bool writeIdle(int timeout_ms);

    /**
     * @brief Writes a freedrive mode control command to the robot
     *
     * @param action Freedrive mode action assigned to this command, such as starting or stopping freedrive.
     * @param timeout_ms The read timeout configuration for the reverse socket running in the external control script on the robot.
     * @return true Freedriver action sent successfully.
     * @return false Fail to send freedriver action.
     */
    ELITE_EXPORT bool writeFreedrive(FreedriveAction action, int timeout_ms);

    /**
     * @brief Sends a stop command to the socket interface which will signal the program running on
     * the robot to no longer listen for commands sent from the remote pc.
     *
     * @param wait_ms Waiting for the robot to disconnect for a certain amount of time. The minimum value is 5.
     * @return true success
     * @return false fail (socket was disconnect or timeout)
     */
    ELITE_EXPORT bool stopControl(int wait_ms = 10000);

    /**
     * @brief Print generated EliRobot script from template
     *
     */
    [[deprecated(
        "Print script is deprecated, instead use ExternalControl plugin or send script to robot. This function will be removed in "
        "June 2027.")]] ELITE_EXPORT void
    printRobotScript();

    /**
     * @brief Is robot connect to server.
     *
     * @return true connected
     * @return false don't
     */
    ELITE_EXPORT bool isRobotConnected();

    /**
     * @brief Zero (tare) the force and torque values measured by the force/torque sensor and applied to the tool TCP. The force and
     * torque values are the force and torque vectors applied to the tool TCP obtained by the `get_tcp_force(True)` script
     * instruction. These vectors have undergone processing such as load compensation.
     *
     * After this command is executed, the current force and torque measurement values will be saved as the force and torque
     * reference values. All subsequent force and torque measurement values will be subtracted by this force and torque reference
     * value (tared).
     *
     * Please note that the above - mentioned force and torque reference values will be updated when this command is executed and
     * will be reset to 0 after the controller is restarted.
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

    /**
     * @brief Registers a callback for robot exceptions.
     *
     * This function registers a callback that will be invoked whenever
     * a robot exception message is received from the primary port.
     *
     * @param cb A callback function that takes a RobotExceptionSharedPtr
     *           representing the received exception.
     */
    ELITE_EXPORT void registerRobotExceptionCallback(std::function<void(RobotExceptionSharedPtr)> cb);

    /**
     * @brief Computes the inverse kinematics using the robot's internal solver.
     *
     * This function calculates the joint angles required for the robot's end-effector
     * to reach the specified Cartesian pose using the **robot's onboard computation**.
     *
     * @param pose A `vector6d_t` representing the desired end-effector pose [x, y, z, Rx, Ry, Rz]
     *             in meters and radians.
     * @param near_joint A `vector6d_t` providing a nearby joint configuration as the initial guess.
     * @param tcp (Optional) A `vector6d_t` specifying the TCP offset. Defaults to {0,0,0,0,0,0}.
     *
     * @return A `std::shared_ptr<vector6d_t>` containing the computed joint angles [J1..J6] in radians.
     *         Returns `nullptr` if the onboard solver fails.
     *
     * @note The computation is performed on the **robot controller itself**, so it uses the
     *       robot's current calibration and does **not** require DH parameters to be set.
     */
    ELITE_EXPORT std::shared_ptr<vector6d_t> getInverseKinematics(const vector6d_t& pose, const vector6d_t& near_joint,
                                                                  const vector6d_t& tcp = {0, 0, 0, 0, 0, 0});

    /**
     * @brief Computes the forward kinematics using the robot's internal solver.
     *
     * This function calculates the Cartesian pose of the robot's end-effector based on the given
     * joint angles using the **robot's onboard computation**, which reflects the current
     * calibration and configuration of the robot.
     *
     * @param joints A `vector6d_t` representing the robot's joint angles [J1..J6] in radians.
     * @param tcp_offset_xyzRPY (Optional) A `vector6d_t` specifying the TCP offset [x, y, z, Rx, Ry, Rz].
     *                          Defaults to {0,0,0,0,0,0}.
     *
     * @return A `std::shared_ptr<vector6d_t>` containing the computed end-effector pose [x, y, z, Rx, Ry, Rz],
     *         where x, y, z are in meters and Rx, Ry, Rz are in radians.
     *         Returns `nullptr` if the onboard solver fails.
     *
     * @note Uses the **robot controller's internal solver**, no DH parameters need to be set.
     */
    ELITE_EXPORT std::shared_ptr<vector6d_t> getForwardKinematics(const vector6d_t& joint,
                                                                  const vector6d_t& tcp = {0, 0, 0, 0, 0, 0});
    /**
     * @brief Configures the robot's Denavit-Hartenberg (DH) parameters for kinematics calculations.
     *
     * This function must be called **before performing inverse kinematics or forward kinematics**
     * calculations that rely on the KDL solver. It sets the robot's link lengths, offsets, and twist angles
     * according to the DH convention.
     *
     * @param dh_a An array of doubles of size `AXIS_COUNT` representing the link lengths (a_i) in meters.
     * @param dh_d An array of doubles of size `AXIS_COUNT` representing the link offsets (d_i) in meters.
     * @param dh_alpha An array of doubles of size `AXIS_COUNT` representing the link twist angles (alpha_i) in radians.
     *
     * @note
     * - Failing to call this function before computing forward or inverse kinematics may result
     *   in incorrect or undefined solutions.
     * - The `AXIS_COUNT` should match the number of joints of the robot (default to 6).
     *
     * @example
     * double dh_a[6] = {0.0, 0.5, 0.3, 0.0, 0.0, 0.0};
     * double dh_d[6] = {0.2, 0.0, 0.0, 0.4, 0.0, 0.1};
     * double dh_alpha[6] = {0.0, -M_PI/2, 0.0, -M_PI/2, M_PI/2, 0.0};
     * robotDriver->setKDLconfig(dh_a, dh_d, dh_alpha);
     */
    ELITE_EXPORT void setKDLconfig(const double dh_a[AXIS_COUNT], const double dh_d[AXIS_COUNT], const double dh_alpha[AXIS_COUNT]);
    /**
    * @brief Computes the inverse kinematics using a local solver on the PC.
    *
    * This function calculates the joint angles required for the robot's end-effector
    * to reach the specified Cartesian pose using **local computation on the PC**.
    *
    * @warning Before calling this function, you **must** configure the robot's DH parameters
    *          using `setKDLconfig()`. Failure to do so may result in incorrect IK results.
    *
    * @param pose A `vector6d_t` representing the desired end-effector pose [x, y, z, Rx, Ry, Rz]
    *             in meters and radians.
    * @param near_joint A `vector6d_t` providing a nearby joint configuration as the initial guess.
    * @param tcp_offset_xyzRPY (Optional) A `vector6d_t` specifying the TCP offset. Defaults to {0,0,0,0,0,0}.
    *
     @return A `std::shared_ptr<vector6d_t>` containing the computed joint angles [J1..J6] in radians.
    *         Returns `nullptr` if no valid solution is found.
    *
    * @note The computation uses the **local PC** for calculation.
    *
    * @example
    * // First, configure the robot DH parameters
    * double dh_a[6] = {...};
    * double dh_d[6] = {...};
    * double dh_alpha[6] = {...};
    * robotDriver.setKDLconfig(dh_a, dh_d, dh_alpha);
    *
    * // Then compute inverse kinematics
    * vector6d_t target_pose = {0.5, 0.0, 0.3, 0.0, 0.0, 0.0};
    * vector6d_t near_joint = {0, 0, 0, 0, 0, 0};
    * auto joint_ptr = robotDriver.getInverseKin(target_pose, near_joint);
    * if(joint_ptr) {
    *     std::cout << "IK solution: "
    *               << (*joint_ptr)[0] << ", "
    *               << (*joint_ptr)[1] << ", "
    *               << (*joint_ptr)[2] << ", ..."
    *               << std::endl;
    * }
    */
    ELITE_EXPORT std::shared_ptr<vector6d_t> getInverseKin(const vector6d_t& pose, const vector6d_t& near_joint,
                                                           const vector6d_t& tcp_offset_xyzRPY = {0, 0, 0, 0, 0, 0});
    /**
     * @brief Computes the forward kinematics using a local solver  on the PC.
     *
     * This function calculates the Cartesian pose of the robot's end-effector based on the given
     * joint angles using **local computation on the PC**.
     *
     * @warning Before calling this function, you **must** configure the robot's DH parameters
     *          using `setKDLconfig()`. Otherwise, the result is incorrect.
     *
     * @param joint A `vector6d_t` representing the robot's joint angles [J1..J6] in radians.
     * @param tcp (Optional) A `vector6d_t` specifying the TCP (Tool Center Point) offset. Defaults to {0,0,0,0,0,0}.
     *
     * @return A `std::shared_ptr<vector6d_t>` containing the computed end-effector pose [x, y, z, Rx, Ry, Rz],
     *         where x, y, z are in meters and Rx, Ry, Rz are in radians.
     *         Returns `nullptr` if the forward kinematics calculation fails.
     *
     * @note Uses the **local PC** for computation.
     */
    ELITE_EXPORT std::shared_ptr<vector6d_t> getForwardKin(const vector6d_t& joints,
                                                           const vector6d_t& tcp_offset_xyzRPY = {0, 0, 0, 0, 0, 0});
};

}  // namespace ELITE

#endif