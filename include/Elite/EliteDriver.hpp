// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// EliteDriver.hpp
// Provides the EliteDriver class for interfacing with Elite Robots..
#ifndef __ELITE_DRIVER_HPP__
#define __ELITE_DRIVER_HPP__

#include <Elite/DataType.hpp>
#include <Elite/EliteOptions.hpp>
#include <Elite/PrimaryPackage.hpp>
#include <Elite/PrimaryPortInterface.hpp>
#include <Elite/SerialCommunication.hpp>

#include <functional>
#include <memory>
#include <string>
#include <vector>

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

    // Maximum duration [S] for constant-velocity extrapolation when no new servoj point arrives.
    float servoj_extrapolate_max_time = 0.08;

    // Deceleration duration [S] used to linearly ramp extrapolation speed down to zero.
    float servoj_decelerate_time = 0.01;

    // Joint velocity threshold [rad/s] used to decide whether joints are stable enough to lock hold position.
    float servoj_hold_velocity_threshold = 0.05;

    // Stable duration [S] required before locking hold position after extrapolation speed reaches zero.
    float servoj_hold_stable_time = 0.04;

    // SDK-managed user frames. The pose is expressed in the base frame.
    std::vector<UserFrame> user_frames;

    // Number of user frame slots generated in the robot script.
    int32_t max_user_frame_count = MAX_USER_FRAME_COUNT;

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
     * @param servoj_extrapolate_max_time Maximum duration [S] for constant-velocity extrapolation.
     * @param servoj_decelerate_time Deceleration duration [S] used to ramp extrapolation speed to zero.
     * @param servoj_hold_velocity_threshold Joint velocity threshold [rad/s] for hold lock decision.
     * @param servoj_hold_stable_time Stable duration [S] required before locking hold position.
     */
    [[deprecated(
        "Construct a EliteDriver object with an argument list is deprecated. Please use"
        "EliteDriver(const EliteDriverConfig& config) instead. This function will be removed in June 2027.")]] ELITE_EXPORT
    EliteDriver(const std::string& robot_ip, const std::string& local_ip, const std::string& script_file,
                bool headless_mode = false, int script_sender_port = 50002, int reverse_port = 50001, int trajectory_port = 50003,
                int script_command_port = 50004, float servoj_time = 0.008, float servoj_lookahead_time = 0.1,
                int servoj_gain = 300, float stopj_acc = 8.0, float servoj_extrapolate_max_time = 0.08,
                float servoj_decelerate_time = 0.01, float servoj_hold_velocity_threshold = 0.05,
                float servoj_hold_stable_time = 0.04);

    /**
     * @brief Destroy the Elite Driver object
     *
     */
    ELITE_EXPORT ~EliteDriver();

    /**
     * @brief Write a servoj() point using the current active user frame.
     *
     * The current active user frame is used only when cartesian is true. The
     * default active frame is the base frame. Joint positions are independent
     * of the active user frame.
     *
     * @param pos Target Cartesian pose or joint positions.
     * @param timeout_ms The read timeout configuration for the reverse socket running in the external control script on the robot.
     * @param cartesian True if pos is a Cartesian pose, false if pos contains joint positions.
     * @return true The point was sent successfully.
     * @return false The point could not be sent.
     */
    ELITE_EXPORT bool writeServoj(const vector6d_t& pos, int timeout_ms, bool cartesian = false);

    /**
     * @brief Write a servoj() point expressed in a specified user frame.
     *
     * The user frame is used only when cartesian is true. Joint positions are
     * independent of the selected user frame.
     *
     * @param pos Target Cartesian pose or joint positions.
     * @param timeout_ms The read timeout configuration for the reverse socket running in the external control script on the robot.
     * @param cartesian True if pos is a Cartesian pose, false if pos contains joint positions.
     * @param user_frame_id -1 for the base frame, or a user frame id in the range
     *                       [0, max_user_frame_count).
     * @return true The point was sent successfully.
     * @return false The frame id or other input is invalid, or the point could not be sent.
     */
    ELITE_EXPORT bool writeServoj(const vector6d_t& pos, int timeout_ms, bool cartesian, int32_t user_frame_id);

    /**
     * @brief Write speedl() velocity using the current active user frame.
     *
     * The current active user frame is used to interpret the velocity. The
     * default active frame is the base frame.
     *
     * @param vel TCP linear and angular velocity [vx, vy, vz, wx, wy, wz].
     * @param timeout_ms The read timeout configuration for the reverse socket running in the external control script on the robot.
     * @return true The velocity was sent successfully.
     * @return false The velocity could not be sent.
     */
    ELITE_EXPORT bool writeSpeedl(const vector6d_t& vel, int timeout_ms);

    /**
     * @brief Write speedl() velocity expressed in a specified user frame.
     *
     * The velocity is projected from the specified user frame into the base
     * frame before it is sent to the robot.
     *
     * @param vel TCP linear and angular velocity [vx, vy, vz, wx, wy, wz].
     * @param timeout_ms The read timeout configuration for the reverse socket running in the external control script on the robot.
     * @param user_frame_id -1 for the base frame, or a user frame id in the range
     *                        [0, max_user_frame_count).
     * @return true The velocity was sent successfully.
     * @return false The frame id or other input is invalid, or the velocity could not be sent.
     */
    ELITE_EXPORT bool writeSpeedl(const vector6d_t& vel, int timeout_ms, int32_t user_frame_id);

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
     * @brief Add or update an SDK-managed user frame from an id and pose.
     *
     * The pose is stored by the SDK and synchronized to the External Control
     * script through the script command socket.
     *
     * @param frame_id User frame slot id in the range [0, max_user_frame_count).
     *                 The id selects which SDK-managed user frame is added or updated.
     * @param pose User frame pose expressed relative to the base frame.
     * @return true The frame was accepted and the update was sent successfully.
     * @return false The frame id is invalid or the update could not be sent.
     */
    ELITE_EXPORT bool setUserFrame(int32_t frame_id, const vector6d_t& pose);

    /**
     * @brief Add or update an SDK-managed user frame from a UserFrame object.
     *
     * This overload uses the object's id and pose fields. The name and valid
     * fields are not updated by this call.
     *
     * @param frame User frame containing the id and pose to synchronize. The
     *              id must be in the range [0, max_user_frame_count), and the
     *              pose must be expressed relative to the base frame.
     * @return true The frame was accepted and the update was sent successfully.
     * @return false The frame id is invalid or the update could not be sent.
     */
    ELITE_EXPORT bool setUserFrame(const UserFrame& frame);

    /**
     * @brief Get an SDK-managed user frame by id.
     *
     * This function reads the SDK-managed cache and does not query the
     * teach pendant's user-frame table.
     *
     * @param frame_id User frame slot id to query, in the range
     *                 [0, max_user_frame_count).
     * @param frame Output user frame.
     * @return true A valid user frame was found.
     * @return false No valid user frame was found for the specified id.
     */
    ELITE_EXPORT bool getUserFrame(int32_t frame_id, UserFrame& frame) const;

    /**
     * @brief Get all SDK-managed user frames.
     *
     * The returned vector is a copy of the SDK-managed cache.
     *
     * @return The currently configured user frames. Returns an empty vector
     *         when no user frames are configured.
     */
    ELITE_EXPORT std::vector<UserFrame> getUserFrames() const;

    /**
     * @brief Select the default user frame for current-frame overloads.
     *
     * The selected frame is used by Cartesian overloads that do not receive an
     * explicit user frame id. This includes the no-frame-id overloads of
     * writeServoj(), writeSpeedl(), and both Cartesian
     * writeTrajectoryPoint() overloads. For writeTrajectoryPoint(), the
     * active frame is used only when cartesian is true. Joint commands,
     * writeSpeedj(), and joint-mode writeTrajectoryPoint() are not affected.
     * The default value is -1, which represents the base frame.
     *
     * @param user_frame_id -1 to select the base frame, or a configured and valid
     *                        SDK-managed user frame id in the range
     *                        [0, max_user_frame_count).
     * @return true The active frame was selected successfully.
     * @return false The frame id is invalid or the user frame is not configured.
     */
    ELITE_EXPORT bool setActiveUserFrame(int32_t user_frame_id);

    /**
     * @brief Get the default user frame used by current-frame overloads.
     *
     * @return The active user frame id. -1 represents the base frame; a
     *         non-negative value identifies an SDK-managed user frame slot.
     */
    ELITE_EXPORT int32_t getActiveUserFrame() const;

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
     * @brief Register a callback for robot-based trajectory feedback frames.
     *
     * @param cb Callback function that will be triggered when the robot reports trajectory progress.
     */
    ELITE_EXPORT void setTrajectoryFeedbackCallback(std::function<void(const TrajectoryMotionFeedback&)> cb);

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
     * @brief Write a timed trajectory point in a specified user frame.
     *
     * The user frame is used only when cartesian is true. Joint positions are
     * independent of the selected user frame.
     *
     * @param positions Desired joint or Cartesian positions.
     * @param time Time for the robot to reach this point.
     * @param blend_radius The radius to be used for blending between control points.
     * @param cartesian True if positions are Cartesian, false if joint-based.
     * @param user_frame_id -1 for the base frame, or a user frame id in the range
     *                       [0, max_user_frame_count).
     * @return true Trajectory point sent successfully.
     * @return false The frame id or other input is invalid, or the point could not be sent.
     */
    ELITE_EXPORT bool writeTrajectoryPoint(const vector6d_t& positions, float time, float blend_radius, bool cartesian,
                                           int32_t user_frame_id);

    /**
     * @brief Writes a trajectory point onto the dedicated socket.
     *
     * @param positions Desired joint or cartesian positions
     * @param blend_radius The radius to be used for blending between control points
     * @param cartesian True, if the point sent is cartesian, false if joint-based
     * @param speed Joint speed for movej or TCP speed for movel
     * @param acceleration Joint acceleration for movej or TCP acceleration for movel
     * @return true Trajectory point sent successfully.
     * @return false Fail to send trajectory point.
     */
    ELITE_EXPORT bool writeTrajectoryPoint(const vector6d_t& positions, float blend_radius, bool cartesian, float speed,
                                           float acceleration);

    /**
     * @brief Write a speed-planned trajectory point in a specified user frame.
     *
     * The time is fixed to zero. The user frame is used only when cartesian
     * is true. Joint positions are independent of the selected user frame.
     *
     * @param positions Desired joint or Cartesian positions.
     * @param blend_radius The radius to be used for blending between control points.
     * @param cartesian True if positions are Cartesian, false if joint-based.
     * @param speed Joint speed for movej or TCP speed for movel.
     * @param acceleration Joint acceleration for movej or TCP acceleration for movel.
     * @param user_frame_id -1 for the base frame, or a user frame id in the range
     *                       [0, max_user_frame_count).
     * @return true Trajectory point sent successfully.
     * @return false The frame id or other input is invalid, or the point could not be sent.
     */
    ELITE_EXPORT bool writeTrajectoryPoint(const vector6d_t& positions, float blend_radius, bool cartesian, float speed,
                                           float acceleration, int32_t user_frame_id);

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
     * @brief Enable or disable collision detection.
     *
     * @param enable true to enable, false to disable
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool setCollisionDetectEnabled(bool enable);

    /**
     * @brief Set collision detection sensitivity.
     *
     * @param ratio Sensitivity ratio in percent. Valid range is [10, 100].
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool setCollisionSensitivity(int32_t ratio);

    /**
     * @brief Set robot mounting plane by dynamically adjusting the gravity direction.
     *
     * @param z_rotation Rotation angle around the robot base Z axis, in radians.
     * @param tilt Mounting plane tilt angle, in radians.
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool setMountingPlane(double z_rotation, double tilt = 0.0);

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
     * @brief Get the primary port interface owned by this driver.
     *
     * The returned object is created and connected by EliteDriver during construction. Its lifetime is managed by EliteDriver, so do
     * not store the reference beyond the lifetime of the driver.
     *
     * @return PrimaryPortInterface& Primary port interface.
     */
    ELITE_EXPORT PrimaryPortInterface& primaryPort();

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
     * @brief Start tool RS485 communication.
     * This function will start a socat process on the robot control cabinet, mapping the serial port to the TCP port you specified.
     * If you want to use this feature, it is recommended to install libssh. If you are using it on a non-Linux system, you must
     * install the libssh library.
     *
     * @param config Serial configuration
     * @param ssh_password SSH password for robot control cabinet
     * @param tcp_port Socat TCP port
     * @return SerialCommunicationSharedPtr A TCP communication object for RS485 communication. nullptr if start fail.
     */
    ELITE_EXPORT SerialCommunicationSharedPtr startToolRs485(const SerialConfig& config, const std::string& ssh_password,
                                                             int tcp_port = 54321);

    /**
     * @brief End tool RS485 communication
     * If you want to use this feature, it is recommended to install libssh. If you are using it on a non-Linux system, you must
     * install the libssh library.
     *
     * @param com TCP communication object for RS485 communication.
     * @param ssh_password SSH password for robot control cabinet
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool endToolRs485(SerialCommunicationSharedPtr com, const std::string& ssh_password);

    /**
     * @brief Start board RS485 communication.
     * This function will start a socat process on the robot control cabinet, mapping the serial port to the TCP port you specified.
     * If you want to use this feature, it is recommended to install libssh. If you are using it on a non-Linux system, you must
     * install the libssh library.
     *
     * @param config Serial configuration
     * @param ssh_password SSH password for robot control cabinet
     * @param tcp_port Socat TCP port
     * @return SerialCommunicationSharedPtr A TCP communication object for RS485 communication. nullptr if start fail.
     */
    ELITE_EXPORT SerialCommunicationSharedPtr startBoardRs485(const SerialConfig& config, const std::string& ssh_password,
                                                              int tcp_port = 54322);

    /**
     * @brief End board RS485 communication
     * If you want to use this feature, it is recommended to install libssh. If you are using it on a non-Linux system, you must
     * install the libssh library.
     *
     * @param com TCP communication object for RS485 communication.
     * @param ssh_password SSH password for robot control cabinet
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool endBoardRs485(SerialCommunicationSharedPtr com, const std::string& ssh_password);
};

}  // namespace ELITE

#endif
