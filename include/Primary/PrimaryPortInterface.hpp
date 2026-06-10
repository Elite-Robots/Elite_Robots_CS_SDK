// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// PrimaryPortInterface.hpp
// Provides the PrimaryPortInterface class for robot primary port communication.
#ifndef __ELITE__PRIMARY_PORT_INTERFACE_HPP__
#define __ELITE__PRIMARY_PORT_INTERFACE_HPP__


#include <Elite/EliteOptions.hpp>
#include <Elite/PrimaryPackage.hpp>
#include <Elite/RobotException.hpp>
#include <functional>
#include <memory>
#include <string>

namespace ELITE {

/**
 * @brief Robot primary port interface
 *
 */
class PrimaryPortInterface {
   private:
    class Impl;
    std::unique_ptr<Impl> impl_;

   public:
    static constexpr int PRIMARY_PORT = 30001;

    ELITE_EXPORT PrimaryPortInterface();
    ELITE_EXPORT ~PrimaryPortInterface();

    /**
     * @brief Connect to robot primary port.
     *  And spawn a background thread for message receiving and parsing.
     * @param ip The robot ip
     * @param port The port(30001)
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool connect(const std::string& ip, int port = PRIMARY_PORT);

    /**
     * @brief Disconnect socket.
     *  And wait for the background thread to finish.
     */
    ELITE_EXPORT void disconnect();

    /**
     * @brief Sends a custom script program to the robot.
     *
     * @param script Script code that shall be executed by the robot.
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool sendScript(const std::string& script);

    /**
     * @brief Power on the robot through the primary port.
     *
     * Sends the controller script command `power on`.
     *
     * @return true script written successfully and robot mode becomes IDLE or RUNNING
     * @return false failed to write script or target state was not reached
     */
    ELITE_EXPORT bool powerOn();

    /**
     * @brief Power off the robot through the primary port.
     *
     * Sends the controller script command `power off`.
     *
     * @return true script written successfully and robot mode becomes POWER_OFF
     * @return false failed to write script or target state was not reached
     */
    ELITE_EXPORT bool powerOff();

    /**
     * @brief Release robot brakes through the primary port.
     *
     * Sends the controller script command `set robotmode run`.
     *
     * @return true script written successfully and robot mode becomes RUNNING
     * @return false failed to write script or target state was not reached
     */
    ELITE_EXPORT bool brakeRelease();

    /**
     * @brief Pause the running task through the primary port.
     *
     * Sends the controller script command `pause task`.
     *
     * @return true script written successfully and runtime state becomes PAUSED
     * @return false failed to write script or target state was not reached
     */
    ELITE_EXPORT bool pauseProgram();

    /**
     * @brief Stop the running task through the primary port.
     *
     * Sends the controller script command `stop task`.
     *
     * @return true script written successfully and runtime state becomes STOPPED
     * @return false failed to write script or target state was not reached
     */
    ELITE_EXPORT bool stopProgram();

    /**
     * @brief Unlock protective stop through the primary port.
     *
     * Sends the controller script command `set unlock protective stop`.
     *
     * @return true script written successfully and safety status is not PROTECTIVE_STOP
     * @return false failed to write script or target state was not reached
     */
    ELITE_EXPORT bool unlockProtectiveStop();

    /**
     * @brief Restart the safety board through the primary port.
     *
     * Sends the controller script command `restart safetyboard`.
     *
     * @return true script written successfully and safety status becomes NORMAL
     * @return false failed to write script or target state was not reached
     */
    ELITE_EXPORT bool safetySystemRestart();

    /**
     * @brief Set the target speed scaling percentage through the primary port.
     *
     * Sends the controller script command `set speed <scaling / 100.0>`.
     *
     * @param scaling Target speed percentage.
     * @return true script written successfully and target speed scaling is confirmed
     * @return false failed to write script or target state was not reached
     */
    ELITE_EXPORT bool setSpeedScaling(int scaling);

    /**
     * @brief Get primary sub-package data.
     *
     * @param pkg Primary sub-package.
     * @param timeout_ms Wait time
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool getPackage(std::shared_ptr<PrimaryPackage> pkg, int timeout_ms);

    /**
     * @brief Get the local IP
     *
     * @return std::string Local IP. If empty, connection had some errors.
     */
    ELITE_EXPORT std::string getLocalIP();

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
};

}  // namespace ELITE

#endif
