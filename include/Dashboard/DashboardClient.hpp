// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// DashboardClient.hpp
// Provides an interface to interact with the robot's dashboard server.
#ifndef __DASHBOARDCLIENT_HPP__
#define __DASHBOARDCLIENT_HPP__

#include <Elite/DataType.hpp>
#include <Elite/EliteException.hpp>
#include <Elite/EliteOptions.hpp>

#include <chrono>
#include <memory>
#include <string>

namespace ELITE {

class DashboardClient {
   public:
    ELITE_EXPORT explicit DashboardClient();
    ELITE_EXPORT virtual ~DashboardClient();

    /**
     * @brief Connect to dashboard server
     *
     * @param ip The IP of dashboard server
     * @param port The IP of dashboard server port
     * @return true connected success
     * @return false connected fail
     */
    ELITE_EXPORT bool connect(const std::string& ip, int port = 29999);

    /**
     * @brief Disconnect
     *
     */
    ELITE_EXPORT void disconnect();

    /**
     * @brief Brake release
     *
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool brakeRelease();

    /**
     * @brief Close safety dialog
     *
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool closeSafetyDialog();

    /**
     * @brief The echo command is used to check the connection status with the dashboard shell server
     *
     * @return true Robot response success
     */
    ELITE_EXPORT bool echo();

    /**
     * @brief Get help string
     *
     * @param cmd The command need help
     * @return std::string The help string of command
     */
    ELITE_EXPORT std::string help(const std::string& cmd);

    /**
     * @brief Add a log message
     * @verbatim
     *  If message inclue '\n' or '\r', will be replace with "\\n" and "\\r"
     * @endverbatim
     *
     * @param message Log content
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool log(const std::string& message);

    /**
     * @brief Pop up a message box that displays the given text or closes the message box that was recently ordered by popup
     *
     * @param arg
     *      "-c": close message box
     *      "-s": pop up message box
     * @param message
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool popup(const std::string& arg, const std::string& message = "");

    /**
     * @brief Quit dashboard and disconnect
     *
     */
    ELITE_EXPORT void quit();

    /**
     * @brief Reboot robot and disconnect
     *
     */
    ELITE_EXPORT void reboot();

    /**
     * @brief Get robot type
     *
     * @return string Robot type
     */
    [[deprecated(
        "This interface is deprecated. Please use robotType() instead. This function will be removed in September "
        "2027.")]] ELITE_EXPORT std::string
    robot();

    /**
     * @brief Get robot type
     *
     * @return string Robot type
     */
    ELITE_EXPORT std::string robotType();

    /**
     * @brief Get robot serial number
     *
     * @return string Robot serial number
     */
    ELITE_EXPORT std::string robotSerialNumber();

    /**
     * @brief Get robot ID
     *
     * @return string Robot ID
     */
    ELITE_EXPORT std::string robotID();

    /**
     * @brief Robot power-on
     *
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool powerOn();

    /**
     * @brief Robot power-off
     *
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool powerOff();

    /**
     * @brief Robot shutdown and disconnect
     *
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT void shutdown();

    /**
     * @brief Get whether remote control mode is enabled.
     *
     * @return true enabled
     * @return false disabled
     */
    ELITE_EXPORT bool remoteControl();

    /**
     * @brief Enable or disable remote control mode.
     *
     * @param enable true to enable, false to disable
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool remoteControl(bool enable);

    /**
     * @brief Get the current control mode.
     *
     * @return RemoteControlMode
     */
    ELITE_EXPORT RemoteControlMode remoteControlMode();

    /**
     * @brief Gets the robot speed scaling percentage
     *
     * @return int Robot speed scaling percentage
     */
    ELITE_EXPORT int speedScaling();

    /**
     * @brief Get robot mode
     *
     * @return RobotMode robot mode
     */
    ELITE_EXPORT RobotMode robotMode();

    /**
     * @brief Get safety mode
     *
     * @return SafetyMode safety mode
     */
    ELITE_EXPORT SafetyMode safetyMode();

    /**
     * @brief Restart safety system
     *
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool safetySystemRestart();

    /**
     * @brief Get task status
     *
     * @return TaskStatus task status
     */
    ELITE_EXPORT TaskStatus runningStatus();

    /**
     * @brief Unlock robot protective stop
     *
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool unlockProtectiveStop();

    /**
     * @brief Query all or given command usage of the dashboard shell
     *
     * @param cmd Command
     * @return string Command usage
     */
    ELITE_EXPORT std::string usage(const std::string& cmd);

    /**
     * @brief Get dashboard version infomation
     *
     * @return string Dashboard version infomation
     */
    ELITE_EXPORT std::string version();

    /**
     * @brief Load robot configuration
     *
     * @param path
     * @return ELITE_EXPORT
     */
    ELITE_EXPORT bool loadConfiguration(const std::string& path);

    /**
     * @brief Get current configuration path
     *
     * @return string The path of configuration
     */
    ELITE_EXPORT std::string configurationPath();

    /**
     * @brief Configuration is modify
     *
     * @return true modified
     */
    ELITE_EXPORT bool isConfigurationModify();

    /**
     * @brief Play program
     *
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool playProgram();

    /**
     * @brief Pause program
     *
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool pauseProgram();

    /**
     * @brief Stop program
     *
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool stopProgram();

    /**
     * @brief Set the speed scaling
     *
     * @param scaling
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool setSpeedScaling(int scaling);

    /**
     * @brief Get the current task path
     *
     * @return string task relative path
     */
    ELITE_EXPORT std::string getTaskPath();

    /**
     * @brief Load task
     *
     * @param path task path
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool loadTask(const std::string& path);

    /**
     * @brief Get the task status
     *
     * @return TaskStatus
     */
    ELITE_EXPORT TaskStatus getTaskStatus();

    /**
     * @brief Is task running
     *
     * @return true running
     */
    ELITE_EXPORT bool taskIsRunning();

    /**
     * @brief Task save status
     *
     * @return true saved
     */
    ELITE_EXPORT bool isTaskSaved();

    /**
     * @brief Send a dashboard command and receive a response.
     *
     * @param cmd Dashboard command
     * @return std::string Response
     */
    ELITE_EXPORT std::string sendAndReceive(const std::string& cmd);

   private:
    class Impl;
    std::unique_ptr<Impl> impl_;

    std::string asyncReadLine(unsigned timeout_ms = 10000);
    void sendCommand(const std::string& cmd);

    std::string sendAndRequest(const std::string& cmd, const std::string& expected = "");
    bool waitForReply(const std::string& cmd, const std::string& expected,
                      const std::chrono::duration<double> timeout = std::chrono::seconds(30));
};

}  // namespace ELITE

#endif
