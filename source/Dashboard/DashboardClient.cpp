// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
#include "DashboardClient.hpp"
#include <Common/BoostAsioCompat.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <mutex>
#include <regex>
#include <thread>
#include "DataType.hpp"
#include "Log.hpp"

using namespace ELITE;
using namespace std::chrono_literals;

class DashboardClient::Impl {
   public:
    std::mutex socket_mutex_;
    BoostIoContext io_context_;
    std::unique_ptr<boost::asio::ip::tcp::socket> socket_ptr_;
    std::unique_ptr<boost::asio::ip::tcp::resolver> resolver_ptr_;

    void disconnect();
};

DashboardClient::DashboardClient() { impl_ = std::make_unique<Impl>(); }

DashboardClient::~DashboardClient() {}

bool DashboardClient::connect(const std::string& ip, int port) {
    bool ret_val = false;
    try {
        std::lock_guard<std::mutex> lock(impl_->socket_mutex_);
        impl_->socket_ptr_.reset(new boost::asio::ip::tcp::socket(impl_->io_context_));
        impl_->resolver_ptr_.reset(new boost::asio::ip::tcp::resolver(impl_->io_context_));
        impl_->socket_ptr_->open(boost::asio::ip::tcp::v4());
        boost::asio::ip::tcp::no_delay no_delay_option(true);
        impl_->socket_ptr_->set_option(no_delay_option);
        boost::asio::socket_base::reuse_address sol_reuse_option(true);
        impl_->socket_ptr_->set_option(sol_reuse_option);
#if defined(__linux) || defined(linux) || defined(__linux__)
        boost::asio::detail::socket_option::boolean<IPPROTO_TCP, TCP_QUICKACK> quickack(true);
        impl_->socket_ptr_->set_option(quickack);
#endif
        boost::asio::ip::tcp::endpoint endpoint(makeBoostAddress(ip), port);
        boost::system::error_code ec = boost::asio::error::would_block;

        impl_->socket_ptr_->async_connect(endpoint, [&](const boost::system::error_code& error) {
            ec = error;
            if (ec) {
                ELITE_LOG_ERROR("Dashboard connect to robot fail: %s", boost::system::system_error(ec).what());
                ret_val = false;
            } else {
                ret_val = true;
            }
        });

        do {
            impl_->io_context_.run_one();
            // TODO: When all tasks in the io_context are completed, the io_context enters a 'stopped' state.
            // How can we prevent the io_context from entering the 'stopped' state without using the 'restart' method?
            if (impl_->io_context_.stopped()) {
                restartBoostIoContext(impl_->io_context_);
            }
        } while (ec == boost::asio::error::would_block);

    } catch (const boost::system::system_error& error) {
        ELITE_LOG_ERROR("Dashboard connect to robot fail: %s", error.what());
        throw EliteException(EliteException::Code::SOCKET_CONNECT_FAIL, error.what());
    }
    asyncReadLine();
    return ret_val;
}

void DashboardClient::disconnect() {
    std::lock_guard<std::mutex> lock(impl_->socket_mutex_);
    impl_->disconnect();
}

void DashboardClient::Impl::disconnect() { socket_ptr_.reset(); }

bool DashboardClient::brakeRelease() {
    std::string response = sendAndRequest("brakeRelease\n", "Brake (Releasing.*|is released).*");
    if (response.empty()) {
        return false;
    }
    return waitForReply("robotMode\n", "robotMode: RUNNING\r\n");
}

bool DashboardClient::closeSafetyDialog() {
    std::string response = sendAndRequest("closeSafetyDialog\n", "closing .* dialog\r\n");
    return !response.empty();
}

bool DashboardClient::echo() {
    std::string response = sendAndRequest("echo\n", "Hello ELITE ROBOTS.\r\n");
    return !response.empty();
}

std::string DashboardClient::help(const std::string& cmd) {
    std::string send_string = "help " + cmd + '\n';
    return sendAndRequest(send_string);
}

bool DashboardClient::log(const std::string& message) {
    std::string message_cpy = message;
    size_t found = message_cpy.find("\n");
    while (found != std::string::npos) {
        message_cpy.replace(found, 1, "\\n");
        found = message_cpy.find("\n", found);
    }

    found = message_cpy.find("\r");
    while (found != std::string::npos) {
        message_cpy.replace(found, 1, "\\r");
        found = message_cpy.find("\r", found);
    }

    std::string send_string = "log -a " + message_cpy + "\n";
    std::string response = sendAndRequest(send_string, "Log has been added.\r\n");
    return !response.empty();
}

bool DashboardClient::popup(const std::string& arg, const std::string& message) {
    std::string send_string;
    if (arg == "-c") {
        send_string = "popup " + arg + "\n";
    } else if (arg == "-s") {
        send_string = "popup " + arg + message + "\n";
    } else {
        throw EliteException(EliteException::Code::ILLEGAL_PARAM, "dashboard popup command");
    }
    std::string response = sendAndRequest(send_string, "Closing popup\r\n|Showing popup with text:.*\\s*");
    return !response.empty();
}

void DashboardClient::quit() {
    sendAndRequest("quit\n");
    impl_->disconnect();
}

void DashboardClient::reboot() {
    sendAndRequest("reboot\n");
    impl_->disconnect();
}

std::string DashboardClient::robot() { return sendAndRequest("robot -t\n"); }

std::string DashboardClient::robotType() {
    return sendAndRequest("robot -t\n");
}

std::string DashboardClient::robotSerialNumber() {
    return sendAndRequest("robot -s\n");
}

std::string DashboardClient::robotID() {
    return sendAndRequest("robot -id\n");
}

bool DashboardClient::powerOn() {
    std::string response = sendAndRequest("robotControl -on\n", "Powering on\r\n");
    return waitForReply("robotMode\n", "robotMode: (RUNNING|IDLE)\r\n");
}

bool DashboardClient::powerOff() {
    std::string response = sendAndRequest("robotControl -off\n", "Powering off\r\n");
    // Beacuse of robot after power off need time to
    // complete some operation (robot still return "POWER_OFF" by "robotMode" command), delay there
    std::this_thread::sleep_for(500ms);
    return waitForReply("robotMode\n", "robotMode: POWER_OFF\r\n");
}

void DashboardClient::shutdown() {
    sendAndRequest("shutdown\n");
    impl_->disconnect();
}

int DashboardClient::speedScaling() {
    std::string request = sendAndRequest("status\n", "Target Speed Fraction:.*");
    std::size_t pos = request.find(": ");
    request = request.substr(pos + 2);
    return std::stoi(request);
}

RobotMode DashboardClient::robotMode() {
    std::string request = sendAndRequest("robotMode\n", "robotMode:.*");
    std::size_t pos = request.find(": ");
    std::string mode = request.substr(pos + 2);
    if (mode == "NO_CONTROLLER") {
        return RobotMode::NO_CONTROLLER;
    } else if (mode == "DISCONNECTED") {
        return RobotMode::DISCONNECTED;
    } else if (mode == "CONFIRM_SAFETY") {
        return RobotMode::CONFIRM_SAFETY;
    } else if (mode == "BOOTING") {
        return RobotMode::BOOTING;
    } else if (mode == "POWER_OFF") {
        return RobotMode::POWER_OFF;
    } else if (mode == "POWER_ON") {
        return RobotMode::POWER_ON;
    } else if (mode == "IDLE") {
        return RobotMode::IDLE;
    } else if (mode == "BACK_DRIVE") {
        return RobotMode::BACKDRIVE;
    } else if (mode == "RUNNING") {
        return RobotMode::RUNNING;
    } else if (mode == "UPDATING") {
        return RobotMode::UPDATING_FIRMWARE;
    } else if (mode == "WAITING_CALIBRATION") {
        return RobotMode::WAITING_CALIBRATION;
    } else {
        return RobotMode::UNKNOWN;
    }
}

SafetyMode DashboardClient::safetyMode() {
    std::string request = sendAndRequest("safety -s\n", "Safety status:.*");
    std::size_t pos = request.find(": ");
    std::string status = request.substr(pos + 2);
    if (status == "NORMAL") {
        return SafetyMode::NORMAL;
    } else if (status == "REDUCED") {
        return SafetyMode::REDUCED;
    } else if (status == "PROTECTIVE_STOP") {
        return SafetyMode::PROTECTIVE_STOP;
    } else if (status == "RECOVERY") {
        return SafetyMode::RECOVERY;
    } else if (status == "SAFEGUARD_STOP") {
        return SafetyMode::SAFEGUARD_STOP;
    } else if (status == "SYSTEM_EMERGENCY_STOP") {
        return SafetyMode::SYSTEM_EMERGENCY_STOP;
    } else if (status == "ROBOT_EMERGENCY_STOP") {
        return SafetyMode::ROBOT_EMERGENCY_STOP;
    } else if (status == "VIOLATION") {
        return SafetyMode::VIOLATION;
    } else if (status == "FAULT") {
        return SafetyMode::FAULT;
    } else if (status == "VALIDATE_JOINT_ID") {
        return SafetyMode::VALIDATE_JOINT_ID;
    } else if (status == "UNDEFINED_SAFETY_MODE") {
        return SafetyMode::UNDEFINED_SAFETY_MODE;
    } else if (status == "AUTOMATIC_MODE_SAFEGUARD_STOP") {
        return SafetyMode::AUTOMATIC_MODE_SAFEGUARD_STOP;
    } else if (status == "SYSTEM_THREE_POSITION_ENABLING_STOP") {
        return SafetyMode::SYSTEM_THREE_POSITION_ENABLING_STOP;
    } else if (status == "TP_THREE_POSITION_ENABLING_STOP") {
        return SafetyMode::TP_THREE_POSITION_ENABLING_STOP;
    } else {
        return SafetyMode::UNKNOWN;
    }
}

bool DashboardClient::safetySystemRestart() {
    sendAndRequest("safety -r\n", "Restarting safety board.*");
    return waitForReply("safety -m\n", "Safety mode: NORMAL\r\n");
}

TaskStatus DashboardClient::runningStatus() {
    std::string request = sendAndRequest("status\n", "RunningStatus:.*");
    std::size_t pos = request.find(": ");
    std::string status = request.substr(pos + 2);
    if (status.find("STOP") != std::string::npos) {
        return TaskStatus::STOPPED;
    } else if (status.find("RUNNING") != std::string::npos) {
        return TaskStatus::PLAYING;
    } else if (status.find("PAUSE") != std::string::npos) {
        return TaskStatus::PAUSED;
    }
    return TaskStatus::STOPPED;
}

bool DashboardClient::unlockProtectiveStop() {
    std::string response = sendAndRequest("unlockProtectiveStop\n", "Protective stop unlocking...\r\n");
    return !response.empty();
}

std::string DashboardClient::usage(const std::string& cmd) { return sendAndRequest("usage " + cmd + "\n"); }

std::string DashboardClient::version() {
    std::string request = sendAndRequest("version\n");
    return request;
}

bool DashboardClient::loadConfiguration(const std::string& path) {
    std::string send_command = "configuration -p " + path + "\n";
    std::string response = sendAndRequest(send_command, "Loading Configuration :.*");
    if (response.empty()) {
        return false;
    }
    return waitForReply("configuration\n", "configuration: Relative path:" + path + "\r\n");
}

std::string DashboardClient::configurationPath() {
    std::string request = sendAndRequest("configuration\n", "configuration: Relative path:.*");
    std::size_t pos = request.find("Relative path:");
    return request.substr(pos + (sizeof("Relative path:") - 1));
}

bool DashboardClient::isConfigurationModify() {
    std::string request = sendAndRequest("configuration -s\n");
    if (request.find("not modified") != std::string::npos) {
        return false;
    } else {
        return true;
    }
}

bool DashboardClient::playProgram() {
    std::string request = sendAndRequest("play\n");
    if (request != "Starting task\r\n") {
        return false;
    }
    return waitForReply("task -s\n", "Task is running\r\n");
}

bool DashboardClient::pauseProgram() {
    std::string request = sendAndRequest("pause\n");
    if (request != "Pausing task\r\n") {
        return false;
    }
    return waitForReply("task -s\n", "Task is paused\r\n");
}

bool DashboardClient::setSpeedScaling(int scaling) {
    std::string send_command = "speed -v " + std::to_string(scaling) + "\n";
    sendAndRequest(send_command);
    // Give some time for the command to take effect
    std::this_thread::sleep_for(200ms);
    return (speedScaling() == scaling);
}

bool DashboardClient::stopProgram() {
    std::string response = sendAndRequest("stop\n");
    if (response != "Stopping task\r\n") {
        return false;
    }
    return waitForReply("task -s\n", "Task is stopped\r\n");
}

std::string DashboardClient::getTaskPath() {
    std::string request = sendAndRequest("task\n");
    const char* kw = "Relative path:";
    constexpr size_t kw_size = sizeof("Relative path:") - 1;
    std::size_t pos = request.find(kw);
    if (pos != std::string::npos) {
        // Exclude prefixes and newline characters.
        return request.substr(pos + kw_size, request.length() - kw_size - 2);
    }
    return request;
}

bool DashboardClient::loadTask(const std::string& path) {
    std::string send_command = "task -p " + path + "\n";
    sendAndRequest(send_command, "Loaded task: .*");
    return waitForReply("task\n", "Relative path:" + path + "\r\n");
}

TaskStatus DashboardClient::getTaskStatus() {
    std::string status_str = sendAndRequest("task -s\n", "Task is .*");
    if (status_str.find("stopped") != std::string::npos) {
        return TaskStatus::STOPPED;
    } else if (status_str.find("paused") != std::string::npos) {
        return TaskStatus::PAUSED;
    } else if (status_str.find("running") != std::string::npos) {
        return TaskStatus::PLAYING;
    } else {
        return TaskStatus::STOPPED;
    }
}

bool DashboardClient::taskIsRunning() {
    std::string request = sendAndRequest("task -r\n", "Task is .*");
    if (request.find("not running") != std::string::npos) {
        return false;
    } else if (request.find("is running") != std::string::npos) {
        return true;
    }
    return false;
}

bool DashboardClient::isTaskSaved() {
    std::string response = sendAndRequest("task -ss\n", "Task is .*");
    if (response == "Task is saved") {
        return true;
    } else {
        return false;
    }
}

std::string DashboardClient::sendAndReceive(const std::string& cmd) {
    if (cmd.back() != '\n') {
        sendCommand(cmd + "\n");
    } else {
        sendCommand(cmd);
    }
    return asyncReadLine();
}

std::string DashboardClient::asyncReadLine(unsigned timeout_ms) {
    boost::system::error_code ec = boost::asio::error::would_block;
    boost::asio::streambuf stream_buffer;
    boost::asio::async_read_until(*impl_->socket_ptr_, stream_buffer, '\n',
                                  [&](const boost::system::error_code& error, std::size_t nb) { ec = error; });

    do {
        runBoostIoContextFor(impl_->io_context_, std::chrono::milliseconds(timeout_ms));
        // TODO: When all tasks in the io_context are completed, the io_context enters a 'stopped' state.
        // How can we prevent the io_context from entering the 'stopped' state without using the 'restart' method?
        if (impl_->io_context_.stopped()) {
            restartBoostIoContext(impl_->io_context_);
        }
    } while (ec == boost::asio::error::would_block);
    if (ec) {
        throw EliteException(EliteException::Code::SOCKET_FAIL, ec.message());
    }
    std::string line(boost::asio::buffers_begin(stream_buffer.data()), boost::asio::buffers_end(stream_buffer.data()));
    return line;
}

void DashboardClient::sendCommand(const std::string& cmd) {
    boost::system::error_code ec;
    impl_->socket_ptr_->send(boost::asio::buffer(cmd), 0, ec);
    if (ec) {
        throw EliteException(EliteException::Code::SOCKET_FAIL, ec.message());
    }
}

std::string DashboardClient::sendAndRequest(const std::string& cmd, const std::string& expected) {
    std::lock_guard<std::mutex> lock(impl_->socket_mutex_);
    if (!impl_->socket_ptr_) {
        ELITE_LOG_ERROR("Dashboard not connect to robot");
        return "";
    }
    sendCommand(cmd);
    std::string response = asyncReadLine();
    if (!expected.empty()) {
        std::smatch match;
        bool ret = std::regex_search(response, match, std::regex(expected));
        if (!ret) {
            throw EliteException(
                EliteException::Code::DASHBOARD_NOT_EXPECT_RECIVE,
                "Dashboard command \"" + cmd + "\" response expected: " + expected + ". But received: " + response);
            return std::string();
        }
        return match[0];
    } else {
        return response;
    }
}

bool DashboardClient::waitForReply(const std::string& cmd, const std::string& expected,
                                   const std::chrono::duration<double> timeout) {
    const std::chrono::duration<double> wait_period = 100ms;
    std::chrono::duration<double> time_done(0);
    std::string response;
    while (time_done < timeout) {
        response = sendAndRequest(cmd);
        if (std::regex_match(response, std::regex(expected))) {
            return true;
        }
        // wait 100ms before trying again
        std::this_thread::sleep_for(wait_period);
        time_done += wait_period;
    }
    return false;
}
