// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
#include "PrimaryPort.hpp"
#include "EliteException.hpp"
#include "Log.hpp"
#include "Utils.hpp"

#include <cmath>
#include <functional>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <vector>

using namespace std::chrono;

namespace ELITE {
using namespace std::chrono;

namespace {

bool waitFor(const std::function<bool()>& predicate, milliseconds timeout, milliseconds period = 100ms) {
    const auto deadline = steady_clock::now() + timeout;
    while (steady_clock::now() < deadline) {
        if (predicate()) {
            return true;
        }
        std::this_thread::sleep_for(period);
    }
    return predicate();
}

bool asBool(uint8_t value) { return value != 0; }

RobotMode toRobotMode(uint8_t mode) {
    switch (mode) {
        case 0:
            return RobotMode::DISCONNECTED;
        case 1:
            return RobotMode::CONFIRM_SAFETY;
        case 2:
            return RobotMode::BOOTING;
        case 3:
            return RobotMode::POWER_OFF;
        case 4:
            return RobotMode::POWER_ON;
        case 5:
            return RobotMode::IDLE;
        case 6:
            return RobotMode::BACKDRIVE;
        case 7:
            return RobotMode::RUNNING;
        case 8:
            return RobotMode::UPDATING_FIRMWARE;
        case 9:
            return RobotMode::WAITING_CALIBRATION;
        default:
            return RobotMode::UNKNOWN;
    }
}

SafetyMode toSafetyMode(uint8_t mode) {
    switch (mode) {
        case 1:
            return SafetyMode::NORMAL;
        case 2:
            return SafetyMode::REDUCED;
        case 3:
            return SafetyMode::PROTECTIVE_STOP;
        case 4:
            return SafetyMode::RECOVERY;
        case 5:
            return SafetyMode::SAFEGUARD_STOP;
        case 6:
            return SafetyMode::SYSTEM_EMERGENCY_STOP;
        case 7:
            return SafetyMode::ROBOT_EMERGENCY_STOP;
        case 8:
            return SafetyMode::VIOLATION;
        case 9:
            return SafetyMode::FAULT;
        case 10:
            return SafetyMode::VALIDATE_JOINT_ID;
        case 11:
            return SafetyMode::UNDEFINED_SAFETY_MODE;
        case 12:
            return SafetyMode::AUTOMATIC_MODE_SAFEGUARD_STOP;
        case 13:
            return SafetyMode::SYSTEM_THREE_POSITION_ENABLING_STOP;
        case 14:
            return SafetyMode::TP_THREE_POSITION_ENABLING_STOP;
        default:
            return SafetyMode::UNKNOWN;
    }
}

}  // namespace

PrimaryPort::PrimaryPort() { message_head_.resize(HEAD_LENGTH); }

PrimaryPort::~PrimaryPort() { disconnect(); }

bool PrimaryPort::connect(const std::string& ip, int port) {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (!socketConnect(ip, port)) {
        return false;
    }
    resetStateData();
    if (!socket_async_thread_) {
        // Start async thread
        socket_async_thread_alive_.store(true);
        socket_async_thread_.reset(new std::thread([&](std::string ip, int port) { socketAsyncLoop(ip, port); }, ip, port));
    }
    return true;
}

void PrimaryPort::disconnect() {
    // Close socket and set thread flag
    {
        std::lock_guard<std::mutex> lock(socket_mutex_);
        socket_async_thread_alive_.store(false);
        socketDisconnect();
        socket_ptr_.reset();
    }
    if (socket_async_thread_ && socket_async_thread_->joinable()) {
        socket_async_thread_->join();
    }
    socket_async_thread_.reset();
}

bool PrimaryPort::sendScript(const std::string& script) {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (!socket_ptr_) {
        ELITE_LOG_ERROR("Don't connect to robot primary port");
        return false;
    }
    auto script_with_newline = std::make_shared<std::string>(script + "\n");
    boost::system::error_code ec;
    socket_ptr_->write_some(boost::asio::buffer(*script_with_newline), ec);
    if (ec) {
        ELITE_LOG_ERROR("Send script to robot fail : %s", boost::system::system_error(ec).what());
        return false;
    } else {
        return true;
    }
}

bool PrimaryPort::powerOn() {
    RobotMode mode = getPrimaryRobotMode();
    if (mode == RobotMode::IDLE || mode == RobotMode::RUNNING) {
        return true;
    }
    if (!sendScript("power on")) {
        return false;
    }
    return waitFor([&]() {
        RobotMode current_mode = getPrimaryRobotMode();
        return current_mode == RobotMode::IDLE || current_mode == RobotMode::RUNNING;
    }, 30s);
}

bool PrimaryPort::powerOff() {
    if (!sendScript("power off")) {
        return false;
    }
    std::this_thread::sleep_for(500ms);
    return waitFor([&]() { return getPrimaryRobotMode() == RobotMode::POWER_OFF; }, 30s);
}

bool PrimaryPort::brakeRelease() {
    if (!waitForRobotModeData(2s)) {
        ELITE_LOG_ERROR("Primary brake release failed: no RobotModeData received from primary port");
        return false;
    }

    RobotMode mode = getPrimaryRobotMode();
    if (mode == RobotMode::RUNNING) {
        return true;
    }
    if (mode != RobotMode::IDLE) {
        ELITE_LOG_ERROR("Primary brake release requires robot mode IDLE. Current mode: %d", static_cast<int>(mode));
        return false;
    }

    SafetyMode safety = getPrimarySafetyMode();
    if (safety != SafetyMode::NORMAL && safety != SafetyMode::REDUCED && safety != SafetyMode::RECOVERY) {
        ELITE_LOG_ERROR("Primary brake release safety mode not allowed: %d", static_cast<int>(safety));
        return false;
    }

    if (!sendScript("set robotmode run")) {
        return false;
    }
    return waitFor([&]() { return getPrimaryRobotMode() == RobotMode::RUNNING; }, 30s);
}

bool PrimaryPort::pauseProgram() {
    if (!waitForRobotModeData(2s)) {
        ELITE_LOG_ERROR("Primary pause task failed: no RobotModeData received from primary port");
        return false;
    }
    TaskStatus status = getPrimaryRunningStatus();
    if (status == TaskStatus::PAUSED) {
        return true;
    }
    if (status != TaskStatus::PLAYING) {
        ELITE_LOG_ERROR("Primary pause task requires runtime state PLAYING. Current state: %d", static_cast<int>(status));
        return false;
    }
    if (!sendScript("pause task")) {
        return false;
    }
    return waitFor([&]() { return getPrimaryRunningStatus() == TaskStatus::PAUSED; }, 30s);
}

bool PrimaryPort::stopProgram() {
    if (!waitForRobotModeData(2s)) {
        ELITE_LOG_ERROR("Primary stop task failed: no RobotModeData received from primary port");
        return false;
    }
    if (getPrimaryRunningStatus() == TaskStatus::STOPPED) {
        return true;
    }
    if (!sendScript("stop task")) {
        return false;
    }
    return waitFor([&]() { return getPrimaryRunningStatus() == TaskStatus::STOPPED; }, 30s);
}

bool PrimaryPort::unlockProtectiveStop() {
    if (!waitForRobotModeData(2s)) {
        ELITE_LOG_ERROR("Primary unlock protective stop failed: no RobotModeData received from primary port");
        return false;
    }
    if (!sendScript("set unlock protective stop")) {
        return false;
    }
    return waitFor([&]() { return !getRobotModeDataSnapshot().is_robot_protective_stopped; }, 5s);
}

bool PrimaryPort::safetySystemRestart() {
    if (!waitForRobotModeData(2s)) {
        ELITE_LOG_ERROR("Primary safety system restart failed: no RobotModeData received from primary port");
        return false;
    }
    if (!sendScript("restart safetyboard")) {
        return false;
    }
    std::this_thread::sleep_for(500ms);
    return waitFor([&]() { return getPrimarySafetyMode() == SafetyMode::NORMAL; }, 30s);
}

bool PrimaryPort::setSpeedScaling(int scaling) {
    if (!waitForRobotModeData(2s)) {
        ELITE_LOG_ERROR("Primary set speed failed: no RobotModeData received from primary port");
        return false;
    }

    std::ostringstream script;
    script << "set speed " << std::fixed << std::setprecision(2) << (static_cast<double>(scaling) / 100.0);
    if (!sendScript(script.str())) {
        return false;
    }
    return waitFor([&]() { return getPrimarySpeedScaling() == scaling; }, 3s);
}

void PrimaryPort::parserRobotModeData(int len, const std::vector<uint8_t>::const_iterator& iter) {
    if (len < ROBOT_MODE_DATA_PKG_LENGTH) {
        ELITE_LOG_ERROR("Primary RobotModeData package len error: %d", len);
        return;
    }

    RobotModeData data;
    int offset = 0;

    offset += sizeof(uint32_t);  // mode_sub_len
    offset += sizeof(uint8_t);   // mode_sub_type

    EndianUtils::unpack(iter + offset, data.timestamp);
    offset += sizeof(uint64_t);

    offset += sizeof(uint8_t);  // reserved
    offset += sizeof(uint8_t);  // reserved

    data.is_robot_power_on = asBool(*(iter + offset));
    offset += sizeof(uint8_t);

    data.is_emergency_stopped = asBool(*(iter + offset));
    offset += sizeof(uint8_t);

    data.is_robot_protective_stopped = asBool(*(iter + offset));
    offset += sizeof(uint8_t);

    data.is_task_running = asBool(*(iter + offset));
    offset += sizeof(uint8_t);

    data.is_task_paused = asBool(*(iter + offset));
    offset += sizeof(uint8_t);

    data.robot_mode = toRobotMode(*(iter + offset));
    offset += sizeof(uint8_t);

    data.robot_control_mode = *(iter + offset);
    offset += sizeof(uint8_t);

    EndianUtils::unpack(iter + offset, data.target_speed_fraction);
    offset += sizeof(double);

    EndianUtils::unpack(iter + offset, data.speed_scaling);
    offset += sizeof(double);

    EndianUtils::unpack(iter + offset, data.target_speed_fraction_limit);
    offset += sizeof(double);

    data.robot_speed_mode = *(iter + offset);
    offset += sizeof(uint8_t);

    offset += sizeof(uint8_t);  // reserved

    data.is_in_package_mode = asBool(*(iter + offset));

    {
        std::lock_guard<std::mutex> lock(robot_mode_data_mutex_);
        robot_mode_data_ = data;
        robot_mode_data_received_ = true;
    }
    robot_mode_data_cv_.notify_all();
}

void PrimaryPort::parserMasterBoardData(int len, const std::vector<uint8_t>::const_iterator& iter) {
    if (len < MASTER_BOARD_DATA_STATUS_MIN_LENGTH) {
        ELITE_LOG_ERROR("Primary MasterBoardData package len error: %d", len);
        return;
    }

    MasterBoardData data;
    int offset = MASTER_BOARD_DATA_STATUS_OFFSET;

    data.safety_mode = toSafetyMode(*(iter + offset));
    offset += sizeof(uint8_t);

    data.is_robot_in_reduced_mode = asBool(*(iter + offset));
    offset += sizeof(uint8_t);

    data.operational_mode_selector_input = asBool(*(iter + offset));
    offset += sizeof(uint8_t);

    data.threeposition_enabling_device_input = asBool(*(iter + offset));
    offset += sizeof(uint8_t);

    data.internal_use = *(iter + offset);

    {
        std::lock_guard<std::mutex> lock(master_board_data_mutex_);
        master_board_data_ = data;
        master_board_data_received_ = true;
    }
    master_board_data_cv_.notify_all();
}

RobotMode PrimaryPort::getPrimaryRobotMode() {
    if (!waitForRobotModeData(500ms)) {
        return RobotMode::UNKNOWN;
    }
    return getRobotModeDataSnapshot().robot_mode;
}

int PrimaryPort::getPrimarySpeedScaling() {
    if (!waitForRobotModeData(500ms)) {
        return 0;
    }
    return static_cast<int>(std::round(getRobotModeDataSnapshot().target_speed_fraction * 100.0));
}

SafetyMode PrimaryPort::getPrimarySafetyMode() {
    if (!waitForMasterBoardData(500ms)) {
        return SafetyMode::UNKNOWN;
    }
    return getMasterBoardDataSnapshot().safety_mode;
}

TaskStatus PrimaryPort::getPrimaryRunningStatus() {
    if (!waitForRobotModeData(500ms)) {
        return TaskStatus::UNKNOWN;
    }
    auto state = getRobotModeDataSnapshot();
    if (state.is_task_paused) {
        return TaskStatus::PAUSED;
    }
    if (state.is_task_running) {
        return TaskStatus::PLAYING;
    }
    return TaskStatus::STOPPED;
}

bool PrimaryPort::waitForRobotModeData(milliseconds timeout) {
    std::unique_lock<std::mutex> lock(robot_mode_data_mutex_);
    return robot_mode_data_cv_.wait_for(lock, timeout, [&]() { return robot_mode_data_received_; });
}

bool PrimaryPort::waitForMasterBoardData(milliseconds timeout) {
    std::unique_lock<std::mutex> lock(master_board_data_mutex_);
    return master_board_data_cv_.wait_for(lock, timeout, [&]() { return master_board_data_received_; });
}

PrimaryPort::RobotModeData PrimaryPort::getRobotModeDataSnapshot() {
    std::lock_guard<std::mutex> lock(robot_mode_data_mutex_);
    return robot_mode_data_;
}

PrimaryPort::MasterBoardData PrimaryPort::getMasterBoardDataSnapshot() {
    std::lock_guard<std::mutex> lock(master_board_data_mutex_);
    return master_board_data_;
}

void PrimaryPort::resetStateData() {
    {
        std::lock_guard<std::mutex> state_lock(robot_mode_data_mutex_);
        robot_mode_data_ = RobotModeData{};
        robot_mode_data_received_ = false;
    }
    {
        std::lock_guard<std::mutex> state_lock(master_board_data_mutex_);
        master_board_data_ = MasterBoardData{};
        master_board_data_received_ = false;
    }
}

bool PrimaryPort::getPackage(std::shared_ptr<PrimaryPackage> pkg, int timeout_ms) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        parser_sub_msg_.insert({pkg->getType(), pkg});
    }

    return pkg->waitUpdate(timeout_ms);
}

bool PrimaryPort::parserMessage() {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (!socket_ptr_ || !socket_ptr_->is_open()) {
        return false;
    }
    // Receive package head and parser it
    boost::system::error_code ec;
    int head_len = boost::asio::read(*socket_ptr_, boost::asio::buffer(message_head_, HEAD_LENGTH), ec);
    if (ec) {
        if (ec == boost::asio::error::would_block) {
            // Data not ready, non blocking mode returns normally
            return true;
        } else {
            ELITE_LOG_ERROR("Primary port receive package head had expection: %s", boost::system::system_error(ec).what());
            return false;
        }
    }
    uint32_t package_len = 0;
    EndianUtils::unpack(message_head_.begin(), package_len);
    if (package_len <= HEAD_LENGTH) {
        ELITE_LOG_ERROR("Primary port package len error: %d", package_len);
        return false;
    }

    return parserMessageBody(message_head_[4], package_len);
}

RobotErrorSharedPtr PrimaryPort::parserRobotError(uint64_t timestamp, RobotError::Source source,
                                                  const std::vector<uint8_t>& msg_body, int offset) {
    int32_t code = 0;
    EndianUtils::unpack(msg_body.begin() + offset, code);
    offset += sizeof(int32_t);

    int32_t sub_code = 0;
    EndianUtils::unpack(msg_body.begin() + offset, sub_code);
    offset += sizeof(int32_t);

    int32_t level = 0;
    EndianUtils::unpack(msg_body.begin() + offset, level);
    offset += sizeof(int32_t);

    uint32_t data_type;
    EndianUtils::unpack(msg_body.begin() + offset, data_type);
    offset += sizeof(uint32_t);

    switch ((RobotError::DataType)data_type) {
        case RobotError::DataType::NONE:
        case RobotError::DataType::UNSIGNED:
        case RobotError::DataType::HEX: {
            uint32_t data;
            EndianUtils::unpack(msg_body.begin() + offset, data);
            return std::make_shared<RobotError>(timestamp, code, sub_code, static_cast<RobotError::Source>(source),
                                                static_cast<RobotError::Level>(level), static_cast<RobotError::DataType>(data_type),
                                                data);

        } break;
        case RobotError::DataType::SIGNED:
        case RobotError::DataType::JOINT: {
            int32_t data;
            EndianUtils::unpack(msg_body.begin() + offset, data);
            return std::make_shared<RobotError>(timestamp, code, sub_code, static_cast<RobotError::Source>(source),
                                                static_cast<RobotError::Level>(level), static_cast<RobotError::DataType>(data_type),
                                                data);
        } break;
        case RobotError::DataType::STRING: {
            std::string data;
            for (size_t i = offset; i < msg_body.size(); i++) {
                data.push_back(msg_body[i]);
            }

            return std::make_shared<RobotError>(timestamp, code, sub_code, static_cast<RobotError::Source>(source),
                                                static_cast<RobotError::Level>(level), static_cast<RobotError::DataType>(data_type),
                                                data);
        } break;
        case RobotError::DataType::FLOAT: {
            float data;
            EndianUtils::unpack(msg_body.begin() + offset, data);
            return std::make_shared<RobotError>(timestamp, code, sub_code, static_cast<RobotError::Source>(source),
                                                static_cast<RobotError::Level>(level), static_cast<RobotError::DataType>(data_type),
                                                data);
        } break;
    }
    return nullptr;
}

RobotRuntimeExceptionSharedPtr PrimaryPort::paraserRuntimeException(uint64_t timestamp, const std::vector<uint8_t>& msg_body,
                                                                    int offset) {
    int32_t line;
    EndianUtils::unpack(msg_body.begin() + offset, line);
    offset += sizeof(int32_t);

    int32_t column;
    EndianUtils::unpack(msg_body.begin() + offset, column);
    offset += sizeof(int32_t);

    std::string text_msg;
    for (size_t i = offset; i < msg_body.size(); i++) {
        text_msg.push_back(msg_body[i]);
    }

    return std::make_shared<RobotRuntimeException>(timestamp, line, column, std::move(text_msg));
}

RobotExceptionSharedPtr PrimaryPort::parserException(const std::vector<uint8_t>& msg_body) {
    uint64_t timestamp;
    int offset = 0;
    EndianUtils::unpack<uint64_t>(msg_body.begin(), timestamp);
    offset += sizeof(uint64_t);

    // Only robot error message
    RobotError::Source source = static_cast<RobotError::Source>(msg_body[offset]);
    offset++;

    RobotException::Type type = static_cast<RobotException::Type>(msg_body[offset]);
    offset++;

    if (type == RobotException::Type::ROBOT_ERROR) {
        return parserRobotError(timestamp, source, msg_body, offset);
    } else if (type == RobotException::Type::SCRIPT_RUNTIME) {
        return paraserRuntimeException(timestamp, msg_body, offset);
    } else {
        return nullptr;
    }
}

bool PrimaryPort::parserMessageBody(int type, int package_len) {
    boost::system::error_code ec;
    int body_len = package_len - HEAD_LENGTH;
    int read_len = 0;
    message_body_.resize(body_len);

    // Receive package body
    boost::asio::async_read(*socket_ptr_, boost::asio::buffer(message_body_, body_len),
                            [&](boost::system::error_code error, std::size_t n) {
                                ec = error;
                                read_len = n;
                            });
    if (io_context_.stopped()) {
        io_context_.restart();
    }
    io_context_.run_for(500ms);
    if (ec) {
        ELITE_LOG_ERROR("Primary port receive package body had expection: %s", boost::system::system_error(ec).what());
        return false;
    }
    if (read_len != body_len) {
        ELITE_LOG_ERROR("Primary port receive package body data length not match. Receive:%d, expect:%d", read_len, body_len);
        return false;
    }

    // If the asynchronous operation completed successfully then the io_context
    // would have been stopped due to running out of work. If it was not
    // stopped, then the io_context::run_for call must have timed out.
    if (!io_context_.stopped()) {
        ELITE_LOG_ERROR("Primary port receive package body timeout");
        io_context_.stop();
        return false;
    }
    // If RobotState message parser others don't do anything.
    if (type == ROBOT_STATE_MSG_TYPE) {
        uint32_t sub_len = 0;
        for (auto iter = message_body_.begin(); iter < message_body_.end(); iter += sub_len) {
            EndianUtils::unpack(iter, sub_len);
            const auto remaining_len = static_cast<size_t>(std::distance(iter, message_body_.end()));
            if (sub_len <= HEAD_LENGTH || sub_len > remaining_len) {
                ELITE_LOG_ERROR("Primary port sub-package len error: %d", sub_len);
                return false;
            }
            int sub_type = *(iter + 4);

            if (sub_type == ROBOT_MODE_DATA_PKG_TYPE) {
                parserRobotModeData(sub_len, iter);
            } else if (sub_type == MASTER_BOARD_DATA_PKG_TYPE) {
                parserMasterBoardData(sub_len, iter);
            }

            std::lock_guard<std::mutex> lock(mutex_);
            auto psm = parser_sub_msg_.find(sub_type);
            if (psm != parser_sub_msg_.end()) {
                psm->second->parser(sub_len, iter);
                psm->second->notifyUpated();
                parser_sub_msg_.erase(sub_type);
            }
        }
    } else if (type == ROBOT_EXCEPTION_MSG_TYPE) {
        if (robot_exception_cb_) {
            RobotExceptionSharedPtr ex = parserException(message_body_);
            if (ex) {
                robot_exception_cb_(ex);
            }
        }
    }
    return true;
}

bool PrimaryPort::socketReconnect(const std::string& ip, int port, bool is_last_connect_success) {
    // Disconnect and reconnect
    std::lock_guard<std::mutex> lock(socket_mutex_);
    socketDisconnect();
    bool connected = socketConnect(ip, port, is_last_connect_success);
    if (connected) {
        resetStateData();
    }
    return connected;
}

void PrimaryPort::socketAsyncLoop(const std::string& ip, int port) {
    bool is_last_connect_success = true;
    while (socket_async_thread_alive_.load()) {
        try {
            if (!parserMessage()) {
                auto now = std::chrono::system_clock::now();
                auto duration = now.time_since_epoch();
                auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
                auto ex = std::make_shared<RobotException>(RobotException::Type::ROBOT_DISCONNECTED, timestamp);
                if (robot_exception_cb_ && is_last_connect_success) {
                    robot_exception_cb_(ex);
                }
                is_last_connect_success = socketReconnect(ip, port, is_last_connect_success);
            }
            std::this_thread::sleep_for(10ms);
        } catch (const std::exception& e) {
            ELITE_LOG_ERROR("Primary port async loop throw exception:%s", e.what());
        }
    }
}

bool PrimaryPort::socketConnect(const std::string& ip, int port, bool is_last_connect_success) {
    try {
        socket_ptr_.reset(new boost::asio::ip::tcp::socket(io_context_));
        socket_ptr_->open(boost::asio::ip::tcp::v4());
        socket_ptr_->set_option(boost::asio::ip::tcp::no_delay(true));
        socket_ptr_->set_option(boost::asio::socket_base::reuse_address(true));
        socket_ptr_->set_option(boost::asio::socket_base::keep_alive(false));
        socket_ptr_->non_blocking(true);
#if defined(__linux) || defined(linux) || defined(__linux__)
        socket_ptr_->set_option(boost::asio::detail::socket_option::boolean<IPPROTO_TCP, TCP_QUICKACK>(true));
#endif
        boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::make_address(ip), port);
        boost::system::error_code connect_ec;
        socket_ptr_->async_connect(endpoint, [&](const boost::system::error_code& ec) { connect_ec = ec; });
        if (io_context_.stopped()) {
            io_context_.restart();
        }
        io_context_.run_for(500ms);
        if (connect_ec) {
            socket_ptr_.reset();
            if (is_last_connect_success) {
                ELITE_LOG_ERROR("Connect to robot primary port fail: %s", boost::system::system_error(connect_ec).what());
            }
            return false;
        }
        // If the asynchronous operation completed successfully then the io_context
        // would have been stopped due to running out of work. If it was not
        // stopped, then the io_context::run_for call must have timed out.
        if (!io_context_.stopped()) {
            if (is_last_connect_success) {
                ELITE_LOG_ERROR("Connect to robot primary port fail: timeout");
            }
            socketDisconnect();
            socket_ptr_.reset();
            io_context_.stop();
            return false;
        }

    } catch (const boost::system::system_error& error) {
        throw EliteException(EliteException::Code::SOCKET_CONNECT_FAIL, error.what());
        return false;
    }
    return true;
}

void PrimaryPort::socketDisconnect() {
    if (socket_ptr_ && socket_ptr_->is_open()) {
        try {
            boost::system::error_code ignore_ec;
            socket_ptr_->cancel(ignore_ec);
            socket_ptr_->shutdown(boost::asio::ip::tcp::socket::shutdown_both, ignore_ec);
            socket_ptr_->close(ignore_ec);
            if (io_context_.stopped()) {
                io_context_.restart();
            }
            io_context_.run_for(500ms);
        } catch (const std::exception& e) {
            ELITE_LOG_WARN("Primary port socket disconnect throw exception:%s", e.what());
        }
    }
}

std::string PrimaryPort::getLocalIP() {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (socket_ptr_ && socket_ptr_->is_open()) {
        boost::system::error_code ignore_ec;
        auto address = socket_ptr_->local_endpoint(ignore_ec).address().to_string();
        if (!ignore_ec) {
            return address;
        }
    }
    return "";
}

}  // namespace ELITE
