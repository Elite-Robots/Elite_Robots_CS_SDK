// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
#include "EliteDriver.hpp"
#include <boost/asio.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include "ControlCommon.hpp"
#include "ControlMode.hpp"
#include "EliteException.hpp"
#include "Log.hpp"
#include "PrimaryPortInterface.hpp"
#include "ReverseInterface.hpp"
#include "ScriptCommandInterface.hpp"
#include "ScriptSender.hpp"
#include "TcpServer.hpp"
#include "TrajectoryInterface.hpp"
#include "SerialCommunicationImpl.hpp"
#include "SshUtils.hpp"

using namespace ELITE;
using namespace std::chrono;

static const std::string SERVER_IP_REPLACE = "{{SERVER_IP_REPLACE}}";
static const std::string REVERSE_PORT_REPLACE = "{{REVERSE_PORT_REPLACE}}";
static const std::string SCRIPT_COMMAND_PORT_REPLACE = "{{SCRIPT_COMMAND_PORT_REPLACE}}";
static const std::string TRAJECTORY_SERVER_PORT_REPLACE = "{{TRAJECTORY_SERVER_PORT_REPLACE}}";
static const std::string SERVO_J_REPLACE = "{{SERVO_J_REPLACE}}";
static const std::string POS_ZOOM_RATIO_REPLACE = "{{POS_ZOOM_RATIO_REPLACE}}";
static const std::string TIME_ZOOM_RATIO_REPLACE = "{{TIME_ZOOM_RATIO_REPLACE}}";
static const std::string COMMON_ZOOM_RATIO_REPLACE = "{{COMMON_ZOOM_RATIO_REPLACE}}";
static const std::string REVERSE_DATA_SIZE_REPLACE = "{{REVERSE_DATA_SIZE_REPLACE}}";
static const std::string TRAJECTORY_DATA_SIZE_REPLACE = "{{TRAJECTORY_DATA_SIZE_REPLACE}}";
static const std::string SCRIPT_COMMAND_DATA_SIZE_REPLACE = "{{SCRIPT_COMMAND_DATA_SIZE_REPLACE}}";
static const std::string STOP_J_REPLACE = "{{STOP_J_REPLACE}}";
static const std::string SERVOJ_TIME_REPLACE = "{{SERVOJ_TIME_REPLACE}}";

class EliteDriver::Impl {
   public:
    Impl() = delete;
    explicit Impl(const std::string& robot_ip) : robot_ip_(robot_ip) {
        reverse_resource_ = std::make_shared<TcpServer::StaticResource>();
    }
    ~Impl() {
        reverse_server_.reset();
        trajectory_server_.reset();
        script_command_server_.reset();
        script_sender_.reset();
        // Must release resource after all servers are destroyed.
        reverse_resource_->shutdown();
        reverse_resource_.reset();
    }

    std::string readScriptFile(const std::string& file);
    void scriptParamWrite(std::string& file_string, const EliteDriverConfig& config);
    int getSocatPid(const std::string& ssh_password, int port);
    std::string robot_script_;
    std::string robot_ip_;
    std::string local_ip_;
    std::unique_ptr<ReverseInterface> reverse_server_;
    std::unique_ptr<TrajectoryInterface> trajectory_server_;
    std::unique_ptr<ScriptSender> script_sender_;
    std::unique_ptr<ScriptCommandInterface> script_command_server_;
    std::unique_ptr<PrimaryPortInterface> primary_port_;
    bool headless_mode_;

    std::shared_ptr<TcpServer::StaticResource> reverse_resource_;
};

std::string EliteDriver::Impl::readScriptFile(const std::string& filepath) {
    std::ifstream ifs;
    ifs.open(filepath);
    if (!ifs) {
        std::stringstream ss;
        ss << "Elite script file '" << filepath << "' doesn't exists.";
        throw EliteException(EliteException::Code::FILE_OPEN_FAIL, ss.str().c_str());
        return std::string();
    }
    std::string content((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));
    ifs.close();
    return content;
}

void EliteDriver::Impl::scriptParamWrite(std::string& file_string, const EliteDriverConfig& config) {
    while (file_string.find(SERVER_IP_REPLACE) != std::string::npos) {
        file_string.replace(file_string.find(SERVER_IP_REPLACE), SERVER_IP_REPLACE.length(), local_ip_);
    }

    while (file_string.find(TRAJECTORY_SERVER_PORT_REPLACE) != std::string::npos) {
        file_string.replace(file_string.find(TRAJECTORY_SERVER_PORT_REPLACE), TRAJECTORY_SERVER_PORT_REPLACE.length(),
                            std::to_string(config.trajectory_port));
    }

    while (file_string.find(REVERSE_PORT_REPLACE) != std::string::npos) {
        file_string.replace(file_string.find(REVERSE_PORT_REPLACE), REVERSE_PORT_REPLACE.length(),
                            std::to_string(config.reverse_port));
    }

    while (file_string.find(SCRIPT_COMMAND_PORT_REPLACE) != std::string::npos) {
        file_string.replace(file_string.find(SCRIPT_COMMAND_PORT_REPLACE), SCRIPT_COMMAND_PORT_REPLACE.length(),
                            std::to_string(config.script_command_port));
    }

    std::ostringstream servoj_replace_str;
    servoj_replace_str << "lookahead_time = " << config.servoj_lookahead_time << ", gain=" << config.servoj_gain;
    while (file_string.find(SERVO_J_REPLACE) != std::string::npos) {
        file_string.replace(file_string.find(SERVO_J_REPLACE), SERVO_J_REPLACE.length(), servoj_replace_str.str());
    }

    while (file_string.find(SERVOJ_TIME_REPLACE) != std::string::npos) {
        file_string.replace(file_string.find(SERVOJ_TIME_REPLACE), SERVOJ_TIME_REPLACE.length(),
                            std::to_string(config.servoj_time));
    }

    while (file_string.find(POS_ZOOM_RATIO_REPLACE) != std::string::npos) {
        file_string.replace(file_string.find(POS_ZOOM_RATIO_REPLACE), POS_ZOOM_RATIO_REPLACE.length(),
                            std::to_string(CONTROL::POS_ZOOM_RATIO));
    }

    while (file_string.find(TIME_ZOOM_RATIO_REPLACE) != std::string::npos) {
        file_string.replace(file_string.find(TIME_ZOOM_RATIO_REPLACE), TIME_ZOOM_RATIO_REPLACE.length(),
                            std::to_string(CONTROL::TIME_ZOOM_RATIO));
    }

    while (file_string.find(COMMON_ZOOM_RATIO_REPLACE) != std::string::npos) {
        file_string.replace(file_string.find(COMMON_ZOOM_RATIO_REPLACE), COMMON_ZOOM_RATIO_REPLACE.length(),
                            std::to_string(CONTROL::COMMON_ZOOM_RATIO));
    }

    while (file_string.find(REVERSE_DATA_SIZE_REPLACE) != std::string::npos) {
        file_string.replace(file_string.find(REVERSE_DATA_SIZE_REPLACE), REVERSE_DATA_SIZE_REPLACE.length(),
                            std::to_string(ReverseInterface::REVERSE_DATA_SIZE));
    }

    while (file_string.find(TRAJECTORY_DATA_SIZE_REPLACE) != std::string::npos) {
        file_string.replace(file_string.find(TRAJECTORY_DATA_SIZE_REPLACE), TRAJECTORY_DATA_SIZE_REPLACE.length(),
                            std::to_string(TrajectoryInterface::TRAJECTORY_MESSAGE_LEN));
    }

    while (file_string.find(SCRIPT_COMMAND_DATA_SIZE_REPLACE) != std::string::npos) {
        file_string.replace(file_string.find(SCRIPT_COMMAND_DATA_SIZE_REPLACE), SCRIPT_COMMAND_DATA_SIZE_REPLACE.length(),
                            std::to_string(ScriptCommandInterface::SCRIPT_COMMAND_DATA_SIZE));
    }

    while (file_string.find(STOP_J_REPLACE) != std::string::npos) {
        file_string.replace(file_string.find(STOP_J_REPLACE), STOP_J_REPLACE.length(), std::to_string(config.stopj_acc));
    }
}

void EliteDriver::init(const EliteDriverConfig& config) {
    ELITE_LOG_DEBUG("Initialization Elite Driver");

    impl_ = std::make_unique<EliteDriver::Impl>(config.robot_ip);

    // First, need to connect to the robot primary port before attempting to obtain the local IP address
    ELITE_LOG_DEBUG("Connecting to robot primary port %s ...", config.robot_ip.c_str());
    impl_->primary_port_ = std::make_unique<PrimaryPortInterface>();
    if (!impl_->primary_port_->connect(impl_->robot_ip_, PrimaryPortInterface::PRIMARY_PORT)) {
        ELITE_LOG_FATAL("Connect robot primary port fail.");
        throw EliteException(EliteException::Code::SOCKET_CONNECT_FAIL, "Connect robot primary port fail.");
    }
    if (config.local_ip.length() <= 0) {
        impl_->local_ip_ = impl_->primary_port_->getLocalIP();
    } else {
        impl_->local_ip_ = config.local_ip;
    }
    ELITE_LOG_DEBUG("Connected to robot primary port.");

    // Generate external control script.
    std::string control_script = impl_->readScriptFile(config.script_file_path);
    ELITE_LOG_DEBUG("Read script file '%s' success.", config.script_file_path.c_str());
    impl_->scriptParamWrite(control_script, config);

    impl_->reverse_server_ = std::make_unique<ReverseInterface>(config.reverse_port, impl_->reverse_resource_);
    ELITE_LOG_DEBUG("Created reverse interface");
    impl_->trajectory_server_ = std::make_unique<TrajectoryInterface>(config.trajectory_port, impl_->reverse_resource_);
    ELITE_LOG_DEBUG("Created trajectory interface");
    impl_->script_command_server_ = std::make_unique<ScriptCommandInterface>(config.script_command_port, impl_->reverse_resource_);
    ELITE_LOG_DEBUG("Created script command interface");

    impl_->headless_mode_ = config.headless_mode;

    if (impl_->headless_mode_) {
        impl_->robot_script_ += "def externalControl():\n";
        std::istringstream control_script_stream(control_script);
        std::string line;
        while (std::getline(control_script_stream, line)) {
            impl_->robot_script_ += "\t" + line + "\n";
        }
        impl_->robot_script_ += "end";

        if (sendExternalControlScript()) {
            ELITE_LOG_DEBUG("Sent external control script to robot.");
        } else {
            ELITE_LOG_DEBUG("Send external control script to robot fail.");
        }
    } else {
        impl_->robot_script_ = control_script;
        impl_->script_sender_ =
            std::make_unique<ScriptSender>(config.script_sender_port, impl_->robot_script_, impl_->reverse_resource_);
        ELITE_LOG_DEBUG("Created script sender");
    }

    ELITE_LOG_DEBUG("Initialization done");
}

EliteDriver::EliteDriver(const EliteDriverConfig& config) { init(config); }

EliteDriver::EliteDriver(const std::string& robot_ip, const std::string& local_ip, const std::string& script_file,
                         bool headless_mode, int script_sender_port, int reverse_port, int trajectory_port, int script_command_port,
                         float servoj_time, float servoj_lookahead_time, int servoj_gain, float stopj_acc) {
    EliteDriverConfig config;
    config.robot_ip = robot_ip;
    config.local_ip = local_ip;
    config.script_file_path = script_file;
    config.headless_mode = headless_mode;
    config.script_sender_port = script_sender_port;
    config.reverse_port = reverse_port;
    config.trajectory_port = trajectory_port;
    config.script_command_port = script_command_port;
    config.servoj_time = servoj_time;
    config.servoj_lookahead_time = servoj_lookahead_time;
    config.servoj_gain = servoj_gain;
    config.stopj_acc = stopj_acc;
    init(config);
}

EliteDriver::~EliteDriver() { impl_.reset(); }

bool EliteDriver::writeServoj(const vector6d_t& pos, int timeout_ms, bool cartesian){
    if (cartesian) {
        return impl_->reverse_server_->writeJointCommand(pos, ControlMode::MODE_POSE, timeout_ms);
    } else {
        return impl_->reverse_server_->writeJointCommand(pos, ControlMode::MODE_SERVOJ, timeout_ms);
    }
}

bool EliteDriver::writeSpeedl(const vector6d_t& vel, int timeout_ms) {
    return impl_->reverse_server_->writeJointCommand(vel, ControlMode::MODE_SPEEDL, timeout_ms);
}

bool EliteDriver::writeSpeedj(const vector6d_t& vel, int timeout_ms) {
    return impl_->reverse_server_->writeJointCommand(vel, ControlMode::MODE_SPEEDJ, timeout_ms);
}

void EliteDriver::setTrajectoryResultCallback(std::function<void(TrajectoryMotionResult)> cb) {
    impl_->trajectory_server_->setMotionResultCallback(cb);
}

bool EliteDriver::writeTrajectoryPoint(const vector6d_t& positions, float time, float blend_radius, bool cartesian) {
    return impl_->trajectory_server_->writeTrajectoryPoint(positions, time, blend_radius, cartesian);
}

bool EliteDriver::writeTrajectoryControlAction(TrajectoryControlAction action, const int point_number, int robot_receive_timeout) {
    return impl_->reverse_server_->writeTrajectoryControlAction(action, point_number, robot_receive_timeout);
}

bool EliteDriver::writeFreedrive(FreedriveAction action, int timeout_ms) {
    return impl_->reverse_server_->writeFreedrive(action, timeout_ms);
}

bool EliteDriver::stopControl(int wait_ms) {
    if (wait_ms < 5) {
        wait_ms = 5;
    }
    if (!impl_->reverse_server_->stopControl()) {
        return false;
    }

    auto start_time = std::chrono::steady_clock::now();
    auto timeout = std::chrono::milliseconds(wait_ms);
    while (impl_->script_command_server_->isRobotConnect() || impl_->reverse_server_->isRobotConnect()) {
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed >= timeout) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return !isRobotConnected();
}

bool EliteDriver::writeIdle(int timeout_ms) {
    return impl_->reverse_server_->writeJointCommand(nullptr, ControlMode::MODE_IDLE, timeout_ms);
}

void EliteDriver::printRobotScript() { std::cout << impl_->robot_script_ << std::endl; }

bool EliteDriver::isRobotConnected() {
    return impl_->reverse_server_->isRobotConnect() && impl_->trajectory_server_->isRobotConnect() &&
           impl_->script_command_server_->isRobotConnect();
}

bool EliteDriver::zeroFTSensor() { return impl_->script_command_server_->zeroFTSensor(); }

bool EliteDriver::setPayload(double mass, const vector3d_t& cog) { return impl_->script_command_server_->setPayload(mass, cog); }

bool EliteDriver::setToolVoltage(const ToolVoltage& vol) { return impl_->script_command_server_->setToolVoltage(vol); }

bool EliteDriver::startForceMode(const vector6d_t& reference_frame, const vector6int32_t& selection_vector,
                                 const vector6d_t& wrench, const ForceMode& mode, const vector6d_t& limits) {
    return impl_->script_command_server_->startForceMode(reference_frame, selection_vector, wrench, mode, limits);
}

bool EliteDriver::endForceMode() { return impl_->script_command_server_->endForceMode(); }

bool EliteDriver::sendScript(const std::string& script) {
    if (!impl_->primary_port_) {
        ELITE_LOG_ERROR("Not connect to robot primary port");
        return false;
    }
    return impl_->primary_port_->sendScript(script);
}

bool EliteDriver::sendExternalControlScript() {
    if (!impl_->headless_mode_) {
        ELITE_LOG_ERROR("Not in headless mode");
        return false;
    }
    return sendScript(impl_->robot_script_);
}

bool EliteDriver::getPrimaryPackage(std::shared_ptr<PrimaryPackage> pkg, int timeout_ms) {
    if (!impl_->primary_port_) {
        ELITE_LOG_ERROR("Not connect to robot primary port");
        return false;
    }
    return impl_->primary_port_->getPackage(pkg, timeout_ms);
}

bool EliteDriver::primaryReconnect() {
    impl_->primary_port_->disconnect();
    return impl_->primary_port_->connect(impl_->robot_ip_);
}

void EliteDriver::registerRobotExceptionCallback(std::function<void(RobotExceptionSharedPtr)> cb) {
    impl_->primary_port_->registerRobotExceptionCallback(cb);
}

int EliteDriver::Impl::getSocatPid(const std::string& ssh_password, int port) {
    auto ps_result =
        SSH_UTILS::executeCommand(robot_ip_, "root", ssh_password, "ps| grep \"[s]ocat tcp-l:"+ std::to_string(port) + "\" | awk '{print $1}'");
    ELITE_LOG_DEBUG("Socat port %d PID:  %s", port, ps_result.c_str());
    if (ps_result.empty()) {
        return -1;
    } else {
        try {
            return std::stoi(ps_result);
        } catch(const std::exception& e) {
            ELITE_LOG_ERROR("Convert socat PID fail 'ps' string: %s. exception: %s", ps_result.c_str(), e.what());
            return -1;
        }
    }
}

SerialCommunicationSharedPtr EliteDriver::startToolRs485(const SerialConfig& config, const std::string& ssh_password,
                                                         int tcp_port) {
    if (!impl_->primary_port_) {
        ELITE_LOG_ERROR("Not connect to robot primary port");
        return nullptr;
    }
    int socat_pid = impl_->getSocatPid(ssh_password, tcp_port);
    if (socat_pid > 0) {
        return std::make_shared<SerialCommunicationImpl>(tcp_port, impl_->robot_ip_, socat_pid);
    }

    std::string baud_rate = std::to_string(static_cast<int>(config.baud_rate));
    std::string parity = std::to_string(static_cast<int>(config.parity));
    std::string stop_bits = std::to_string(static_cast<int>(config.stop_bits));
    std::string script = "sec tool_rs485_config():\n";
    script += "    set_tool_analog_io_work_mode(0)\n";
    script += "    tool_serial_config(True," + baud_rate + "," + parity + "," + stop_bits + ")\n";
    script += "end\n";
    if (!impl_->primary_port_->sendScript(script)) {
        ELITE_LOG_ERROR("Send tool_rs485_config script fail.");
        return nullptr;
    }
    std::string cmd = "bash -lc 'socat tcp-l:" + std::to_string(tcp_port) +
                      ",reuseaddr,fork,nodelay file:/dev/ttyTCI0,nonblock,raw,waitlock=/var/run/tty0 > /dev/null 2>&1 &'";
    SSH_UTILS::executeCommand(impl_->robot_ip_, "root", ssh_password, cmd);

    for (size_t i = 0; i < 10; i++) {
        socat_pid = impl_->getSocatPid(ssh_password, tcp_port);
        if (socat_pid > 0) {
            break;
        }
        std::this_thread::sleep_for(100ms);
    }
    if (socat_pid < 0) {
        return nullptr;
    }
    return std::make_shared<SerialCommunicationImpl>(tcp_port, impl_->robot_ip_, socat_pid);
}

bool EliteDriver::endToolRs485(SerialCommunicationSharedPtr com, const std::string& ssh_password) {
    if (!com) {
        return false;
    }
    if (com->getSocatPid() < 0) {
        return false;
    }
    SSH_UTILS::executeCommand(impl_->robot_ip_, "root", ssh_password, "kill " + std::to_string(com->getSocatPid()));
    return true;
}

SerialCommunicationSharedPtr EliteDriver::startBoardRs485(const SerialConfig& config, const std::string& ssh_password,
                                                         int tcp_port) {
    if (!impl_->primary_port_) {
        ELITE_LOG_ERROR("Not connect to robot primary port");
        return nullptr;
    }
    int socat_pid = impl_->getSocatPid(ssh_password, tcp_port);
    if (socat_pid > 0) {
        return std::make_shared<SerialCommunicationImpl>(tcp_port, impl_->robot_ip_, socat_pid);
    }

    std::string baud_rate = std::to_string(static_cast<int>(config.baud_rate));
    std::string parity = std::to_string(static_cast<int>(config.parity));
    std::string stop_bits = std::to_string(static_cast<int>(config.stop_bits));
    std::string script = "sec board_rs485_config():\n";
    script += "    masterboard_serial_config(True," + baud_rate + "," + parity + "," + stop_bits + ")\n";
    script += "end\n";
    if (!impl_->primary_port_->sendScript(script)) {
        ELITE_LOG_ERROR("Send board_rs485_config script fail.");
        return nullptr;
    }
    std::string cmd = "bash -lc 'socat tcp-l:" + std::to_string(tcp_port) +
                      ",reuseaddr,fork,nodelay file:/dev/ttyBoard,nonblock,raw,waitlock=/var/run/tty1 > /dev/null 2>&1 &'";
    SSH_UTILS::executeCommand(impl_->robot_ip_, "root", ssh_password, cmd);

    for (size_t i = 0; i < 10; i++) {
        socat_pid = impl_->getSocatPid(ssh_password, tcp_port);
        if (socat_pid > 0) {
            break;
        }
        std::this_thread::sleep_for(100ms);
    }
    if (socat_pid < 0) {
        return nullptr;
    }
    return std::make_shared<SerialCommunicationImpl>(tcp_port, impl_->robot_ip_, socat_pid);
}

bool EliteDriver::endBoardRs485(SerialCommunicationSharedPtr com, const std::string& ssh_password) {
    if (!com) {
        return false;
    }
    if (com->getSocatPid() < 0) {
        return false;
    }
    SSH_UTILS::executeCommand(impl_->robot_ip_, "root", ssh_password, "kill " + std::to_string(com->getSocatPid()));
    return true;
}