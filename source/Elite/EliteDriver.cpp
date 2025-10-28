#include "EliteDriver.hpp"
#include <Eigen/Dense>
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

using namespace ELITE;

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
static const std::string SERVOJ_QUEUE_PRE_RECV_SIZE_REPLACE = "{{SERVOJ_QUEUE_PRE_RECV_SIZE_REPLACE}}";
static const std::string SERVOJ_QUEUE_PRE_RECV_TIMEOUT_REPLACE = "{{SERVOJ_QUEUE_PRE_RECV_TIMEOUT_REPLACE}}";

class EliteDriver::Impl {
   public:
    Impl() = delete;
    explicit Impl(const std::string& robot_ip) : robot_ip_(robot_ip) { TcpServer::start(); }
    ~Impl() {
        reverse_server_.reset();
        trajectory_server_.reset();
        script_command_server_.reset();
        script_sender_.reset();
        TcpServer::stop();
    }

    std::string readScriptFile(const std::string& file);
    void scriptParamWrite(std::string& file_string, const EliteDriverConfig& config);
    std::string robot_script_;
    std::string robot_ip_;
    std::string local_ip_;
    std::unique_ptr<ReverseInterface> reverse_server_;
    std::unique_ptr<TrajectoryInterface> trajectory_server_;
    std::unique_ptr<ScriptSender> script_sender_;
    std::unique_ptr<ScriptCommandInterface> script_command_server_;
    std::unique_ptr<PrimaryPortInterface> primary_port_;

    std::unique_ptr<Kinematics> kinematics_;
    bool headless_mode_;
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

    while (file_string.find(SERVOJ_QUEUE_PRE_RECV_SIZE_REPLACE) != std::string::npos) {
        file_string.replace(file_string.find(SERVOJ_QUEUE_PRE_RECV_SIZE_REPLACE), SERVOJ_QUEUE_PRE_RECV_SIZE_REPLACE.length(),
                            std::to_string(config.servoj_queue_pre_recv_size));
    }

    float servoj_queue_pre_recv_timeout = 0;
    if (config.servoj_queue_pre_recv_timeout <= 0) {
        servoj_queue_pre_recv_timeout = config.servoj_queue_pre_recv_size * config.servoj_time;
    } else {
        servoj_queue_pre_recv_timeout = config.servoj_queue_pre_recv_timeout;
    }
    while (file_string.find(SERVOJ_QUEUE_PRE_RECV_TIMEOUT_REPLACE) != std::string::npos) {
        file_string.replace(file_string.find(SERVOJ_QUEUE_PRE_RECV_TIMEOUT_REPLACE), SERVOJ_QUEUE_PRE_RECV_TIMEOUT_REPLACE.length(),
                            std::to_string(servoj_queue_pre_recv_timeout));
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

    impl_->reverse_server_ = std::make_unique<ReverseInterface>(config.reverse_port);
    ELITE_LOG_DEBUG("Created reverse interface");
    impl_->trajectory_server_ = std::make_unique<TrajectoryInterface>(config.trajectory_port);
    ELITE_LOG_DEBUG("Created trajectory interface");
    impl_->script_command_server_ = std::make_unique<ScriptCommandInterface>(config.script_command_port);
    ELITE_LOG_DEBUG("Created script command interface");

    impl_->kinematics_ = std::make_unique<Kinematics>();
    ELITE_LOG_DEBUG("Created Kinematics");
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
        impl_->script_sender_ = std::make_unique<ScriptSender>(config.script_sender_port, impl_->robot_script_);
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

bool EliteDriver::writeServoj(const vector6d_t& pos, int timeout_ms, bool cartesian, bool queue_mode) {
    if (cartesian) {
        if (queue_mode) {
            return impl_->reverse_server_->writeJointCommand(pos, ControlMode::MODE_POSE_QUEUE, timeout_ms);
        } else {
            return impl_->reverse_server_->writeJointCommand(pos, ControlMode::MODE_POSE, timeout_ms);
        }
    } else {
        if (queue_mode) {
            return impl_->reverse_server_->writeJointCommand(pos, ControlMode::MODE_SERVOJ_QUEUE, timeout_ms);
        } else {
            return impl_->reverse_server_->writeJointCommand(pos, ControlMode::MODE_SERVOJ, timeout_ms);
        }
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

std::shared_ptr<vector6d_t> EliteDriver::getInverseKinematics(const vector6d_t& pose, const vector6d_t& near_joint,
                                                              const vector6d_t& tcp) {
    return impl_->script_command_server_->getInverseKinematics(pose, near_joint, tcp);
}

bool EliteDriver::primaryReconnect() {
    impl_->primary_port_->disconnect();
    return impl_->primary_port_->connect(impl_->robot_ip_);
}

void EliteDriver::registerRobotExceptionCallback(std::function<void(RobotExceptionSharedPtr)> cb) {
    impl_->primary_port_->registerRobotExceptionCallback(cb);
}

std::shared_ptr<vector6d_t> EliteDriver::getForwardKinematics(const vector6d_t& joint_positions, const vector6d_t& tcp) {
    return impl_->script_command_server_->getForwardKinematics(joint_positions, tcp);
}

void EliteDriver::setKDLconfig(const double dh_a[AXIS_COUNT], const double dh_d[AXIS_COUNT], const double dh_alpha[AXIS_COUNT]) {
    impl_->kinematics_->updateKinConfig(dh_a, dh_d, dh_alpha);
}

std::shared_ptr<vector6d_t> EliteDriver::getInverseKin(const vector6d_t& pose, const vector6d_t& joints_near_vec,
                                                       const vector6d_t& tcp_offset_xyzRPY) {
    JOINTS joints_near;
    MAT4X4 mat;
    double x = pose[0];
    double y = pose[1];
    double z = pose[2];
    double rx = pose[3];
    double ry = pose[4];
    double rz = pose[5];
    KDL::Rotation rot = KDL::Rotation::RPY(rx, ry, rz);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) mat[i][j] = rot(i, j);
    mat[0][3] = x;
    mat[1][3] = y;
    mat[2][3] = z;
    mat[3][0] = 0.0;
    mat[3][1] = 0.0;
    mat[3][2] = 0.0;
    mat[3][3] = 1.0;
    for (int i = 0; i < AXIS_COUNT; ++i) {
        joints_near[i] = joints_near_vec[i];
    }
    JOINTS joints_result{};
    if (impl_->kinematics_->inverse(mat, joints_near, joints_result, tcp_offset_xyzRPY) != 0) {
        return nullptr;
    }

    auto result = std::make_shared<vector6d_t>();
    for (int i = 0; i < AXIS_COUNT; ++i) {
        (*result)[i] = joints_result[i];
    }
    return result;
}

std::shared_ptr<vector6d_t> EliteDriver::getForwardKin(const vector6d_t& joints, const vector6d_t& tcp_offset_xyzRPY) {
    MAT4X4 mat;
    JOINTS joints_array;
    auto result = std::make_shared<vector6d_t>();
    for (size_t i = 0; i < 6; ++i) {
        joints_array[i] = joints[i];
    }
    impl_->kinematics_->forward(joints_array, mat, tcp_offset_xyzRPY);
    (*result)[0] = mat[0][3];  // X
    (*result)[1] = mat[1][3];  // Y
    (*result)[2] = mat[2][3];  // Z
    KDL::Rotation rot(mat[0][0], mat[0][1], mat[0][2], mat[1][0], mat[1][1], mat[1][2], mat[2][0], mat[2][1], mat[2][2]);
    double roll, pitch, yaw;
    rot.GetRPY(roll, pitch, yaw);
    (*result)[3] = roll;
    (*result)[4] = pitch;
    (*result)[5] = yaw;
    return result;
}
