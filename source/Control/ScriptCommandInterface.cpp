#include "ScriptCommandInterface.hpp"
#include "ControlCommon.hpp"
#include "Log.hpp"

namespace ELITE {

ScriptCommandInterface::ScriptCommandInterface(int port) : ReversePort(port, 24) {
    server_->setReceiveCallback([this](const uint8_t* data, int length) {
        recv_buffer_.insert(recv_buffer_.end(), data, data + length);
        while (recv_buffer_.size() >= 6 * sizeof(uint32_t)) {
            const uint8_t* frame_data = recv_buffer_.data();
            this->handReceive(frame_data, 6 * sizeof(uint32_t));

            recv_buffer_.erase(recv_buffer_.begin(), recv_buffer_.begin() + 6 * sizeof(uint32_t));
        }
    });
    server_->startListen();
}

ScriptCommandInterface::~ScriptCommandInterface() {}

bool ScriptCommandInterface::zeroFTSensor() {
    int32_t buffer[SCRIPT_COMMAND_DATA_SIZE] = {0};
    buffer[0] = htonl(static_cast<int32_t>(Cmd::ZERO_FTSENSOR));
    return write(buffer, sizeof(buffer)) > 0;
}

bool ScriptCommandInterface::setPayload(double mass, const vector3d_t& cog) {
    int32_t buffer[SCRIPT_COMMAND_DATA_SIZE] = {0};
    buffer[0] = htonl(static_cast<int32_t>(Cmd::SET_PAYLOAD));
    buffer[1] = htonl(static_cast<int32_t>((mass * CONTROL::COMMON_ZOOM_RATIO)));
    buffer[2] = htonl(static_cast<int32_t>((cog[0] * CONTROL::COMMON_ZOOM_RATIO)));
    buffer[3] = htonl(static_cast<int32_t>((cog[1] * CONTROL::COMMON_ZOOM_RATIO)));
    buffer[4] = htonl(static_cast<int32_t>((cog[2] * CONTROL::COMMON_ZOOM_RATIO)));
    return write(buffer, sizeof(buffer)) > 0;
}

bool ScriptCommandInterface::setToolVoltage(const ToolVoltage& vol) {
    int32_t buffer[SCRIPT_COMMAND_DATA_SIZE] = {0};
    buffer[0] = htonl(static_cast<int32_t>(Cmd::SET_TOOL_VOLTAGE));
    buffer[1] = htonl(static_cast<int32_t>(vol) * CONTROL::COMMON_ZOOM_RATIO);
    return write(buffer, sizeof(buffer)) > 0;
}

bool ScriptCommandInterface::startForceMode(const vector6d_t& task_frame, const vector6int32_t& selection_vector,
                                            const vector6d_t& wrench, const ForceMode& mode, const vector6d_t& limits) {
    int32_t buffer[SCRIPT_COMMAND_DATA_SIZE] = {0};
    buffer[0] = htonl(static_cast<int32_t>(Cmd::START_FORCE_MODE));
    int32_t* bp = &buffer[1];
    for (auto& tf : task_frame) {
        *bp = htonl(static_cast<int32_t>((tf * CONTROL::COMMON_ZOOM_RATIO)));
        bp++;
    }
    for (auto& sv : selection_vector) {
        *bp = htonl(sv);
        bp++;
    }
    for (auto& wr : wrench) {
        *bp = htonl(static_cast<int32_t>((wr * CONTROL::COMMON_ZOOM_RATIO)));
        bp++;
    }
    *bp = htonl(static_cast<int32_t>(mode));
    bp++;
    for (auto& li : limits) {
        *bp = htonl(static_cast<int32_t>((li * CONTROL::COMMON_ZOOM_RATIO)));
        bp++;
    }
    return write(buffer, sizeof(buffer)) > 0;
}

bool ScriptCommandInterface::endForceMode() {
    int32_t buffer[SCRIPT_COMMAND_DATA_SIZE] = {0};
    buffer[0] = htonl(static_cast<int32_t>(Cmd::END_FORCE_MODE));
    return write(buffer, sizeof(buffer)) > 0;
}
void ScriptCommandInterface::handReceive(const uint8_t* data, int data_length) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    size_t num_elements = data_length / sizeof(uint32_t);
    last_data_.resize(num_elements);
    std::memcpy(last_data_.data(), data, num_elements * sizeof(uint32_t));

    for (size_t i = 0; i < num_elements; ++i) {
        uint32_t temp = ntohl(last_data_[i]);

        last_data_[i] = static_cast<int32_t>(temp);
    }
    data_received_ = true;
}
std::shared_ptr<vector6d_t> ScriptCommandInterface::getInverseKinematics(const vector6d_t& pose, const vector6d_t& near_joint,
                                                                         const vector6d_t& tcp) {
    int32_t buffer[SCRIPT_COMMAND_DATA_SIZE] = {0};
    buffer[0] = htonl(static_cast<int32_t>(Cmd::INVERSE_KINEMATICS));
    buffer[1] = htonl(static_cast<int32_t>(pose[0] * CONTROL::COMMON_ZOOM_RATIO));
    buffer[2] = htonl(static_cast<int32_t>(pose[1] * CONTROL::COMMON_ZOOM_RATIO));
    buffer[3] = htonl(static_cast<int32_t>(pose[2] * CONTROL::COMMON_ZOOM_RATIO));
    buffer[4] = htonl(static_cast<int32_t>(pose[3] * CONTROL::COMMON_ZOOM_RATIO));
    buffer[5] = htonl(static_cast<int32_t>(pose[4] * CONTROL::COMMON_ZOOM_RATIO));
    buffer[6] = htonl(static_cast<int32_t>(pose[5] * CONTROL::COMMON_ZOOM_RATIO));
    buffer[7] = htonl(static_cast<int32_t>(near_joint[0] * CONTROL::COMMON_ZOOM_RATIO));
    buffer[8] = htonl(static_cast<int32_t>(near_joint[1] * CONTROL::COMMON_ZOOM_RATIO));
    buffer[9] = htonl(static_cast<int32_t>(near_joint[2] * CONTROL::COMMON_ZOOM_RATIO));
    buffer[10] = htonl(static_cast<int32_t>(near_joint[3] * CONTROL::COMMON_ZOOM_RATIO));
    buffer[11] = htonl(static_cast<int32_t>(near_joint[4] * CONTROL::COMMON_ZOOM_RATIO));
    buffer[12] = htonl(static_cast<int32_t>(near_joint[5] * CONTROL::COMMON_ZOOM_RATIO));
    buffer[13] = htonl(static_cast<int32_t>(tcp[0] * CONTROL::COMMON_ZOOM_RATIO));
    buffer[14] = htonl(static_cast<int32_t>(tcp[1] * CONTROL::COMMON_ZOOM_RATIO));
    buffer[15] = htonl(static_cast<int32_t>(tcp[2] * CONTROL::COMMON_ZOOM_RATIO));
    buffer[16] = htonl(static_cast<int32_t>(tcp[3] * CONTROL::COMMON_ZOOM_RATIO));
    buffer[17] = htonl(static_cast<int32_t>(tcp[4] * CONTROL::COMMON_ZOOM_RATIO));
    buffer[18] = htonl(static_cast<int32_t>(tcp[5] * CONTROL::COMMON_ZOOM_RATIO));

    if (write(buffer, sizeof(buffer)) <= 0) {
        return nullptr;
    }
    while (true) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (data_received_) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    data_received_ = false;
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (last_data_.size() != 6) {
        return nullptr;
    }
    auto result = std::make_shared<vector6d_t>();
    for (size_t i = 0; i < 6; ++i) {
        (*result)[i] = static_cast<double>(last_data_[i]) / CONTROL::COMMON_ZOOM_RATIO;
    }
    if (!result) {
    }
    return result;
}

std::shared_ptr<vector6d_t> ScriptCommandInterface::getForwardKinematics(const vector6d_t& joint, const vector6d_t& tcp) {
    int32_t buffer[SCRIPT_COMMAND_DATA_SIZE] = {0};
    buffer[0] = htonl(static_cast<int32_t>(Cmd::FORWARD_KINEMATICS));
    buffer[1] = htonl(static_cast<int32_t>(joint[0] * CONTROL::COMMON_ZOOM_RATIO));
    buffer[2] = htonl(static_cast<int32_t>(joint[1] * CONTROL::COMMON_ZOOM_RATIO));
    buffer[3] = htonl(static_cast<int32_t>(joint[2] * CONTROL::COMMON_ZOOM_RATIO));
    buffer[4] = htonl(static_cast<int32_t>(joint[3] * CONTROL::COMMON_ZOOM_RATIO));
    buffer[5] = htonl(static_cast<int32_t>(joint[4] * CONTROL::COMMON_ZOOM_RATIO));
    buffer[6] = htonl(static_cast<int32_t>(joint[5] * CONTROL::COMMON_ZOOM_RATIO));
    buffer[7] = htonl(static_cast<int32_t>(tcp[0] * CONTROL::COMMON_ZOOM_RATIO));
    buffer[8] = htonl(static_cast<int32_t>(tcp[1] * CONTROL::COMMON_ZOOM_RATIO));
    buffer[9] = htonl(static_cast<int32_t>(tcp[2] * CONTROL::COMMON_ZOOM_RATIO));
    buffer[10] = htonl(static_cast<int32_t>(tcp[3] * CONTROL::COMMON_ZOOM_RATIO));
    buffer[11] = htonl(static_cast<int32_t>(tcp[4] * CONTROL::COMMON_ZOOM_RATIO));
    buffer[12] = htonl(static_cast<int32_t>(tcp[5] * CONTROL::COMMON_ZOOM_RATIO));
    if (write(buffer, sizeof(buffer)) <= 0) {
        return nullptr;

    }
    while (true) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (data_received_) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    data_received_ = false;
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (last_data_.size() != 6) {
        return nullptr;
    }
    auto result = std::make_shared<vector6d_t>();
    for (size_t i = 0; i < 6; ++i) {
        (*result)[i] = static_cast<double>(last_data_[i]) / CONTROL::COMMON_ZOOM_RATIO;
    }
    if (!result) {
    }
    return result;
}

}  // namespace ELITE