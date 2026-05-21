// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
#include <future>

#include "ControlCommon.hpp"
#include "Log.hpp"
#include "ScriptCommandInterface.hpp"

namespace ELITE {

ScriptCommandInterface::ScriptCommandInterface(int port, std::shared_ptr<TcpServer::StaticResource> resource)
    : ReversePort(port, 4, resource) {
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

}  // namespace ELITE