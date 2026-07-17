// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
#include "ReverseInterface.hpp"
#include <cmath>
#include "ControlCommon.hpp"
#include "EliteException.hpp"
#include "Log.hpp"

using namespace ELITE;

ReverseInterface::ReverseInterface(int port, std::shared_ptr<TcpServer::StaticResource> resource) : ReversePort(port, 4, resource) {
    server_->startListen();
}

ReverseInterface::~ReverseInterface() {}

bool ReverseInterface::writeJointCommand(const vector6d_t& pos, ControlMode mode, int timeout) {
    return writeJointCommand(&pos, mode, timeout);
}

bool ReverseInterface::writeJointCommand(const vector6d_t* pos, ControlMode mode, int timeout) {
    int32_t data[REVERSE_DATA_SIZE] = {0};
    const vector6d_t& position = *pos;
    data[0] = htonl(timeout);
    data[REVERSE_DATA_SIZE - 1] = htonl((int)mode);
    if (pos) {
        for (size_t i = 0; i < 6; i++) {
            int32_t rounded_value = static_cast<int32_t>(std::round(position[i] * CONTROL::POS_ZOOM_RATIO));
            data[i + 1] = ::htonl(rounded_value);
        }
    }

    return write(data, sizeof(data)) > 0;
}

bool ReverseInterface::writeTrajectoryControlAction(TrajectoryControlAction action, int point_number, int timeout) {
    int32_t data[REVERSE_DATA_SIZE] = {0};
    data[0] = htonl(timeout);
    data[1] = htonl((int)action);
    data[2] = htonl(point_number);
    data[REVERSE_DATA_SIZE - 1] = htonl((int)ControlMode::MODE_TRAJECTORY);
    return write(data, sizeof(data)) > 0;
}

bool ReverseInterface::writeFreedrive(FreedriveAction action, int timeout_ms) {
    int32_t data[REVERSE_DATA_SIZE] = {0};
    data[0] = htonl(timeout_ms);
    data[1] = htonl((int)action);
    data[REVERSE_DATA_SIZE - 1] = htonl((int)ControlMode::MODE_FREEDRIVE);
    return write(data, sizeof(data)) > 0;
}

bool ReverseInterface::stopControl() {
    int32_t data[REVERSE_DATA_SIZE];
    data[0] = 0;
    data[REVERSE_DATA_SIZE - 1] = htonl((int)ControlMode::MODE_STOPPED);

    return write(data, sizeof(data)) > 0;
}
