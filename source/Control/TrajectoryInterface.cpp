// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
#include "TrajectoryInterface.hpp"
#include <boost/asio.hpp>
#include <cstring>
#include "ControlCommon.hpp"
#include "EliteException.hpp"
#include "Log.hpp"

using namespace ELITE;

TrajectoryInterface::TrajectoryInterface(int port, std::shared_ptr<TcpServer::StaticResource> resource_)
    : ReversePort(port, TRAJECTORY_FEEDBACK_LEN * sizeof(int32_t), resource_) {
    server_->setReceiveCallback([&](const uint8_t data[], int nb) {
        if (nb != TRAJECTORY_FEEDBACK_LEN * static_cast<int>(sizeof(int32_t))) {
            return;
        }

        auto decode_int32 = [](int32_t raw_value) -> int32_t {
            uint32_t network_value = static_cast<uint32_t>(raw_value);
            uint32_t host_value = ::ntohl(network_value);
            return static_cast<int32_t>(host_value);
        };

        int32_t raw_feedback[TRAJECTORY_FEEDBACK_LEN] = {0};
        std::memcpy(raw_feedback, data, sizeof(raw_feedback));
        TrajectoryMotionFeedback feedback;
        feedback.message_type = static_cast<TrajectoryFeedbackMessageType>(decode_int32(raw_feedback[0]));
        feedback.point_index = decode_int32(raw_feedback[1]);
        feedback.total_points = decode_int32(raw_feedback[2]);
        feedback.result = decode_int32(raw_feedback[3]);
        for (size_t i = 0; i < feedback.point.size(); ++i) {
            feedback.point[i] = static_cast<double>(decode_int32(raw_feedback[4 + i])) / CONTROL::POS_ZOOM_RATIO;
        }

        if (motion_feedback_func_) {
            motion_feedback_func_(feedback);
        }

        if (feedback.message_type == TrajectoryFeedbackMessageType::RESULT && motion_result_func_) {
            motion_result_func_(static_cast<TrajectoryMotionResult>(feedback.result));
        }
    });
    server_->startListen();
}

TrajectoryInterface::~TrajectoryInterface() {}

bool TrajectoryInterface::writeTrajectoryPoint(const vector6d_t& positions, float time, float blend_radius, bool cartesian) {
    return writeTrajectoryPoint(positions, time, blend_radius, cartesian, 0.0f, 0.0f);
}

bool TrajectoryInterface::writeTrajectoryPoint(const vector6d_t& positions, float blend_radius, bool cartesian, float speed,
                                               float acceleration) {
    return writeTrajectoryPoint(positions, 0.0f, blend_radius, cartesian, speed, acceleration);
}

bool TrajectoryInterface::writeTrajectoryPoint(const vector6d_t& positions, float time, float blend_radius, bool cartesian,
                                               float speed, float acceleration) {
    int32_t buffer[TRAJECTORY_MESSAGE_LEN] = {0};
    for (size_t i = 0; i < 6; i++) {
        int32_t rounded_pos = static_cast<int32_t>(::round(positions[i] * CONTROL::POS_ZOOM_RATIO));
        buffer[i] = ::htonl(rounded_pos);
    }
    buffer[6] = htonl(round(speed * CONTROL::COMMON_ZOOM_RATIO));
    buffer[7] = htonl(round(acceleration * CONTROL::COMMON_ZOOM_RATIO));
    buffer[18] = htonl(round(time * CONTROL::TIME_ZOOM_RATIO));
    buffer[19] = htonl(round(blend_radius * CONTROL::POS_ZOOM_RATIO));
    if (cartesian) {
        buffer[20] = htonl((int)TrajectoryMotionType::CARTESIAN);
    } else {
        buffer[20] = htonl((int)TrajectoryMotionType::JOINT);
    }

    return write(buffer, sizeof(buffer)) > 0;
}
