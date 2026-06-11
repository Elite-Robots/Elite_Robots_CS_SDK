// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
#include "PrimaryStatePackage.hpp"
#include "Log.hpp"
#include "Utils.hpp"

namespace ELITE {
namespace {

static constexpr int ROBOT_MODE_DATA_PKG_LENGTH = 53;
static constexpr int MASTER_BOARD_DATA_STATUS_OFFSET = 4 + 1 + 4 + 4 + 3 + 8 * 3 + 3 + 8 * 3 + 4 * 4;
static constexpr int MASTER_BOARD_DATA_STATUS_FIELD_LENGTH = 5;
static constexpr int MASTER_BOARD_DATA_STATUS_MIN_LENGTH =
    MASTER_BOARD_DATA_STATUS_OFFSET + MASTER_BOARD_DATA_STATUS_FIELD_LENGTH;

}  // namespace

void RobotModeDataPackage::parser(int len, const std::vector<uint8_t>::const_iterator& iter) {
    if (len < ROBOT_MODE_DATA_PKG_LENGTH) {
        ELITE_LOG_ERROR("Primary RobotModeData package len error: %d", len);
        return;
    }

    int offset = 0;
    offset += sizeof(uint32_t);  // mode_sub_len
    offset += sizeof(uint8_t);   // mode_sub_type

    EndianUtils::unpack(iter + offset, data_.timestamp);
    offset += sizeof(uint64_t);

    offset += sizeof(uint8_t);  // reserved
    offset += sizeof(uint8_t);  // reserved

    data_.is_robot_power_on = static_cast<bool>(*(iter + offset));
    offset += sizeof(uint8_t);

    data_.is_emergency_stopped = static_cast<bool>(*(iter + offset));
    offset += sizeof(uint8_t);

    data_.is_robot_protective_stopped = static_cast<bool>(*(iter + offset));
    offset += sizeof(uint8_t);

    data_.is_task_running = static_cast<bool>(*(iter + offset));
    offset += sizeof(uint8_t);

    data_.is_task_paused = static_cast<bool>(*(iter + offset));
    offset += sizeof(uint8_t);

    data_.robot_mode = static_cast<RobotMode>(*(iter + offset));
    offset += sizeof(uint8_t);

    data_.robot_control_mode = *(iter + offset);
    offset += sizeof(uint8_t);

    EndianUtils::unpack(iter + offset, data_.target_speed_fraction);
    offset += sizeof(double);

    EndianUtils::unpack(iter + offset, data_.speed_scaling);
    offset += sizeof(double);

    EndianUtils::unpack(iter + offset, data_.target_speed_fraction_limit);
    offset += sizeof(double);

    data_.robot_speed_mode = *(iter + offset);
    offset += sizeof(uint8_t);

    offset += sizeof(uint8_t);  // reserved

    data_.is_in_package_mode = static_cast<bool>(*(iter + offset));
}

void MasterBoardDataPackage::parser(int len, const std::vector<uint8_t>::const_iterator& iter) {
    if (len < MASTER_BOARD_DATA_STATUS_MIN_LENGTH) {
        ELITE_LOG_ERROR("Primary MasterBoardData package len error: %d", len);
        return;
    }

    int offset = MASTER_BOARD_DATA_STATUS_OFFSET;

    data_.safety_mode = static_cast<SafetyMode>(*(iter + offset));
    offset += sizeof(uint8_t);

    data_.is_robot_in_reduced_mode = static_cast<bool>(*(iter + offset));
    offset += sizeof(uint8_t);

    data_.operational_mode_selector_input = static_cast<bool>(*(iter + offset));
    offset += sizeof(uint8_t);

    data_.threeposition_enabling_device_input = static_cast<bool>(*(iter + offset));
    offset += sizeof(uint8_t);

    data_.internal_use = *(iter + offset);
}

}  // namespace ELITE
