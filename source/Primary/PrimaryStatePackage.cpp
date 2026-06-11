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

    data_.is_robot_power_on = asBool(*(iter + offset));
    offset += sizeof(uint8_t);

    data_.is_emergency_stopped = asBool(*(iter + offset));
    offset += sizeof(uint8_t);

    data_.is_robot_protective_stopped = asBool(*(iter + offset));
    offset += sizeof(uint8_t);

    data_.is_task_running = asBool(*(iter + offset));
    offset += sizeof(uint8_t);

    data_.is_task_paused = asBool(*(iter + offset));
    offset += sizeof(uint8_t);

    data_.robot_mode = toRobotMode(*(iter + offset));
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

    data_.is_in_package_mode = asBool(*(iter + offset));
}

void MasterBoardDataPackage::parser(int len, const std::vector<uint8_t>::const_iterator& iter) {
    if (len < MASTER_BOARD_DATA_STATUS_MIN_LENGTH) {
        ELITE_LOG_ERROR("Primary MasterBoardData package len error: %d", len);
        return;
    }

    int offset = MASTER_BOARD_DATA_STATUS_OFFSET;

    data_.safety_mode = toSafetyMode(*(iter + offset));
    offset += sizeof(uint8_t);

    data_.is_robot_in_reduced_mode = asBool(*(iter + offset));
    offset += sizeof(uint8_t);

    data_.operational_mode_selector_input = asBool(*(iter + offset));
    offset += sizeof(uint8_t);

    data_.threeposition_enabling_device_input = asBool(*(iter + offset));
    offset += sizeof(uint8_t);

    data_.internal_use = *(iter + offset);
}

}  // namespace ELITE
