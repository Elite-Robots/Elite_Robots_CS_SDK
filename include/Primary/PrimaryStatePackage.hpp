// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// PrimaryStatePackage.hpp
// Provides primary port state package parsers.
#ifndef __ELITE__PRIMARY_STATE_PACKAGE_HPP__
#define __ELITE__PRIMARY_STATE_PACKAGE_HPP__

#include <Elite/DataType.hpp>
#include <Elite/EliteOptions.hpp>
#include <Elite/PrimaryPackage.hpp>

#include <cstdint>
#include <vector>

namespace ELITE {

/**
 * @brief RobotModeData sub-package data in the robot's primary port.
 *
 */
struct RobotModeData {
    uint64_t timestamp = 0;
    bool is_robot_power_on = false;
    bool is_emergency_stopped = false;
    bool is_robot_protective_stopped = false;
    bool is_task_running = false;
    bool is_task_paused = false;
    RobotMode robot_mode = RobotMode::UNKNOWN;
    uint8_t robot_control_mode = 0;
    double target_speed_fraction = 0.0;
    double speed_scaling = 0.0;
    double target_speed_fraction_limit = 0.0;
    uint8_t robot_speed_mode = 0;
    bool is_in_package_mode = false;
};

/**
 * @brief MasterBoardData status fields in the robot's primary port.
 *
 */
struct MasterBoardData {
    SafetyMode safety_mode = SafetyMode::UNKNOWN;
    bool is_robot_in_reduced_mode = false;
    bool operational_mode_selector_input = false;
    bool threeposition_enabling_device_input = false;
    uint8_t internal_use = 0;
};

/**
 * @brief The RobotModeData message in the robot's primary port.
 *
 */
class RobotModeDataPackage : public PrimaryPackage {
   private:
    /// The RobotModeData sub-package type
    static constexpr int ROBOT_MODE_DATA_PKG_TYPE = 0;

    RobotModeData data_;

   public:
    ELITE_EXPORT RobotModeDataPackage() : PrimaryPackage(ROBOT_MODE_DATA_PKG_TYPE) {}
    ELITE_EXPORT ~RobotModeDataPackage() override = default;

    /**
     * @brief Parser message from robot. Internal use.
     *
     * @param len The len of sub-package
     * @param iter Position of the sub-package in the entire package
     */
    ELITE_EXPORT void parser(int len, const std::vector<uint8_t>::const_iterator& iter) override;

    /**
     * @brief Get parsed RobotModeData.
     *
     * @return RobotModeData parsed data snapshot
     */
    ELITE_EXPORT RobotModeData data() const { return data_; }
};

/**
 * @brief The MasterBoardData message in the robot's primary port.
 *
 */
class MasterBoardDataPackage : public PrimaryPackage {
   private:
    /// The MasterBoardData sub-package type
    static constexpr int MASTER_BOARD_DATA_PKG_TYPE = 3;

    MasterBoardData data_;

   public:
    ELITE_EXPORT MasterBoardDataPackage() : PrimaryPackage(MASTER_BOARD_DATA_PKG_TYPE) {}
    ELITE_EXPORT ~MasterBoardDataPackage() override = default;

    /**
     * @brief Parser message from robot. Internal use.
     *
     * @param len The len of sub-package
     * @param iter Position of the sub-package in the entire package
     */
    ELITE_EXPORT void parser(int len, const std::vector<uint8_t>::const_iterator& iter) override;

    /**
     * @brief Get parsed MasterBoardData.
     *
     * @return MasterBoardData parsed data snapshot
     */
    ELITE_EXPORT MasterBoardData data() const { return data_; }
};

}  // namespace ELITE

#endif
