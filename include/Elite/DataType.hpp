/**
 * @file DataType.hpp
 * @author yanxiaojia
 * @brief The data type of sdk
 * @date 2024-08-21
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef __DATA_TYPE_HPP__
#define __DATA_TYPE_HPP__

#include <Elite/EliteOptions.hpp>

#include <array>
#include <cstdint>

#if (ELITE_SDK_COMPILE_STANDARD >= 17)
#include <variant>
#elif (ELITE_SDK_COMPILE_STANDARD == 14)
#include <boost/variant.hpp>
#endif

namespace ELITE {

enum class RobotMode : int32_t {
    UNKNOWN = -2,
    NO_CONTROLLER = -1,
    DISCONNECTED = 0,
    CONFIRM_SAFETY = 1,
    BOOTING = 2,
    POWER_OFF = 3,
    POWER_ON = 4,
    IDLE = 5,
    BACKDRIVE = 6,
    RUNNING = 7,
    UPDATING_FIRMWARE = 8,
    WAITING_CALIBRATION = 9,
};

enum class JointMode : int32_t {
    MODE_RESET = 235,
    MODE_SHUTTING_DOWN = 236,
    MODE_BACKDRIVE = 238,
    MODE_POWER_OFF = 239,
    MODE_READY_FOR_POWEROFF = 240,
    MODE_NOT_RESPONDING = 245,
    MODE_MOTOR_INITIALISATION = 246,
    MODE_BOOTING = 247,
    MODE_BOOTLOADER = 249,
    MODE_VIOLATION = 251,
    MODE_FAULT = 252,
    MODE_RUNNING = 253,
    MODE_IDLE = 255
};

enum class SafetyMode : int32_t {
    UNKNOWN = -2,
    NORMAL = 1,
    REDUCED = 2,
    PROTECTIVE_STOP = 3,
    RECOVERY = 4,
    SAFEGUARD_STOP = 5,
    SYSTEM_EMERGENCY_STOP = 6,
    ROBOT_EMERGENCY_STOP = 7,
    VIOLATION = 8,
    FAULT = 9,
    VALIDATE_JOINT_ID = 10,
    UNDEFINED_SAFETY_MODE = 11,
    AUTOMATIC_MODE_SAFEGUARD_STOP = 12,
    SYSTEM_THREE_POSITION_ENABLING_STOP = 13,
    TP_THREE_POSITION_ENABLING_STOP = 14,
};

enum class ToolMode : uint32_t {
    MODE_RESET = 235,
    MODE_SHUTTING_DOWN = 236,
    MODE_POWER_OFF = 239,
    MODE_NOT_RESPONDING = 245,
    MODE_BOOTING = 247,
    MODE_BOOTLOADER = 249,
    MODE_FAULT = 252,
    MODE_RUNNING = 253,
    MODE_IDLE = 255
};

enum class ToolDigitalMode : uint8_t {
    /// All available
    SINGLE_NEEDLE,
    /// 0 and 1 are both available
    DOUBLE_NEEDLE_1,
    /// 2 and 3 are both available
    DOUBLE_NEEDLE_2,
    /// All not available
    TRIPLE_NEEDLE
};

enum class ToolDigitalOutputMode : uint8_t {
    /// push/pull mode
    PUSH_PULL_MODE = 0,
    /// PNP
    SOURCING_PNP_MODE = 1,
    /// NPN
    SINKING_NPN_MODE = 2
};

enum class TaskStatus { UNKNOWN, PLAYING, PAUSED, STOPPED };

enum class TrajectoryMotionResult : int {
    /// Successful execution
    SUCCESS = 0,
    /// Canceled by user
    CANCELED = 1,
    /// Aborted due to error during execution
    FAILURE = 2
};

enum class TrajectoryControlAction : int {
    /// Represents command to cancel currently active trajectory.
    CANCEL = -1,
    /// Represents no new control command.
    NOOP = 0,
    /// Represents command to start a new trajectory.
    START = 1,
};

enum class ToolVoltage : int {
    OFF = 0,    // 0V
    V_12 = 12,  // 12V
    V_24 = 24   // 24V
};

enum class ForceMode : int {
    /// The force frame is the force reference frame.
    FIX,
    /// The Y-axis in the force frame points from the TCP
    /// origin of the robot to the origin of the force reference frame.
    POINT,
    /// The X-axis in the force frame is the projection of the TCP motion direction
    /// vector in the X-Y plane belonging to the force reference frame.
    MOTION,
    /// The force frame is the TCP frame.
    TCP,
};

enum class FreedriveAction : int {
    /// Represents command to stop freedrive mode.
    FREEDRIVE_END = -1,
    /// Represents keep running in freedrive mode.
    FREEDRIVE_NOOP = 0,
    /// Represents command to start freedrive mode.
    FREEDRIVE_START = 1
};

// 4x4 Pose Matrix
struct PoseMatrix {
    std::array<std::array<double, 4>, 4> data{{
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    }};
    PoseMatrix operator*(const PoseMatrix& other) const;
    PoseMatrix& operator*=(const PoseMatrix& other);
    PoseMatrix inverse() const;
};

using vector3d_t = std::array<double, 3>;
using vector6d_t = std::array<double, 6>;
using vector6int32_t = std::array<int32_t, 6>;
using vector6uint32_t = std::array<uint32_t, 6>;
#if (ELITE_SDK_COMPILE_STANDARD >= 17)
using RtsiTypeVariant = std::variant<bool, int8_t, uint8_t, int16_t, uint16_t, int32_t, uint32_t, int64_t, uint64_t, double,
                                     vector3d_t, vector6d_t, vector6int32_t, vector6uint32_t>;
#elif (ELITE_SDK_COMPILE_STANDARD == 14)
using RtsiTypeVariant = boost::variant<bool, int8_t, uint8_t, int16_t, uint16_t, int32_t, uint32_t, int64_t, uint64_t, double,
                                       vector3d_t, vector6d_t, vector6int32_t, vector6uint32_t>;
#endif

}  // namespace ELITE

#endif