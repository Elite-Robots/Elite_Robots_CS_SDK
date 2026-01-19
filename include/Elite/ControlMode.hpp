// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// ControlMode.hpp
// Provides the ControlMode enum class for robot control modes.
#ifndef __CONTROL_MODE_HPP__
#define __CONTROL_MODE_HPP__

namespace ELITE {

enum class ControlMode : int {
    MODE_STOPPED = -2,         // When this is set, the program is expected to stop and exit.
    MODE_UNINITIALIZED = -1,   // Startup default until another mode is sent to the script.
    MODE_IDLE = 0,             // Set when no controller is currently active controlling the robot.
    MODE_SERVOJ = 1,           // Set when servoj control is active.
    MODE_SPEEDJ = 2,           // Set when speedj control is active.
    MODE_TRAJECTORY = 3,       // Set when trajectory forwarding is active.
    MODE_SPEEDL = 4,           // Set when cartesian velocity control is active.
    MODE_POSE = 5,             // Set when cartesian pose control is active.
    MODE_FREEDRIVE = 6,        // Set when freedrive mode is active.
    MODE_TOOL_IN_CONTACT = 7,  // Set tool in contact.(Not use now, coming soon)
};

}  // namespace ELITE

#endif
