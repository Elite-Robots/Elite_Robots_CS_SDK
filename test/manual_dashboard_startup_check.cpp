// SPDX-License-Identifier: MIT
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "Dashboard/DashboardClient.hpp"
#include "Elite/DataType.hpp"

namespace {

const char* robotModeName(ELITE::RobotMode mode) {
    switch (mode) {
        case ELITE::RobotMode::NO_CONTROLLER:
            return "NO_CONTROLLER";
        case ELITE::RobotMode::DISCONNECTED:
            return "DISCONNECTED";
        case ELITE::RobotMode::CONFIRM_SAFETY:
            return "CONFIRM_SAFETY";
        case ELITE::RobotMode::BOOTING:
            return "BOOTING";
        case ELITE::RobotMode::POWER_OFF:
            return "POWER_OFF";
        case ELITE::RobotMode::POWER_ON:
            return "POWER_ON";
        case ELITE::RobotMode::IDLE:
            return "IDLE";
        case ELITE::RobotMode::BACKDRIVE:
            return "BACKDRIVE";
        case ELITE::RobotMode::RUNNING:
            return "RUNNING";
        case ELITE::RobotMode::UPDATING_FIRMWARE:
            return "UPDATING_FIRMWARE";
        case ELITE::RobotMode::WAITING_CALIBRATION:
            return "WAITING_CALIBRATION";
        case ELITE::RobotMode::UNKNOWN:
        default:
            return "UNKNOWN";
    }
}

const char* safetyModeName(ELITE::SafetyMode mode) {
    switch (mode) {
        case ELITE::SafetyMode::NORMAL:
            return "NORMAL";
        case ELITE::SafetyMode::REDUCED:
            return "REDUCED";
        case ELITE::SafetyMode::PROTECTIVE_STOP:
            return "PROTECTIVE_STOP";
        case ELITE::SafetyMode::RECOVERY:
            return "RECOVERY";
        case ELITE::SafetyMode::SAFEGUARD_STOP:
            return "SAFEGUARD_STOP";
        case ELITE::SafetyMode::SYSTEM_EMERGENCY_STOP:
            return "SYSTEM_EMERGENCY_STOP";
        case ELITE::SafetyMode::ROBOT_EMERGENCY_STOP:
            return "ROBOT_EMERGENCY_STOP";
        case ELITE::SafetyMode::VIOLATION:
            return "VIOLATION";
        case ELITE::SafetyMode::FAULT:
            return "FAULT";
        case ELITE::SafetyMode::VALIDATE_JOINT_ID:
            return "VALIDATE_JOINT_ID";
        case ELITE::SafetyMode::UNDEFINED_SAFETY_MODE:
            return "UNDEFINED_SAFETY_MODE";
        case ELITE::SafetyMode::AUTOMATIC_MODE_SAFEGUARD_STOP:
            return "AUTOMATIC_MODE_SAFEGUARD_STOP";
        case ELITE::SafetyMode::SYSTEM_THREE_POSITION_ENABLING_STOP:
            return "SYSTEM_THREE_POSITION_ENABLING_STOP";
        case ELITE::SafetyMode::TP_THREE_POSITION_ENABLING_STOP:
            return "TP_THREE_POSITION_ENABLING_STOP";
        case ELITE::SafetyMode::UNKNOWN:
        default:
            return "UNKNOWN";
    }
}

const char* taskStatusName(ELITE::TaskStatus status) {
    switch (status) {
        case ELITE::TaskStatus::PLAYING:
            return "PLAYING";
        case ELITE::TaskStatus::PAUSED:
            return "PAUSED";
        case ELITE::TaskStatus::STOPPED:
            return "STOPPED";
        case ELITE::TaskStatus::UNKNOWN:
        default:
            return "UNKNOWN";
    }
}

template <typename Fn>
bool tryStep(const char* name, Fn&& fn) {
    try {
        bool ok = fn();
        std::cout << name << ": " << (ok ? "OK" : "FAIL") << '\n';
        return ok;
    } catch (const std::exception& e) {
        std::cout << name << ": EXCEPTION: " << e.what() << '\n';
        return false;
    }
}

bool waitRobotMode(ELITE::DashboardClient& dashboard, ELITE::RobotMode expected, int timeout_ms) {
    int count = 0;
    while (count < timeout_ms / 100) {
        auto mode = dashboard.robotMode();
        if (mode == expected) {
            return true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        count++;
    }

    return false;
}

bool waitRobotReadyAfterPowerOn(ELITE::DashboardClient& dashboard, int timeout_ms) {
    int count = 0;
    while (count < timeout_ms / 100) {
        auto mode = dashboard.robotMode();
        if (mode == ELITE::RobotMode::IDLE || mode == ELITE::RobotMode::RUNNING) {
            return true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        count++;
    }

    return false;
}

bool ensurePowerOff(ELITE::DashboardClient& dashboard) {
    auto mode = dashboard.robotMode();
    if (mode == ELITE::RobotMode::POWER_OFF) {
        std::cout << "ensure POWER_OFF: SKIP, already POWER_OFF\n";
        return true;
    }

    std::cout << "ensure POWER_OFF: current robotMode=" << robotModeName(mode) << '\n';
    if (!tryStep("powerOff", [&] { return dashboard.powerOff(); })) {
        mode = dashboard.robotMode();
        if (mode != ELITE::RobotMode::POWER_OFF) {
            std::cout << "after failed powerOff robotMode=" << robotModeName(mode) << '\n';
            return false;
        }
    }

    return waitRobotMode(dashboard, ELITE::RobotMode::POWER_OFF, 30000);
}

bool runStartup(ELITE::DashboardClient& dashboard) {
    auto safety = dashboard.safetyMode();
    auto mode = dashboard.robotMode();
    auto task = dashboard.getTaskStatus();
    std::cout << "initial safety=" << safetyModeName(safety) << ", robotMode=" << robotModeName(mode)
              << ", task=" << taskStatusName(task) << '\n';

    if (safety == ELITE::SafetyMode::PROTECTIVE_STOP) {
        if (!tryStep("unlockProtectiveStop", [&] { return dashboard.unlockProtectiveStop(); })) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        safety = dashboard.safetyMode();
        std::cout << "after unlock safety=" << safetyModeName(safety) << '\n';
    }

    tryStep("closeSafetyDialog(optional)", [&] { return dashboard.closeSafetyDialog(); });

    task = dashboard.getTaskStatus();
    if (task != ELITE::TaskStatus::STOPPED) {
        if (!tryStep("stopProgram", [&] { return dashboard.stopProgram(); })) {
            task = dashboard.getTaskStatus();
            if (task != ELITE::TaskStatus::STOPPED) {
                return false;
            }
        }
    } else {
        std::cout << "stopProgram: SKIP, task already stopped\n";
    }

    safety = dashboard.safetyMode();
    mode = dashboard.robotMode();
    std::cout << "before power safety=" << safetyModeName(safety) << ", robotMode=" << robotModeName(mode) << '\n';

    if (mode == ELITE::RobotMode::POWER_OFF) {
        if (!tryStep("powerOn", [&] { return dashboard.powerOn(); })) {
            mode = dashboard.robotMode();
            if (mode != ELITE::RobotMode::IDLE && mode != ELITE::RobotMode::RUNNING) {
                std::cout << "after failed powerOn robotMode=" << robotModeName(mode) << '\n';
                return false;
            }
        }

        if (!waitRobotReadyAfterPowerOn(dashboard, 30000)) {
            std::cout << "wait IDLE/RUNNING after powerOn: FAIL\n";
            return false;
        }
    } else if (mode == ELITE::RobotMode::POWER_ON || mode == ELITE::RobotMode::BOOTING) {
        if (!waitRobotReadyAfterPowerOn(dashboard, 30000)) {
            std::cout << "wait IDLE/RUNNING from " << robotModeName(mode) << ": FAIL\n";
            return false;
        }
    } else {
        std::cout << "powerOn: SKIP, robot already not POWER_OFF\n";
    }

    mode = dashboard.robotMode();
    std::cout << "after power robotMode=" << robotModeName(mode) << '\n';

    if (mode == ELITE::RobotMode::IDLE) {
        if (!tryStep("brakeRelease", [&] { return dashboard.brakeRelease(); })) {
            mode = dashboard.robotMode();
            if (mode != ELITE::RobotMode::RUNNING) {
                std::cout << "after failed brakeRelease robotMode=" << robotModeName(mode) << '\n';
                return false;
            }
        }

        if (!waitRobotMode(dashboard, ELITE::RobotMode::RUNNING, 30000)) {
            std::cout << "wait RUNNING after brakeRelease: FAIL\n";
            return false;
        }
    } else if (mode == ELITE::RobotMode::RUNNING) {
        std::cout << "brakeRelease: SKIP, robot already RUNNING\n";
    } else {
        std::cout << "brakeRelease: FAIL, unsupported robotMode=" << robotModeName(mode) << '\n';
        return false;
    }

    std::cout << "final safety=" << safetyModeName(dashboard.safetyMode())
              << ", robotMode=" << robotModeName(dashboard.robotMode())
              << ", task=" << taskStatusName(dashboard.getTaskStatus()) << '\n';
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string ip = argc > 1 ? argv[1] : "172.16.100.82";
    const int cycles = argc > 2 ? std::stoi(argv[2]) : 1;
    ELITE::DashboardClient dashboard;

    if (!tryStep("dashboard.connect", [&] { return dashboard.connect(ip); })) {
        return 1;
    }

    for (int i = 0; i < cycles; ++i) {
        std::cout << "========== cycle " << (i + 1) << "/" << cycles << " ==========\n";

        if (!ensurePowerOff(dashboard)) {
            return 2;
        }

        if (!runStartup(dashboard)) {
            return 3;
        }

        if (i + 1 < cycles) {
            if (!ensurePowerOff(dashboard)) {
                return 4;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }

    return 0;
}
