// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
#include <Elite/DashboardClient.hpp>
#include <Elite/DataType.hpp>
#include <Elite/EliteDriver.hpp>
#include <Elite/Log.hpp>
#include <Elite/RtsiIOInterface.hpp>
#include <Elite/RtUtils.hpp>

#include <boost/program_options.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <thread>

#if defined(__linux) || defined(linux) || defined(__linux__)
#include <sys/mman.h>
#endif

using namespace ELITE;
using namespace std::chrono;
namespace po = boost::program_options;

static std::unique_ptr<EliteDriver> s_driver;
static std::unique_ptr<RtsiIOInterface> s_rtsi_client;
static std::unique_ptr<DashboardClient> s_dashboard;

struct TrapezoidalPoint {
    double t;
    double pos;
    double vel;
    double acc;
};

std::vector<TrapezoidalPoint> trapezoidalSpeedPlan(double start, double end, double vmax, double amax, double dt) {
    std::vector<TrapezoidalPoint> trajectory;

    // total displacement
    double dq = end - start;
    double dir = (dq >= 0) ? 1.0 : -1.0;
    dq = std::fabs(dq);

    // the time required for acceleration and deceleration
    double t_acc = vmax / amax;
    double d_acc = 0.5 * amax * t_acc * t_acc;

    double t_flat = 0.0;
    if (dq < 2 * d_acc) {
        // Fail to reach the maximum speed; switch to a triangular speed curve.
        t_acc = std::sqrt(dq / amax);
        d_acc = 0.5 * amax * t_acc * t_acc;
        t_flat = 0.0;
    } else {
        t_flat = (dq - 2 * d_acc) / vmax;
    }

    // Total time
    double T = 2 * t_acc + t_flat;

    for (double t = 0.0; t <= T + 1e-6; t += dt) {
        double pos, vel, acc;

        if (t < t_acc) {
            // acc
            acc = dir * amax;
            vel = acc * t;
            pos = start + dir * (0.5 * amax * t * t);
        } else if (t < t_acc + t_flat) {
            // constant
            acc = 0.0;
            vel = dir * vmax;
            pos = start + dir * (d_acc + vmax * (t - t_acc));
        } else if (t <= T) {
            // dec
            double td = t - (t_acc + t_flat);
            acc = -dir * amax;
            vel = dir * vmax - amax * td * dir;
            pos = end - dir * (0.5 * amax * (T - t) * (T - t));
        } else {
            // finish
            acc = 0.0;
            vel = 0.0;
            pos = end;
        }

        trajectory.push_back({t, pos, vel, acc});
    }

    return trajectory;
}



int main(int argc, char** argv) {
#if defined(__linux) || defined(linux) || defined(__linux__)
    mlockall(MCL_CURRENT | MCL_FUTURE);
    pthread_t handle = pthread_self();
    RT_UTILS::setThreadFiFoScheduling(handle, RT_UTILS::getThreadFiFoMaxPriority());
    RT_UTILS::bindThreadToCpus(handle, 2);
#endif

    EliteDriverConfig config;
    std::string output_file;
    double max_speed = 0;
    double max_acc = 0;

    // Parser param
    po::options_description desc(
        "Usage:\n"
        "\t./servoj_plan_example <--robot-ip=ip> [--local-ip=\"\"] [--use-headless-mode=true]\n"
        "Parameters:");
    desc.add_options()
        ("help,h", "Print help message")
        ("robot-ip", po::value<std::string>(&config.robot_ip)->required(),
            "\tRequired. IP address of the robot.")
        ("use-headless-mode", po::value<bool>(&config.headless_mode)->required()->implicit_value(true),
            "\tRequired. Use headless mode.")
        ("local-ip", po::value<std::string>(&config.local_ip)->default_value(""),
            "\tOptional. IP address of the local network interface.")
        ("max-speed", po::value<double>(&max_speed)->default_value(2.0),
            "\tOptional. The joint max speed")
        ("max-acc", po::value<double>(&max_acc)->default_value(2.0),
            "\tOptional. The joint max acc")
        ("output-file", po::value<std::string>(&output_file)->default_value("servoj_plan_data.csv"),
            "\tOptional. The joint max acc");

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help")) {
            std::cout << desc << std::endl;
            return 0;
        }

        po::notify(vm);
    } catch (const po::error& e) {
        std::cerr << "Argument error: " << e.what() << "\n\n";
        std::cerr << desc << "\n";
        return 1;
    }

    if (config.headless_mode) {
        ELITE_LOG_WARN("Use headless mode. Please ensure the robot is not in local mode.");
    } else {
        ELITE_LOG_WARN(
            "It needs to be correctly configured, and the External Control plugin should be inserted into the task tree.");
    }

    config.script_file_path = "external_control.script";
    config.servoj_time = 0.004;
    s_driver = std::make_unique<EliteDriver>(config);
    s_rtsi_client = std::make_unique<RtsiIOInterface>("output_recipe.txt", "input_recipe.txt", 250);
    s_dashboard = std::make_unique<DashboardClient>();
    ELITE_LOG_INFO("Connecting to the dashboard");
    if (!s_dashboard->connect(config.robot_ip)) {
        ELITE_LOG_FATAL("Failed to connect to the dashboard.");
        return 1;
    }
    ELITE_LOG_INFO("Successfully connected to the dashboard");

    ELITE_LOG_INFO("Connecting to the RTSI");
    if (!s_rtsi_client->connect(config.robot_ip)) {
        ELITE_LOG_FATAL("Failed to connect to the RTSI.");
        return 1;
    }
    ELITE_LOG_INFO("Successfully connected to the RTSI");

    VersionInfo version = s_rtsi_client->getControllerVersion();
    ELITE_LOG_INFO("Controller version is %s", version.toString().c_str());

    ELITE_LOG_INFO("Start powering on...");
    if (!s_dashboard->powerOn()) {
        ELITE_LOG_FATAL("Power-on failed");
        return 1;
    }
    ELITE_LOG_INFO("Power-on succeeded");

    ELITE_LOG_INFO("Start releasing brake...");
    if (!s_dashboard->brakeRelease()) {
        ELITE_LOG_FATAL("Release brake fail");
        return 1;
    }
    ELITE_LOG_INFO("Brake released");

    if (config.headless_mode) {
        if (!s_driver->isRobotConnected()) {
            if (!s_driver->sendExternalControlScript()) {
                ELITE_LOG_FATAL("Fail to send external control script");
                return 1;
            }
        }
    } else {
        if (!config.headless_mode && !s_dashboard->playProgram()) {
            ELITE_LOG_FATAL("Fail to play program");
            return 1;
        }
    }

    ELITE_LOG_INFO("Wait external control script run...");
    while (!s_driver->isRobotConnected()) {
        std::this_thread::sleep_for(10ms);
    }
    ELITE_LOG_INFO("External control script is running");

    bool positive_rotation = false;
    bool negative_rotation = false;
    vector6d_t actual_joint = s_rtsi_client->getActualJointPositions();
    vector6d_t target_joint = actual_joint;
    double increment = 0;
    auto next = steady_clock::now();
    constexpr double JOINT_FINAL_TARGET = 3.0;
    std::vector<TrapezoidalPoint> plan_joint;
    
    while (!(positive_rotation && negative_rotation)) {
        if (!positive_rotation) {
            plan_joint = trapezoidalSpeedPlan(target_joint[5], JOINT_FINAL_TARGET, max_speed, max_acc, config.servoj_time);
            positive_rotation = true;
        } else if (!negative_rotation) {
            plan_joint = trapezoidalSpeedPlan(target_joint[5], -JOINT_FINAL_TARGET, max_speed, max_acc, config.servoj_time);
            negative_rotation = true;
        }
        
        for (auto& point: plan_joint) {
            target_joint[5] = point.pos;
            if (!s_driver->writeServoj(target_joint, 100, false)) {
                ELITE_LOG_FATAL("Send servoj command to robot fail");
                goto end;
            }
            next += 4ms;
            std::this_thread::sleep_until(next);
        }
    }

end:
    ELITE_LOG_INFO("Motion finish");
    s_driver->stopControl();

    return 0;
}
