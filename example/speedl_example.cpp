// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
#include <Elite/DataType.hpp>
#include <Elite/EliteDriver.hpp>
#include <Elite/Log.hpp>

#include <boost/program_options.hpp>
#include <iostream>
#include <memory>
#include <thread>

using namespace ELITE;
namespace po = boost::program_options;

static std::unique_ptr<EliteDriver> s_driver;

int main(int argc, char** argv) {
    EliteDriverConfig config;
    // Parser param
    po::options_description desc(
        "Usage:\n"
        "\t./speedl_example <--robot-ip=ip> [--local-ip=\"\"] [--use-headless-mode=true]\n"
        "Parameters:");
    desc.add_options()
        ("help,h", "Print help message")
        ("robot-ip", po::value<std::string>(&config.robot_ip)->required(),
            "\tRequired. IP address of the robot.")
        ("use-headless-mode", po::value<bool>(&config.headless_mode)->required()->implicit_value(true),
            "\tRequired. Use headless mode.")
        ("local-ip", po::value<std::string>(&config.local_ip)->default_value(""),
            "\tOptional. IP address of the local network interface.");

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
    s_driver = std::make_unique<EliteDriver>(config);
    PrimaryPortInterface& primary = s_driver->primaryPort();

    ELITE_LOG_INFO("Start powering on...");
    if (!primary.powerOn()) {
        ELITE_LOG_FATAL("Power-on failed");
        return 1;
    }
    ELITE_LOG_INFO("Power-on succeeded");

    ELITE_LOG_INFO("Start releasing brake...");
    if (!primary.brakeRelease()) {
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
        ELITE_LOG_INFO("Please start the External Control task from the robot side.");
    }

    ELITE_LOG_INFO("Wait external control script run...");
    while (!s_driver->isRobotConnected()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ELITE_LOG_INFO("External control script is running");

    vector6d_t speedl_vector = {0, 0, -0.02, 0, 0, 0};
    s_driver->writeSpeedl(speedl_vector, 0);
    std::this_thread::sleep_for(std::chrono::seconds(5));

    speedl_vector = {0, 0, 0.02, 0, 0, 0};
    s_driver->writeSpeedl(speedl_vector, 0);
    std::this_thread::sleep_for(std::chrono::seconds(5));

    s_driver->stopControl();

    return 0;
}
