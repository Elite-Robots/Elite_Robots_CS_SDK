// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
#include <Elite/EliteDriver.hpp>
#include <Elite/Log.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <iostream>
#include <thread>

using namespace ELITE;
namespace po = boost::program_options;
using namespace std::chrono;

int main(int argc, char** argv) {
    EliteDriverConfig config;
    std::string pw;
    // Parser param
    po::options_description desc(
        "Usage:\n"
        "\t./serial_example <--robot-ip=ip> [--local-ip=\"\"] [--use-headless-mode=true]\n"
        "Parameters:");
    desc.add_options()
        ("help,h", "Print help message")
        ("robot-ip", po::value<std::string>(&config.robot_ip)->required(),"\tRequired. IP address of the robot.")
        ("use-headless-mode", po::value<bool>(&config.headless_mode)->required()->implicit_value(true), "\tRequired. Use headless mode.")
        ("ssh-pw", po::value<std::string>(&pw)->required(), "\tRequired. Controller box OS ssh password.")
        ("local-ip", po::value<std::string>(&config.local_ip)->default_value(""), "\tOptional. IP address of the local network interface.");

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
    auto driver = std::make_unique<EliteDriver>(config);
    PrimaryPortInterface& primary = driver->primaryPort();

    ELITE_LOG_INFO("Start powering on...");
    if (!primary.powerOn()) {
        ELITE_LOG_FATAL("Power-on failed");
        return 1;
    }
    ELITE_LOG_INFO("Power-on succeeded");

    ELITE_LOG_INFO("Start releasing brake...");
    if (!primary.brakeRelease()) {
        ELITE_LOG_FATAL("Release brake fail");
        return 1;
    }
    ELITE_LOG_INFO("Brake released");

    if (config.headless_mode) {
        if (!driver->isRobotConnected()) {
            if (!driver->sendExternalControlScript()) {
                ELITE_LOG_FATAL("Fail to send external control script");
                return 1;
            }
        }
    } else {
        ELITE_LOG_INFO("Please start the External Control task from the robot side.");
    }

    ELITE_LOG_INFO("Wait external control script run...");
    while (!driver->isRobotConnected()) {
        std::this_thread::sleep_for(10ms);
    }
    ELITE_LOG_INFO("External control script is running");

    SerialConfig serial_config;
    serial_config.baud_rate = SerialConfig::BaudRate::BR_115200;
    serial_config.parity = SerialConfig::Parity::NONE;
    serial_config.stop_bits = SerialConfig::StopBits::ONE;
    auto serial = driver->startToolRs485(serial_config, pw);
    if (!serial) {
        ELITE_LOG_FATAL("Start serial communication fail.");
        return 1;
    }

    ELITE_LOG_INFO("Connecting to serial socat server...");
    if(!serial->connect(1000)) {
        ELITE_LOG_FATAL("Can't connect socat server");
        return 1;
    }
    ELITE_LOG_INFO("Connected to serial socat server.");

    ELITE_LOG_INFO("Send data to serial...");
    std::string hello_str = "hello world";
    if (serial->write((const uint8_t*)hello_str.c_str(), hello_str.size()) <= 0) {
        ELITE_LOG_FATAL("Send data to serial fail.");
        return 1;
    }
    ELITE_LOG_INFO("Data sent.");

    ELITE_LOG_INFO("Read data from serial...");
    std::string receive_str;
    receive_str.resize(hello_str.size());
    if(serial->read((uint8_t*)receive_str.data(), receive_str.size(), 5000) <= 0) {
        ELITE_LOG_INFO("Read data to serial fail.");
    }
    ELITE_LOG_INFO("Receive:%s", receive_str.c_str());

    ELITE_LOG_INFO("Ending serial communication...");
    serial->disconnect();
    driver->endToolRs485(serial, pw);
    driver->stopControl();
    ELITE_LOG_INFO("Serial communication ended.");

    return 0;
}
