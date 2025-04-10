#include "RtsiIOInterface.hpp"
#include "EliteException.hpp"
#include "Log.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>

using namespace ELITE;

RtsiIOInterface::RtsiIOInterface(const std::string& output_recipe_file, const std::string& input_recipe_file, double frequency) 
    : output_recipe_string_(readRecipe(output_recipe_file))
    , input_recipe_string_(readRecipe(input_recipe_file))
    , target_frequency_(frequency){

}

RtsiIOInterface::~RtsiIOInterface() {
    disconnect();
}

bool RtsiIOInterface::connect(const std::string& ip) {
    if (isConnected() || recv_thread_) {
        disconnect();
    }
    
    RtsiClientInterface::connect(ip);
    
    if(!negotiateProtocolVersion()) {
        ELITE_LOG_FATAL("RTSI negitiate protocol version fail.");
        return false;
    }

    controller_version_ = RtsiClientInterface::getControllerVersion();

    // Setup input and output recipe. 
    // Send start signal
    try {
        setupRecipe();
        if (!start()) {
            ELITE_LOG_FATAL("RTSI start signal send fail.");
            return false;
        }
    } catch(const EliteException& e) {
        if (e == EliteException::Code::RTSI_UNKNOW_VARIABLE_TYPE) {
            ELITE_LOG_FATAL("RTSI setup recipe fail. Check recipe files.");
            disconnect();
            return false;
        } else {
            throw e;
        }
    }

    // The recv thread must create after setup recipe, because 'output_recipe_' get in setup 
    is_recv_thread_alive_ = true;
    recv_thread_.reset(new std::thread([&](){
        recvLoop();
    }));
    // Wait for recv_thread_ run
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return true;
}

void RtsiIOInterface::disconnect() {
    if (recv_thread_ && recv_thread_->joinable()) {
        is_recv_thread_alive_ = false;
        recv_thread_->join();
    }
    RtsiClientInterface::disconnect();
}

VersionInfo RtsiIOInterface::getControllerVersion() {
    return controller_version_;
}

bool RtsiIOInterface::setSpeedScaling(double slider) {
    if (input_recipe_) {
        if(!setInputRecipeValue("speed_slider_mask", 1)) {
            return false;
        }
        if(!setInputRecipeValue("speed_slider_fraction", slider)) {
            return false;
        }
    }
    
    return true;
}

bool RtsiIOInterface::setStandardDigital(int index, bool level) {
    if (input_recipe_) {
        uint16_t digital_mask = 1 << index;
        if(!setInputRecipeValue("standard_digital_output_mask", digital_mask)) {
            return false;
        }
        uint16_t digital = level << index;
        if(!setInputRecipeValue("standard_digital_output", digital)) {
            return false;
        }
    }
    
    return true;
}

bool RtsiIOInterface::setConfigureDigital(int index, bool level) {
    if (input_recipe_) {
        uint8_t digital_mask = 1 << index;
        if(!setInputRecipeValue("configurable_digital_output_mask", digital_mask)) {
            return false;
        }
        uint8_t digital = level << index;
        if(!setInputRecipeValue("configurable_digital_output", digital)) {
            return false;
        }
    }
    return true;
}

bool RtsiIOInterface::setAnalogOutputVoltage(int index, double value) {
    if (input_recipe_) {
        uint8_t mask = 1 << index;
        // value = (max - min) * level + min
        // level = (value - min) / (max - min)
        double level = value / 10.0;
        if(!setInputRecipeValue("standard_analog_output_type", 3)) {
            return false;
        }
        if (index == 0) {
            if(!setInputRecipeValue("standard_analog_output_mask", 1)) {
                return false;
            }

            if(!setInputRecipeValue("standard_analog_output_0", level)) {
                return false;
            }
        } else if(index == 1) {
            if(!setInputRecipeValue("standard_analog_output_mask", 2)) {
                return false;
            }

            if(!setInputRecipeValue("standard_analog_output_1", level)) {
                return false;
            }
        } else {
            if(!setInputRecipeValue("standard_analog_output_mask", 0)) {
                return false;
            }
        }
    }
    return true;
}

bool RtsiIOInterface::setAnalogOutputCurrent(int index, double value) {
    if (input_recipe_) {
        uint8_t mask = 1 << index;
        // value = (max - min) * level + min
        // level = (value - min) / (max - min)
        double level = (value - 0.004) / (0.02 - 0.004);
        if(!setInputRecipeValue("standard_analog_output_type", 0)) {
            return false;
        }
        if (index == 0) {
            if(!setInputRecipeValue("standard_analog_output_mask", 1)) {
                return false;
            }

            if(!setInputRecipeValue("standard_analog_output_0", level)) {
                return false;
            }
        } else if(index == 1) {
            if(!setInputRecipeValue("standard_analog_output_mask", 2)) {
                return false;
            }

            if(!setInputRecipeValue("standard_analog_output_1", level)) {
                return false;
            }
        } else {
            if(!setInputRecipeValue("standard_analog_output_mask", 0)) {
                return false;
            }
        }
    }
    return true;
}

bool RtsiIOInterface::setExternalForceTorque(const vector6d_t& value) {
    if (input_recipe_) {
        if(!setInputRecipeValue("external_force_torque", value)) {
            return false;
        }
    }
    return true;
}

bool RtsiIOInterface::setToolDigitalOutput(int index, bool level) {
    if (input_recipe_) {
        uint8_t mask = 1 << index;
        if(!setInputRecipeValue("tool_digital_output_mask", mask)) {
            return false;
        }
        uint8_t digital = level << index;
        if(!setInputRecipeValue("tool_digital_output", digital)) {
            return false;
        }
    }
    return true;
}

double RtsiIOInterface::getTimestamp() {
    double result = 0;
    getRecipeValue("timestamp", result);
    return result;
}

double RtsiIOInterface::getPayloadMass() {
    double result = 0;
    getRecipeValue("payload_mass", result);
    return result;
}

vector3d_t RtsiIOInterface::getPayloadCog() {
    vector3d_t result;
    getRecipeValue("payload_cog", result);
    return result;
}

vector6d_t RtsiIOInterface::getTargetJointPositions() {
    vector6d_t result{0};
    getRecipeValue("target_joint_positions", result);
    return result;
}

uint32_t RtsiIOInterface::getScriptControlLine() {
    uint32_t result{0};
    getRecipeValue("script_control_line", result);
    return result;
}

vector6d_t RtsiIOInterface::getTargetJointVelocity() {
    vector6d_t result{0};
    getRecipeValue("target_joint_speeds", result);
    return result;
}

vector6d_t RtsiIOInterface::getActualJointPositions() {
    vector6d_t result{0};
    getRecipeValue("actual_joint_positions", result);
    return result;
}

vector6d_t RtsiIOInterface::getActualJointTorques() {
    vector6d_t result{0};
    getRecipeValue("actual_joint_torques", result);
    return result;
}

vector6d_t RtsiIOInterface::getActualJointVelocity() {
    vector6d_t result{0};
    getRecipeValue("actual_joint_speeds", result);
    return result;
}

vector6d_t RtsiIOInterface::getActualJointCurrent() {
    vector6d_t result{0};
    getRecipeValue("actual_joint_current", result);
    return result;
}

vector6d_t RtsiIOInterface::getActualJointTemperatures() {
    vector6d_t result{0};
    getRecipeValue("joint_temperatures", result);
    return result;
}

vector6d_t RtsiIOInterface::getAcutalTCPPose() {
    vector6d_t result{0};
    getRecipeValue("actual_TCP_pose", result);
    return result;
}

vector6d_t RtsiIOInterface::getAcutalTCPVelocity() {
    vector6d_t result{0};
    getRecipeValue("actual_TCP_speed", result);
    return result;
}

vector6d_t RtsiIOInterface::getAcutalTCPForce() {
    vector6d_t result{0};
    getRecipeValue("actual_TCP_force", result);
    return result;
}

vector6d_t RtsiIOInterface::getTargetTCPPose() {
    vector6d_t result{0};
    getRecipeValue("target_TCP_pose", result);
    return result;
}

vector6d_t RtsiIOInterface::getTargetTCPVelocity() {
    vector6d_t result{0};
    getRecipeValue("target_TCP_speed", result);
    return result;
}

uint32_t RtsiIOInterface::getDigitalInputBits() {
    uint32_t result{0};
    getRecipeValue("actual_digital_input_bits", result);
    return result;
}

uint32_t RtsiIOInterface::getDigitalOutputBits() {
    uint32_t result{0};
    getRecipeValue("actual_digital_output_bits", result);
    return result;
}

RobotMode RtsiIOInterface::getRobotMode() {
    int32_t result = 0;
    getRecipeValue("robot_mode", result);
    return static_cast<RobotMode>(result);
}

std::array<JointMode, 6> RtsiIOInterface::getJointMode() {
    vector6int32_t modes;
    getRecipeValue("joint_mode", modes);
    std::array<JointMode, 6> joint_modes;
    std::memcpy(joint_modes.data(), modes.data(), sizeof(joint_modes));
    return joint_modes;
}

SafetyMode RtsiIOInterface::getSafetyStatus() {
    int32_t result = 0;
    getRecipeValue("safety_status", result);
    return static_cast<SafetyMode>(result);
}

double RtsiIOInterface::getActualSpeedScaling() {
    double result = 0;
    getRecipeValue("speed_scaling", result);
    return result;
}

double RtsiIOInterface::getTargetSpeedScaling() {
    double result = 0;
    getRecipeValue("target_speed_fraction", result);
    return result;
}

double RtsiIOInterface::getRobotVoltage() {
    double result = 0;
    getRecipeValue("actual_robot_voltage", result);
    return result;
}

double RtsiIOInterface::getRobotCurrent() {
    double result = 0;
    getRecipeValue("actual_robot_current", result);
    return result;
}

TaskStatus RtsiIOInterface::getRuntimeState() {
    uint32_t result = 0;
    getRecipeValue("runtime_state", result);
    return static_cast<TaskStatus>(result);
}

vector3d_t RtsiIOInterface::getElbowPosition() {
    vector3d_t result{0};
    getRecipeValue("elbow_position", result);
    return result;
}

vector3d_t RtsiIOInterface::getElbowVelocity() {
    vector3d_t result{0};
    getRecipeValue("elbow_velocity", result);
    return result;
}

uint32_t RtsiIOInterface::getRobotStatus() {
    uint32_t result{0};
    getRecipeValue("robot_status_bits", result);
    return result;
}

uint32_t RtsiIOInterface::getSafetyStatusBits() {
    uint32_t result{0};
    getRecipeValue("safety_status_bits", result);
    return result;
}

uint32_t RtsiIOInterface::getAnalogIOTypes() {
    uint32_t result{0};
    getRecipeValue("analog_io_types", result);
    return result;
}

double RtsiIOInterface::getAnalogInput(int index) {
    double result{0};
    if (index == 0) {
        getRecipeValue("standard_analog_input0", result);
    } else {
        getRecipeValue("standard_analog_input1", result);
    }
    return result;
}

double RtsiIOInterface::getAnalogOutput(int index) {
    double result{0};
    if (index == 0) {
        getRecipeValue("standard_analog_output0", result);
    } else {
        getRecipeValue("standard_analog_output1", result);
    }
    return result;
}

double RtsiIOInterface::getIOCurrent() {
    double result = 0;
    getRecipeValue("io_current", result);
    return result;
}

ToolMode RtsiIOInterface::getToolMode() {
    uint32_t result{0};
    getRecipeValue("tool_mode", result);
    return static_cast<ToolMode>(result);
}


uint32_t RtsiIOInterface::getToolAnalogInputType() {
    uint32_t result{0};
    getRecipeValue("tool_analog_input_types", result);
    return result;
}

uint32_t RtsiIOInterface::getToolAnalogOutputType() {
    uint32_t result{0};
    getRecipeValue("tool_analog_output_types", result);
    return result;
}

double RtsiIOInterface::getToolAnalogInput() {
    double result = 0;
    getRecipeValue("tool_analog_input", result);
    return result;
}

double RtsiIOInterface::getToolAnalogOutput() {
    double result = 0;
    getRecipeValue("tool_analog_output", result);
    return result;
}

double RtsiIOInterface::getToolOutputVoltage() {
    double result = 0;
    getRecipeValue("tool_output_voltage", result);
    return result;
}

double RtsiIOInterface::getToolOutputCurrent() {
    double result = 0;
    getRecipeValue("tool_output_current", result);
    return result;
}

double RtsiIOInterface::getToolOutputTemperature() {
    double result = 0;
    getRecipeValue("tool_temperature", result);
    return result;
}

ToolDigitalMode RtsiIOInterface::getToolDigitalMode() {
    uint8_t result = 0;
    getRecipeValue("tool_digital_mode", result);
    return static_cast<ToolDigitalMode>(result);
}

ToolDigitalOutputMode RtsiIOInterface::getToolDigitalOutputMode(int index) {
    uint8_t result = 0;
    if (index == 0) {
        getRecipeValue("tool_digital0_mode", result);
        return static_cast<ToolDigitalOutputMode>(result);
    } else if(index == 1) {
        getRecipeValue("tool_digital1_mode", result);
        return static_cast<ToolDigitalOutputMode>(result);
    } else if(index == 2) {
        getRecipeValue("tool_digital2_mode", result);
        return static_cast<ToolDigitalOutputMode>(result);
    } else if(index == 3) {
        getRecipeValue("tool_digital3_mode", result);
        return static_cast<ToolDigitalOutputMode>(result);
    }
    return ToolDigitalOutputMode();
}

uint32_t RtsiIOInterface::getOutBoolRegisters0To31() {
    uint32_t result = 0;
    getRecipeValue("output_bit_registers0_to_31", result);
    return result;
}

uint32_t RtsiIOInterface::getOutBoolRegisters32To63() {
    uint32_t result = 0;
    getRecipeValue("output_bit_registers32_to_63", result);
    return result;
}

uint32_t RtsiIOInterface::getInBoolRegisters0To31() {
    uint32_t result = 0;
    getRecipeValue("input_bit_registers0_to_31", result);
    return result;
}

uint32_t RtsiIOInterface::getInBoolRegisters32To63() {
    uint32_t result = 0;
    getRecipeValue("input_bit_registers32_to_63", result);
    return result;
}

bool RtsiIOInterface::getInBoolRegister(int index) {
    bool result = 0;
    getRecipeValue("input_bit_register" + std::to_string(index), result);
    return result;
}

bool RtsiIOInterface::getOutBoolRegister(int index) {
    bool result = 0;
    getRecipeValue("output_bit_register" + std::to_string(index), result);
    return result;
}

int32_t RtsiIOInterface::getInIntRegister(int index) {
    int32_t result = 0;
    getRecipeValue("input_int_register" + std::to_string(index), result);
    return result;
}

int32_t RtsiIOInterface::getOutIntRegister(int index) {
    int32_t result = 0;
    getRecipeValue("output_int_register" + std::to_string(index), result);
    return result;
}

double RtsiIOInterface::getInDoubleRegister(int index) {
    double result = 0;
    getRecipeValue("input_double_register" + std::to_string(index), result);
    return result;
}

double RtsiIOInterface::getOutDoubleRegister(int index) {
    double result = 0;
    getRecipeValue("output_double_register" + std::to_string(index), result);
    return result;
}

std::vector<std::string> RtsiIOInterface::readRecipe(const std::string& recipe_file) {
    std::vector<std::string> recipe;
    std::ifstream file(recipe_file);
    if (file.fail()) {
        std::stringstream msg;
        msg << "Opening file '" << recipe_file << "' failed with error: " << strerror(errno);
        throw EliteException(EliteException::Code::FILE_OPEN_FAIL, msg.str());
    }

    if (file.peek() == std::ifstream::traits_type::eof()) {
        std::stringstream msg;
        msg << "The recipe '" << recipe_file << "' file is empty exiting ";
        throw EliteException(EliteException::Code::FILE_OPEN_FAIL, msg.str());
    }

    std::string line;
    while (std::getline(file, line)) {
        recipe.push_back(line);
    }

    return recipe;
}

void RtsiIOInterface::setupRecipe() {
    input_recipe_ = setupInputRecipe(input_recipe_string_);
    output_recipe_ = setupOutputRecipe(output_recipe_string_, target_frequency_);
}

void RtsiIOInterface::recvLoop() {
    // Calculate the ideal cycle time.
    double period_ms = (1 / target_frequency_) * 1000;
    ELITE_LOG_INFO("RTSI IO interface sync thread start, period %lfms", period_ms);
    while (is_recv_thread_alive_) {
        try {
            receiveData(output_recipe_, false);
            if (input_new_cmd_) {
                send(input_recipe_);
                input_new_cmd_ = false;
            }
        } catch(const std::exception& e) {
            is_recv_thread_alive_ = false;
        }
    }
    ELITE_LOG_INFO("RTSI IO interface sync thread dropped");
}

template<typename T>
void RtsiIOInterface::getRecipeValue(const std::string& name, T& out_value) {
    if (output_recipe_) {
        output_recipe_->getValue(name, out_value);
    }
}

template<typename T>
bool RtsiIOInterface::setInputRecipeValue(const std::string &name, const T& value) {
    bool ret = input_recipe_->setValue(name, value);
    input_new_cmd_ = true;
    return ret;
}

template void RtsiIOInterface::getRecipeValue<double>(const std::string &name, double& out_value);
template void RtsiIOInterface::getRecipeValue<bool>(const std::string &name, bool& out_value);
template void RtsiIOInterface::getRecipeValue<int>(const std::string &name, int& out_value);

template bool RtsiIOInterface::setInputRecipeValue<double>(const std::string &name, const double& value);
template bool RtsiIOInterface::setInputRecipeValue<bool>(const std::string &name, const bool& value);
template bool RtsiIOInterface::setInputRecipeValue<int>(const std::string &name, const int& value);
