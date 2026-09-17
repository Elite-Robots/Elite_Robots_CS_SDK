// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// RtsiIOInterface.hpp
// Wraps the RtsiClientInterface class for RTSI I/O communication with the robot.
#ifndef __RTSI_IO_INTERFACE_HPP__
#define __RTSI_IO_INTERFACE_HPP__

#include <Elite/DataType.hpp>
#include <Elite/EliteOptions.hpp>
#include <Elite/RtsiClientInterface.hpp>
#include <Elite/RtsiRecipe.hpp>
#include <Elite/VersionInfo.hpp>

#include <atomic>
#include <memory>
#include <string>
#include <thread>

namespace ELITE {

/**
 * @brief The RTSI interface has been functionally encapsulated.
 *
 */
class RtsiIOInterface : protected RtsiClientInterface {
   public:
    RtsiIOInterface() = delete;

    /**
     * @brief Construct a new Rtsi I O Interface object
     *
     * @param output_recipe_file Output recipe configuration file
     * @param input_recipe_file Input recipe configuration file. If empty
     * @param frequency Output frequency
     */
    ELITE_EXPORT explicit RtsiIOInterface(const std::string& output_recipe_file, const std::string& input_recipe_file,
                                          double frequency);

    /**
     * @brief Construct a new Rtsi I O Interface object
     *
     * @param output_recipe_file Output recipe configuration
     * @param input_recipe_file Input recipe configuration
     * @param frequency Output frequency
     */
    ELITE_EXPORT explicit RtsiIOInterface(const std::vector<std::string>& output_recipe,
                                          const std::vector<std::string>& input_recipe, double frequency);

    ELITE_EXPORT virtual ~RtsiIOInterface();

    /**
     * @brief Connect to RTSI server
     *
     * @param ip The IP of RTSI server
     * @return true connected success
     * @return false connected fail
     */
    ELITE_EXPORT virtual bool connect(const std::string& ip);

    /**
     * @brief Disconnect
     *
     * @note The function will block to wait for thread finish
     */
    ELITE_EXPORT virtual void disconnect();

    /**
     * @brief Get connection state
     *
     * @return true connected
     * @return false disconnect
     */
    ELITE_EXPORT virtual bool isConnected();

    /**
     * @brief Is start to sync robot data
     *
     * @return true started
     * @return false not started
     */
    ELITE_EXPORT virtual bool isStarted();

    /**
     * @brief Get the Controller Version
     *
     * @return std::tuple<uint32_t, uint32_t, uint32_t, uint32_t>
     * A tuple type where the data is, in order, major version, minor version, bugfix, and build.
     */
    ELITE_EXPORT virtual VersionInfo getControllerVersion();

    /**
     * @brief Set the robot speed scaling
     *
     * @param scaling The target scaling
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool setSpeedScaling(double scaling);

    /**
     * @brief Set the standard digital
     *
     * @param index The index of standard digital IO
     * @param level High or low level
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool setStandardDigital(int index, bool level);

    /**
     * @brief Set the configurable digital
     *
     * @param index The index of configurable digital IO
     * @param level High or low level
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool setConfigureDigital(int index, bool level);

    /**
     * @brief Set the output voltage
     *
     * @param index The index of analog IO
     * @param value The voltage. Unit: V, range [0. 10]V
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool setAnalogOutputVoltage(int index, double value);

    /**
     * @brief Set the output current
     *
     * @param index The index of analog IO
     * @param value The current. Unit: A, range [0.004, 0.2]A
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool setAnalogOutputCurrent(int index, double value);

    /**
     * @brief Used to input external force sensor data into the system. This data takes effect when ft_rtsi_input_enable is set to
     * true.
     *
     * @param value external force sensor data
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool setExternalForceTorque(const vector6d_t& value);

    /**
     * @brief Set the tool digital output level
     *
     * @param index The index of tool output IO
     * @param level level
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT bool setToolDigitalOutput(int index, bool level);

    /**
     * @return double timestamp. Unit: second.
     */
    ELITE_EXPORT double getTimestamp();

    /**
     * @return double Terminal payload, unit: kg
     */
    ELITE_EXPORT double getPayloadMass();

    /**
     * @return vector3d_t Barycenter of terminal payload, unit: m
     */
    ELITE_EXPORT vector3d_t getPayloadCog();

    /**
     * @return uint32_t The line number of the running script
     */
    ELITE_EXPORT uint32_t getScriptControlLine();

    /**
     * @return vector6d_t Target joint positions. Unit is rad.
     */
    ELITE_EXPORT vector6d_t getTargetJointPositions();

    /**
     * @return vector6d_t Target joint velocity. Unit is rad/s.
     */
    ELITE_EXPORT vector6d_t getTargetJointVelocity();

    /**
     * @return vector6d_t Actual joint positions. Unit is rad.
     */
    ELITE_EXPORT vector6d_t getActualJointPositions();

    /**
     * @return vector6d_t Actual joint torques. Unit is N*m.
     */
    ELITE_EXPORT vector6d_t getActualJointTorques();

    /**
     * @return vector6d_t Actual joint velocity. Unit is rad/s.
     */
    ELITE_EXPORT vector6d_t getActualJointVelocity();

    /**
     * @return vector6d_t Actual joint current. Unit is A.
     */
    ELITE_EXPORT vector6d_t getActualJointCurrent();

    /**
     * @return vector6d_t Actual joint temperatures. Unit is Celsius degrees.
     */
    ELITE_EXPORT vector6d_t getActualJointTemperatures();

    /**
     * @return vector6d_t Actual cartesian coordinates of the tool: [x, y, z, rx, ry, rz],
     * where x, y, z is a position vection, and rx, ry, rz is a rotation vector.
     */
    [[deprecated("Use getActualTCPPose() instead. This function will be removed in June 2027.")]] ELITE_EXPORT vector6d_t
    getAcutalTCPPose();
    ELITE_EXPORT vector6d_t getActualTCPPose();
    /**
     * @param user_frame_pose User frame pose relative to the base frame:
     *                        [x, y, z, rx, ry, rz], in meters and radians.
     * @return Actual TCP pose expressed in the specified user frame.
     */
    ELITE_EXPORT vector6d_t getActualTCPPose(const vector6d_t& user_frame_pose);
    /**
     * @param user_frame User frame whose pose is relative to the base frame.
     * @return Actual TCP pose expressed in the specified user frame.
     */
    ELITE_EXPORT vector6d_t getActualTCPPose(const UserFrame& user_frame);

    /**
     * @return vector6d_t Actual speed of the tool given in cartesian coordinates: [x, y, z, rx, ry, rz]/s,
     * where x, y, z is a position vection, and rx, ry, rz is a rotation vector.
     */
    [[deprecated("Use getActualTCPPose() instead. This function will be removed in June 2027.")]] ELITE_EXPORT vector6d_t
    getAcutalTCPVelocity();
    ELITE_EXPORT vector6d_t getActualTCPVelocity();

    /**
     * @returns vector6d_t Generalized forces in the TCP. (Subtract the force data caused by the load.)
     */
    [[deprecated("Use getActualTCPPose() instead. This function will be removed in June 2027.")]] ELITE_EXPORT vector6d_t
    getAcutalTCPForce();
    ELITE_EXPORT vector6d_t getActualTCPForce();

    /**
     * @returns vector6d_t Target cartesian coordinates of the tool: [x, y, z, rx, ry, rz],
     * where x, y, z is a position vection, and rx, ry, rz is a rotation vector.
     */
    ELITE_EXPORT vector6d_t getTargetTCPPose();

    /**
     * @return vector6d_t Target speed of the tool given in cartesian coordinates: [x, y, z, rx, ry, rz],
     * where x, y, z is a position vection, and rx, ry, rz is a rotation vector.
     */
    ELITE_EXPORT vector6d_t getTargetTCPVelocity();

    /**
     * @return uint32_t The bit values of all digital input I/Os.
     * @note
     *      bits 0 to 15 is standard digital input
     *      bits 16 to 24 is configure digital input
     *      bits 24 to 28 is tool digital input
     */
    ELITE_EXPORT uint32_t getDigitalInputBits();

    /**
     * @return uint32_t The bit values of all digital output I/Os.
     * @note
     *      bits 0 to 15 is standard digital input
     *      bits 16 to 24 is configure digital input
     *      bits 24 to 28 is tool digital input
     */
    ELITE_EXPORT uint32_t getDigitalOutputBits();

    /**
     * @return RobotMode Robot mode
     */
    ELITE_EXPORT RobotMode getRobotMode();

    /**
     * @return std::array<JointMode, 6> Every joint mode
     */
    ELITE_EXPORT std::array<JointMode, 6> getJointMode();

    /**
     * @return SafetyMode Robot safety mode
     */
    ELITE_EXPORT SafetyMode getSafetyStatus();

    /**
     * @return double Actual robot speed scaling
     */
    ELITE_EXPORT double getActualSpeedScaling();

    /**
     * @return double Target robot speed scaling
     */
    ELITE_EXPORT double getTargetSpeedScaling();

    /**
     * @return double Robot voltage (48V)
     */
    ELITE_EXPORT double getRobotVoltage();

    /**
     * @return double Robot current
     */
    ELITE_EXPORT double getRobotCurrent();

    /**
     * @return uint32_t Program state
     */
    ELITE_EXPORT TaskStatus getRuntimeState();

    /**
     * @return vector3d_t The real-time position of the robot's elbow: [x, y, z]
     */
    ELITE_EXPORT vector3d_t getElbowPosition();

    /**
     * @return vector3d_t The real-time velocity of the robot's elbow: [x, y, z]/s
     */
    ELITE_EXPORT vector3d_t getElbowVelocity();

    /**
     * @return uint32_t Robot status. bits 0-2: is power on | is program running | is freedrive button pressed
     */
    ELITE_EXPORT uint32_t getRobotStatus();

    /**
     * @return uint32_t Robot safety bits. Bits 0-10: is normal mode | is reduced mode | is protective stopped | is recovery mode |
     * is safeguard stopped | is system emergency stopped | is robot emergency stopped | is emergency stopped | is violation | is
     * fault | is stoppeddue to safety
     */
    ELITE_EXPORT uint32_t getSafetyStatusBits();

    /**
     * @return uint32_t Analog IO type. Bits 0-3: analog input 0 (bit 0), analog input 1 (bit 1), analog output 0 (bit 2), analog
     * output 1 (bit 3). 0:  current mode; 1: voltage mode
     */
    ELITE_EXPORT uint32_t getAnalogIOTypes();

    /**
     * @param index The index of standard analog input: [0, 1]
     * @return double Standard analog input [A or V]
     */
    ELITE_EXPORT double getAnalogInput(int index);

    /**
     * @param index The index of standard analog output: [0, 1]
     * @return double Standard analog output, unit:[A or V]
     */
    ELITE_EXPORT double getAnalogOutput(int index);

    /**
     * @return double Mainboard IO current, unit: A
     */
    ELITE_EXPORT double getIOCurrent();

    /**
     * @return double Get tool mode
     */
    ELITE_EXPORT ToolMode getToolMode();

    /**
     * @return uint32_t Tool analog input IO mode, 0: current mode, 1: voltage mode
     */
    ELITE_EXPORT uint32_t getToolAnalogInputType();

    /**
     * @return uint32_t Tool analog output IO mode, 0: current mode, 1: voltage mode
     */
    ELITE_EXPORT uint32_t getToolAnalogOutputType();

    /**
     * @return double Tool analog input value, range: current mode[0.004-0.02]A,  voltage mode[0-10]V
     */
    ELITE_EXPORT double getToolAnalogInput();

    /**
     * @return double Tool analog output value, range: current mode[0.004-0.02]A,  voltage mode[0-10]V
     */
    ELITE_EXPORT double getToolAnalogOutput();

    /**
     * @return double Tool voltage value, unit: V
     */
    ELITE_EXPORT double getToolOutputVoltage();

    /**
     * @return double Tool current value, unit: A
     */
    ELITE_EXPORT double getToolOutputCurrent();

    /**
     * @return double Tool temperature value, unit: Celsius degrees
     */
    ELITE_EXPORT double getToolOutputTemperature();

    /**
     * @return double Tool digital mode: single-needle, double-needle mode 1, double-needle mode 2, triple-needle
     */
    ELITE_EXPORT ToolDigitalMode getToolDigitalMode();

    /**
     * @param index The index of tool digital
     * @return double Tool digital output mode: push/pull, npn, pnp
     */
    ELITE_EXPORT ToolDigitalOutputMode getToolDigitalOutputMode(int index);

    /**
     * @return uint32_t Output the value (0~31) of the boolean register.
     */
    ELITE_EXPORT uint32_t getOutBoolRegisters0To31();

    /**
     * @return uint32_t Output the value (32~63) of the boolean register.
     */
    ELITE_EXPORT uint32_t getOutBoolRegisters32To63();

    /**
     * @return uint32_t Input the value (0~31) of the boolean register.
     */
    ELITE_EXPORT uint32_t getInBoolRegisters0To31();

    /**
     * @return uint32_t Input the value (32~63) of the boolean register.
     */
    ELITE_EXPORT uint32_t getInBoolRegisters32To63();

    /**
     * @param index The index of bool registers, range: [64, 127]
     * @return bool Input bool register value
     */
    ELITE_EXPORT bool getInBoolRegister(int index);

    /**
     * @param index The index of bool registers
     * @return bool Output bool register value
     */
    ELITE_EXPORT bool getOutBoolRegister(int index);

    /**
     * @param index The index of int registers
     * @return int Input int register value
     */
    ELITE_EXPORT int32_t getInIntRegister(int index);

    /**
     * @param index The index of int registers
     * @return int Output int register value
     */
    ELITE_EXPORT int32_t getOutIntRegister(int index);

    /**
     * @param index The index of int registers
     * @return double Input double register value
     */
    ELITE_EXPORT double getInDoubleRegister(int index);

    /**
     * @param index The index of int registers
     * @return double Output double register value
     */
    ELITE_EXPORT double getOutDoubleRegister(int index);

    /**
     * @brief Get data from output recipe
     *
     * @tparam T data type
     * @param name Variable name
     * @param out_value Output value
     */
    template <typename T>
    bool getRecipeValue(const std::string& name, T& out_value) {
        if (output_recipe_) {
            return output_recipe_->getValue(name, out_value);
        }
        return false;
    }

    /**
     * @brief Set the input recipe value
     *
     * @tparam T data type
     * @param name Variable name
     * @param value Set value
     * @return false fail
     */
    template <typename T>
    bool setInputRecipeValue(const std::string& name, const T& value) {
        if (input_recipe_) {
            bool ret = input_recipe_->setValue(name, value);
            input_new_cmd_ = true;
            return ret;
        }
        return false;
    }

   private:
    std::atomic_bool input_new_cmd_;
    std::vector<std::string> input_recipe_string_;
    std::vector<std::string> output_recipe_string_;
    double target_frequency_;

    std::shared_ptr<RtsiRecipe> input_recipe_;
    std::shared_ptr<RtsiRecipe> output_recipe_;

    std::unique_ptr<std::thread> recv_thread_;
    std::atomic<bool> is_recv_thread_alive_;
    VersionInfo controller_version_;

    /**
     * @brief Continuously receive and parse data messages.
     *
     */
    void recvLoop();

    /**
     * @brief Setup input and output recipe
     *
     */
    void setupRecipe();

    /**
     * @brief Reads output or input recipe from a file
     *
     * @param file Recipe file
     * @return std::vector<std::string> The field for the subscription item
     */
    std::vector<std::string> readRecipe(const std::string& file);
};

// getRecipeValue() interface instantiate
template bool RtsiIOInterface::getRecipeValue<double>(const std::string& name, double& out_value);
template bool RtsiIOInterface::getRecipeValue<bool>(const std::string& name, bool& out_value);
template bool RtsiIOInterface::getRecipeValue<int8_t>(const std::string& name, int8_t& out_value);
template bool RtsiIOInterface::getRecipeValue<uint8_t>(const std::string& name, uint8_t& out_value);
template bool RtsiIOInterface::getRecipeValue<int16_t>(const std::string& name, int16_t& out_value);
template bool RtsiIOInterface::getRecipeValue<uint16_t>(const std::string& name, uint16_t& out_value);
template bool RtsiIOInterface::getRecipeValue<int32_t>(const std::string& name, int32_t& out_value);
template bool RtsiIOInterface::getRecipeValue<uint32_t>(const std::string& name, uint32_t& out_value);
template bool RtsiIOInterface::getRecipeValue<int64_t>(const std::string& name, int64_t& out_value);
template bool RtsiIOInterface::getRecipeValue<uint64_t>(const std::string& name, uint64_t& out_value);
template bool RtsiIOInterface::getRecipeValue<vector3d_t>(const std::string& name, vector3d_t& out_value);
template bool RtsiIOInterface::getRecipeValue<vector6d_t>(const std::string& name, vector6d_t& out_value);
template bool RtsiIOInterface::getRecipeValue<vector6int32_t>(const std::string& name, vector6int32_t& out_value);
template bool RtsiIOInterface::getRecipeValue<vector6uint32_t>(const std::string& name, vector6uint32_t& out_value);

// setInputRecipeValue() interface instantiate
template bool RtsiIOInterface::setInputRecipeValue<double>(const std::string& name, const double& value);
template bool RtsiIOInterface::setInputRecipeValue<bool>(const std::string& name, const bool& value);
template bool RtsiIOInterface::setInputRecipeValue<int8_t>(const std::string& name, const int8_t& value);
template bool RtsiIOInterface::setInputRecipeValue<uint8_t>(const std::string& name, const uint8_t& value);
template bool RtsiIOInterface::setInputRecipeValue<int16_t>(const std::string& name, const int16_t& value);
template bool RtsiIOInterface::setInputRecipeValue<uint16_t>(const std::string& name, const uint16_t& value);
template bool RtsiIOInterface::setInputRecipeValue<int32_t>(const std::string& name, const int32_t& value);
template bool RtsiIOInterface::setInputRecipeValue<uint32_t>(const std::string& name, const uint32_t& value);
template bool RtsiIOInterface::setInputRecipeValue<vector6d_t>(const std::string& name, const vector6d_t& value);
template bool RtsiIOInterface::setInputRecipeValue<vector6int32_t>(const std::string& name, const vector6int32_t& value);

}  // namespace ELITE

#endif
