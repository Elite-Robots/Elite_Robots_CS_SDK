# RTSI

## Introduction
RTSI is the real-time communication interface of Elite robots, which can be used to obtain the robot's status, set the I/O, etc. The SDK provides two interfaces for RTSI: `RtsiClientInterface` and `RtsiIOInterface`. `RtsiClientInterface` requires manual operations such as connection and version verification. `RtsiIOInterface`, on the other hand, encapsulates most of the interfaces. In actual tests, the real-time performance of `RtsiIOInterface` is slightly worse, while the real-time performance of `RtsiClientInterface` depends on the user's code.

# RtsiClientInterface Class

## Introduction
RTSI client

## Header File
```cpp
#include <Elite/RtsiClientInterface.hpp>
```

## Interfaces

### ***Connection***
```cpp
void connect(const std::string& ip, int port = 30004)
```
- ***Function***
Connects to the RTSI port of the robot.
- ***Parameters***
    - ip: The IP address of the robot.
    - port: The RTSI port.

---

### ***Disconnection***
```cpp
void disconnect();
```
- ***Function***
Disconnects from the robot.

---

### ***Protocol Version Verification***
```cpp
bool negotiateProtocolVersion(uint16_t version = DEFAULT_PROTOCOL_VERSION)
```
- ***Function***
Sends the protocol version verification.
- ***Parameters***
    - version: The protocol version.
- ***Return Value***: Returns true if the verification is successful, and false otherwise.

---

### ***Get the Version of Elite CS Controller***
```cpp
VersionInfo getControllerVersion()
```
- ***Function***
Gets the version of Elite CS Controller.
- ***Return Value***: The version of Elite CS Controller.

---

### ***Configure the Output Subscription Recipe***
```cpp
RtsiRecipeSharedPtr setupOutputRecipe(const std::vector<std::string>& recipe_list, double frequency = 250)
```
- ***Function***
Configures the output subscription recipe.
- ***Parameters***
    - recipe_list: The recipe string. For specific content, refer to the Elite official document "RTSI User Manual".
    - frequency: The update frequency.
- ***Return Value***: The output subscription recipe. If it is nullptr, it means the subscription fails.

---

### ***Configure the Input Subscription Recipe***
```cpp
RtsiRecipeSharedPtr setupInputRecipe(const std::vector<std::string>& recipe)
```
- ***Function***
Configures the input subscription recipe.
- ***Parameters***
    - recipe: The recipe string. For specific content, refer to the Elite official document "RTSI User Manual".
- ***Return Value***: The input subscription recipe. If it is nullptr, it means the subscription fails.

---

### ***Start Data Synchronization***
```cpp
bool start()
```
- ***Function***
Starts data synchronization.
- ***Return Value***: Returns true if successful, and false if failed.

---

### ***Pause Data Synchronization***
```cpp
bool pause()
```
- ***Function***
Pauses data synchronization.
- ***Return Value***: Returns true if successful, and false if failed.

---

### ***Send the Input Subscription Recipe***
```cpp
void send(RtsiRecipeSharedPtr& recipe)
```
- ***Function***
Sends the input subscription recipe and sets the data for the robot.
- ***Parameters***
    - recipe: The input subscription recipe.

---

### Receive Output Subscription
```cpp
int receiveData(std::vector<RtsiRecipeSharedPtr>& recipes, bool read_newest = false)
```
- ***Function***
Receives the recipe data of the output subscription.
- ***Parameters***
    - recipes: The list of output subscription recipes. Only one recipe is received, and the data of the recipe in the list is updated. It is recommended that read_newest be false.
    - read_newest: Whether to receive the latest data packet. (There may be multiple data packets in the system cache.)
- ***Return Value***: The ID of the received recipe.

---

### Receive Output Subscription
```cpp
bool receiveData(RtsiRecipeSharedPtr recipe, bool read_newest = false)
```
- ***Function***
Receives the recipe data of the output subscription.
- ***Parameters***
    - recipe: The output subscription recipe. In the case of multiple recipes, if the received recipe is not the input recipe, the data of this recipe will not be updated.
    - read_newest: Whether to receive the latest data packet. (There may be multiple data packets in the system cache.)
- ***Return Value***: Returns true if the recipe is updated successfully.

---

### ***Connection Status***
```cpp
bool isConnected()
```
- ***Function***
Checks the connection status.
- ***Return Value***: Returns true if the connection is normal, and false otherwise.

---

### ***Synchronization Status***
```cpp
bool isStarted()
```
- ***Function***
Checks whether the robot data synchronization has been started.
- ***Return Value***: Returns true if yes, and false if no.

---

### ***Readable Status***
```cpp
bool isReadAvailable()
```
- ***Function***
Checks whether there is readable data. It is usually used to determine whether there is readable data in the buffer when receiving the robot's status.
- ***Return Value***: Returns true if yes, and false if no.

---

# RtsiIOInterface Class

## Introduction
It inherits from the `RtsiClientInterface` class. This interface further encapsulates `RtsiClientInterface` and starts a thread internally to synchronize the robot data.

## Header File
```cpp
#include <Elite/RtsiIOInterface.hpp>
```

## Constructor and Destructor

### ***Constructor***
```cpp
RtsiIOInterface::RtsiIOInterface(const std::string& output_recipe_file, const std::string& input_recipe_file, double frequency)
```
- ***Function***
Initializes the data and reads the two files `output_recipe_file` and `input_recipe_file` to obtain the subscription recipes. The format of the recipe file is:  
```
Subscription Item 1
Subscription Item 2
```
- ***Parameters***
    - output_recipe_file: The path of the output recipe file. If empty, no subscription will be made.
    - input_recipe_file: The path of the input recipe file. If empty, no subscription will be made.
    - frequency: The data synchronization frequency.

---

```cpp
RtsiIOInterface(const std::vector<std::string>& output_recipe, const std::vector<std::string>& input_recipe, double frequency)
```
- ***Function***

    Initialize data

- ***Parameters***
    - output_recipe: Input recipe string. If empty, no subscription will be made.

    - input_recipe: Output recipe string. If empty, no subscription will be made.

    - frequency: Data synchronization frequency.


---

### ***Destructor***
```cpp
RtsiIOInterface::~RtsiIOInterface()
```
- ***Function***
Disconnects the socket, ends the data synchronization thread, and releases the resources.

## Interfaces

### ***Connection***
```cpp
bool connect(const std::string& ip)
```
- ***Function***
Connects to the robot's RTSI port, performs version verification, obtains the robot controller version information, configures the input and output subscription recipes, and starts the data synchronization thread.
- ***Parameters***
    - ip: The IP address of the robot.
- ***Return Value***: Returns true if successful, and false if failed.

---

### ***Disconnection***
```cpp
void disconnect()
```
- ***Function***
Disconnects the socket from the robot and ends the data synchronization thread.

---

### Get the Controller Version
```cpp
virtual VersionInfo getControllerVersion()
```
- ***Function***
Gets the controller version information.
- ***Return Value***: A VersionInfo object containing the major version number, minor version number, patch number, and build number.

---

### Set the Speed Scaling
```cpp
bool setSpeedScaling(double scaling)
```
- ***Function***
Sets the speed scaling of the robot.
- ***Parameters***
    - scaling: The target speed scaling.
- ***Return Value***: Returns true if the setting is successful, and false if failed.

---

### Set the Standard Digital I/O
```cpp
bool setStandardDigital(int index, bool level)
```
- ***Function***
Sets the level of the standard digital I/O.
- ***Parameters***
    - index: The index of the standard digital I/O.
    - level: High/Low level.
- ***Return Value***: Returns true if the setting is successful, and false if failed.

---

### Set the Configurable Digital I/O
```cpp
bool setConfigureDigital(int index, bool level)
```
- ***Function***
Sets the level of the configurable digital I/O.
- ***Parameters***
    - index: The index of the configurable digital I/O.
    - level: High/Low level.
- ***Return Value***: Returns true if the setting is successful, and false if failed.

---

### Set the Analog Output Voltage
```cpp
bool setAnalogOutputVoltage(int index, double value)
```
- ***Function***
Sets the analog output voltage.
- ***Parameters***
    - index: The index of the analog I/O.
    - value: The voltage value (unit: V, range [0,10]V).
- ***Return Value***: Returns true if the setting is successful, and false if failed.

---

### Set the Analog Output Current
```cpp
bool setAnalogOutputCurrent(int index, double value)
```
- ***Function***
Sets the analog output current.
- ***Parameters***
    - index: The index of the analog I/O.
    - value: The current value (unit: A, range [0.004,0.2]A).
- ***Return Value***: Returns true if the setting is successful, and false if failed.

---

### Set the External Force and Torque
```cpp
bool setExternalForceTorque(const vector6d_t& value)
```
- ***Function***
Inputs the data from the external force sensor (effective when ft_rtsi_input_enable is set to true).
- ***Parameters***
    - value: The data from the external force sensor.
- ***Return Value***: Returns true if the setting is successful, and false if failed.

---

### Set the Tool Digital Output
```cpp
bool setToolDigitalOutput(int index, bool level)
```
- ***Function***
Sets the level of the tool digital output.
- ***Parameters***
    - index: The index of the tool output I/O.
    - level: The level value.
- ***Return Value***: Returns true if the setting is successful, and false if failed.

---

### Get the Timestamp
```cpp
double getTimestamp()
```
- ***Function***
Gets the timestamp.
- ***Return Value***: The timestamp (unit: seconds).

---

### Get the Payload Mass
```cpp
double getPayloadMass()
```
- ***Function***
Gets the mass of the end effector payload.
- ***Return Value***: The payload mass (unit: kg).

---

### Get the Payload Center of Gravity
```cpp
vector3d_t getPayloadCog()
```
- ***Function***
Gets the center of gravity of the end effector payload.
- ***Return Value***: The coordinates of the payload center of gravity (unit: m).

---

### Get the Script Control Line Number
```cpp
uint32_t getScriptControlLine()
```
- ***Function***
Gets the line number of the running script.
- ***Return Value***: The line number of the script.

---

### Get the Target Joint Positions
```cpp
vector6d_t getTargetJointPositions()
```
- ***Function***
Gets the target joint positions.
- ***Return Value***: The joint positions (unit: rad).

---

### Get the Target Joint Velocity
```cpp
vector6d_t getTargetJointVelocity()
```
- ***Function***
Gets the target joint velocity.
- ***Return Value***: The joint velocity (unit: rad/s).

---

### Get the Actual Joint Positions
```cpp
vector6d_t getActualJointPositions()
```
- ***Function***
Gets the actual joint positions.
- ***Return Value***: The joint positions (unit: rad).

---

### Get the Actual Joint Torques
```cpp
vector6d_t getActualJointTorques()
```
- ***Function***
Gets the actual joint torques.
- ***Return Value***: The joint torques (unit: N*m).

---

### Get the Actual Joint Velocity
```cpp
vector6d_t getActualJointVelocity()
```
- ***Function***
Gets the actual joint velocity.
- ***Return Value***: The joint velocity (unit: rad/s).

---

### Get the Actual Joint Current
```cpp
vector6d_t getActualJointCurrent()
```
- ***Function***
Gets the actual joint current.
- ***Return Value***: The joint current (unit: A).

---

### Get the Actual Joint Temperatures
```cpp
vector6d_t getActualJointTemperatures()
```
- ***Function***
Gets the actual joint temperatures.
- ***Return Value***: The joint temperatures (unit: degrees Celsius).

---

### Get the Actual TCP Pose
```cpp
vector6d_t getActualTCPPose()
vector6d_t getActualTCPPose(const vector6d_t& user_frame_pose)
vector6d_t getActualTCPPose(const UserFrame& user_frame)
```
- ***Function***
Gets the actual Cartesian coordinates of the tool. The no-argument overload returns the pose in the base frame. When a user-frame pose relative to the base frame is provided, the returned pose is expressed in that user frame.
- ***Parameters***
    - user_frame_pose: Optional user-frame pose relative to the base frame, formatted as `[x, y, z, rx, ry, rz]`, with meters for position and radians for orientation.
    - user_frame: User frame object. Its `pose` field, expressed relative to the base frame, is used for the conversion.
- ***Return Value***: `[x, y, z, rx, ry, rz]`, where x, y, z are position vectors and rx, ry, rz are rotation vectors.

---

### Get the Actual TCP Velocity
```cpp
vector6d_t getActualTCPVelocity()
```
- ***Function***
Gets the actual Cartesian velocity of the tool.
- ***Return Value***: [x, y, z, rx, ry, rz]/s, where x, y, z are position vectors, and rx, ry, rz are rotation vectors.

---

### Get the Actual TCP Force
```cpp
vector6d_t getActualTCPForce()
```
- ***Function***
Gets the generalized force of the TCP (subtracting the force data caused by the payload).
- ***Return Value***: The TCP force vector.

---

### Get the Target TCP Pose
```cpp
vector6d_t getTargetTCPPose()
```
- ***Function***
Gets the target Cartesian coordinates of the tool.
- ***Return Value***: [x, y, z, rx, ry, rz], where x, y, z are position vectors, and rx, ry, rz are rotation vectors.

---

### Get the Target TCP Velocity
```cpp
vector6d_t getTargetTCPVelocity()
```
- ***Function***
Gets the target Cartesian velocity of the tool.
- ***Return Value***: [x, y, z, rx, ry, rz], where x, y, z are position vectors, and rx, ry, rz are rotation vectors.

---

### Get the Digital Input Bits
```cpp
uint32_t getDigitalInputBits()
```
- ***Function***
Gets the bit values of all digital input I/Os.
- ***Return Value***:
    - bits 0-15: Standard digital input.
    - bits 16-24: Configurable digital input.
    - bits 24-28: Tool digital input.

---

### Get the Digital Output Bits
```cpp
uint32_t getDigitalOutputBits()
```
- ***Function***
Gets the bit values of all digital output I/Os.
- ***Return Value***:
    - bits 0-15: Standard digital input.
    - bits 16-24: Configurable digital input.
    - bits 24-28: Tool digital input.

---

### Get the Robot Mode
```cpp
RobotMode getRobotMode()
```
- ***Function***
Gets the robot mode.
- ***Return Value***: An enumeration value of RobotMode.

---

### Get the Joint Mode
```cpp
std::array<JointMode, 6> getJointMode()
```
- ***Function***
Gets the mode of each joint.
- ***Return Value***: An array containing 6 JointMode values.

---

### Get the Safety Status
```cpp
SafetyMode getSafetyStatus()
```
- ***Function***
Gets the robot's safety mode.
- ***Return Value***: An enumeration value of SafetyMode.

---

### Get the Actual Speed Scaling
```cpp
double getActualSpeedScaling()
```
- ***Function***
Gets the actual speed scaling of the robot.
- ***Return Value***: The speed scaling value.

---

### Get the Target Speed Scaling
```cpp
double getTargetSpeedScaling()
```
- ***Function***
Gets the target speed scaling of the robot.
- ***Return Value***: The speed scaling value.

---

### Get the Robot Voltage
```cpp
double getRobotVoltage()
```
- ***Function***
Gets the robot voltage (48V).
- ***Return Value***: The voltage value (unit: V).

---

### Get the Robot Current
```cpp
double getRobotCurrent()
```
- ***Function***
Gets the robot current.
- ***Return Value***: The current value (unit: A).

---

### Get the Runtime State
```cpp
TaskStatus getRuntimeState()
```
- ***Function***
Gets the program state.
- ***Return Value***: An enumeration value of TaskStatus.

---

### Get the Elbow Position
```cpp
vector3d_t getElbowPosition()
```
- ***Function***
Gets the real-time position of the robot's elbow.
- ***Return Value***: [x, y, z] coordinates.

---

### Get the Elbow Velocity
```cpp
vector3d_t getElbowVelocity()
```
- ***Function***
Gets the real-time velocity of the robot's elbow.
- ***Return Value***: [x, y, z]/s velocity vector.

---

### Get Robot Status
```cpp
uint32_t getRobotStatus()
```
- ***Description***

    Get robot status.

- ***Return Value***：
    - bits 0-2: Power on | Program running | Freedrive button pressed

---

### Get Safety Status Bits
```cpp
uint32_t getSafetyStatusBits()
```
- ***Description***

    Get robot safety status bits.

- ***Return Value***：
    - bits 0-10: Normal mode | Reduced mode | Protective stop | Recovery mode | Safeguard stop | System emergency stop | Robot emergency stop | Emergency stop | Violation | Fault | Safety stop

---

### Get Analog IO Types
```cpp
uint32_t getAnalogIOTypes()
```
- ***Description***

    Get analog IO types.

- ***Return Value***：
    - bits 0-3: Analog input 0(bit 0), analog input 1(bit 1), analog output 0(bit 2), analog output 1(bit 3)
    - 0: Current mode; 1: Voltage mode

---

### Get Analog Input
```cpp
double getAnalogInput(int index)
```
- ***Description***

    Get standard analog input value.

- ***Parameters***

    - index: Standard analog input index[0,1]

- ***Return Value***: Analog input value (unit: A or V)

---

### Get Analog Output
```cpp
double getAnalogOutput(int index)
```
- ***Description***

    Get standard analog output value.

- ***Parameters***

    - index: Standard analog output index[0,1]

- ***Return Value***: Analog output value (unit: A or V)

---

### Get IO Current
```cpp
double getIOCurrent()
```
- ***Description***

    Get mainboard IO current.

- ***Return Value***: Current value (unit: A)

---

### Get Tool Mode
```cpp
ToolMode getToolMode()
```
- ***Description***

    Get tool mode.

- ***Return Value***: ToolMode enumeration value

---

### Get Tool Analog Input Type
```cpp
uint32_t getToolAnalogInputType()
```
- ***Description***

    Get tool analog input IO mode.

- ***Return Value***: 0: Current mode, 1: Voltage mode

---

### Get Tool Analog Output Type
```cpp
uint32_t getToolAnalogOutputType()
```
- ***Description***

    Get tool analog output IO mode.

- ***Return Value***: 0: Current mode, 1: Voltage mode

---

### Get Tool Analog Input
```cpp
double getToolAnalogInput()
```
- ***Description***

    Get tool analog input value.

- ***Return Value***：
    - Current mode range[0.004-0.02]A
    - Voltage mode range[0-10]V

---

### Get Tool Analog Output
```cpp
double getToolAnalogOutput()
```
- ***Description***

    Get tool analog output value.

- ***Return Value***：
    - Current mode range[0.004-0.02]A
    - Voltage mode range[0-10]V

---

### Get Tool Output Voltage
```cpp
double getToolOutputVoltage()
```
- ***Description***

    Get tool output voltage.

- ***Return Value***: Voltage value (unit: V)

---

### Get Tool Output Current
```cpp
double getToolOutputCurrent()
```
- ***Description***

    Get tool output current.

- ***Return Value***: Current value (unit: A)

---

### Get Tool Output Temperature
```cpp
double getToolOutputTemperature()
```
- ***Description***

    Get tool output temperature.

- ***Return Value***: Temperature value (unit: Celsius)

---

### Get Tool Digital Mode
```cpp
ToolDigitalMode getToolDigitalMode()
```
- ***Description***

    Get tool digital mode.

- ***Return Value***: Single pin, Dual pin mode 1, Dual pin mode 2, Triple pin

---

### Get Tool Digital Output Mode
```cpp
ToolDigitalOutputMode getToolDigitalOutputMode(int index)
```
- ***Description***

    Get tool digital output mode.

- ***Parameters***

    - index: Tool digital index

- ***Return Value***: Push/Pull, NPN, PNP

---

### Get Output Boolean Registers 0-31
```cpp
uint32_t getOutBoolRegisters0To31()
```
- ***Description***

    Get output values of boolean registers 0-31.

- ***Return Value***: Register value

---

### Get Output Boolean Registers 32-63
```cpp
uint32_t getOutBoolRegisters32To63()
```
- ***Description***

    Get output values of boolean registers 32-63.

- ***Return Value***: Register value

---

### Get Input Boolean Registers 0-31
```cpp
uint32_t getInBoolRegisters0To31()
```
- ***Description***

    Get input values of boolean registers 0-31.

- ***Return Value***: Register value

---

### Get Input Boolean Registers 32-63
```cpp
uint32_t getInBoolRegisters32To63()
```
- ***Description***

    Get input values of boolean registers 32-63.

- ***Return Value***: Register value

---

### Get Input Boolean Register
```cpp
bool getInBoolRegister(int index)
```
- ***Description***

    Get input value of a specified boolean register.

- ***Parameters***

    - index: Boolean register index[64,127]

- ***Return Value***: Register value

---

### Get Output Boolean Register
```cpp
bool getOutBoolRegister(int index)
```
- ***Description***

    Get output value of a specified boolean register.

- ***Parameters***

    - index: Boolean register index

- ***Return Value***: Register value

---

### Get Input Integer Register
```cpp
int32_t getInIntRegister(int index)
```
- ***Description***

    Get input value of a specified integer register.

- ***Parameters***

    - index: Integer register index

- ***Return Value***: Register value

---

### Get Output Integer Register
```cpp
int32_t getOutIntRegister(int index)
```
- ***Description***

    Get output value of a specified integer register.

- ***Parameters***

    - index: Integer register index

- ***Return Value***: Register value

---

### Get Input Double Register
```cpp
double getInDoubleRegister(int index)
```
- ***Description***

    Get input value of a specified double register.

- ***Parameters***

    - index: Double register index

- ***Return Value***: Register value

---

### Get Output Double Register
```cpp
double getOutDoubleRegister(int index)
```
- ***Description***

    Get output value of a specified double register.

- ***Parameters***

    - index: Double register index

- ***Return Value***: Register value

---

### Get Output Recipe Value
```cpp
template<typename T>
bool getRecipeValue(const std::string& name, T& out_value);
```
- ***Description***

    Get the value of a specified subscription item in the output recipe.

- ***Parameters***
    - name: Subscription item name

    - out_value: Output value. Note that the type of this value must match the type specified in the RTSI documentation.

- ***Return Value***: Returns true on success, false on failure.

---

### Set Input Recipe Value
```cpp
template <typename T>
bool setInputRecipeValue(const std::string& name, const T& value)
```
- ***Description***

    Set the value of a specified subscription item in the input recipe.

- ***Parameters***
    - name: Subscription item name

    - value: Input value. Note that the type of this value must match the type specified in the RTSI documentation.

- ***Return Value***: Returns true on success, false on failure.

---

# RtsiRecipe Class

## Introduction

This interface provides basic operations for RTSI recipes. The `RtsiRecipe` class can only be obtained through the `RtsiClientInterface::setupOutputRecipe()` and `RtsiClientInterface::setupInputRecipe()` interfaces.

## Header File
```cpp
#include <Elite/RtsiRecipe.hpp>
```

## Interface

### ***Get Value***
```cpp
template<typename T>
bool getValue(const std::string& name, T& out_value)
```
- ***Description***

    Get the value of a subscription item in the recipe.

- ***Parameters***

    - name: Subscription item name.

    - out_value: Output value of the subscription item. Note that the type of this value must match the type specified in the RTSI documentation.

- ***Return Value***: Returns true on success, false on failure.

---

### ***Set Value***
```cpp
template<typename T>
bool setValue(const std::string& name, const T& value)
```
- ***Description***

    Set the value of a subscription item in the recipe.

- ***Parameters***

    - name: Subscription item name.

    - value: Value to set for the subscription item. Note that the type of this value must match the type specified in the RTSI documentation.

- ***Return Value***: Returns true on success, false on failure.

---

### Get Recipe
```cpp
const std::vector<std::string>& getRecipe()
```
- ***Description***
    
    Get the list of recipe subscription item names.

- ***Return Value***: List of recipe subscription item names.

---

### Get Recipe ID
```cpp
int getID()
```
- ***Description***
    
    Get the recipe ID.

- ***Return Value***: Recipe ID.

---
