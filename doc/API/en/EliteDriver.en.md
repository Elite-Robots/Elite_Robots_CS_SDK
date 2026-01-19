# EliteDriver Class

## Introduction
The EliteDriver is the main class for data interaction with the robot. It is responsible for establishing all necessary socket connections and handling the data exchange with the robot. The EliteDriver sends control scripts to the robot. After the robot runs the control script, it will establish communication with the EliteDriver, receive motion data, and send the motion results when necessary.

## Header File
```cpp
#include <Elite/EliteDriver.hpp>
```

## Constructor and Destructor

### ***Constructor***
```cpp
EliteDriver(const EliteDriverConfig& config)
```
- ***Function***

    Creates an EliteDriver object and initializes necessary connections for robot communication.  
    This function will throw exceptions in the following cases:  
    1. TCP server creation fails (typically due to port being occupied).
    2. Failed to connect to the robot's primary port.

- ***Parameters***
    - config: Configuration, refer to [Configuration](./EliteDriverConfig.en.md)


### ***Constructor***(Deprecated)
```cpp
EliteDriver::EliteDriver(
    const std::string& robot_ip, 
    const std::string& local_ip, 
    const std::string& script_file,
    bool headless_mode = false, 
    int script_sender_port = 50002, 
    int reverse_port = 50001,
    int trajectory_port = 50003, 
    int script_command_port = 50004, 
    float servoj_time = 0.008,
    float servoj_lookhead_time = 0.1, 
    int servoj_gain = 300, 
    float stopj_acc = 8.0);
```
- ***Function***
Creates an EliteDriver object and initializes the necessary connections for communication with the robot.
- ***Parameters***
    - robot_ip: The IP address of the robot.
    - local_ip: The local IP address.
    - script_file: The template file of the control script.
    - headless_mode: Whether to run in headless mode. After using this mode, there is no need to use the 'External Control' plugin. If this parameter is true, then in the constructor, a control script will be sent to the robot's primary port once.
    - script_sender_port: script_sender_port: Port for sending control scripts. If this port cannot be connected, the `External Control` plugin will stop.
    - reverse_port: The port for reverse communication.
    - trajectory_port: The port for sending trajectory points.
    - script_command_port: The port for sending script commands.
    - servoj_time: The time parameter for servo motion.
    - servoj_lookhead_time: The look-ahead time for servo motion, ranging from [0.03, 0.2] seconds.
    - servoj_gain: The servo gain.
    - stopj_acc: The acceleration for stopping motion (rad/s²).

---

### ***Destructor***
```cpp
~EliteDriver::EliteDriver()
```
- ***Function***
Releases resources, and the socket will be closed during destruction.

---

## Motion Control

### ***Control Joint Position***
```cpp
bool writeServoj(const vector6d_t& pos, int timeout_ms, bool cartesian = false)
```
- ***Function***
Sends a servo motion instruction to the robot.
- ***Parameters***
    - pos: The target position.

    - timeout_ms: Sets the timeout for the robot to read the next instruction. If it is less than or equal to 0, it will wait indefinitely.

    - `cartesian`: Set to `true` if sending Cartesian coordinates, `false` for joint-based positions.  
    
- ***Return Value***: Returns true if the instruction is sent successfully, and false if it fails.

---

### ***Control End-effector Velocity***
```cpp
bool writeSpeedl(const vector6d_t& vel, int timeout_ms)
```
- ***Function***
Sends a linear velocity control instruction to the robot.
- ***Parameters***
    - vel: The linear velocity [x, y, z, rx, ry, rz].
    - timeout_ms: Sets the timeout for the robot to read the next instruction. If it is less than or equal to 0, it will wait indefinitely.
- ***Return Value***: Returns true if the instruction is sent successfully, and false if it fails.

---

### ***Make the Robot Idle***
```cpp
bool writeIdle(int timeout_ms)
```
- ***Function***
Sends an idle instruction. If the robot is in motion, it will make the robot stop moving.
- ***Parameters***
    - timeout_ms: Sets the timeout for the robot to read the next instruction. If it is less than or equal to 0, it will wait indefinitely.
- ***Return Value***: Returns true if the instruction is sent successfully, and false if it fails.

---

### ***Freedrive***
```cpp
bool writeFreedrive(FreedriveAction action, int timeout_ms)
```
- ***Function***

Send commands for Freedrive mode, such as enabling and stopping Freedrive.

- ***Parameters***
    - action: Freedrive action, including: START, END, NOOP
    - timeout_ms: Set the timeout for the robot to read the next instruction. If it is less than or equal to 0, it will wait indefinitely.

- *** Note***: After writing the 'START' action, the next instruction needs to be written within the timeout period, which can be written as' NOOP '.

---

## Trajectory Motion

### ***Set Trajectory Motion Result Callback***
```cpp
void setTrajectoryResultCallback(std::function<void(TrajectoryMotionResult)> cb)
```
- ***Function***
Registers a callback function for when the trajectory is completed.
One way to control the robot is to send all the waypoints to the robot at once. When the execution is completed, the callback function registered here will be triggered.
- ***Parameters***
    - cb: The callback function when the execution is completed.

---

### ***Write Trajectory Waypoint***
```cpp
bool writeTrajectoryPoint(const vector6d_t& positions, float time, float blend_radius, bool cartesian)
```
- ***Function***
Writes a trajectory waypoint to a specific socket.
- ***Parameters***
    - positions: The waypoint.
    - time: The time to reach the waypoint.
    - blend_radius: The transition radius between two waypoints.
    - cartesian: If the sent point is Cartesian, it is True. If it is joint-based, it is false.
- ***Return Value***: Returns true if the instruction is sent successfully, and false if it fails.

---

### ***Trajectory Control Action***
```cpp
bool writeTrajectoryControlAction(TrajectoryControlAction action, const int point_number, int timeout_ms)
```
- ***Function***
Sends a trajectory control instruction.
- ***Parameters***
    - action: The action of trajectory control.
    - point_number: The number of waypoints.
    - timeout_ms: Sets the timeout for the robot to read the next instruction. If it is less than or equal to 0, it will wait indefinitely.
- ***Return Value***: Returns true if the instruction is sent successfully, and false if it fails.
- ***Note***: After writing the `START` action, the next instruction needs to be written within the timeout period, and the `NOOP` can be written.

---

## Robot Configuration
### ***Zero the Force Sensor***
```cpp
bool zeroFTSensor()
```
- ***Function***
Resets (zeros) the force/torque values measured by the force/torque sensor applied to the tool TCP. The force/torque values are the force/torque vectors obtained by the get_tcp_force(True) script instruction, and these vectors have been processed with load compensation, etc.
After this instruction is executed, the current force/torque measurement value will be saved as the force/torque reference value, and all subsequent force/torque measurement values will be subtracted by this force/torque reference value (zeroed).
Please note that the above force/torque reference value will be updated when this instruction is executed, and will be reset to 0 after the controller is restarted.
- ***Return Value***: Returns true if the instruction is sent successfully, and false if it fails.

---

### ***Set the End-effector Payload***
```cpp
bool setPayload(double mass, const vector3d_t& cog)
```
- ***Function***
This command is used to set the mass, center of gravity, and moment of inertia of the robot's payload.
- ***Parameters***
    - mass: The mass of the payload.
    - cog: The coordinates of the center of gravity of the payload (relative to the flange frame).
- ***Return Value***: Returns true if the instruction is sent successfully, and false if it fails.

---

### ***Set the Tool Voltage***
```cpp
bool setToolVoltage(const ToolVoltage& vol)
```
- ***Function***
Sets the tool voltage.
- ***Parameters***
    - vol: The tool voltage.
- ***Return Value***: Returns true if the instruction is sent successfully, and false if it fails.

---

### ***Enable the Force Control Mode***
```cpp
bool startForceMode(const vector6d_t& reference_frame, const vector6int32_t& selection_vector, const vector6d_t& wrench, const ForceMode& mode, const vector6d_t& limits)
```
- ***Function***
Enables the force control mode.
- ***Parameters***
    - reference_frame: The pose vector that defines the force control reference coordinate system. This pose vector is defined relative to the base coordinate system. The format is [X,Y,Z,Rx,Ry,Rz], where X, Y, and Z represent the position in meters; Rx, Ry, and Rz represent the pose in radians. Rx, Ry, and Rz are defined using the standard RPY angles. selection_vector: A six-dimensional vector composed of 0s and 1s, used to define the force control axes in the force control coordinate system. 1 represents a force control axis, and 0 represents a non-force control axis.
    - selection_vector: A six-dimensional vector composed of 0s and 1s, used to define the force control axes in the force control coordinate system. 1 represents a force control axis, and 0 represents a non-force control axis.
    - wrench: The target force/torque applied by the robot to the environment. The robot will adjust its pose along/around the force control axes to achieve the specified target force/torque. The format is [Fx,Fy,Fz,Mx,My,Mz], where Fx, Fy, and Fz represent the force applied along the force control axes in Newtons; Mx, My, and Mz represent the torque applied around the force control axes in Newton-meters. This value is invalid for non-force control axes. Due to joint safety limitations, the actual applied force/torque may be lower than the set target force/torque. The actual force/torque applied to the environment can be read using the get_tcp_force script instruction in a separate thread.
    - mode: The force control mode parameter, an integer type data ranging from 0 to 3, used to define the force control mode, that is, how the force control coordinate system is defined or how it is transformed from the force control reference coordinate system.
        - 0: Fixed mode. The force control coordinate system is the force control reference coordinate system.
        - 1: Point mode. The Y-axis of the force control coordinate system points from the origin of the robot TCP to the origin of the force control reference coordinate system.
        - 2: Motion mode. The X-axis of the force control coordinate system is the projection of the TCP movement direction vector on the X-Y plane of the force control reference coordinate system.
    - limits: The speed limit parameter, a six-dimensional vector of float type data. The format is [Vx,Vy,Vz,ωx,ωy,ωz], where Vx, Vy, and Vz represent the maximum allowed TCP speed along the axis in m/s; ωx, ωy, and ωz represent the maximum allowed TCP speed around the axis in rad/s. This speed limit parameter is invalid for non-force control axes, and non-force control axes still execute the original trajectory on that axis.
- ***Return Value***: Returns true if the instruction is sent successfully, and false if it fails.

---

### ***Disable the Force Control Mode***
```cpp
bool endForceMode()
```
- ***Function***
Disables the force control mode.
- ***Return Value***: Returns true if the instruction is sent successfully, and false if it fails.

---

## Others

### ***Stop External Control***
```cpp
bool stopControl(int wait_ms = 10000)
```
- ***Function***
Sends a stop instruction to the robot. The robot will exit the control script and stop receiving instructions from the PC.

- ***Parameters***
    - wait_ms: Waiting for the robot to disconnect for a certain amount of time(ms). Range >5ms.

- ***Return Value***: 

    Return true for successful instruction sending and false for failure. The following situations will return false:
    - Disconnected from the robot.
    - Not disconnected from the robot during the waiting time.

---

### Is robot connected
```cpp
bool isRobotConnected()
```
- ***Function***
Checks whether the connection to the robot is established.
- ***Return Value***: Returns true if connected, and false if not connected.

---

### Send Script
```cpp
bool sendScript(const std::string& script)
```
- ***Function***
Sends an executable script to port 30001 of the robot.
- ***Parameters***
    - script: The script to be sent.
- ***Return Value***: Returns true if the sending is successful, and false if it fails.

---

### ***Send Control Script***
```cpp
bool sendExternalControlScript()
```
- ***Function***
Sends an external control script to the robot. It can be used to establish or restore control of the robot.
- ***Return Value***: Returns true if the sending is successful, and false if it fails.

---

### ***Get the Data Packet from the Robot's Primary Port***
```cpp
bool getPrimaryPackage(std::shared_ptr<PrimaryPackage> pkg, int timeout_ms)
```
- ***Function***
Gets the data packet from port 30001 of the robot.
- ***Parameters***
    - pkg: The data packet to be obtained (refer to [PrimaryPort](./PrimaryPort.cn.md)).
    - timeout_ms: The timeout for obtaining the packet.
- ***Return Value***: Returns true if the obtaining is successful, and false if it fails.

---

### ***Reconnect to the Robot's Primary Port***
```cpp
bool primaryReconnect()
```
- ***Function***
Re-establishes the connection to port 30001 of the robot.
- ***Return Value***: Returns true if successful, and false if it fails. 

---

### ***Register Robot Exception Callback***
```cpp
void registerRobotExceptionCallback(std::function<void(RobotExceptionSharedPtr)> cb)
```

- ***Functionality***
    Registers a callback function for robot exceptions. This callback will be invoked when an exception message is received from the robot's primary port. The callback function takes a parameter of type `RobotExceptionSharedPtr`, representing the exception information.

- ***Parameters***
    - `cb`: The callback function to handle received robot exceptions. The parameter is a shared pointer to a robot exception (see: [RobotException](./RobotException.en.md)).

---

### ***Enable Tool RS485 Communication***
```cpp
SerialCommunicationSharedPtr startToolRs485(const SerialConfig& config, int tcp_port = 54321)
```

- ***Description***  
    Enables tool RS485 communication. This interface launches a socat process on the robot controller to forward data from the tool RS485 serial port to the specified TCP/IP port.

- ***Parameters***
    - `config`: Serial port configuration.
    - `ssh_password` : The SSH login password for the robot control cabinet operating system.
    - `tcp_port`: TCP port.

- ***Return Value***: An object for operating the serial port, which essentially functions as a TCP client. See [serial communication](./SerialCommunication.en.md).
- ***Note***: If you want to use this feature, it is recommended to install libssh. If you are using it on a non-Linux system, you must install the libssh library.

---

### ***Disable Tool RS485 Communication***
```cpp
bool endToolRs485(SerialCommunicationSharedPtr com, const std::string& ssh_password)
```

- ***Description***  
    Disables tool RS485 communication.

- ***Parameters***
    - `comm_ptr`: The serial communication object. See [serial communication](./SerialCommunication.en.md).
    - `com` : The SSH login password for the robot control cabinet operating system.

- ***Return Value***: Indicates whether the tool RS485 communication was successfully disabled.
- ***Note***: If you want to use this feature, it is recommended to install libssh. If you are using it on a non-Linux system, you must install the libssh library.

---

### ***Enable Board RS485 Communication***
```cpp
SerialCommunicationSharedPtr startBoardRs485(const SerialConfig& config, int tcp_port = 54321)
```

- ***Description***  
    Enables board RS485 communication. This interface launches a socat process on the robot controller to forward data from the tool RS485 serial port to the specified TCP/IP port.

- ***Parameters***
    - `config`: Serial port configuration.
    - `ssh_password` : The SSH login password for the robot control cabinet operating system.
    - `tcp_port`: TCP port.

- ***Return Value***: An object for operating the serial port, which essentially functions as a TCP client. See [serial communication](./SerialCommunication.en.md).
- ***Note***: If you want to use this feature, it is recommended to install libssh. If you are using it on a non-Linux system, you must install the libssh library.

---

### ***Disable Board RS485 Communication***
```cpp
bool endBoardRs485(SerialCommunicationSharedPtr com, const std::string& ssh_password)
```

- ***Description***  
    Disables board RS485 communication.

- ***Parameters***
    - `comm_ptr`: The serial communication object. See [serial communication](./SerialCommunication.en.md).
    - `com` : The SSH login password for the robot control cabinet operating system.

- ***Return Value***: Indicates whether the tool RS485 communication was successfully disabled.
- ***Note***: If you want to use this feature, it is recommended to install libssh. If you are using it on a non-Linux system, you must install the libssh library.

---