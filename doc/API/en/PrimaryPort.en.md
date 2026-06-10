# Primary Port

## Introduction
The SDK provides interfaces for connecting to the robot's port 30001, sending scripts, and a framework for parsing the data from port 30001. Only partial data packets are parsed in the SDK. If you want to parse other data packets, you need to write the parsing code manually.

# PrimaryPortInterface Class

## Introduction
This interface provides methods for communicating with the robot's primary port interface.

## Header File of PrimaryPortInterface
```cpp
#include <Elite/PrimaryPortInterface.hpp>
```

## Constructor of the PrimaryPortInterface Class

### ***Constructor***
```cpp
PrimaryPortInterface::PrimaryPortInterface()
```
- ***Function***
Initializes the data. Note that this constructor does not connect to the robot.

---

## Communication

### ***Connection***
```cpp
bool connect(const std::string& ip, int port = PRIMARY_PORT)
```
- ***Function***
Connects to the robot's port 30001 (by default).
- ***Parameters***
    - ip: The IP address of the robot.
    - timeout_ms: Sets the timeout for the robot to read the next instruction. If it is less than or equal to 0, it will wait indefinitely.
- ***Return Value***: Returns true if successful, and false if failed.

- ***Note***
    1. Warning: Repeated calls to this function without intermediate disconnect() will force-close the active connection.
    2. Usage constraint: Call rate must be ≤ 2Hz (once per 500ms minimum interval).

---

### ***Disconnection***
```cpp
void disconnect()
```
- ***Function***
Disconnects from the robot.

- ***Note***

    Suggest adding ~500ms delay between this function and connect calls.

---

### ***Send Script***
```cpp
bool sendScript(const std::string& script)
```
- ***Function***
Sends an executable script to the robot.
- ***Parameters***
    - script: The script to be sent.
- ***Return Value***: Returns true if the sending is successful, and false if failed.

---

### ***Power On Robot***
```cpp
bool powerOn()
```
- ***Function***

    Sends the normal script command `power on` through port 30001, and confirms the robot mode through Primary/30001 state packages.

- ***Return Value***: Returns true if the script is sent successfully and `RobotModeData.robot_mode` becomes `IDLE` or `RUNNING`; returns false if sending fails or waiting for the target state times out.

- ***Note***

    If the current robot mode is already `IDLE` or `RUNNING`, this interface returns true directly.

---

### ***Power Off Robot***
```cpp
bool powerOff()
```
- ***Function***

    Sends the normal script command `power off` through port 30001, and confirms the robot mode through Primary/30001 state packages.

- ***Return Value***: Returns true if the script is sent successfully and `RobotModeData.robot_mode` becomes `POWER_OFF`; returns false if sending fails or waiting for the target state times out.

---

### ***Release Brakes***
```cpp
bool brakeRelease()
```
- ***Function***

    Sends the normal script command `set robotmode run` through port 30001 to release the robot brakes.

- ***Return Value***: Returns true if the script is sent successfully and `RobotModeData.robot_mode` becomes `RUNNING`; returns false if preconditions are not met, sending fails, or waiting for the target state times out.

- ***Note***

    This interface first checks Primary/30001 state packages: the robot mode must be `IDLE`, and the safety mode must be `NORMAL`, `REDUCED`, or `RECOVERY`. If the current robot mode is already `RUNNING`, this interface returns true directly.

---

### ***Pause Task***
```cpp
bool pauseProgram()
```
- ***Function***

    Sends the normal script command `pause task` through port 30001 to pause the currently running task.

- ***Return Value***: Returns true if the script is sent successfully and the task state becomes `PAUSED`; returns false if no task is running, sending fails, or waiting for the target state times out.

- ***Note***

    This interface determines the task state from `RobotModeData.is_task_running` and `RobotModeData.is_task_paused`. It sends the pause command only when the current task state is `PLAYING`. If the task is already `PAUSED`, it returns true directly.

---

### ***Stop Task***
```cpp
bool stopProgram()
```
- ***Function***

    Sends the normal script command `stop task` through port 30001 to stop the current task.

- ***Return Value***: Returns true if the script is sent successfully and the task state becomes `STOPPED`; returns false if sending fails or waiting for the target state times out.

- ***Note***

    This interface determines the task state from `RobotModeData.is_task_running` and `RobotModeData.is_task_paused`. If the current task state is already `STOPPED`, it returns true directly.

---

### ***Unlock Protective Stop***
```cpp
bool unlockProtectiveStop()
```
- ***Function***

    Sends the normal script command `set unlock protective stop` through port 30001 to unlock the robot protective stop state.

- ***Return Value***: Returns true if the script is sent successfully and `RobotModeData.is_robot_protective_stopped` becomes false; returns false if sending fails or waiting for the target state times out.

---

### ***Restart Safety System***
```cpp
bool safetySystemRestart()
```
- ***Function***

    Sends the normal script command `restart safetyboard` through port 30001 to restart the safety board.

- ***Return Value***: Returns true if the script is sent successfully and `MasterBoardData.safety_mode` becomes `NORMAL`; returns false if sending fails or waiting for the target state times out.

---

### ***Set Speed Scaling***
```cpp
bool setSpeedScaling(int scaling)
```
- ***Function***

    Sends the normal script command `set speed <scaling / 100.0>` through port 30001 to set the target robot speed scaling.

- ***Parameters***
    - scaling: Target speed percentage.

- ***Return Value***: Returns true if the script is sent successfully and `RobotModeData.target_speed_fraction` is read back as the target value; returns false if sending fails or waiting for the target state times out.

- ***Note***

    Primary/30001 state parsing is only used internally by these control APIs to confirm command results. It is not exposed as a public `PrimaryPortInterface` state-query API. Use RTSI for external robot-state reads.

---

### ***Get Data Packet***
```cpp
bool getPackage(std::shared_ptr<PrimaryPackage> pkg, int timeout_ms)
```
- ***Function***
Retrieves and parses the robot's data packet.
- ***Parameters***
    - pkg: The data packet to be retrieved.
    - timeout_ms: The waiting timeout.
- ***Return Value***: Returns true if the retrieval is successful, and false if failed.

---

### ***Get local IP***
```cpp
std::string getLocalIP()
```
- ***Function***

    Get local IP address

- ***Return Value***：Local IP address. If empty, connection had some errors.

---

### ***Register Robot Exception Callback***
```cpp
void registerRobotExceptionCallback(std::function<void(RobotExceptionSharedPtr)> cb)
```

- ***Functionality***
    Registers a callback function for robot exceptions. This callback will be invoked when an exception message is received from the robot's primary port. The callback function takes a parameter of type `RobotExceptionSharedPtr`, representing the exception information.

- ***Parameters***
    - `cb`: The callback function to handle received robot exceptions. The parameter is a shared pointer to a robot exception (see: [RobotException](./RobotException.en.md)).

# PrimaryPackage Class

## Introduction
This class is mainly used to be inherited to obtain the data from the Primary port.
The SDK does not provide parsing for all data packets. If you need other data packets, you can refer to the implementation of `KinematicsInfo` in `RobotConfPackage.hpp` to write the data packet parsing code you need. Refer to the Elite official document "CS_User Manual_Robot Status Message.xlsx" for the message format.

## Header File of PrimaryPackage
```cpp
#include <Elite/PrimaryPackage.hpp>
```

## Constructor of the PrimaryPackage Class
### ***Constructor***
```cpp
PrimaryPackage::PrimaryPackage(int type)
```
- ***Function***
Initializes the data.
- ***Parameters***
    - type: The type of the data packet (refer to the Elite official document: CS_User Manual_Robot Status Message.xlsx).

---

## Pure Virtual Function

### Parse Message
```cpp
void parser(int len, const std::vector<uint8_t>::const_iterator& iter)
```
- ***Function***
The specific implementation is to be completed by subclasses. It parses the sub-message of the robot status message from the Primary port. When an instance of a subclass is passed as a parameter to `PrimaryPortInterface::getPackage()`, this function will be called.
- ***Parameters***
    - len: The length of the sub-message.
    - iter: The position of the sub-message in the whole message.

---

## Others

### ***Get Message Type***
```cpp
int getType()
```
- ***Function***
Gets the type of the data packet.
- ***Return Value***: The type of the data packet.

---

## Internal Use in the SDK
### Wait for Update
```cpp
bool waitUpdate(int timeout_ms)
```
- ***Function***
Waits for the data packet to be updated. It is called in the `getPackage()` function.
- ***Parameters***
    - timeout_ms: The timeout.
- ***Return Value***: Returns true if it does not time out, and false if it times out.

---

# KinematicsInfo Class

## Introduction
This is for parsing the kinematics data packet in the robot configuration data. `PrimaryPackage` is the parent class of this interface. In addition to the methods in the `PrimaryPackage` class, the main content of this interface is the robot's DH parameters.

## Header File of KinematicsInfo
```cpp
#include <Elite/RobotConfPackage.hpp>
```

## DH Parameters

- `vector6d_t dh_a_`

- `vector6d_t dh_d_`

- `vector6d_t dh_alpha_`
