# DashboardClient Class

## Introduction
The DashboardClient class provides an interface for interacting with the robot's dashboard server, which is used to execute various robot control commands and query the status.

## Header File
```cpp
#include <Elite/DashboardClient.hpp>
```

## Interfaces

### Connect to the Server
```cpp
bool connect(const std::string& ip, int port = 29999)
```
- ***Function***
Connects to the dashboard server.
- ***Parameters***
    - ip: The IP address of the dashboard server.
    - port: The port of the dashboard server (default is 29999).
- ***Return Value***: Returns true if the connection is successful, and false if it fails.

---

### Disconnect
```cpp
void disconnect()
```
- ***Function***
Disconnects from the dashboard server.

---

### Brake Release
```cpp
bool brakeRelease()
```
- ***Function***
Releases the robot's brakes.
- ***Return Value***: Returns true if the operation is successful, and false if it fails.

---

### Close the Safety Pop-up Window
```cpp
bool closeSafetyDialog()
```
- ***Function***
Closes the safety message pop-up window.
- ***Return Value***: Returns true if the operation is successful, and false if it fails.

---

### Connection Check
```cpp
bool echo()
```
- ***Function***
Checks the connection status with the dashboard shell server.
- ***Return Value***: Returns true if the robot responds successfully.

---

### Get Help Information
```cpp
std::string help(const std::string& cmd)
```
- ***Function***
Gets the help information for the specified command.
- ***Parameters***
    - cmd: The command that needs help.
- ***Return Value***: The help string for the command.

---

### Add Log
```cpp
bool log(const std::string& message)
```
- ***Function***
Adds a log message ('\n' or '\r' in the message will be replaced with "\\n" and "\\r").
- ***Parameters***
    - message: The content of the log.
- ***Return Value***: Returns true if the operation is successful, and false if it fails.

---

### Pop-up/Close the Message Box
```cpp
bool popup(const std::string& arg, const std::string& message = "")
```
- ***Function***
Pops up or closes the message box.
- ***Parameters***
    - arg:
        - "-c": Closes the message box.
        - "-s": Pops up the message box.
    - message: The content of the message (optional).
- ***Return Value***: Returns true if the operation is successful, and false if it fails.

---

### Quit the Dashboard
```cpp
void quit()
```
- ***Function***
Quits the dashboard and disconnects the connection.

---

### Reboot the Robot
```cpp
void reboot()
```
- ***Function***
Reboots the robot and disconnects the connection.

---

### Get the Robot Type
```cpp
std::string robot()
```
- ***Function***
Gets the type of the robot.
- ***Return Value***: The string representing the robot type.

---

### Power On the Robot
```cpp
bool powerOn()
```
- ***Function***
Powers on the robot.
- ***Return Value***: Returns true if the operation is successful, and false if it fails.

---

### Power Off the Robot
```cpp
bool powerOff()
```
- ***Function***
Powers off the robot.
- ***Return Value***: Returns true if the operation is successful, and false if it fails.

---

### Shut Down the Robot
```cpp
void shutdown()
```
- ***Function***
Shuts down the robot and disconnects the connection.

---

### Get the Remote Control Status
```cpp
bool remoteControl()
```
- ***Function***
Gets whether remote control mode is enabled.
- ***Return Value***: Returns true if remote control mode is enabled, and false otherwise.

---

### Set the Remote Control Status
```cpp
bool remoteControl(bool enable)
```
- ***Function***
Enables or disables remote control mode.
- ***Parameters***
    - enable: True to enable remote control mode, false to disable it.
- ***Return Value***: Returns true if the operation is successful, and false if it fails.

---

### Get the Control Mode
```cpp
RemoteControlMode remoteControlMode()
```
- ***Function***
Gets the current control mode.
- ***Return Value***: Returns `RemoteControlMode::REMOTE`, `RemoteControlMode::LOCAL`, `RemoteControlMode::NONE`, or `RemoteControlMode::UNKNOWN`.

---

### Get the Speed Scaling
```cpp
int speedScaling()
```
- ***Function***
Gets the percentage of the robot's speed scaling.
- ***Return Value***: The percentage of the speed scaling.

---

### Get the Robot Mode
```cpp
RobotMode robotMode()
```
- ***Function***
Gets the current mode of the robot.
- ***Return Value***: An enumeration value of RobotMode.

---

### Get the Safety Mode
```cpp
SafetyMode safetyMode()
```
- ***Function***
Gets the safety mode.
- ***Return Value***: An enumeration value of SafetyMode.

---

### Restart the Safety System
```cpp
bool safetySystemRestart()
```
- ***Function***
Restarts the safety system.
- ***Return Value***: Returns true if the operation is successful, and false if it fails.

---

### Get the Task Status
```cpp
TaskStatus runningStatus()
```
- ***Function***
Gets the running status of the task.
- ***Return Value***: An enumeration value of TaskStatus.

---

### Unlock the Protective Stop
```cpp
bool unlockProtectiveStop()
```
- ***Function***
Unlocks the robot's protective stop.
- ***Return Value***: Returns true if the operation is successful, and false if it fails.

---

### Get the Command Usage
```cpp
std::string usage(const std::string& cmd)
```
- ***Function***
Queries the usage of the dashboard shell command.
- ***Parameters***
    - cmd: The command to be queried.
- ***Return Value***: The string of the command usage.

---

### Get the Dashboard Version
```cpp
std::string version()
```
- ***Function***
Gets the version information of the dashboard.
- ***Return Value***: The string of the version information.

---

### Load the Robot Configuration
```cpp
bool loadConfiguration(const std::string& path)
```
- ***Function***
Loads the robot configuration file.
- ***Parameters***
    - path: The path of the configuration file.
- ***Return Value***: Returns true if the loading is successful, and false if it fails.

---

### Get the Configuration Path
```cpp
std::string configurationPath()
```
- ***Function***
Gets the path of the current configuration file.
- ***Return Value***: The string of the configuration file path.

---

### Check the Configuration Modification
```cpp
bool isConfigurationModify()
```
- ***Function***
Checks whether the configuration has been modified.
- ***Return Value***: Returns true if it has been modified.

---

### Run the Program
```cpp
bool playProgram()
```
- ***Function***
Runs the program.
- ***Return Value***: Returns true if the operation is successful, and false if it fails.

---

### Pause the Program
```cpp
bool pauseProgram()
```
- ***Function***
Pauses the program.
- ***Return Value***: Returns true if the operation is successful, and false if it fails.

---

### Stop the Program
```cpp
bool stopProgram()
```
- ***Function***
Stops the program.
- ***Return Value***: Returns true if the operation is successful, and false if it fails.

---

### Set the Speed Scaling
```cpp
bool setSpeedScaling(int scaling)
```
- ***Function***
Sets the speed scaling.
- ***Parameters***
    - scaling: The speed scaling to be set.
- ***Return Value***: Returns true if the setting is successful, and false if it fails.

---

### Get the Task Path
```cpp
std::string getTaskPath()
```
- ***Function***
Gets the current task path.
- ***Return Value***: The string of the relative task path.

---

### Load the Task
```cpp
bool loadTask(const std::string& path)
```
- ***Function***
Loads the task.
- ***Parameters***
    - path: The path of the task.
- ***Return Value***: Returns true if the loading is successful, and false if it fails.

---

### Get the Task Status
```cpp
TaskStatus getTaskStatus()
```
- ***Function***
Gets the task status.
- ***Return Value***: An enumeration value of TaskStatus.

---

### Check the Task Running Status
```cpp
bool taskIsRunning()
```
- ***Function***
Checks whether the task is running.
- ***Return Value***: Returns true if it is running.

---

### Check the Task Saving Status
```cpp
bool isTaskSaved()
```
- ***Function***
Checks whether the task has been saved.
- ***Return Value***: Returns true if it has been saved.

---

### Send and Receive the Command
```cpp
std::string sendAndReceive(const std::string& cmd)
```
- ***Function***
Sends a dashboard command and receives the response.
- ***Parameters***
    - cmd: The dashboard command to be sent.
- ***Return Value***: The string of the command response.

---
