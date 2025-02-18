#ifndef __SCRIPT_COMMAND_INTERFACE_HPP__
#define __SCRIPT_COMMAND_INTERFACE_HPP__

#include "TcpServer.hpp"
#include "DataType.hpp"

#include <memory>
#include <boost/asio.hpp>

namespace ELITE
{

class ScriptCommandInterface
{
private:
    enum class Cmd{
        ZERO_FTSENSOR = 0,
        SET_PAYLOAD = 1,
        SET_TOOL_VOLTAGE = 2,
        START_FORCE_MODE = 3,
        END_FORCE_MODE = 4,
    };

    std::unique_ptr<TcpServer> server_;
    std::shared_ptr<boost::asio::ip::tcp::socket> client_;
    std::mutex client_mutex_;

    /**
     * @brief Close client and reset clinet point
     * 
     */
    void clientDisconnect();

    /**
     * @brief Send socket data to client
     * 
     * @param buffer data buffer
     * @param size buffer size
     * @return int 
     */
    int write(int32_t buffer[], int size);

    /**
     * @brief Not real read data. Check connection state.
     * 
     */
    void asyncRead();

public:
    static constexpr int SCRIPT_COMMAND_DATA_SIZE = 26;

    ScriptCommandInterface() = delete;

    /**
     * @brief Construct a new Script Command Interface object
     * 
     * @param port Server port
     */
    ScriptCommandInterface(int port);
    
    ~ScriptCommandInterface();

    /**
     * @brief zero the force/torque applied to the TCP measured by the sensor(tare sensor).
     * 
     * @return true 
     * @return false 
     */
    bool zeroFTSensor();

    /**
     * @brief This command is used to set the mass, 
     * center of gravity and moment of inertia of the robot payload
     * 
     * @param mass The mass of the payload
     * @param cog The coordinates of the center of gravity of the payload (relative to the flange frame).
     * @return true success
     * @return false fail
     */
    bool setPayload(double mass, const vector3d_t& cog);

    /**
     * @brief Set the tool voltage
     * 
     * @param vol Tool voltage
     * @return true success
     * @return false fail
     */
    bool setToolVoltage(const ToolVoltage& vol);

    /**
     * @brief This command is used to enable force control mode and the robot will be controlled in the force control mode. 
     * 
     * @param reference_frame A pose vector that defines the force reference frame relative to the base frame. 
     * The format is [X,Y,Z,Rx,Ry,Rz], where X, Y, and Z represent position with the unit of m, Rx, Ry, and RZ
     * represent pose with the unit of rad which is defined by standard Euler angles.
     * @param selection_vector a 6-dimensional vector consisting of 0 and 1 that defines the compliant axis in the force frame. 
     * 1 represents the axis is compliant and 0 represents the axis is non compliant.  
     * @param wrench The force/torque applied to the environment by the robot. 
     * The robot moves/rotates along the compliant axis to adjust its pose to achieve the target force/torque. 
     * The format is [Fx,Fy,Fz,Mx,My,Mz], where Fx, Fy, and Fz represent the force applied along the 
     * compliant axis with the unit of N, Mx, My, and Mz represent the torque applied about the 
     * compliant axis with the unit of Nm. This value is invalid for the non-compliant axis. Due to the 
     * safety restrictions of joints, the actual applied force/torque is lower than the set one. In the 
     * separate thread, the command get_tcp_force may be used to read the actual force/torque applied to the environment.
     * @param mode The parameter for force control mode
     * @param limits The parameter for the speed limit. The format is [Vx,Vy,Vz,ωx,ωy,ωz],
     * where Vx, Vy, and Vz represent the maximum speed for TCP along 
     * the compliant axis with the unit of m/s, ωx, ωy, and ωz represent the maximum speed for TCP 
     * about the compliant axis with the unit of rad/s. This parameter is invalid for the non-compliant 
     * axis whose trajectory will be as set before.
     * @return true success 
     * @return false fail
     */
    bool startForceMode(const vector6d_t& task_frame, const vector6int32_t& selection_vector,
                             const vector6d_t& wrench, const ForceMode& mode, const vector6d_t& limits);
    
    /**
     * @brief This command is used to disable the force control mode. It also will be performed when the procedure ends. 
     * 
     * @return true success
     * @return false fail
     */
    bool endForceMode();

    /**
     * @brief Is robot connect to server.
     * 
     * @return true connected
     * @return false don't
     */
    bool isRobotConnect();

};





} // namespace ELITE


#endif