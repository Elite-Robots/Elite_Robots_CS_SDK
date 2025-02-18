#ifndef __REVERSE_INTERFACE_HPP__
#define __REVERSE_INTERFACE_HPP__

#include "TcpServer.hpp"
#include "ControlMode.hpp"
#include "DataType.hpp"

#include <boost/asio.hpp>
#include <mutex>

namespace ELITE
{

/**
 * @brief The ReverseInterface class handles communication to the robot. 
 * It starts a server and waits for the robot to connect via its EliteRobot program.
 * 
 * 
 */
class ReverseInterface
{
private:
    int port_;
    std::unique_ptr<TcpServer> server_;
    std::shared_ptr<boost::asio::ip::tcp::socket> client_;
    std::mutex client_mutex_;

    /**
     * @brief Client disconnect
     * 
     */
    void clientDisconnect();

    /**
     * @brief Not real read data. Check connection state.
     * 
     */
    void asyncRead();

    /**
     * @brief Write buffer to socket.
     * 
     * @return int data
     */
    int write(int32_t buffer[], int size);

public:
    static const int REVERSE_DATA_SIZE = 8;

    ReverseInterface() = delete;

    /**
     * @brief Construct a new Reverse Interface object include TcpServer
     * 
     * @param port Port the Server is started on
     */
    ReverseInterface(int port);

    /**
     * @brief Destroy the Reverse Interface object. Will disconnect robot connection.
     * 
     */
    ~ReverseInterface();

    /**
     * @brief Writes needed information to the robot to be read by the EliteRobot program.
     * 
     * @param pos 
     * @param mode 
     * @param timeout_ms 
     * @return true 
     * @return false 
     */
    bool writeJointCommand(const vector6d_t& pos, ControlMode mode, int timeout_ms);
    bool writeJointCommand(const vector6d_t* pos, ControlMode mode, int timeout_ms);

    /**
     * @brief Writes needed information to the robot to be read by the EliteRobot program.
     * 
     * @param action Trajectory action assigned to this command. See documentation of TrajectoryControlAction for details on possible values.
     * @param point_number The number of points of the trajectory to be executed
     * @param timeout_ms The read timeout configuration for the reverse socket running in the external control script on the robot.
     * @return true success
     * @return false fail
     */
    bool writeTrajectoryControlAction(TrajectoryControlAction action, const int point_number, int timeout_ms);

    /**
     * @brief Finish external control script.
     * 
     * @return true success
     * @return false fail
     */
    bool stopControl();

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