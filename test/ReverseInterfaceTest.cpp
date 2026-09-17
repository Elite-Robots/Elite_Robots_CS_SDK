#include <gtest/gtest.h>
#include <boost/asio.hpp>
#include <memory>
#include <thread>

#include "ReverseInterface.hpp"
#include "ControlCommon.hpp"
#include "TcpServer.hpp"

#define REVERSE_INTERFACE_TEST_PORT 50002

using namespace ELITE;
using namespace std::chrono;

class TcpClient
{
public:
    boost::asio::io_context io_context;
    std::unique_ptr<boost::asio::ip::tcp::socket> socket_ptr;
    std::unique_ptr<boost::asio::ip::tcp::resolver> resolver_ptr;
    TcpClient() = default;
    
    TcpClient(const std::string& ip, int port) {
        connect(ip, port);
    }

    ~TcpClient() = default;

    void connect(const std::string& ip, int port) {
        try {
            socket_ptr.reset(new boost::asio::ip::tcp::socket(io_context));
            resolver_ptr.reset(new boost::asio::ip::tcp::resolver(io_context));
            socket_ptr->open(boost::asio::ip::tcp::v4());
            boost::asio::ip::tcp::no_delay no_delay_option(true);
            socket_ptr->set_option(no_delay_option);
            boost::asio::socket_base::reuse_address sol_reuse_option(true);
            socket_ptr->set_option(sol_reuse_option);
#if defined(__linux) || defined(linux) || defined(__linux__)
            boost::asio::detail::socket_option::boolean<IPPROTO_TCP, TCP_QUICKACK> quickack(true);
            socket_ptr->set_option(quickack);
#endif
            boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::make_address(ip), port);
            socket_ptr->async_connect(endpoint, [&](const boost::system::error_code& error) {
                if (error) {
                    throw boost::system::system_error(error);
                }
            });
            io_context.run();
        
        } catch(const boost::system::system_error &error) {
            throw error;
        }
    }
    
};

TEST(REVERSE_INTERFACE, trajectory_control_action) {
    auto tcp_resource = std::make_shared<TcpServer::StaticResource>();
    std::unique_ptr<ReverseInterface> reverse_ins = std::make_unique<ReverseInterface>(REVERSE_INTERFACE_TEST_PORT, tcp_resource);
    std::unique_ptr<TcpClient> client = std::make_unique<TcpClient>();

    EXPECT_NO_THROW(client->connect("127.0.0.1", REVERSE_INTERFACE_TEST_PORT));

    std::this_thread::sleep_for(100ms);

    reverse_ins->writeTrajectoryControlAction(TrajectoryControlAction::START, 10, 100);

    int32_t buffer[ReverseInterface::REVERSE_DATA_SIZE];
    int recv_num = client->socket_ptr->read_some(boost::asio::buffer(buffer, sizeof(buffer)));

    // check size
    EXPECT_EQ(recv_num, sizeof(buffer));
    // timeout
    EXPECT_EQ(::htonl(buffer[0]), 100);
    // data 1
    EXPECT_EQ(::htonl(buffer[1]), (int)TrajectoryControlAction::START);
    // data 1
    EXPECT_EQ(::htonl(buffer[2]), 10);
    // control mode
    EXPECT_EQ(::htonl(buffer[8]), (int)ControlMode::MODE_TRAJECTORY);

    client->socket_ptr->close();    
    tcp_resource->shutdown();
}

TEST(REVERSE_INTERFACE, joint_idle_command) {
    auto tcp_resource = std::make_shared<TcpServer::StaticResource>();
    std::unique_ptr<ReverseInterface> reverse_ins = std::make_unique<ReverseInterface>(REVERSE_INTERFACE_TEST_PORT, tcp_resource);
    std::unique_ptr<TcpClient> client = std::make_unique<TcpClient>();

    EXPECT_NO_THROW(client->connect("127.0.0.1", REVERSE_INTERFACE_TEST_PORT));

    std::this_thread::sleep_for(100ms);

    reverse_ins->writeJointCommand({1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f}, ControlMode::MODE_IDLE, 100);

    int32_t buffer[ReverseInterface::REVERSE_DATA_SIZE];
    int recv_num = client->socket_ptr->read_some(boost::asio::buffer(buffer, sizeof(buffer)));

    client->socket_ptr->close();

    // check size
    EXPECT_EQ(recv_num, sizeof(buffer));
    // timeout
    EXPECT_EQ(::htonl(buffer[0]), 100);
    // data 1
    EXPECT_EQ(::htonl(buffer[1]), 1 * CONTROL::POS_ZOOM_RATIO);
    // data 1
    EXPECT_EQ(::htonl(buffer[2]), 2 * CONTROL::POS_ZOOM_RATIO);
    // data 1
    EXPECT_EQ(::htonl(buffer[3]), 3 * CONTROL::POS_ZOOM_RATIO);
    // data 1
    EXPECT_EQ(::htonl(buffer[4]), 4 * CONTROL::POS_ZOOM_RATIO);
    // data 1
    EXPECT_EQ(::htonl(buffer[5]), 5 * CONTROL::POS_ZOOM_RATIO);
    // data 1
    EXPECT_EQ(::htonl(buffer[6]), 6 * CONTROL::POS_ZOOM_RATIO);
    // control mode
    EXPECT_EQ(::htonl(buffer[8]), (int)ControlMode::MODE_IDLE);
    tcp_resource->shutdown();
}

TEST(REVERSE_INTERFACE, joint_command_send_nullptr) {
    auto tcp_resource = std::make_shared<TcpServer::StaticResource>();
    std::unique_ptr<ReverseInterface> reverse_ins = std::make_unique<ReverseInterface>(REVERSE_INTERFACE_TEST_PORT, tcp_resource);
    std::unique_ptr<TcpClient> client = std::make_unique<TcpClient>();

    EXPECT_NO_THROW(client->connect("127.0.0.1", REVERSE_INTERFACE_TEST_PORT));

    std::this_thread::sleep_for(100ms);

    EXPECT_TRUE(reverse_ins->writeJointCommand(nullptr, ControlMode::MODE_IDLE, 100));
    tcp_resource->shutdown();
}


TEST(REVERSE_INTERFACE, disconnect) { 
    auto tcp_resource = std::make_shared<TcpServer::StaticResource>();
    std::unique_ptr<ReverseInterface> reverse_ins;

    reverse_ins.reset(new ReverseInterface(REVERSE_INTERFACE_TEST_PORT, tcp_resource));

    std::unique_ptr<TcpClient> client;
    client.reset(new TcpClient());
    EXPECT_NO_THROW(client->connect("127.0.0.1", REVERSE_INTERFACE_TEST_PORT));

    std::this_thread::sleep_for(50ms);

    EXPECT_TRUE(reverse_ins->isRobotConnect());

    client.reset();

    std::this_thread::sleep_for(50ms);

    EXPECT_FALSE(reverse_ins->isRobotConnect());
    tcp_resource->shutdown();
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
