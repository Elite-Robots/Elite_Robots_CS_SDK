#include <gtest/gtest.h>
#include <boost/asio.hpp>
#include <condition_variable>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>

#include "TrajectoryInterface.hpp"
#include "ControlCommon.hpp"

using namespace ELITE;
using namespace std::chrono;

#define SCRIPT_COMMAND_INTERFACE_TEST_PORT 50004
#define TRAJECTORY_INTERFACE_TEST_PORT 50003


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

static void send_feedback_frame(TcpClient& client, int32_t message_type, int32_t point_index, int32_t total_points, int32_t result,
                                const vector6d_t& point) {
    int32_t frame[TrajectoryInterface::TRAJECTORY_FEEDBACK_LEN] = {0};
    frame[0] = htonl(message_type);
    frame[1] = htonl(point_index);
    frame[2] = htonl(total_points);
    frame[3] = htonl(result);
    for (size_t i = 0; i < point.size(); ++i) {
        frame[4 + i] = htonl(static_cast<int32_t>(std::lround(point[i] * CONTROL::POS_ZOOM_RATIO)));
    }
    client.socket_ptr->send(boost::asio::buffer(frame, sizeof(frame)));
}

TEST(TRAJECTORY_INTERFACE, write_point) {
    auto tcp_resource = std::make_shared<TcpServer::StaticResource>();
    std::unique_ptr<TrajectoryInterface> trajectory_ins = std::make_unique<TrajectoryInterface>(TRAJECTORY_INTERFACE_TEST_PORT, tcp_resource);
    std::unique_ptr<TcpClient> client = std::make_unique<TcpClient>();

    EXPECT_NO_THROW(client->connect("127.0.0.1", TRAJECTORY_INTERFACE_TEST_PORT));

    std::this_thread::sleep_for(50ms);

    TrajectoryMotionFeedback feedback;
    TrajectoryMotionResult motion_result = TrajectoryMotionResult::FAILURE;
    std::mutex callback_mutex;
    std::condition_variable callback_cv;
    bool has_feedback = false;
    bool has_result = false;
    trajectory_ins->setMotionResultCallback([&](TrajectoryMotionResult result) {
        {
            std::lock_guard<std::mutex> lock(callback_mutex);
            motion_result = result;
            has_result = true;
        }
        callback_cv.notify_one();
    });
    trajectory_ins->setMotionFeedbackCallback([&](const TrajectoryMotionFeedback& value) {
        {
            std::lock_guard<std::mutex> lock(callback_mutex);
            feedback = value;
            has_feedback = true;
        }
        callback_cv.notify_one();
    });

    trajectory_ins->writeTrajectoryPoint({1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f}, 100, 1.230, true);

    int32_t buffer[TrajectoryInterface::TRAJECTORY_MESSAGE_LEN];
    int recv_len = client->socket_ptr->read_some(boost::asio::buffer(buffer, sizeof(buffer)));

    EXPECT_EQ(recv_len, sizeof(buffer));

    EXPECT_EQ(::htonl(buffer[0]), 1 * CONTROL::POS_ZOOM_RATIO);
    EXPECT_EQ(::htonl(buffer[1]), 2 * CONTROL::POS_ZOOM_RATIO);
    EXPECT_EQ(::htonl(buffer[2]), 3 * CONTROL::POS_ZOOM_RATIO);
    EXPECT_EQ(::htonl(buffer[3]), 4 * CONTROL::POS_ZOOM_RATIO);
    EXPECT_EQ(::htonl(buffer[4]), 5 * CONTROL::POS_ZOOM_RATIO);
    EXPECT_EQ(::htonl(buffer[5]), 6 * CONTROL::POS_ZOOM_RATIO);

    // time
    EXPECT_EQ(::htonl(buffer[18]), 100 * CONTROL::TIME_ZOOM_RATIO);
    // blend
    EXPECT_EQ(::htonl(buffer[19]), 1.230 * CONTROL::POS_ZOOM_RATIO);
    // mode
    EXPECT_EQ(::htonl(buffer[20]), (int)TrajectoryMotionType::CARTESIAN);

    vector6d_t point{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    send_feedback_frame(*client, (int)TrajectoryFeedbackMessageType::ACTIVE_POINT, 0, 1, -1, point);
    {
        std::unique_lock<std::mutex> lock(callback_mutex);
        EXPECT_TRUE(callback_cv.wait_for(lock, 500ms, [&]() {
            return has_feedback && feedback.message_type == TrajectoryFeedbackMessageType::ACTIVE_POINT;
        }));
    }
    EXPECT_EQ(feedback.message_type, TrajectoryFeedbackMessageType::ACTIVE_POINT);
    EXPECT_EQ(feedback.point_index, 0);
    EXPECT_EQ(feedback.total_points, 1);
    EXPECT_EQ(feedback.result, -1);
    EXPECT_DOUBLE_EQ(feedback.point[0], 1.0);

    send_feedback_frame(*client, (int)TrajectoryFeedbackMessageType::RESULT, 1, 1, (int)TrajectoryMotionResult::SUCCESS, point);

    {
        std::unique_lock<std::mutex> lock(callback_mutex);
        EXPECT_TRUE(callback_cv.wait_for(lock, 500ms, [&]() {
            return has_result && has_feedback && feedback.message_type == TrajectoryFeedbackMessageType::RESULT;
        }));
    }

    EXPECT_EQ(motion_result, TrajectoryMotionResult::SUCCESS);
    EXPECT_EQ(feedback.message_type, TrajectoryFeedbackMessageType::RESULT);
    EXPECT_EQ(feedback.result, (int)TrajectoryMotionResult::SUCCESS);
}

TEST(TRAJECTORY_INTERFACE, disconnect) { 
    auto tcp_resource = std::make_shared<TcpServer::StaticResource>();
    std::unique_ptr<TrajectoryInterface> trajectory_ins;

    trajectory_ins.reset(new TrajectoryInterface(TRAJECTORY_INTERFACE_TEST_PORT, tcp_resource));

    std::unique_ptr<TcpClient> client;
    client.reset(new TcpClient());
    EXPECT_NO_THROW(client->connect("127.0.0.1", TRAJECTORY_INTERFACE_TEST_PORT));

    std::this_thread::sleep_for(50ms);

    EXPECT_TRUE(trajectory_ins->isRobotConnect());

    client.reset();

    std::this_thread::sleep_for(50ms);

    EXPECT_FALSE(trajectory_ins->isRobotConnect());
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
