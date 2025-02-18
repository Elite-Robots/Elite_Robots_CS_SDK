#include "TrajectoryInterface.hpp"
#include "ControlCommon.hpp"
#include "EliteException.hpp"
#include "Log.hpp"
#include <boost/asio.hpp>

using namespace ELITE;



TrajectoryInterface::TrajectoryInterface(int port) {
    server_.reset(new TcpServer(port));

    server_->setConnectCallback([&](std::shared_ptr<boost::asio::ip::tcp::socket> client){
        ELITE_LOG_INFO("Trajectory interface accept new connection.");
        {
            std::lock_guard<std::mutex> lock(client_mutex_);
            if (client_) {
                clientDisconnect();
            }
            client_ = client;
        }
        receiveResult();
    });
}


TrajectoryInterface::~TrajectoryInterface() {
    
}


void TrajectoryInterface::receiveResult() {
    std::lock_guard<std::mutex> lock(client_mutex_);
    if (!client_) {
        return;
    }
    if (!client_->is_open()) {
        client_.reset();
        return;
    }
    client_->async_read_some(boost::asio::buffer(&motion_result_, sizeof(motion_result_)), [&](boost::system::error_code ec, std::size_t len){
        if (len <= 0 || ec) {
            ELITE_LOG_INFO("Connection to trajectory interface dropped.");
            client_->close();
            return;
        }
        
        motion_result_ = (TrajectoryMotionResult)htonl((int)motion_result_);
        if (motion_result_func_) {
            motion_result_func_(motion_result_);
        }
        receiveResult();
    });
}


bool TrajectoryInterface::writeTrajectoryPoint( const vector6d_t& positions, 
                                                float time, 
                                                float blend_radius, 
                                                bool cartesian) {
    std::lock_guard<std::mutex> lock(client_mutex_);
    if (!client_) {
        return false;
    }
    int32_t buffer[TRAJECTORY_MESSAGE_LEN] = {0};
    for (size_t i = 0; i < 6; i++) {
        buffer[i] = htonl(round(positions[i] * CONTROL::POS_ZOOM_RATIO));
    }
    buffer[18] = htonl(round(time * CONTROL::TIME_ZOOM_RATIO));
    buffer[19] = htonl(round(blend_radius * CONTROL::POS_ZOOM_RATIO));
    if (cartesian) {
        buffer[20] = htonl((int)TrajectoryMotionType::CARTESIAN);
    } else {
        buffer[20] = htonl((int)TrajectoryMotionType::JOINT);
    }

    return write(buffer, sizeof(buffer)) > 0;
}

void TrajectoryInterface::clientDisconnect() {
    if (client_) {
        ELITE_LOG_INFO("Connection to trajectory interface dropped.");
        client_.reset();
    }
}

int TrajectoryInterface::write(int32_t buffer[], int size) {
    try {
        if(client_->write_some(boost::asio::buffer(buffer, size)) < size) {
            clientDisconnect();
            return size;
        }
        return size;
    } catch(const boost::system::system_error &error) {
        clientDisconnect();
        return -1;
    }
}

bool TrajectoryInterface::isRobotConnect() {
    std::lock_guard<std::mutex> lock(client_mutex_);
    if (client_) {
        return client_->is_open();
    } else {
        return false;
    }
}