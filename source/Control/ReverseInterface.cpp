#include "ReverseInterface.hpp"
#include "ControlCommon.hpp"
#include "EliteException.hpp"
#include "Log.hpp"

using namespace ELITE;

ReverseInterface::ReverseInterface(int port) : port_(port) {
    server_ = std::make_unique<TcpServer>(port);
    server_->setConnectCallback([&](std::shared_ptr<boost::asio::ip::tcp::socket> client) {
        {
            std::lock_guard<std::mutex> lock(client_mutex_);
            clientDisconnect();
            client_ = client;
        }
        asyncRead();
    });
}

ReverseInterface::~ReverseInterface() {
    clientDisconnect();
}

void ReverseInterface::asyncRead() {
    std::lock_guard<std::mutex> lock(client_mutex_);
    if (!client_) {
        return;
    }
    if (!client_->is_open()) {
        client_.reset();
        return;
    }
    std::shared_ptr<int> no_use;
    no_use.reset(new int);
    client_->async_read_some(boost::asio::buffer(no_use.get(), sizeof(int)), [&, no_use](boost::system::error_code ec, std::size_t len){
        if (len <= 0 || ec) {
            ELITE_LOG_INFO("Connection to reverse interface dropped.");
            client_->close();
            asyncRead();
            return;
        } else {
            asyncRead();
        }
    });
}

int ReverseInterface::write(int32_t buffer[], int size) {
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

void ReverseInterface::clientDisconnect() {
    if (client_) {
        ELITE_LOG_INFO("Connection to reverse interface dropped.");
        client_.reset();
    }
}

bool ReverseInterface::writeJointCommand(const vector6d_t& pos, ControlMode mode, int timeout) {
    return writeJointCommand(&pos, mode, timeout);
}

bool ReverseInterface::writeJointCommand(const vector6d_t* pos, ControlMode mode, int timeout) {
    std::lock_guard<std::mutex> lock(client_mutex_);
    if (!client_) {
        return false;
    }
    int32_t data[REVERSE_DATA_SIZE] = {0};
    data[0] = htonl(timeout);
    data[REVERSE_DATA_SIZE - 1] = htonl((int)mode);
    if (pos) {
        for (size_t i = 0; i < 6; i++) {
            data[i + 1] = htonl(round((*pos)[i] * CONTROL::POS_ZOOM_RATIO));
        }
    }
    
    return write(data, sizeof(data)) > 0;
}

bool ReverseInterface::writeTrajectoryControlAction(TrajectoryControlAction action, int point_number, int timeout) {
    std::lock_guard<std::mutex> lock(client_mutex_);
    if (!client_) {
        return false;
    }
    int32_t data[REVERSE_DATA_SIZE] = {0};
    data[0] = htonl(timeout);
    data[1] = htonl((int)action);
    data[2] = htonl(point_number);
    data[REVERSE_DATA_SIZE - 1] = htonl((int)ControlMode::MODE_TRAJECTORY);
    return write(data, sizeof(data)) > 0;
}

bool ReverseInterface::stopControl() {
    std::lock_guard<std::mutex> lock(client_mutex_);
    if (!client_) {
        return false;
    }
    int32_t data[REVERSE_DATA_SIZE];
    data[0] = 0;
    data[REVERSE_DATA_SIZE - 1] = htonl((int)ControlMode::MODE_STOPPED);
    
    return write(data, sizeof(data)) > 0;
}

bool ReverseInterface::isRobotConnect() {
    std::lock_guard<std::mutex> lock(client_mutex_);
    if (client_) {
        return client_->is_open();
    } else {
        return false;
    }
}
