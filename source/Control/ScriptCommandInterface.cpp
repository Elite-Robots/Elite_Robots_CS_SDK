#include "ScriptCommandInterface.hpp"
#include "ControlCommon.hpp"
#include "Log.hpp"

namespace ELITE
{

ScriptCommandInterface::ScriptCommandInterface(int port) {
    server_.reset(new TcpServer(port));

    server_->setConnectCallback([&](std::shared_ptr<boost::asio::ip::tcp::socket> client){
        {
            std::lock_guard<std::mutex> lock(client_mutex_);
            if (client_) {
                clientDisconnect();
            }
            client_ = client;
        }
        asyncRead();
    });
}

ScriptCommandInterface::~ScriptCommandInterface() {
    clientDisconnect();
}

void ScriptCommandInterface::clientDisconnect() {
    if (client_) {
        client_.reset();
        ELITE_LOG_INFO("Connection to script command interface dropped.");
    }
}

void ScriptCommandInterface::asyncRead() {
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
            ELITE_LOG_INFO("Connection to script command interface dropped.");
            client_->close();
            asyncRead();
            return;
        } else {
            asyncRead();
        }
    });
}

bool ScriptCommandInterface::zeroFTSensor() {
    std::lock_guard<std::mutex> lock(client_mutex_);
    if (!client_) {
        return false;
    }
    int32_t buffer[SCRIPT_COMMAND_DATA_SIZE] = {0};
    buffer[0] = htonl(static_cast<int32_t>(Cmd::ZERO_FTSENSOR));
    return write(buffer, sizeof(buffer)) > 0;
}

bool ScriptCommandInterface::setPayload(double mass, const vector3d_t& cog) {
    std::lock_guard<std::mutex> lock(client_mutex_);
    if (!client_) {
        return false;
    }
    int32_t buffer[SCRIPT_COMMAND_DATA_SIZE] = {0};
    buffer[0] = htonl(static_cast<int32_t>(Cmd::SET_PAYLOAD));
    buffer[1] = htonl(static_cast<int32_t>((mass * CONTROL::COMMON_ZOOM_RATIO)));
    buffer[2] = htonl(static_cast<int32_t>((cog[0] * CONTROL::COMMON_ZOOM_RATIO)));
    buffer[3] = htonl(static_cast<int32_t>((cog[1] * CONTROL::COMMON_ZOOM_RATIO)));
    buffer[4] = htonl(static_cast<int32_t>((cog[2] * CONTROL::COMMON_ZOOM_RATIO)));
    return write(buffer, sizeof(buffer)) > 0;
}

bool ScriptCommandInterface::setToolVoltage(const ToolVoltage& vol) {
    std::lock_guard<std::mutex> lock(client_mutex_);
    if (!client_) {
        return false;
    }
    int32_t buffer[SCRIPT_COMMAND_DATA_SIZE] = {0};
    buffer[0] = htonl(static_cast<int32_t>(Cmd::SET_TOOL_VOLTAGE));
    buffer[1] = htonl(static_cast<int32_t>(vol) * CONTROL::COMMON_ZOOM_RATIO);
    return write(buffer, sizeof(buffer)) > 0;
}

bool ScriptCommandInterface::startForceMode(const vector6d_t& task_frame, 
                                                 const vector6int32_t& selection_vector,
                                                 const vector6d_t& wrench, 
                                                 const ForceMode& mode, 
                                                 const vector6d_t& limits) {

    std::lock_guard<std::mutex> lock(client_mutex_);
    if (!client_) {
        return false;
    }
    int32_t buffer[SCRIPT_COMMAND_DATA_SIZE] = {0};
    buffer[0] = htonl(static_cast<int32_t>(Cmd::START_FORCE_MODE));
    int32_t* bp = &buffer[1];
    for (auto& tf : task_frame) {
        *bp = htonl(static_cast<int32_t>((tf * CONTROL::COMMON_ZOOM_RATIO)));
        bp++;
    }
    for (auto& sv : selection_vector) {
        *bp = htonl(sv);
        bp++;
    }
    for (auto& wr : wrench) {
        *bp = htonl(static_cast<int32_t>((wr * CONTROL::COMMON_ZOOM_RATIO)));
        bp++;
    }
    *bp = htonl(static_cast<int32_t>(mode));
    bp++;
    for (auto& li : limits) {
        *bp = htonl(static_cast<int32_t>((li * CONTROL::COMMON_ZOOM_RATIO)));
        bp++;
    }
    return write(buffer, sizeof(buffer)) > 0;
}

bool ScriptCommandInterface::endForceMode() {
    std::lock_guard<std::mutex> lock(client_mutex_);
    if (!client_) {
        return false;
    }
    int32_t buffer[SCRIPT_COMMAND_DATA_SIZE] = {0};
    buffer[0] = htonl(static_cast<int32_t>(Cmd::END_FORCE_MODE));
    return write(buffer, sizeof(buffer)) > 0;
}

int ScriptCommandInterface::write(int32_t buffer[], int size) {
    try {
        if(client_->write_some(boost::asio::buffer(buffer, size)) < size) {
            clientDisconnect();
            return -1;
        }
        return size;
    } catch(const boost::system::system_error &error) {
        clientDisconnect();
        return -1;
    }
}

bool ScriptCommandInterface::isRobotConnect() {
    std::lock_guard<std::mutex> lock(client_mutex_);
    if (client_) {
        return client_->is_open();
    } else {
        return false;
    }
}

} // namespace ELITE