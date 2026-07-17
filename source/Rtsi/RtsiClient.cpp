// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
#include "RtsiClient.hpp"
#include "EliteException.hpp"
#include "RtsiRecipeInternal.hpp"
#include "Utils.hpp"
#include "VersionInfo.hpp"
#include "Log.hpp"

#include <array>
#include <iostream>

using namespace ELITE;

#define RTSI_HEADR_SIZE (3)

void RtsiClient::connect(const std::string& ip, int port) {
    try {
        // If reconnect, the buffer not clean
        socket_ptr_.reset(new boost::asio::ip::tcp::socket(io_context_));
        resolver_ptr_.reset(new boost::asio::ip::tcp::resolver(io_context_));
        socket_ptr_->open(boost::asio::ip::tcp::v4());
        socket_ptr_->set_option(boost::asio::ip::tcp::no_delay(true));
        socket_ptr_->set_option(boost::asio::socket_base::reuse_address(true));
        socket_ptr_->set_option(boost::asio::socket_base::keep_alive(false));
#if defined(__linux) || defined(linux) || defined(__linux__)
        socket_ptr_->set_option(boost::asio::detail::socket_option::boolean<IPPROTO_TCP, TCP_QUICKACK>(true));
        socket_ptr_->set_option(boost::asio::detail::socket_option::integer<SOL_SOCKET, SO_PRIORITY>(6));
#endif
        boost::asio::ip::tcp::endpoint endpoint(makeBoostAddress(ip), port);
        socket_ptr_->async_connect(endpoint, [&](const boost::system::error_code& error) {
            if (!error) {
                connection_state = ConnectionState::CONNECTED;
            } else {
                connection_state = ConnectionState::DISCONNECTED;
                ELITE_LOG_ERROR("Connect to RTSI server %s:%d fail: %s", ip.c_str(), port, error.message().c_str());
            }
        });
        io_context_.run();

    } catch (const boost::system::system_error& error) {
        throw EliteException(EliteException::Code::SOCKET_CONNECT_FAIL, error.what());
    }
}

void RtsiClient::disconnect() {
    socketDisconnect();
}

bool RtsiClient::negotiateProtocolVersion(uint16_t version) {
    std::vector<uint8_t> payload{(uint8_t)(version >> 8), (uint8_t)version};
    sendAll(PackageType::REQUEST_PROTOCOL_VERSION, payload);
    bool is_accept = false;
    receive(PackageType::REQUEST_PROTOCOL_VERSION, [&](int len, const std::vector<uint8_t>& package) {
        // According to the RTSI document, does the fourth byte of the message represent whether the version check is successful.
        is_accept = package[3];
    });
    return is_accept;
}

VersionInfo RtsiClient::getControllerVersion() {
    sendAll(PackageType::GET_ELITE_CONTROL_VERSION);
    VersionInfo version;
    receive(PackageType::GET_ELITE_CONTROL_VERSION, [&](int len, const std::vector<uint8_t>& package) {
        int offset = RTSI_HEADR_SIZE;
        EndianUtils::unpack(package, offset, version.major);
        EndianUtils::unpack(package, offset, version.minor);
        EndianUtils::unpack(package, offset, version.bugfix);
        EndianUtils::unpack(package, offset, version.build);
    });
    return version;
}

RtsiRecipeSharedPtr RtsiClient::setupOutputRecipe(const std::vector<std::string>& recipe_list, double frequency) {
    // The first eight bytes of the payload section in the output subscription message are the frequency.
    std::vector<uint8_t> payload = EndianUtils::pack(frequency);
    for (auto i : recipe_list) {
        std::copy(i.begin(), i.end(), std::back_inserter(payload));
        payload.push_back(',');
    }
    // Remove the last redundant ','.
    payload.pop_back();
    sendAll(PackageType::CONTROL_PACKAGE_SETUP_OUTPUTS, payload);

    RtsiRecipeInternal* recipe = new RtsiRecipeInternal(recipe_list);
    receive(PackageType::CONTROL_PACKAGE_SETUP_OUTPUTS,
            [&](int len, const std::vector<uint8_t>& package) { recipe->parserTypePackage(len, package); });
    RtsiRecipeSharedPtr result(static_cast<RtsiRecipe*>(recipe));
    return result;
}

RtsiRecipeSharedPtr RtsiClient::setupInputRecipe(const std::vector<std::string>& recipe_list) {
    std::vector<uint8_t> payload;
    for (auto i : recipe_list) {
        std::copy(i.begin(), i.end(), std::back_inserter(payload));
        payload.push_back(',');
    }
    // Remove the last redundant ','.
    payload.pop_back();
    sendAll(PackageType::CONTROL_PACKAGE_SETUP_INPUTS, payload);

    RtsiRecipeInternal* recipe = new RtsiRecipeInternal(recipe_list);
    receive(PackageType::CONTROL_PACKAGE_SETUP_INPUTS,
            [&](int len, const std::vector<uint8_t>& package) { recipe->parserTypePackage(len, package); });
    RtsiRecipeSharedPtr result(static_cast<RtsiRecipe*>(recipe));
    return result;
}

bool RtsiClient::start() {
    sendAll(PackageType::CONTROL_PACKAGE_START);
    bool is_start = false;
    receive(PackageType::CONTROL_PACKAGE_START, [&](int len, const std::vector<uint8_t>& package) {
        // According to the RTSI document, does the fourth byte of the message represent whether data transmission has started
        // successfully.
        is_start = package[3];
        if (is_start) {
            connection_state = ConnectionState::STARTED;
        }
    });
    return is_start;
}

bool RtsiClient::pause() {
    sendAll(PackageType::CONTROL_PACKAGE_PAUSE);
    bool is_pause = false;
    receive(PackageType::CONTROL_PACKAGE_PAUSE, [&](int len, const std::vector<uint8_t>& package) {
        // According to the RTSI document, does the fourth byte of the message represent whether data transmission has paused
        // successfully.
        is_pause = package[3];
        if (is_pause) {
            connection_state = ConnectionState::STOPED;
        }
    });
    return is_pause;
}

bool RtsiClient::isConnected() { return connection_state != ConnectionState::DISCONNECTED; }

bool RtsiClient::isStarted() { return connection_state == ConnectionState::STARTED; }

bool RtsiClient::isReadAvailable() { return socket_ptr_ ? socket_ptr_->available() : false; }

void RtsiClient::send(RtsiRecipeSharedPtr& recipe) {
    sendAll(PackageType::DATA_PACKAGE, static_cast<RtsiRecipeInternal*>(recipe.get())->packToBytes());
}

int RtsiClient::receiveData(std::vector<RtsiRecipeSharedPtr>& recipes, bool read_newest) {
    int result_id = -1;
    receive(
        PackageType::DATA_PACKAGE,
        [&](int len, const std::vector<uint8_t>& package) {
            // Referring to the RTSI document, the fourth byte of the message is the recipe ID.
            int recipe_id = package[3];
            for (size_t i = 0; i < recipes.size(); i++) {
                if (!recipes[i]) {
                    break;
                }
                if (recipes[i]->getID() == recipe_id) {
                    static_cast<RtsiRecipeInternal*>(recipes[i].get())->parserDataPackage(len, package);
                    result_id = recipe_id;
                    break;
                }
            }
        },
        read_newest);
    return result_id;
}

bool RtsiClient::receiveData(RtsiRecipeSharedPtr recipe, bool read_newest) {
    bool result = false;
    receive(
        PackageType::DATA_PACKAGE,
        [&](int len, const std::vector<uint8_t>& package) {
            // Referring to the RTSI document, the fourth byte of the message is the recipe ID.
            int recipe_id = package[3];
            if (recipe->getID() == recipe_id) {
                static_cast<RtsiRecipeInternal*>(recipe.get())->parserDataPackage(len, package);
                result = true;
            }
        },
        read_newest);

    return result;
}

void RtsiClient::sendAll(const PackageType& cmd, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> message(RTSI_HEADR_SIZE);
    uint16_t message_len = RTSI_HEADR_SIZE + payload.size();

    // Build package header
    message[0] = (uint8_t)(message_len >> 8);
    message[1] = (uint8_t)message_len;
    message[2] = static_cast<uint8_t>(cmd);
    // Push back payload
    std::copy(payload.begin(), payload.end(), std::back_inserter(message));

    boost::system::error_code ec;
    socket_ptr_->write_some(boost::asio::buffer(message), ec);
    if (ec) {
        ELITE_LOG_FATAL("RTSI socket send fail: %s", ec.message().c_str());
        throw EliteException(EliteException::Code::SOCKET_FAIL, ec.message());
    }
}

void RtsiClient::socketDisconnect() {
    socket_ptr_.reset();
    connection_state = DISCONNECTED;
}

int RtsiClient::receiveSocket(std::vector<uint8_t>& buff, int size, int offset, unsigned timeout_ms) {
    if (buff.size() < (size + offset)) {
        buff.resize(size + offset);
    }
    int read_len = 0;
    boost::asio::async_read(*socket_ptr_, boost::asio::buffer(buff.data() + offset, size),
                            [&](const boost::system::error_code& ec, std::size_t nb) {
                                if (ec) {
                                    ELITE_LOG_FATAL("RTSI socket receive fail: %s", ec.message().c_str());
                                    throw EliteException(EliteException::Code::SOCKET_FAIL, ec.message());
                                }
                                read_len = nb;
                            });

    // Restart the io_context, as it may have been left in the "stopped" state
    // by a previous operation.
    if (io_context_.stopped()) {
        restartBoostIoContext(io_context_);
    }

    // Block until the asynchronous operation has completed, or timed out. If
    // the pending asynchronous operation is a composed operation, the deadline
    // applies to the entire operation, rather than individual operations on
    // the socket.
    runBoostIoContextFor(io_context_, std::chrono::milliseconds(timeout_ms));

    // If the asynchronous operation completed successfully then the io_context
    // would have been stopped due to running out of work. If it was not
    // stopped, then the io_context::run_for call must have timed out.
    if (!io_context_.stopped()) {
        // Disconnect to cancel the outstanding asynchronous operation.
        socketDisconnect();

        // Clear socket receive or send operation
        auto work = makeBoostIoWorkGuard(io_context_);
        io_context_.run();
        work.reset();

        return -1;
    }
    return read_len;
}

void RtsiClient::receive(const PackageType& target_type, std::function<void(int, const std::vector<uint8_t>&)> parser_func,
                         bool read_newest) {
    std::vector<uint8_t> buff(4069);
    // Receive RTSI package head
    while (receiveSocket(buff, RTSI_HEADR_SIZE, 0) > 0) {
        // Parser package head
        uint16_t pkg_len;
        EndianUtils::unpack(buff.begin(), pkg_len);
        PackageType pkg_type = static_cast<PackageType>(buff[2]);

        // Receive RTSI package body
        if (receiveSocket(buff, (pkg_len - RTSI_HEADR_SIZE), RTSI_HEADR_SIZE) <= 0) {
            break;
        }

        if (target_type == pkg_type) {
            parser_func(pkg_len, buff);
            if (!read_newest) {
                return;
            } else {
                if (socket_ptr_->available() >= RTSI_HEADR_SIZE) {
                    continue;
                } else {
                    break;
                }
            }
        }
    }
}
