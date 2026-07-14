// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
#include "SerialCommunicationImpl.hpp"
#include "Log.hpp"

namespace ELITE {

SerialCommunicationImpl::SerialCommunicationImpl(int tcp_port, const std::string& ip, int socat_pid)
    : robot_ip_(ip), tcp_port_(tcp_port), socat_pid_(socat_pid), socket_(io_context_) {}

SerialCommunicationImpl::~SerialCommunicationImpl() {}

bool SerialCommunicationImpl::connect(int timeout_ms) {
    disconnect();
    try {
        std::lock_guard<std::mutex> lock(socket_mutex_);
        socket_.open(boost::asio::ip::tcp::v4());
        boost::asio::ip::tcp::no_delay no_delay_option(true);
        socket_.set_option(no_delay_option);
        boost::asio::socket_base::reuse_address sol_reuse_option(true);
        socket_.set_option(sol_reuse_option);
#if defined(__linux) || defined(linux) || defined(__linux__)
        boost::asio::detail::socket_option::boolean<IPPROTO_TCP, TCP_QUICKACK> quickack(true);
        socket_.set_option(quickack);
#endif
        boost::asio::ip::tcp::endpoint endpoint(makeBoostAddress(robot_ip_), tcp_port_);
        auto ec_ptr = std::make_shared<boost::system::error_code>(boost::asio::error::would_block);
        socket_.async_connect(endpoint, [ec_ptr](const boost::system::error_code& error) { *ec_ptr = error; });
        if (io_context_.stopped()) {
            restartBoostIoContext(io_context_);
        }
        runBoostIoContextFor(io_context_, std::chrono::milliseconds(timeout_ms));
        if (*ec_ptr) {
            ELITE_LOG_ERROR("Serial connect to robot fail: %s", boost::system::system_error(*ec_ptr).what());
            return false;
        }
    } catch (const boost::system::system_error& error) {
        ELITE_LOG_ERROR("Serial connect to robot fail: %s", error.what());
        return false;
    }
    return true;
}

void SerialCommunicationImpl::disconnect() {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    socketDisconnect();
}

void SerialCommunicationImpl::socketDisconnect() {
    if (!io_context_.stopped()) {
        io_context_.stop();
    }

    if (socket_.is_open()) {
        boost::system::error_code ignore_ec;
        socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ignore_ec);
        socket_.cancel(ignore_ec);
        socket_.close(ignore_ec);
    }
}

bool SerialCommunicationImpl::isConnected() {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (socket_.is_open()) {
        return true;
    }
    return false;
}

int SerialCommunicationImpl::write(const uint8_t* data, size_t size) {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (!socket_.is_open()) {
        return -1;
    }
    boost::system::error_code ec;
    int ret = boost::asio::write(socket_, boost::asio::buffer(data, size), ec);

    if (ec) {
        ELITE_LOG_DEBUG("Serial socket send fail: %s", ec.message().c_str());
        return -1;
    }
    return ret;
}

int SerialCommunicationImpl::read(uint8_t* data, size_t size, int timeout_ms) {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (!socket_.is_open()) {
        return -1;
    }
    int read_len = 0;
    boost::system::error_code error_code;
    if (timeout_ms <= 0) {
        read_len = boost::asio::read(socket_, boost::asio::buffer(data, size), error_code);
        if (error_code && read_len <= 0) {
            ELITE_LOG_ERROR("Serial socket receive fail: %s", error_code.message().c_str());
            return -1;
        }
        return read_len;
    } else {
        if (io_context_.stopped()) {
            restartBoostIoContext(io_context_);
        }
        boost::asio::async_read(socket_, boost::asio::buffer(data, size), [&](const boost::system::error_code& ec, std::size_t nb) {
            error_code = ec;
            read_len = nb;
        });
        runBoostIoContextFor(io_context_, std::chrono::milliseconds(timeout_ms));
        if (error_code && read_len <= 0) {
            ELITE_LOG_ERROR("Serial socket receive fail: %s", error_code.message().c_str());
            return -1;
        }
        return read_len;
    }
}

}  // namespace ELITE
