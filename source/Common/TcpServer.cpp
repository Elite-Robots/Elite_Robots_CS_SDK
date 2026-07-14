// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
#include "TcpServer.hpp"
#include <chrono>
#include <iostream>
#include <thread>
#include "Common/RtUtils.hpp"
#include "EliteException.hpp"
#include "Log.hpp"

namespace ELITE {

namespace {
constexpr int BIND_RETRY_TIMES = 30;
constexpr auto BIND_RETRY_INTERVAL = std::chrono::milliseconds(10);
}

TcpServer::TcpServer(int port, int recv_buf_size, std::shared_ptr<StaticResource> resource) : read_buffer_(recv_buf_size) {
    resource_ = resource;
    boost::system::error_code ec;
    bool logged_address_in_use = false;
    for (int i = 0; i < BIND_RETRY_TIMES; ++i) {
        try {
            acceptor_ = std::make_unique<boost::asio::ip::tcp::acceptor>(
                *(resource_->io_context_ptr_), boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port), true);
            ec = boost::system::error_code();
            break;
        } catch (const boost::system::system_error& error) {
            ec = error.code();
            acceptor_.reset();

            if (ec != boost::asio::error::address_in_use || i == BIND_RETRY_TIMES - 1) {
                break;
            }

            if (!logged_address_in_use) {
                ELITE_LOG_WARN("TCP port %d is in use, retry bind for up to %lld ms", port,
                               static_cast<long long>(BIND_RETRY_INTERVAL.count() * (BIND_RETRY_TIMES - 1)));
                logged_address_in_use = true;
            }
            std::this_thread::sleep_for(BIND_RETRY_INTERVAL);
        }
    }

    if (ec) {
        ELITE_LOG_FATAL("Create TCP server on port %d fail: %s", port, ec.message().c_str());
        throw EliteException(EliteException::Code::SOCKET_FAIL, ec.message());
    }

    acceptor_->listen(1, ec);
    if (ec) {
        ELITE_LOG_FATAL("TCP port %d listen fail: %s", port, ec.message().c_str());
        throw EliteException(EliteException::Code::SOCKET_FAIL, ec.message());
    }
}

TcpServer::~TcpServer() {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (acceptor_ && acceptor_->is_open()) {
        boost::system::error_code ec;
        acceptor_->cancel(ec);
        acceptor_->close(ec);
        acceptor_.reset();
    }
    if (socket_) {
        boost::system::error_code ec;
        closeSocket(socket_, ec);
        socket_.reset();
    }
}

void TcpServer::setReceiveCallback(ReceiveCallback cb) { 
    std::lock_guard<std::mutex> lock(receive_cb_mutex_);
    receive_cb_ = std::move(cb); 
}

void TcpServer::unsetReceiveCallback() {  
    std::lock_guard<std::mutex> lock(receive_cb_mutex_);
    receive_cb_ = nullptr; 
}

void TcpServer::startListen() { doAccept(); }

void TcpServer::doAccept() {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (!acceptor_) {
        return;
    }
    auto new_socket = std::make_shared<boost::asio::ip::tcp::socket>(*(resource_->io_context_ptr_));
    std::weak_ptr<TcpServer> weak_self = shared_from_this();
    // Accept call back
    auto accept_cb = [weak_self, new_socket](boost::system::error_code ec) {
        boost::system::error_code ignore_ec;
        if (auto self = weak_self.lock()) {
            if (!ec) {
                std::lock_guard<std::mutex> lock(self->socket_mutex_);
                // Close old connection
                if (self->socket_ && self->socket_->is_open()) {
                    auto local_point = self->socket_->local_endpoint(ignore_ec);
                    auto remote_point = self->socket_->remote_endpoint(ignore_ec);
                    self->closeSocket(self->socket_, ignore_ec);
                    ELITE_LOG_INFO("TCP port %d has new connection and close old client: %s:%d %s", local_point.port(),
                                   remote_point.address().to_string().c_str(), remote_point.port(),
                                   boost::system::system_error(ignore_ec).what());
                }
                // Socket set option
                new_socket->set_option(boost::asio::socket_base::reuse_address(true), ignore_ec);
                new_socket->set_option(boost::asio::ip::tcp::no_delay(true), ignore_ec);
                new_socket->set_option(boost::asio::socket_base::keep_alive(true), ignore_ec);
#if defined(__linux) || defined(linux) || defined(__linux__)
                new_socket->set_option(boost::asio::detail::socket_option::boolean<IPPROTO_TCP, TCP_QUICKACK>(true));
                new_socket->set_option(boost::asio::detail::socket_option::integer<SOL_SOCKET, SO_PRIORITY>(6));
#endif
                // Update alive socket
                self->socket_ = new_socket;
                // If accept success, get local and remote endpoint
                // Save endpoint info for log
                self->local_endpoint_ = new_socket->local_endpoint(ignore_ec);
                if (ignore_ec) {
                    ELITE_LOG_WARN("Get local endpoint fail: %s", ignore_ec.message().c_str());
                    ignore_ec = boost::system::error_code();
                }
                self->remote_endpoint_ = new_socket->remote_endpoint(ignore_ec);
                if (ignore_ec) {
                    ELITE_LOG_WARN("Get remote endpoint fail: %s", ignore_ec.message().c_str());
                }
                ELITE_LOG_INFO("TCP port %d accept client: %s:%d %s", self->local_endpoint_.port(),
                               self->remote_endpoint_.address().to_string().c_str(), self->remote_endpoint_.port(),
                               boost::system::system_error(ec).what());
                // Start async read
                self->doRead(new_socket);
            } else {
                std::lock_guard<std::mutex> lock(self->socket_mutex_);
                // Close old connection
                if (self->socket_ && self->socket_->is_open()) {
                    auto local_point = self->socket_->local_endpoint(ignore_ec);
                    auto remote_point = self->socket_->remote_endpoint(ignore_ec);
                    self->closeSocket(self->socket_, ignore_ec);
                    ELITE_LOG_ERROR("TCP port %d accept new connection fail(%s), and close old connection %s:%d %s",
                                    local_point.port(), boost::system::system_error(ec).what(),
                                    remote_point.address().to_string().c_str(), remote_point.port(),
                                    boost::system::system_error(ignore_ec).what());
                }
                self->socket_.reset();
            }
            self->doAccept();
        }
    };

    acceptor_->async_accept(*new_socket, accept_cb);
}

void TcpServer::doRead(std::shared_ptr<boost::asio::ip::tcp::socket> sock) {
    std::weak_ptr<TcpServer> weak_self = shared_from_this();
    auto read_cb = [weak_self, sock](boost::system::error_code ec, std::size_t n) {
        if (auto self = weak_self.lock()) {
            if (!ec) {
                self->callReceiveCallback(self->read_buffer_.data(), n);
                // Continue read
                self->doRead(sock);
            } else {
                if (sock->is_open()) {
                    boost::system::error_code ignore_ec;
                    self->closeSocket(sock, ignore_ec);
                    ELITE_LOG_INFO("TCP port %d close client: %s:%d %s. Reason: %s", self->local_endpoint_.port(),
                                   self->remote_endpoint_.address().to_string().c_str(), self->remote_endpoint_.port(),
                                   boost::system::system_error(ignore_ec).what(), boost::system::system_error(ec).what());
                }
            }
        }
    };
    boost::asio::async_read(*sock, boost::asio::buffer(read_buffer_), read_cb);
}

int TcpServer::writeClient(void* data, int size) {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (socket_) {
        try {
            boost::system::error_code ec;
            int wb = boost::asio::write(*socket_, boost::asio::buffer(data, size), ec);
            if (ec) {
                ELITE_LOG_DEBUG("Port %d write TCP client fail: %s", local_endpoint_.port(), ec.message().c_str());
                return -1;
            }
            return wb;
        } catch (const boost::system::system_error& e) {
            ELITE_LOG_DEBUG("Port %d write TCP client exception: %s", local_endpoint_.port(), e.what());
            return -1;
        }
    }
    return -1;
}

bool TcpServer::isClientConnected() {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (socket_) {
        return socket_->is_open();
    }
    return false;
}

void TcpServer::closeSocket(std::shared_ptr<boost::asio::ip::tcp::socket> sock, boost::system::error_code& ec) {
    if (sock->is_open()) {
        sock->cancel(ec);
        sock->shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
        sock->close(ec);
    }
}

void TcpServer::callReceiveCallback(const uint8_t data[], int size) {
    std::lock_guard<std::mutex> lock(receive_cb_mutex_);
    if (receive_cb_) {
        receive_cb_(data, size);
    }
}

TcpServer::StaticResource::StaticResource() {
    if (server_thread_) {
        return;
    }
    if (!io_context_ptr_) {
        io_context_ptr_ = std::make_shared<BoostIoContext>();
    }
    work_guard_ptr_ = makeBoostIoWorkGuard(*io_context_ptr_);
    auto io_ctx = io_context_ptr_;
    server_thread_.reset(new std::thread([io_ctx]() {
        try {
            if (io_ctx->stopped()) {
                restartBoostIoContext(*io_ctx);
            }
            io_ctx->run();
            ELITE_LOG_INFO("TCP server exit thread");
        } catch (const boost::system::system_error& e) {
            ELITE_LOG_FATAL("TCP server thread error: %s", e.what());
        }
    }));

    std::thread::native_handle_type thread_headle = server_thread_->native_handle();
    RT_UTILS::setThreadFiFoScheduling(thread_headle, RT_UTILS::getThreadFiFoMaxPriority());
}

void TcpServer::StaticResource::shutdown() {
    if (shutting_down_.exchange(true)) {
        return;
    }
    work_guard_ptr_.reset();
    io_context_ptr_->stop();
    if (server_thread_ && server_thread_->joinable()) {
        if (std::this_thread::get_id() != server_thread_->get_id()) {
            server_thread_->join();
        } else {
            server_thread_->detach();
            ELITE_LOG_WARN("TcpServer::StaticResource's thread is waiting for itself; setting it to detach.");
        }
    }
    server_thread_.reset();
    io_context_ptr_.reset();
}

TcpServer::StaticResource::~StaticResource() {
    shutdown();
}

}  // namespace ELITE
