// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
#include "ScriptSender.hpp"
#include <boost/asio.hpp>
#include "ControlCommon.hpp"
#include "EliteException.hpp"
#include "Log.hpp"

using namespace ELITE;

ScriptSender::ScriptSender(int port, const std::string& program, std::shared_ptr<TcpServer::StaticResource> resource)
    : program_(program), TcpServer(port, 0, resource) {
    doAccept();
}

ScriptSender::~ScriptSender() {}

void ScriptSender::doAccept() {
    auto new_socket = std::make_shared<boost::asio::ip::tcp::socket>(*(resource_->io_context_ptr_));
    // Accept call back
    auto accept_cb = [this, new_socket](boost::system::error_code ec) {
        if (!ec) {
            responseRequest(new_socket);
        }
        ScriptSender::doAccept();
    };
    acceptor_->listen(1);
    acceptor_->async_accept(*new_socket, accept_cb);
}

void ScriptSender::responseRequest(std::shared_ptr<boost::asio::ip::tcp::socket> sock) {
    boost::asio::async_read_until(*sock, recv_request_buffer_, '\n', [&, sock](boost::system::error_code ec, std::size_t len) {
        if (ec) {
            if (sock->is_open()) {
                ELITE_LOG_INFO("Connection to script sender interface dropped: %s", boost::system::system_error(ec).what());
            }
        } else {
            ELITE_LOG_INFO("Robot request external control script.");
            std::string request;
            std::istream response_stream(&recv_request_buffer_);
            std::getline(response_stream, request);
            if (request == PROGRAM_REQUEST_) {
                boost::system::error_code wec;
                sock->write_some(boost::asio::buffer(program_), wec);
                if (wec) {
                    ELITE_LOG_ERROR("Script sender send script fail: %s", boost::system::system_error(wec).what());
                    return;
                }
            }
            responseRequest(sock);
        }
    });
}
