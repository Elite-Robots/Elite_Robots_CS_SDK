#include "TcpServer.hpp"
#include "EliteException.hpp"
#include "Log.hpp"
#include <iostream>

using namespace ELITE;

TcpServer::TcpServer(int port) : port_(port) {
    loop_keep_alive_ = true;
    server_thread_ = std::make_unique<std::thread>([&]() { 
        serverLoop(); 
    });
}

TcpServer::~TcpServer() {
    loop_keep_alive_ = false;
    io_context_.stop();
    if (server_thread_->joinable()) {
        server_thread_->join();
    }
    
}

void TcpServer::doAccept(boost::asio::ip::tcp::acceptor &acceptor) {
    acceptor.async_accept([&](boost::system::error_code ec, boost::asio::ip::tcp::socket client_socket) {
        std::shared_ptr<boost::asio::ip::tcp::socket> client_socket_ptr = std::make_shared<boost::asio::ip::tcp::socket>(std::move(client_socket));
        client_socket_ptr->set_option(boost::asio::ip::tcp::no_delay(true));
        if (new_connect_function_) {
            new_connect_function_(client_socket_ptr);
        }
        doAccept(acceptor);
    });
}


void TcpServer::serverLoop() {
    boost::asio::ip::tcp::acceptor acceptor(io_context_, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port_));
    acceptor.listen(1);
    if (!acceptor.is_open()) {
        throw EliteException(EliteException::Code::SOCKET_FAIL);
    }
    
    while (loop_keep_alive_) {
        try {
            doAccept(acceptor);
        
            if(io_context_.stopped()) {
                io_context_.restart();
            }
            io_context_.run();
        } catch(const boost::system::system_error &error) {
            ELITE_LOG_INFO("TCP server %d has error: %s", port_, error.what());
            continue;
        }
    }
}
