// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// TcpServer.hpp
// Provides utility functions for string manipulation.
#ifndef __TCP_SERVER_HPP__
#define __TCP_SERVER_HPP__

#include <atomic>
#include <Common/BoostAsioCompat.hpp>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace ELITE {

class TcpServer : public std::enable_shared_from_this<TcpServer> {
   public:
    // Boost io_context and backend thread.
    // All servers use the same io_comtext and thread.
    class StaticResource {
       public:
        std::unique_ptr<std::thread> server_thread_;
        std::shared_ptr<BoostIoWorkGuard> work_guard_ptr_;
        std::shared_ptr<BoostIoContext> io_context_ptr_;
        StaticResource();
        ~StaticResource();
        void shutdown();

        StaticResource(const StaticResource&) = delete;
        StaticResource& operator=(const StaticResource&) = delete;

       private:
        std::atomic<bool> shutting_down_{false};
    };

    // Read callback
    using ReceiveCallback = std::function<void(const uint8_t[], int)>;

    /**
     * @brief Construct a new Tcp Server object
     *
     * @param port Listen port
     * @param recv_buf_size
     * @note Ensure that the start() method has been called before instantiation
     */
    TcpServer(int port, int recv_buf_size, std::shared_ptr<StaticResource> resource);

    /**
     * @brief Destroy the Tcp Server object
     *
     */
    ~TcpServer();

    /**
     * @brief Set the Receive Callback
     *
     * @param cb receive callback
     */
    void setReceiveCallback(ReceiveCallback cb);

    /**
     * @brief Unset the Receive Callback
     *
     */
    void unsetReceiveCallback();

    /**
     * @brief Write data to client
     *
     * @param data data
     * @param size The number of bytes in the data
     * @return int Success send bytes
     */
    int writeClient(void* data, int size);

    /**
     * @brief Start listen port
     *
     */
    void startListen();

    /**
     * @brief Determine if there is a client connected
     *
     * @return true Connected
     * @return false Disconnected
     */
    bool isClientConnected();

   protected:
    std::unique_ptr<boost::asio::ip::tcp::acceptor> acceptor_;
    std::shared_ptr<StaticResource> resource_;

   private:
    // Save connected client. In this project, each server is only connected to one client.
    std::shared_ptr<boost::asio::ip::tcp::socket> socket_;
    boost::asio::ip::tcp::endpoint remote_endpoint_;
    boost::asio::ip::tcp::endpoint local_endpoint_;

    std::vector<uint8_t> read_buffer_;
    ReceiveCallback receive_cb_;
    std::mutex receive_cb_mutex_;
    std::mutex socket_mutex_;

    /**
     * @brief Async accept client connection and add async read task
     *
     */
    virtual void doAccept();

    /**
     * @brief Async receive
     *
     * @param sock Client socket
     */
    void doRead(std::shared_ptr<boost::asio::ip::tcp::socket> sock);

    /**
     * @brief Cancle client asnyc task and close client connection
     *
     * @param sock client socket
     * @param ec boost error code (Only close() function ec is available).
     */
    void closeSocket(std::shared_ptr<boost::asio::ip::tcp::socket> sock, boost::system::error_code& ec);

    /**
     * @brief Call receive callback
     *
     * @param data received data
     * @param size received data size
     */
    void callReceiveCallback(const uint8_t data[], int size);
};

}  // namespace ELITE
#endif
