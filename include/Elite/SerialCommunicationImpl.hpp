// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// SerialCommunicationImpl.hpp
// Provides the SerialCommunication class for RS485 communication over TCP.
#ifndef __ELITE__SERIAL_COMMUNICATION_IMPL_HPP__
#define __ELITE__SERIAL_COMMUNICATION_IMPL_HPP__


#include <Common/BoostAsioCompat.hpp>
#include <Elite/SerialCommunication.hpp>
#include <mutex>

namespace ELITE {

/**
 * @brief RS485 communication class.
 *
 * This class provides an interface for RS485 communication over TCP.
 *
 */
class SerialCommunicationImpl : public SerialCommunication {
   private:
    int tcp_port_;
    int socat_pid_;
    std::string robot_ip_;
    std::mutex socket_mutex_;
    BoostIoContext io_context_;
    boost::asio::ip::tcp::socket socket_;
    
    void socketDisconnect();
   public:
    /**
     * @brief Construct a new Serial Communication object
     *
     */
    SerialCommunicationImpl(int tcp_port, const std::string& ip, int socat_pid);

    /**
     * @brief Destroy the Serial Communication object
     *
     */
    virtual ~SerialCommunicationImpl();

    /**
     * @brief Connect to the RS485 TCP server.
     *
     * @param timeout_ms Timeout in milliseconds.
     * @return true success
     * @return false fail
     */
    virtual bool connect(int timeout_ms);

    /**
     * @brief Disconnect from the RS485 TCP server.
     *
     */
    virtual void disconnect();

    /**
     * @brief Write data to the RS485 TCP server.
     *
     * @param data data buffer
     * @param size data size
     * @return int success write size, -1 fail
     */
    virtual int write(const uint8_t* data, size_t size);

    /**
     * @brief Read data from the RS485 TCP server.
     *
     * @param data data buffer
     * @param size data size
     * @param timeout_ms timeout in milliseconds
     * @return int success read size, -1 fail
     */
    virtual int read(uint8_t* data, size_t size, int timeout_ms);

    /**
     * @brief Check if connected to the RS485 TCP server.
     *
     * @return true connected
     * @return false disconnect
     */
    virtual bool isConnected();

    /**
     * @brief Get the Socat PID
     * 
     * @return int socat pid 
     */
    virtual int getSocatPid() const { return socat_pid_; }
};

}  // namespace ELITE

#endif  // __ELITE__SERIAL_COMMUNICATION_IMPL_HPP__
