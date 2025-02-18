#ifndef __TCP_SERVER_HPP__
#define __TCP_SERVER_HPP__

#include <boost/asio.hpp>
#include <memory>
#include <functional>
#include <vector>

namespace ELITE
{

class TcpServer {
private:
    boost::asio::io_context io_context_;
    bool loop_keep_alive_;
    int port_;
    std::unique_ptr<std::thread> server_thread_;
    std::function<void (std::shared_ptr<boost::asio::ip::tcp::socket>)> new_connect_function_;

    /**
     * @brief TCP server loop. Run boost library async interface.
     * 
     */
    void serverLoop();

    /**
     * @brief Accept new connection is connected
     * 
     * @param acceptor 
     */
    void doAccept(boost::asio::ip::tcp::acceptor &acceptor);

public:
    TcpServer() = delete;

    /**
     * @brief Construct a new Tcp Server object
     * 
     * @param port Server port
     */
    TcpServer(int port);
    
    /**
     * @brief Destroy the Tcp Server object. Will join the serverLoop() finish.
     * 
     */
    ~TcpServer();

    /**
     * @brief Set callback function when new connection in.
     * 
     * @param func 
     */
    void setConnectCallback(std::function<void (std::shared_ptr<boost::asio::ip::tcp::socket>)> func) {
        new_connect_function_ = func;
    }

};



} // namespace ELITE


#endif
