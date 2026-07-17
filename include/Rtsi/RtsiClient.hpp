// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// RtsiClient.hpp
// Provides the RtsiClient class for RTSI communication with the robot.
#ifndef __RTSICLIENT_HPP__
#define __RTSICLIENT_HPP__

#include "RtsiRecipe.hpp"
#include "VersionInfo.hpp"
#include <Common/BoostAsioCompat.hpp>

#include <array>
#include <functional>
#include <memory>
#include <tuple>
#include <vector>

namespace ELITE {

/**
 * @brief
 *      Rtsi client
 *
 */
class RtsiClient {
   public:
    static constexpr uint16_t DEFAULT_PROTOCOL_VERSION = 1;

    RtsiClient() = default;
    virtual ~RtsiClient() = default;

    /**
     * @brief Connect to robot RTSI server
     *
     * @param ip The robot IP
     * @param port RTSI port
     */
    void connect(const std::string& ip, int port = 30004);

    /**
     * @brief Disconnect
     *
     */
    void disconnect();

    /**
     * @brief Verify the protocol version.
     *
     * @param version The version of RTSI
     * @return bool true if successful
     */
    bool negotiateProtocolVersion(uint16_t version = DEFAULT_PROTOCOL_VERSION);

    /**
     * @brief Get the Controller Version object
     *
     * @return std::tuple<uint32_t, uint32_t, uint32_t, uint32_t>
     * A tuple type where the data is, in order, major version, minor version, bugfix, and build.
     */
    VersionInfo getControllerVersion();

    /**
     * @brief Subscribe to output variables.
     *
     * @param recipe The list of recipe. The variable names are explained in the document.
     * @param frequency Setup output frenqucy
     * @return RtsiRecipeSharedPtr The data recipe
     */
    RtsiRecipeSharedPtr setupOutputRecipe(const std::vector<std::string>& recipe_list, double frequency = 250);

    /**
     * @brief Subscribe to input variables.
     *
     * @param recipe The list of recipe. The variable names are explained in the document.
     * @return RtsiRecipeSharedPtr The data recipe
     */
    RtsiRecipeSharedPtr setupInputRecipe(const std::vector<std::string>& recipe);

    /**
     * @brief Send start signal to server
     *
     * @return true Start successfully
     * @return false Start fail
     */
    bool start();

    /**
     * @brief Send pause signal to server
     *
     * @return true Pause successfully
     * @return false Pause fail
     */
    bool pause();

    /**
     * @brief Send an recipe to controller
     *
     * @param recipe  The recipe sent to the controller.
     */
    void send(RtsiRecipeSharedPtr& recipe);

    /**
     * @brief Receive RTSI output recipes data
     *
     * @param recipes The recipe you want to receive. Note that only one recipe will be received.
     * @param read_newest If want to parser the newest message
     * @return int The ID of recipe which is received. If -1, not match recipe
     */
    int receiveData(std::vector<RtsiRecipeSharedPtr>& recipes, bool read_newest = false);

    /**
     * @brief Receive RTSI output recipe data
     *
     * @param recipe The recipe you want to receive.
     * @param read_newest If want to parser the newest message
     * @return true success
     * @return false false
     */
    bool receiveData(RtsiRecipeSharedPtr recipe, bool read_newest = false);

    /**
     * @brief Get connection state
     *
     * @return true connected
     * @return false disconnect
     */
    bool isConnected();

    /**
     * @brief Is start to sync robot data
     *
     * @return true started
     * @return false not started
     */
    bool isStarted();

    /**
     * @brief This function is used to determine have bytes that may be read without blocking.
     *
     * @return true has bytes
     * @return false don't has
     */
    bool isReadAvailable();

   private:
    enum class PackageType : uint8_t;

    BoostIoContext io_context_;
    std::unique_ptr<boost::asio::ip::tcp::socket> socket_ptr_;
    std::unique_ptr<boost::asio::ip::tcp::resolver> resolver_ptr_;

    enum ConnectionState { DISCONNECTED, CONNECTED, STARTED, STOPED };
    ConnectionState connection_state;

    /**
     * @brief Rtsi package type
     *
     */
    enum class PackageType : uint8_t {
        REQUEST_PROTOCOL_VERSION = 86,       // ascii V
        GET_ELITE_CONTROL_VERSION = 118,     // ascii v
        TEXT_MESSAGE = 77,                   // ascii M
        DATA_PACKAGE = 85,                   // ascii U
        CONTROL_PACKAGE_SETUP_OUTPUTS = 79,  // ascii O
        CONTROL_PACKAGE_SETUP_INPUTS = 73,   // ascii I
        CONTROL_PACKAGE_START = 83,          // ascii S
        CONTROL_PACKAGE_PAUSE = 80           // ascii P
    };

    /**
     * @brief Send an package to RTSI server
     *
     * @param cmd Send package type
     * @param payload Package payload
     */
    void sendAll(const PackageType& cmd, const std::vector<uint8_t>& payload = std::vector<uint8_t>());

    /**
     * @brief Receive socket bytes from RTSI server
     *
     * @param buff Data buffer
     * @param size Size of buffer
     * @param offset Offset of buffer
     * @param timeout_ms Timeout(ms)
     * @return int The number of bytes recieved
     */
    int receiveSocket(std::vector<uint8_t>& buff, int size, int offset, unsigned timeout_ms = 1000);

    /**
     * @brief Loop receive util target package come
     *
     * @param target_type Target package type
     * @param parser_func When receive target type, will call the parser function
     * @param read_newest If want to parser the newest message
     */
    void receive(const PackageType& target_type, std::function<void(int, const std::vector<uint8_t>&)> parser_func,
                 bool read_newest = false);

    /**
     * @brief Close socket connection
     *
     */
    void socketDisconnect();
};

}  // namespace ELITE

#endif
