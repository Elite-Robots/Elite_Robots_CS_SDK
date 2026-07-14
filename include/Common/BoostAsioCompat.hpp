// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// BoostAsioCompat.hpp
// Provides compatibility aliases for Boost.Asio versions shipped by older Ubuntu releases.
#ifndef __ELITE_BOOST_ASIO_COMPAT_HPP__
#define __ELITE_BOOST_ASIO_COMPAT_HPP__

#include <boost/asio.hpp>
#include <boost/version.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

namespace ELITE {

#if BOOST_VERSION >= 106600
using BoostIoContext = boost::asio::io_context;
using BoostIoWorkGuard = boost::asio::executor_work_guard<BoostIoContext::executor_type>;

inline std::shared_ptr<BoostIoWorkGuard> makeBoostIoWorkGuard(BoostIoContext& io_context) {
    return std::make_shared<BoostIoWorkGuard>(boost::asio::make_work_guard(io_context));
}

inline boost::asio::ip::address makeBoostAddress(const std::string& ip) { return boost::asio::ip::make_address(ip); }

inline void restartBoostIoContext(BoostIoContext& io_context) { io_context.restart(); }

template <typename Rep, typename Period>
void runBoostIoContextFor(BoostIoContext& io_context, const std::chrono::duration<Rep, Period>& timeout) {
    io_context.run_for(timeout);
}
#else
using BoostIoContext = boost::asio::io_service;
using BoostIoWorkGuard = boost::asio::io_service::work;

inline std::shared_ptr<BoostIoWorkGuard> makeBoostIoWorkGuard(BoostIoContext& io_context) {
    return std::make_shared<BoostIoWorkGuard>(io_context);
}

inline boost::asio::ip::address makeBoostAddress(const std::string& ip) { return boost::asio::ip::address::from_string(ip); }

inline void restartBoostIoContext(BoostIoContext& io_context) { io_context.reset(); }

template <typename Rep, typename Period>
void runBoostIoContextFor(BoostIoContext& io_context, const std::chrono::duration<Rep, Period>& timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!io_context.stopped() && std::chrono::steady_clock::now() < deadline) {
        if (io_context.poll_one() == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}
#endif

}  // namespace ELITE

#endif
