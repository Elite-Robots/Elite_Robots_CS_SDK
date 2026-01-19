// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// SshUtils.hpp
// Provides utility functions for SSH and SCP operations.
#ifndef __ELITE__SSH_UTILS_HPP__
#define __ELITE__SSH_UTILS_HPP__

#include <Elite/EliteOptions.hpp>

#include <functional>
#include <string>

namespace ELITE {

namespace SSH_UTILS {
/**
 * @brief Log in to the server via SSH, execute commands, and return the output
 * of the commands.
 *
 * @param host SSH server IP
 * @param user user name
 * @param password user password
 * @param cmd Want execute commands
 * @return std::string The result of the command.
 */
ELITE_EXPORT std::string executeCommand(const std::string &host, const std::string &user, const std::string &password, const std::string &cmd);

/**
 * @brief Download files via SCP.
 *
 * @param server SSH server IP
 * @param user User name
 * @param password User password
 * @param remote_path Remote file path
 * @param local_path Save path (the file name needs to be included).
 * @param progress_cb Download progress callback function.
 *      f_z: File size.
 *      r_z: Downloaded size.
 *      err: Error information (nullptr when there is no error)
 * @return true sucess
 * @return false fail
 */
ELITE_EXPORT bool downloadFile(const std::string &server, const std::string &user, const std::string &password, const std::string &remote_path,
                  const std::string &local_path, std::function<void(int f_z, int r_z, const char *err)> progress_cb);

/**
 * @brief Download file via SCP
 *
 * @param server SSH server IP
 * @param user User name
 * @param password User password
 * @param remote_path Remote file path (the file name needs to be included).
 * @param local_path Save path
 * @param progress_cb Download progress callback function.
 *      f_z: File size.
 *      w_z: Uploaded size.
 *      err: Error information (nullptr when there is no error)
 * @return true sucess
 * @return false fail
 */
ELITE_EXPORT bool uploadFile(const std::string &server, const std::string &user, const std::string &password, const std::string &remote_path,
                const std::string &local_path, std::function<void(int f_z, int w_z, const char *err)> progress_cb);

}  // namespace SSH_UTILS

}  // namespace ELITE

#endif