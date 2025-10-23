#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#if defined(__linux) || defined(linux) || defined(__linux__)
#include <sys/wait.h>
#include <unistd.h>
#elif defined(_WIN32) || defined(_WIN64)
#define NOMINMAX
#include <io.h>
#define stat _stat64
#endif

#ifdef ELITE_USE_LIB_SSH
#include <libssh/libssh.h>
#endif

#ifndef FILE_PERMISSIONS
#define FILE_PERMISSIONS 0644
#endif



#include "Common/SshUtils.hpp"
#include "Elite/Log.hpp"

namespace ELITE {

namespace SSH_UTILS {

std::string executeCommand(const std::string& host, const std::string& user, const std::string& password, const std::string& cmd) {
#ifdef ELITE_USE_LIB_SSH
    ssh_session session = ssh_new();
    if (!session) {
        ELITE_LOG_ERROR("Failed to create SSH session");
        return "";
    }

    ssh_options_set(session, SSH_OPTIONS_HOST, host.c_str());
    ssh_options_set(session, SSH_OPTIONS_USER, user.c_str());

    if (ssh_connect(session) != SSH_OK) {
        ELITE_LOG_ERROR("SSH connection failed: %s", ssh_get_error(session));
        ssh_free(session);
        return "";
    }

    if (ssh_userauth_password(session, nullptr, password.c_str()) != SSH_AUTH_SUCCESS) {
        ELITE_LOG_ERROR("Authentication failed: %s", ssh_get_error(session));
        ssh_disconnect(session);
        ssh_free(session);
        return "";
    }

    ssh_channel channel = ssh_channel_new(session);
    if (!channel) {
        ELITE_LOG_ERROR("Failed to create SSH channel");
        ssh_disconnect(session);
        ssh_free(session);
        return "";
    }

    if (ssh_channel_open_session(channel) != SSH_OK) {
        ELITE_LOG_ERROR("Failed to open SSH channel: %s", ssh_get_error(session));
        ssh_channel_free(channel);
        ssh_disconnect(session);
        ssh_free(session);
        return "";
    }

    if (ssh_channel_request_exec(channel, cmd.c_str()) != SSH_OK) {
        ELITE_LOG_ERROR("Failed to execute command: ", ssh_get_error(session));
        ssh_channel_close(channel);
        ssh_channel_free(channel);
        ssh_disconnect(session);
        ssh_free(session);
        return "";
    }

    char buffer[4096];
    int nbytes;
    std::string result;
    while ((nbytes = ssh_channel_read(channel, buffer, sizeof(buffer) - 1, 0)) > 0) {
        buffer[nbytes] = '\0';
        result += buffer;
    }

    ssh_channel_send_eof(channel);
    ssh_channel_close(channel);
    ssh_channel_free(channel);
    ssh_disconnect(session);
    ssh_free(session);

    return result;
#else
#if defined(__linux) || defined(linux) || defined(__linux__)
    int pipefd[2];
    if (pipe(pipefd) == -1) {
        char buf[256] = {0};
        ELITE_LOG_ERROR("Execute cmd \"%s\" fail: %s", cmd.c_str(), strerror_r(errno, buf, sizeof(buf)));
        return "";
    }

    pid_t pid = fork();
    if (pid == -1) {
        char buf[256] = {0};
        ELITE_LOG_ERROR("Execute cmd \"%s\" fail: %s", cmd.c_str(), strerror_r(errno, buf, sizeof(buf)));
        return "";
    }

    if (pid == 0) {  // child process
        // Close reader
        close(pipefd[0]);
        // Redirect stdout to a pipe.
        dup2(pipefd[1], STDOUT_FILENO);
        // Close the writter (which has been duplicated to stdout).
        close(pipefd[1]);
        execlp("sshpass", "sshpass", "-p", password.c_str(), "ssh", "-o", "StrictHostKeyChecking=no", (user + "@" + host).c_str(),
               cmd.c_str(), nullptr);
        char err_buf[256] = {0};
        ELITE_LOG_ERROR("Execute cmd \"%s\" fail: %d", cmd.c_str(), strerror_r(errno, err_buf, sizeof(err_buf)));
        exit(1);
    } else {
        // Close the writter
        close(pipefd[1]);
        char buffer[256];
        std::ostringstream result;

        // Read child porcess output
        ssize_t bytesRead;
        while ((bytesRead = read(pipefd[0], buffer, sizeof(buffer) - 1)) > 0) {
            buffer[bytesRead] = '\0';
            result << buffer;
        }
        // Close reader
        close(pipefd[0]);
        int status;
        waitpid(pid, &status, 0);
        if (WIFEXITED(status)) {
            ELITE_LOG_INFO("Execute command \"%s\" exited with status: %d", cmd.c_str(), WEXITSTATUS(status));
        }
        return result.str();
    }
#else
    return "";
#endif
#endif
}

static bool scpCommand(const std::string& password, const std::string& path1, const std::string& path2) {
#if defined(__linux) || defined(linux) || defined(__linux__)
    pid_t pid = fork();
    if (pid == -1) {
        char err_buf[256] = {0};
        ELITE_LOG_ERROR("scp path1: \"%s\" path2: \"%s\" fail: %d", path1.c_str(), path2.c_str(),
                        strerror_r(errno, err_buf, sizeof(err_buf)));
        return false;
    }

    if (pid == 0) {
        execlp("sshpass", "sshpass", "-p", password.c_str(), "scp", "-o", "StrictHostKeyChecking=no", path1.c_str(), path2.c_str(),
               nullptr);
        perror("execlp failed");
        exit(1);
    } else {
        int status;
        waitpid(pid, &status, 0);
        if (WIFEXITED(status)) {
            ELITE_LOG_INFO("scp path1: \"%s\" path2: \"%s\" exited with status: %d", path1.c_str(), path2.c_str(),
                           WEXITSTATUS(status));
            if (status) {
                return false;
            } else {
                return true;
            }
        }
        return false;
    }
#else
    return false;
#endif
}

bool downloadFile(const std::string& server, const std::string& user, const std::string& password, const std::string& remote_path,
                  const std::string& local_path, std::function<void(int f_z, int r_z, const char* err)> progress_cb) {
#ifdef ELITE_USE_LIB_SSH
    // Read 1 MB each time.
    constexpr int CHUNK_SIZE = 1048576;
    ssh_session session = ssh_new();
    if (!session) {
        ELITE_LOG_ERROR("Failed to create SSH session");
        return false;
    }

    ssh_options_set(session, SSH_OPTIONS_HOST, server.c_str());
    ssh_options_set(session, SSH_OPTIONS_USER, user.c_str());

    if (ssh_connect(session) != SSH_OK) {
        ELITE_LOG_ERROR("SSH connection failed: %s", ssh_get_error(session));
        ssh_free(session);
        return false;
    }

    if (ssh_userauth_password(session, nullptr, password.c_str()) != SSH_AUTH_SUCCESS) {
        ELITE_LOG_ERROR("SSH authentication failed: %s", ssh_get_error(session));
        ssh_disconnect(session);
        ssh_free(session);
        return false;
    }

    ssh_scp scp = ssh_scp_new(session, SSH_SCP_READ, remote_path.c_str());
    if (!scp || ssh_scp_init(scp) != SSH_OK) {
        ELITE_LOG_ERROR("Failed to initialize SCP: %s", ssh_get_error(session));
        ssh_scp_free(scp);
        ssh_disconnect(session);
        ssh_free(session);
        return false;
    }

    if (ssh_scp_pull_request(scp) != SSH_SCP_REQUEST_NEWFILE) {
        ELITE_LOG_ERROR("Failed to pull SCP request: %s", ssh_get_error(session));
        ssh_scp_free(scp);
        ssh_disconnect(session);
        ssh_free(session);
        return false;
    }

    int file_size = ssh_scp_request_get_size(scp);
    std::string file_name = ssh_scp_request_get_filename(scp);
    ELITE_LOG_INFO("Downloading: %s (%d bytes)", file_name.c_str(), file_size);

    std::ofstream local_file(local_path, std::ios::binary);
    if (!local_file) {
        ELITE_LOG_ERROR("Failed to open local file: %s", local_path.c_str());
        ssh_scp_free(scp);
        ssh_disconnect(session);
        ssh_free(session);
        return false;
    }

    // Read in chunks.
    int total_read = 0;
    std::vector<char> buffer(CHUNK_SIZE);
    while (total_read < file_size) {
        int bytes_to_read = std::min(CHUNK_SIZE, file_size - total_read);
        int bytes_read = ssh_scp_read(scp, buffer.data(), bytes_to_read);

        if (bytes_read == SSH_ERROR) {
            const char* ssh_err = ssh_get_error(session);
            progress_cb(file_size, total_read, ssh_err);
            ELITE_LOG_ERROR("SCP read error: %s", ssh_err);
            break;
        }

        local_file.write(buffer.data(), bytes_read);
        total_read += bytes_read;

        if (progress_cb) {
            progress_cb(file_size, total_read, nullptr);
        }
    }

    ELITE_LOG_INFO("Download complete!");

    local_file.close();
    ssh_scp_free(scp);
    ssh_disconnect(session);
    ssh_free(session);
    return true;
#else
    (void)progress_cb;
    return scpCommand(password, (user + "@" + server + ":" + remote_path), local_path);
#endif
}

bool uploadFile(const std::string& server, const std::string& user, const std::string& password, const std::string& remote_path,
                const std::string& local_path, std::function<void(int f_z, int r_z, const char* err)> progress_cb) {
#ifdef ELITE_USE_LIB_SSH
    // Write 1 MB each time.
    constexpr int CHUNK_SIZE = 1048576;
    ssh_session session = ssh_new();
    if (!session) {
        ELITE_LOG_ERROR("Failed to create SSH session");
    }

    ssh_options_set(session, SSH_OPTIONS_HOST, server.c_str());
    ssh_options_set(session, SSH_OPTIONS_USER, user.c_str());

    if (ssh_connect(session) != SSH_OK) {
        ELITE_LOG_ERROR("SSH connection failed: %s", ssh_get_error(session));
        ssh_free(session);
        return false;
    }

    if (ssh_userauth_password(session, nullptr, password.c_str()) != SSH_AUTH_SUCCESS) {
        ELITE_LOG_ERROR("SSH authentication failed: %s", ssh_get_error(session));
        ssh_disconnect(session);
        ssh_free(session);
        return false;
    }

    std::ifstream local_file(local_path, std::ios::binary | std::ios::ate);
    if (!local_file) {
        ELITE_LOG_ERROR("Failed to open local file: %s", local_path.c_str());
        ssh_disconnect(session);
        ssh_free(session);
        return false;
    }

    size_t total_size = local_file.tellg();
    local_file.seekg(0, std::ios::beg);

    ssh_scp scp = ssh_scp_new(session, SSH_SCP_WRITE | SSH_SCP_RECURSIVE, remote_path.c_str());
    if (!scp) {
        ELITE_LOG_ERROR("SCP session creation failed: %s", ssh_get_error(session));
        ssh_disconnect(session);
        ssh_free(session);
        return false;
    }

    if (ssh_scp_init(scp) != SSH_OK) {
        ELITE_LOG_ERROR("SCP initialization failed: ", ssh_get_error(session));
        ssh_scp_free(scp);
        ssh_disconnect(session);
        ssh_free(session);
        return false;
    }

#if defined(_WIN32) || defined(_WIN64)
#define FILE_PERMISSIONS (S_IREAD | S_IWRITE)
#elif defined(__linux) || defined(linux) || defined(__linux__) || defined(__APPLE__)
#define FILE_PERMISSIONS (S_IRUSR | S_IWUSR)
#endif
    // File's infomation
    if (ssh_scp_push_file(scp, remote_path.c_str(), total_size, FILE_PERMISSIONS) != SSH_OK) {
        ELITE_LOG_ERROR("Failed to push file info: ", ssh_get_error(session));
        ssh_scp_free(scp);
        ssh_disconnect(session);
        ssh_free(session);
        return false;
    }

    // Write in chunks.
    std::vector<char> buffer(CHUNK_SIZE);
    size_t uploaded_size = 0;
    int last_percent = -1;

    ELITE_LOG_INFO("Uploading: %s (%d bytes)", local_path.c_str(), total_size);

    while (local_file) {
        local_file.read(buffer.data(), sizeof(buffer));
        std::streamsize bytes_read = local_file.gcount();
        if (bytes_read > 0) {
            if (ssh_scp_write(scp, buffer.data(), bytes_read) != SSH_OK) {
                ELITE_LOG_ERROR("Failed to write to SCP session");
                const char* ssh_err = ssh_get_error(session);
                progress_cb(total_size, uploaded_size, ssh_err);
                ssh_scp_free(scp);
                ssh_disconnect(session);
                ssh_free(session);
                return false;
            }
            uploaded_size += bytes_read;
        }
        if (progress_cb) {
            progress_cb(total_size, uploaded_size, nullptr);
        }
    }
    ELITE_LOG_INFO("Upload complete!");

    local_file.close();
    ssh_scp_close(scp);
    ssh_scp_free(scp);
    ssh_disconnect(session);
    ssh_free(session);
    return true;
#else
    (void)progress_cb;
    return scpCommand(password, local_path, (user + "@" + server + ":" + remote_path));
#endif
}

}  // namespace SSH_UTILS

}  // namespace ELITE
