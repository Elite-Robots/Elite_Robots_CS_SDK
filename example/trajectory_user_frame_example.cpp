// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// Cartesian trajectory example in the base frame and in an SDK-managed user
// frame.

#include <Elite/DataType.hpp>
#include <Elite/EliteDriver.hpp>
#include <Elite/Log.hpp>
#include <Elite/RtsiIOInterface.hpp>

#include <atomic>
#include <boost/program_options.hpp>
#include <chrono>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace ELITE;
namespace po = boost::program_options;
using namespace std::chrono_literals;

namespace {

constexpr int32_t kFrameBId = 1;
constexpr double kPi = 3.14159265358979323846;

class TrajectoryControl {
   public:
    explicit TrajectoryControl(const EliteDriverConfig& config) : config_(config) {
        driver_ = std::make_unique<EliteDriver>(config_);
    }

    ~TrajectoryControl() {
        if (driver_) {
            driver_->stopControl();
        }
    }

    EliteDriver& driver() { return *driver_; }

    bool startControl() {
        PrimaryPortInterface& primary = driver_->primaryPort();

        ELITE_LOG_INFO("Start powering on...");
        if (!primary.powerOn()) {
            ELITE_LOG_FATAL("Power-on failed");
            return false;
        }
        ELITE_LOG_INFO("Power-on succeeded");

        ELITE_LOG_INFO("Start releasing brake...");
        if (!primary.brakeRelease()) {
            ELITE_LOG_FATAL("Brake release failed");
            return false;
        }
        ELITE_LOG_INFO("Brake released");

        if (config_.headless_mode) {
            if (!driver_->isRobotConnected() && !driver_->sendExternalControlScript()) {
                ELITE_LOG_FATAL("Fail to send external control script");
                return false;
            }
        } else {
            ELITE_LOG_INFO("Please start the External Control task from the robot side.");
        }

        ELITE_LOG_INFO("Wait external control script run...");
        if (!waitForRobotConnectionStable(300ms, 5s)) {
            ELITE_LOG_FATAL("Timed out waiting for external control script to become stable");
            return false;
        }
        ELITE_LOG_INFO("External control script is running");
        return true;
    }

    bool moveTrajectory(const std::vector<vector6d_t>& target_points, float point_time, float blend_radius, bool cartesian,
                        float speed, float acceleration, int32_t user_frame_id) {
        current_point_.store(-1);
        total_points_.store(0);
        last_result_.store(-1);

        std::promise<TrajectoryMotionResult> move_done_promise;
        auto move_done_future = move_done_promise.get_future();
        driver_->setTrajectoryResultCallback([&move_done_promise](TrajectoryMotionResult result) {
            try {
                move_done_promise.set_value(result);
            } catch (const std::future_error&) {
            }
        });
        driver_->setTrajectoryFeedbackCallback([&](const TrajectoryMotionFeedback& feedback) {
            {
                std::lock_guard<std::mutex> lock(feedback_mutex_);
                last_feedback_ = feedback;
            }

            if (feedback.message_type == TrajectoryFeedbackMessageType::ACTIVE_POINT) {
                current_point_.store(feedback.point_index);
                total_points_.store(feedback.total_points);
                ELITE_LOG_INFO("Trajectory point %d/%d is active", feedback.point_index + 1, feedback.total_points);
                ELITE_LOG_INFO("Active point target = [%lf, %lf, %lf, %lf, %lf, %lf]", feedback.point[0], feedback.point[1],
                               feedback.point[2], feedback.point[3], feedback.point[4], feedback.point[5]);
            } else if (feedback.message_type == TrajectoryFeedbackMessageType::POINT_DONE) {
                current_point_.store(feedback.point_index);
                total_points_.store(feedback.total_points);
                ELITE_LOG_INFO("Trajectory point %d/%d is done", feedback.point_index + 1, feedback.total_points);
                ELITE_LOG_INFO("Done point target = [%lf, %lf, %lf, %lf, %lf, %lf]", feedback.point[0], feedback.point[1],
                               feedback.point[2], feedback.point[3], feedback.point[4], feedback.point[5]);
            } else if (feedback.message_type == TrajectoryFeedbackMessageType::RESULT) {
                last_result_.store(feedback.result);
                ELITE_LOG_INFO("Trajectory result frame received: %d", feedback.result);
            }
        });

        ELITE_LOG_INFO("Trajectory motion start in frame id: %d", user_frame_id);
        if (!driver_->writeTrajectoryControlAction(TrajectoryControlAction::START, target_points.size(), 200)) {
            ELITE_LOG_ERROR("Failed to start trajectory motion");
            return false;
        }

        for (const auto& point : target_points) {
            bool point_sent = false;
            if (point_time > 0.0f) {
                point_sent = driver_->writeTrajectoryPoint(point, point_time, blend_radius, cartesian, user_frame_id);
            } else {
                point_sent = driver_->writeTrajectoryPoint(point, blend_radius, cartesian, speed, acceleration, user_frame_id);
            }
            if (!point_sent) {
                ELITE_LOG_ERROR("Failed to write trajectory point");
                return false;
            }
            if (!driver_->writeTrajectoryControlAction(TrajectoryControlAction::NOOP, 0, 200)) {
                ELITE_LOG_ERROR("Failed to send NOOP command");
                return false;
            }
        }

        int last_logged_point = -2;
        const auto deadline = std::chrono::steady_clock::now() + 20s;
        while (move_done_future.wait_for(50ms) != std::future_status::ready) {
            if (std::chrono::steady_clock::now() >= deadline) {
                ELITE_LOG_ERROR("Timed out waiting for trajectory result");
                return false;
            }

            const int current_point = current_point_.load();
            const int total_points = total_points_.load();
            if (current_point != last_logged_point && current_point >= 0 && total_points > 0) {
                const TrajectoryMotionFeedback feedback = getLastFeedback();
                ELITE_LOG_INFO("Cached progress says current point is %d/%d", current_point + 1, total_points);
                ELITE_LOG_INFO("Cached point value = [%lf, %lf, %lf, %lf, %lf, %lf]", feedback.point[0], feedback.point[1],
                               feedback.point[2], feedback.point[3], feedback.point[4], feedback.point[5]);
                last_logged_point = current_point;
            }
            if (!driver_->writeTrajectoryControlAction(TrajectoryControlAction::NOOP, 0, 200)) {
                ELITE_LOG_ERROR("Failed to send NOOP command");
                return false;
            }
        }

        const auto result = move_done_future.get();
        ELITE_LOG_INFO("Trajectory motion completed with result: %d", result);

        if (!driver_->writeIdle(0)) {
            ELITE_LOG_ERROR("Failed to write idle command");
            return false;
        }
        return result == TrajectoryMotionResult::SUCCESS;
    }

    bool moveTrajectoryByTime(const std::vector<vector6d_t>& target_points, float point_time, float blend_radius, bool cartesian,
                              int32_t user_frame_id) {
        return moveTrajectory(target_points, point_time, blend_radius, cartesian, 0.0f, 0.0f, user_frame_id);
    }

    bool moveToJointTargetBySpeed(const vector6d_t& point, float speed, float acceleration) {
        return moveTrajectory({point}, 0.0f, 0.0f, false, speed, acceleration, BASE_USER_FRAME_ID);
    }

   private:
    bool waitForRobotConnectionStable(std::chrono::milliseconds stable_period, std::chrono::milliseconds timeout) {
        const auto start_time = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point connected_since;

        while (std::chrono::steady_clock::now() - start_time < timeout) {
            if (driver_->isRobotConnected()) {
                if (connected_since == std::chrono::steady_clock::time_point{}) {
                    connected_since = std::chrono::steady_clock::now();
                } else if (std::chrono::steady_clock::now() - connected_since >= stable_period) {
                    return true;
                }
            } else {
                connected_since = std::chrono::steady_clock::time_point{};
            }
            std::this_thread::sleep_for(10ms);
        }

        return false;
    }

    TrajectoryMotionFeedback getLastFeedback() {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        return last_feedback_;
    }

    std::unique_ptr<EliteDriver> driver_;
    EliteDriverConfig config_;
    std::atomic<int> current_point_{-1};
    std::atomic<int> total_points_{0};
    std::atomic<int> last_result_{-1};
    std::mutex feedback_mutex_;
    TrajectoryMotionFeedback last_feedback_;
};

}  // namespace

int main(int argc, const char** argv) {
    constexpr float kJointSpeed = 0.5f;
    constexpr float kJointAcceleration = 0.8f;
    constexpr float kPointTime = 3.0f;

    EliteDriverConfig config;
    config.script_file_path = "external_control.script";
    config.headless_mode = true;
    config.max_user_frame_count = 2;

    po::options_description description(
        "Usage:\n"
        "  ./trajectory_user_frame_example --robot-ip <ip> [options]\n\n"
        "The robot first runs the trajectory_example points in BASE, then runs\n"
        "the same points in frame_b. frame_b is a 90-degree rotation around\n"
        "the base Z axis.\n\n"
        "Options:");
    description.add_options()
        ("help,h", "Print help message")
        ("robot-ip", po::value<std::string>(&config.robot_ip)->required(), "Robot IP address")
        ("local-ip", po::value<std::string>(&config.local_ip)->default_value(""), "Local IP address")
        ("use-headless-mode", po::value<bool>(&config.headless_mode)->default_value(true)->implicit_value(true),
         "Send and run External Control automatically");

    po::variables_map variables;
    try {
        po::store(po::parse_command_line(argc, argv, description), variables);
        if (variables.count("help")) {
            std::cout << description << "\n";
            return 0;
        }
        po::notify(variables);
    } catch (const po::error& error) {
        std::cerr << "Argument error: " << error.what() << "\n\n" << description << "\n";
        return 1;
    }

    const vector6d_t frame_b_pose{{0.0, 0.0, 0.0, 0.0, 0.0, kPi}};
    config.user_frames.push_back(UserFrame{kFrameBId, "frame_b", frame_b_pose, true});

    std::unique_ptr<TrajectoryControl> trajectory_control;
    std::unique_ptr<RtsiIOInterface> rtsi_client;
    try {
        trajectory_control = std::make_unique<TrajectoryControl>(config);
        rtsi_client = std::make_unique<RtsiIOInterface>("output_recipe.txt", "input_recipe.txt", 250);

        ELITE_LOG_INFO("Connecting to the RTSI");
        if (!rtsi_client->connect(config.robot_ip)) {
            throw std::runtime_error("Fail to connect or config to the RTSI");
        }
        ELITE_LOG_INFO("Successfully connected to the RTSI");

        ELITE_LOG_INFO("Starting trajectory control...");
        if (!trajectory_control->startControl()) {
            throw std::runtime_error("Failed to start trajectory control");
        }
        ELITE_LOG_INFO("Trajectory control started");

        vector6d_t actual_joints = rtsi_client->getActualJointPositions();
        actual_joints[3] = -1.57;
        ELITE_LOG_INFO("Moving to the same joint configuration as trajectory_example");
        if (!trajectory_control->moveToJointTargetBySpeed(actual_joints, kJointSpeed, kJointAcceleration)) {
            throw std::runtime_error("Failed to move to the initial joint configuration");
        }
        ELITE_LOG_INFO("Initial joint configuration reached");

        UserFrame frame_b;
        if (!trajectory_control->driver().getUserFrame(kFrameBId, frame_b)) {
            throw std::runtime_error("Failed to get frame_b from SDK cache");
        }

        const vector6d_t base_start = rtsi_client->getActualTCPPose();
        std::vector<vector6d_t> base_trajectory;
        vector6d_t base_pose = base_start;
        base_pose[2] -= 0.2;
        base_trajectory.push_back(base_pose);
        base_pose[1] -= 0.2;
        base_trajectory.push_back(base_pose);
        base_pose[1] += 0.2;
        base_pose[2] += 0.2;
        base_trajectory.push_back(base_pose);

        ELITE_LOG_INFO("Executing trajectory in BASE, %zu points", base_trajectory.size());
        if (!trajectory_control->moveTrajectoryByTime(base_trajectory, kPointTime, 0.0f, true, BASE_USER_FRAME_ID)) {
            throw std::runtime_error("Base-frame trajectory failed");
        }

        const vector6d_t user_start = rtsi_client->getActualTCPPose(frame_b);
        std::vector<vector6d_t> user_trajectory;
        vector6d_t user_pose = user_start;
        user_pose[2] -= 0.2;
        user_trajectory.push_back(user_pose);
        user_pose[1] -= 0.2;
        user_trajectory.push_back(user_pose);
        user_pose[1] += 0.2;
        user_pose[2] += 0.2;
        user_trajectory.push_back(user_pose);

        ELITE_LOG_INFO("Executing trajectory in frame_b, %zu points", user_trajectory.size());
        if (!trajectory_control->moveTrajectoryByTime(user_trajectory, kPointTime, 0.0f, true, kFrameBId)) {
            throw std::runtime_error("frame_b trajectory failed");
        }

        trajectory_control->driver().stopControl();
        rtsi_client->disconnect();
    } catch (const std::exception& error) {
        std::cerr << "Example failed: " << error.what() << "\n";
        if (trajectory_control) {
            trajectory_control->driver().stopControl();
        }
        if (rtsi_client) {
            rtsi_client->disconnect();
        }
        return 1;
    }

    return 0;
}
