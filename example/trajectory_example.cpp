// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
#include <Elite/DataType.hpp>
#include <Elite/EliteDriver.hpp>
#include <Elite/Log.hpp>
#include <Elite/RtsiIOInterface.hpp>

#include <atomic>
#include <boost/program_options.hpp>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

using namespace ELITE;
namespace po = boost::program_options;

class TrajectoryControl {
   private:
    std::unique_ptr<EliteDriver> driver_;
    EliteDriverConfig config_;
    std::atomic<int> current_point_{-1};
    std::atomic<int> total_points_{0};
    std::atomic<int> last_result_{-1};
    std::mutex feedback_mutex_;
    TrajectoryMotionFeedback last_feedback_;

   public:
    explicit TrajectoryControl(const EliteDriverConfig& config) {
        config_ = config;
        driver_ = std::make_unique<EliteDriver>(config);
    }

    ~TrajectoryControl() {
        if (driver_) {
            driver_->stopControl();
        }
    }

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
            if (!driver_->isRobotConnected()) {
                if (!driver_->sendExternalControlScript()) {
                    ELITE_LOG_FATAL("Fail to send external control script");
                    return false;
                }
            }
        } else {
            ELITE_LOG_INFO("Please start the External Control task from the robot side.");
        }

        ELITE_LOG_INFO("Wait external control script run...");
        while (!driver_->isRobotConnected()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        ELITE_LOG_INFO("External control script is running");
        return true;
    }

    bool moveTrajectoryByTime(const std::vector<vector6d_t>& target_points, float point_time, float blend_radius, bool is_cartesian) {
        return moveTrajectory(target_points, point_time, blend_radius, is_cartesian, 0.0f, 0.0f);
    }

    bool moveTrajectoryBySpeed(const std::vector<vector6d_t>& target_points, float blend_radius, bool is_cartesian, float speed,
                               float acceleration) {
        return moveTrajectory(target_points, 0.0f, blend_radius, is_cartesian, speed, acceleration);
    }

    bool moveToJointTargetBySpeed(const vector6d_t& point, float speed, float acceleration) {
        return moveTrajectoryBySpeed({point}, 0.0f, false, speed, acceleration);
    }

   private:
    bool moveTrajectory(const std::vector<vector6d_t>& target_points, float point_time, float blend_radius, bool is_cartesian, float speed,
                        float acceleration) {
        current_point_.store(-1);
        total_points_.store(0);
        last_result_.store(-1);

        std::promise<TrajectoryMotionResult> move_done_promise;
        driver_->setTrajectoryResultCallback([&](TrajectoryMotionResult result) { move_done_promise.set_value(result); });
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
            } else if (feedback.message_type == TrajectoryFeedbackMessageType::RESULT) {
                last_result_.store(feedback.result);
                ELITE_LOG_INFO("Trajectory result frame received: %d", feedback.result);
            }
        });

        ELITE_LOG_INFO("Trajectory motion start");
        if (!driver_->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::START, target_points.size(), 200)) {
            ELITE_LOG_ERROR("Failed to start trajectory motion");
            return false;
        }

        for (const auto& point : target_points) {
            bool point_sent = false;
            if (point_time > 0.0f) {
                point_sent = driver_->writeTrajectoryPoint(point, point_time, blend_radius, is_cartesian);
            } else {
                point_sent = driver_->writeTrajectoryPoint(point, blend_radius, is_cartesian, speed, acceleration);
            }

            if (!point_sent) {
                ELITE_LOG_ERROR("Failed to write trajectory point");
                return false;
            }
            if (!driver_->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::NOOP, 0, 200)) {
                ELITE_LOG_ERROR("Failed to send NOOP command");
                return false;
            }
        }

        std::future<TrajectoryMotionResult> move_done_future = move_done_promise.get_future();
        int last_logged_point = -2;
        while (move_done_future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
            int current_point = current_point_.load();
            int total_points = total_points_.load();
            if (current_point != last_logged_point && current_point >= 0 && total_points > 0) {
                TrajectoryMotionFeedback feedback = getLastFeedback();
                ELITE_LOG_INFO("Cached progress says current point is %d/%d", current_point + 1, total_points);
                ELITE_LOG_INFO("Cached point value = [%lf, %lf, %lf, %lf, %lf, %lf]", feedback.point[0], feedback.point[1],
                               feedback.point[2], feedback.point[3], feedback.point[4], feedback.point[5]);
                last_logged_point = current_point;
            }
            if (!driver_->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::NOOP, 0, 200)) {
                ELITE_LOG_ERROR("Failed to send NOOP command");
                return false;
            }
        }

        auto result = move_done_future.get();
        ELITE_LOG_INFO("Trajectory motion completed with result: %d", result);

        if (!driver_->writeIdle(0)) {
            ELITE_LOG_ERROR("Failed to write idle command");
            return false;
        }

        return result == TrajectoryMotionResult::SUCCESS;
    }

    TrajectoryMotionFeedback getLastFeedback() {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        return last_feedback_;
    }
};

int main(int argc, const char** argv) {
    constexpr float kJointSpeed = 0.5f;
    constexpr float kJointAcceleration = 0.8f;
    constexpr float kPointTime = 3.0f;
    constexpr float kCartesianSpeed = 0.15f;
    constexpr float kCartesianAcceleration = 0.3f;

    EliteDriverConfig config;
    // Parser param
    po::options_description desc(
        "Usage:\n"
        "\t./trajectory_example <--robot-ip=ip> [--local-ip=\"\"] [--use-headless-mode=true]\n"
        "Parameters:");
    desc.add_options()("help,h", "Print help message")("robot-ip", po::value<std::string>(&config.robot_ip)->required(),
                                                       "\tRequired. IP address of the robot.")(
        "use-headless-mode", po::value<bool>(&config.headless_mode)->required()->implicit_value(true),
        "\tRequired. Use headless mode.")("local-ip", po::value<std::string>(&config.local_ip)->default_value(""),
                                          "\tOptional. IP address of the local network interface.");

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help")) {
            std::cout << desc << std::endl;
            return 0;
        }

        po::notify(vm);
    } catch (const po::error& e) {
        std::cerr << "Argument error: " << e.what() << "\n\n";
        std::cerr << desc << "\n";
        return 1;
    }

    if (config.headless_mode) {
        ELITE_LOG_WARN("Use headless mode. Please ensure the robot is not in local mode.");
    } else {
        ELITE_LOG_WARN(
            "It needs to be correctly configured, and the External Control plugin should be inserted into the task tree.");
    }

    config.script_file_path = "external_control.script";
    std::unique_ptr<TrajectoryControl> trajectory_control = std::make_unique<TrajectoryControl>(config);
    std::unique_ptr<RtsiIOInterface> rtsi_client = std::make_unique<RtsiIOInterface>("output_recipe.txt", "input_recipe.txt", 250);

    ELITE_LOG_INFO("Connecting to the RTSI");
    if (!rtsi_client->connect(config.robot_ip)) {
        ELITE_LOG_FATAL("Fail to connect or config to the RTSI.");
        throw std::runtime_error("Fail to connect or config to the RTSI");
    }
    ELITE_LOG_INFO("Successfully connected to the RTSI");

    ELITE_LOG_INFO("Starting trajectory control...");
    if (!trajectory_control->startControl()) {
        ELITE_LOG_FATAL("Failed to start trajectory control.");
        return 1;
    }
    ELITE_LOG_INFO("Trajectory control started");

    vector6d_t actual_joints = rtsi_client->getActualJointPositions();
    actual_joints[3] = -1.57;

    ELITE_LOG_INFO("Moving joints to target: [%lf, %lf, %lf, %lf, %lf, %lf]",
                   actual_joints[0], actual_joints[1], actual_joints[2], actual_joints[3], actual_joints[4], actual_joints[5]);
    if (!trajectory_control->moveToJointTargetBySpeed(actual_joints, kJointSpeed, kJointAcceleration)) {
        ELITE_LOG_FATAL("Failed to move joints to target.");
        return 1;
    }
    ELITE_LOG_INFO("Joints moved to target");

    vector6d_t actual_pose = rtsi_client->getActualTCPPose();
    std::vector<vector6d_t> trajectory;

    actual_pose[2] -= 0.2;
    trajectory.push_back(actual_pose);

    actual_pose[1] -= 0.2;
    trajectory.push_back(actual_pose);

    actual_pose[1] += 0.2;
    actual_pose[2] += 0.2;
    trajectory.push_back(actual_pose);

    ELITE_LOG_INFO("Executing trajectory with time control, %zu points", trajectory.size());
    if (!trajectory_control->moveTrajectoryByTime(trajectory, kPointTime, 0.0f, true)) {
        ELITE_LOG_FATAL("Failed to move trajectory with time control.");
        return 1;
    }
    ELITE_LOG_INFO("Time-controlled trajectory completed");

    ELITE_LOG_INFO("Executing trajectory with speed control, %zu points", trajectory.size());
    if (!trajectory_control->moveTrajectoryBySpeed(trajectory, 0.0f, true, kCartesianSpeed, kCartesianAcceleration)) {
        ELITE_LOG_FATAL("Failed to move trajectory with speed control.");
        return 1;
    }
    ELITE_LOG_INFO("Speed-controlled trajectory completed");

    return 0;
}
