// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.

#include <Elite/ClassLoader.hpp>
#include <Elite/Log.hpp>
#include <Elite/PoseAlgebraBase.hpp>

#include <boost/program_options.hpp>
#include <iomanip>
#include <iostream>
#include <string>

using namespace ELITE;
namespace po = boost::program_options;

namespace {

void printVector6d(const std::string& name, const vector6d_t& value) {
    std::cout << name << " = [";
    for (size_t i = 0; i < value.size(); ++i) {
        std::cout << std::fixed << std::setprecision(6) << value[i];
        if (i + 1 < value.size()) {
            std::cout << ", ";
        }
    }
    std::cout << "]\n";
}

void printPoseMatrix(const std::string& name, const PoseMatrix& value) {
    std::cout << name << " =\n";
    for (size_t i = 0; i < 4; ++i) {
        std::cout << "  [";
        for (size_t j = 0; j < 4; ++j) {
            std::cout << std::fixed << std::setprecision(6) << value.data[i][j];
            if (j + 1 < 4) {
                std::cout << ", ";
            }
        }
        std::cout << "]\n";
    }
}

bool checkResult(const std::string& operation, bool ok, const PoseAlgebraResult& result) {
    if (!ok) {
        ELITE_LOG_ERROR("%s failed. error=%d, message=%s", operation.c_str(), static_cast<int>(result.error),
                        result.message.c_str());
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, const char** argv) {
#if defined(ELITE_COMPILE_POSE_ALG_PLUGIN)
    std::string plugin_lib_path;
    std::string plugin_class = "ELITE::EigenPoseAlgebra";

    po::options_description desc(
        "Usage:\n"
        "\t./pose_algebra_example <--plugin-lib=path> [--plugin-class=class_name]\n"
        "Parameters:");
    desc.add_options()("help,h", "Print help message")(
        "plugin-lib", po::value<std::string>(&plugin_lib_path)->required(),
        "\tRequired. Path to pose algebra plugin library.")(
        "plugin-class", po::value<std::string>(&plugin_class)->default_value("ELITE::EigenPoseAlgebra"),
        "\tOptional. Plugin class name.");

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

    ClassLoader loader(plugin_lib_path);

    if (!loader.loadLib()) {
        ELITE_LOG_FATAL("Failed to load pose algebra plugin library: %s", plugin_lib_path.c_str());
        return 1;
    }

    auto pose_algebra = loader.createUniqueInstance<PoseAlgebraBase>(plugin_class);
    if (pose_algebra == nullptr) {
        ELITE_LOG_FATAL("Failed to create PoseAlgebraBase instance: %s", plugin_class.c_str());
        return 1;
    }

    const vector6d_t base_pose{0.400000, -0.200000, 0.500000, 0.100000, 0.200000, -0.300000};
    const vector6d_t tool_offset{0.050000, 0.000000, 0.120000, 0.000000, 0.000000, 1.570796};

    PoseAlgebraResult result;

    PoseMatrix base_matrix;
    if (!checkResult("vectorToMatrix(base_pose)", pose_algebra->vectorToMatrix(base_pose, base_matrix, result), result)) {
        return 1;
    }

    PoseMatrix tool_matrix;
    if (!checkResult("vectorToMatrix(tool_offset)", pose_algebra->vectorToMatrix(tool_offset, tool_matrix, result), result)) {
        return 1;
    }

    PoseMatrix composed_matrix;
    if (!checkResult("multiply(base_matrix, tool_matrix)",
                     pose_algebra->multiply(base_matrix, tool_matrix, composed_matrix, result), result)) {
        return 1;
    }

    PoseMatrix tool_in_base_matrix;
    if (!checkResult("worldToLocal(base_matrix, composed_matrix)",
                     pose_algebra->worldToLocal(base_matrix, composed_matrix, tool_in_base_matrix, result), result)) {
        return 1;
    }

    PoseMatrix recovered_world_matrix;
    if (!checkResult("localToWorld(base_matrix, tool_in_base_matrix)",
                     pose_algebra->localToWorld(base_matrix, tool_in_base_matrix, recovered_world_matrix, result),
                     result)) {
        return 1;
    }

    PoseMatrix inverse_base_matrix;
    if (!checkResult("inverse(base_matrix)", pose_algebra->inverse(base_matrix, inverse_base_matrix, result), result)) {
        return 1;
    }

    PoseMatrix identity_check;
    if (!checkResult("multiply(base_matrix, inverse_base_matrix)",
                     pose_algebra->multiply(base_matrix, inverse_base_matrix, identity_check, result), result)) {
        return 1;
    }

    vector6d_t composed_pose;
    if (!checkResult("matrixToVector(composed_matrix)",
                     pose_algebra->matrixToVector(composed_matrix, composed_pose, result), result)) {
        return 1;
    }

    vector6d_t tool_in_base_pose;
    if (!checkResult("worldToLocal(base_pose, composed_pose)",
                     pose_algebra->worldToLocal(base_pose, composed_pose, tool_in_base_pose, result), result)) {
        return 1;
    }

    vector6d_t recovered_world_pose;
    if (!checkResult("localToWorld(base_pose, tool_in_base_pose)",
                     pose_algebra->localToWorld(base_pose, tool_in_base_pose, recovered_world_pose, result), result)) {
        return 1;
    }

    vector6d_t added_pose;
    if (!checkResult("add(base_pose, tool_offset)", pose_algebra->add(base_pose, tool_offset, added_pose, result), result)) {
        return 1;
    }

    vector6d_t recovered_pose;
    if (!checkResult("subtract(added_pose, tool_offset)",
                     pose_algebra->subtract(added_pose, tool_offset, recovered_pose, result), result)) {
        return 1;
    }

    PoseDistance dist_pose;
    if (!checkResult("distance(base_pose, composed_pose)",
                     pose_algebra->distance(base_pose, composed_pose, dist_pose, result), result)) {
        return 1;
    }

    std::cout << "=== Pose Algebra Example (Eigen Plugin) ===\n";
    printVector6d("base_pose", base_pose);
    printVector6d("tool_offset", tool_offset);
    printPoseMatrix("base_matrix", base_matrix);
    printPoseMatrix("composed_matrix", composed_matrix);
    printPoseMatrix("tool_in_base_matrix", tool_in_base_matrix);
    printPoseMatrix("recovered_world_matrix", recovered_world_matrix);
    printPoseMatrix("identity_check", identity_check);
    printVector6d("composed_pose", composed_pose);
    printVector6d("tool_in_base_pose", tool_in_base_pose);
    printVector6d("recovered_world_pose", recovered_world_pose);
    printVector6d("added_pose", added_pose);
    printVector6d("recovered_pose", recovered_pose);
    std::cout << "distance.linear_distance  = " << std::fixed << std::setprecision(6) << dist_pose.linear_distance
              << "\n";
    std::cout << "distance.angular_distance = " << std::fixed << std::setprecision(6) << dist_pose.angular_distance
              << "\n";

    return 0;
#else
    std::cout << "pose_algebra_example: ELITE_COMPILE_POSE_ALG_PLUGIN is OFF, plugin demo is skipped.\n";
    std::cout << "Reconfigure with: cmake -DELITE_COMPILE_POSE_ALG_PLUGIN=TRUE ..\n";
    return 0;
#endif
}
