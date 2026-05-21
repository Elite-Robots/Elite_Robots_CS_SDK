#include <Elite/ClassLoader.hpp>
#include <Elite/PoseAlgebraBase.hpp>
#include <Elite/Log.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstdlib>
#include <fstream>
#include <string>
#include <iostream>

using namespace ELITE;

namespace {

bool fileExists(const std::string& path) {
    return static_cast<bool>(std::ifstream(path));
}

std::string findPluginLibraryPath(const std::string& lib_name) {
    const std::array<std::string, 5> candidates = {
        "../build/plugin/pose_algebra/" + lib_name,
        "build/plugin/pose_algebra/" + lib_name,
        "plugin/pose_algebra/" + lib_name,
        "../plugin/pose_algebra/" + lib_name,
        "./" + lib_name,
    };

    std::cout << "Looking for library: " << lib_name << std::endl;
    for (const auto& candidate : candidates) {
        std::cout << "  Checking: " << candidate << std::endl;
        if (fileExists(candidate)) {
            std::cout << "  Found at: " << candidate << std::endl;
            return candidate;
        }
    }
    std::cout << "  Not found!" << std::endl;
    return "";
}

}  // namespace

TEST(PoseAlgebraTest, EigenPluginWorks) {
    const std::string plugin_path = findPluginLibraryPath("libelite_eigen_pose_algebra.so");
    if (plugin_path.empty()) {
        GTEST_SKIP() << "Eigen pose algebra plugin library not found in expected build paths";
    }

    ClassLoader loader(plugin_path);
    ASSERT_TRUE(loader.loadLib()) << "Failed to load plugin: " << plugin_path;

    std::cout << "Typeid(PoseAlgebraBase).name() = " << typeid(PoseAlgebraBase).name() << std::endl;

    auto algebra = loader.createUniqueInstance<PoseAlgebraBase>("ELITE::EigenPoseAlgebra");
    ASSERT_NE(algebra, nullptr) << "Failed to create ELITE::EigenPoseAlgebra instance";
}

TEST(PoseAlgebraTest, ElitePluginWorks) {
    const std::string plugin_path = findPluginLibraryPath("libelite_pose_algebra.so");
    if (plugin_path.empty()) {
        GTEST_SKIP() << "Elite pose algebra plugin library not found in expected build paths";
    }

    ClassLoader loader(plugin_path);
    ASSERT_TRUE(loader.loadLib()) << "Failed to load plugin: " << plugin_path;

    std::cout << "Typeid(PoseAlgebraBase).name() = " << typeid(PoseAlgebraBase).name() << std::endl;

    auto algebra = loader.createUniqueInstance<PoseAlgebraBase>("ELITE::ElitePoseAlgebra");
    ASSERT_NE(algebra, nullptr) << "Failed to create ELITE::ElitePoseAlgebra instance";
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
