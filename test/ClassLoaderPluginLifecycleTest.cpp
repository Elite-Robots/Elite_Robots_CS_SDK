#include <Elite/ClassLoader.hpp>
#include <Elite/KinematicsBase.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstdlib>
#include <fstream>
#include <string>

using namespace ELITE;

namespace {

bool fileExists(const std::string& path) {
    return static_cast<bool>(std::ifstream(path));
}

std::string findPluginLibraryPath() {
    const std::array<std::string, 3> candidates = {
        "../plugin/kinematics/libelite_kdl_kinematics.so",
        "plugin/kinematics/libelite_kdl_kinematics.so",
        "./libelite_kdl_kinematics.so",
    };

    for (const auto& candidate : candidates) {
        if (fileExists(candidate)) {
            return candidate;
        }
    }
    return "";
}

}  // namespace

TEST(ClassLoaderPluginLifecycleTest, PluginCanBeCreated) {
    const std::string plugin_path = findPluginLibraryPath();
    if (plugin_path.empty()) {
        GTEST_SKIP() << "Kinematics plugin library not found in expected build paths";
    }

    ClassLoader loader(plugin_path);
    ASSERT_TRUE(loader.loadLib()) << "Failed to load plugin: " << plugin_path;

    auto plugin = loader.createUniqueInstance<KinematicsBase>("ELITE::KdlKinematicsPlugin");
    ASSERT_NE(plugin, nullptr) << "Failed to create ELITE::KdlKinematicsPlugin instance";
}

TEST(ClassLoaderPluginLifecycleTest, ProcessExitDoesNotCrashAfterPluginUse) {
    const std::string plugin_path = findPluginLibraryPath();
    if (plugin_path.empty()) {
        GTEST_SKIP() << "Kinematics plugin library not found in expected build paths";
    }

    EXPECT_EXIT(
        {
            ClassLoader loader(plugin_path);
            if (!loader.loadLib()) {
                std::_Exit(2);
            }
            auto plugin = loader.createUniqueInstance<KinematicsBase>("ELITE::KdlKinematicsPlugin");
            if (!plugin) {
                std::_Exit(3);
            }
            plugin.reset();
            std::exit(0);
        },
        ::testing::ExitedWithCode(0),
        "");
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
