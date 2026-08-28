cmake_minimum_required(VERSION 3.5)

project(elite_eigen_pose_algebra_plugin LANGUAGES CXX)

find_package(Eigen3 REQUIRED)

set(PLUGIN_TARGET elite_eigen_pose_algebra)

add_library(
    ${PLUGIN_TARGET}
    SHARED
    source/EigenPoseAlgebra.cpp
)

target_include_directories(
    ${PLUGIN_TARGET}
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Ubuntu 16.04 ships an Eigen 3.3 beta package whose CMake config only
# provides EIGEN3_INCLUDE_DIRS and does not define the Eigen3::Eigen target.
if(NOT TARGET Eigen3::Eigen)
    target_include_directories(${PLUGIN_TARGET} PRIVATE ${EIGEN3_INCLUDE_DIRS})
endif()

target_link_libraries(
    ${PLUGIN_TARGET}
    PRIVATE
    elite_cs_series_sdk::shared
)

if(TARGET Eigen3::Eigen)
    target_link_libraries(${PLUGIN_TARGET} PRIVATE Eigen3::Eigen)
endif()

set_target_properties(
    ${PLUGIN_TARGET}
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${ELITE_POSE_ALGEBRA_PLUGIN_BUILD_PATH}
)

if(ELITE_INSTALL)
    install(
        TARGETS ${PLUGIN_TARGET}
        LIBRARY DESTINATION ${ELITE_POSE_ALGEBRA_PLUGIN_INSTALL_PATH}
    )
endif()
