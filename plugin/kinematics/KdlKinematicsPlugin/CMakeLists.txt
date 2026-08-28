cmake_minimum_required(VERSION 3.5)

project(elite_kdl_kinematics_plugin LANGUAGES CXX)

find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

set(PLUGIN_TARGET elite_kdl_kinematics)

add_library(
    ${PLUGIN_TARGET} 
    SHARED
    source/KdlKinematicsPlugin.cpp
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
    orocos-kdl
)

if(TARGET Eigen3::Eigen)
    target_link_libraries(${PLUGIN_TARGET} PRIVATE Eigen3::Eigen)
endif()

set_target_properties(
    ${PLUGIN_TARGET} 
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${ELITE_KINEMATICS_PLUGIN_BUILD_PATH}
)

if(ELITE_INSTALL)
    install(
        TARGETS ${PLUGIN_TARGET}
        LIBRARY DESTINATION ${ELITE_KINEMATICS_PLUGIN_INSTALL_PATH}
    )
endif()
