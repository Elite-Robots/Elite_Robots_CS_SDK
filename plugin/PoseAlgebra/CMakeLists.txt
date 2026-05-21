cmake_minimum_required(VERSION 3.16)

include(GNUInstallDirs)

message(STATUS "Configuring pose algebra plugins")

set(ELITE_POSE_ALGEBRA_PLUGIN_INSTALL_PATH "${CMAKE_INSTALL_LIBDIR}/elite-cs-series-sdk-plugins/pose-algebra")
message(STATUS "Pose algebra plugins will be installed to: ${ELITE_POSE_ALGEBRA_PLUGIN_INSTALL_PATH}")

set(ELITE_POSE_ALGEBRA_PLUGIN_BUILD_PATH "${CMAKE_BINARY_DIR}/plugin/pose_algebra")
message(STATUS "Pose algebra plugins will be build to: ${ELITE_POSE_ALGEBRA_PLUGIN_BUILD_PATH}")

file(GLOB POSE_ALG_PLUGIN_DIRS
    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    *
)

foreach(dir ${POSE_ALG_PLUGIN_DIRS})
    if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
        if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir}/CMakeLists.txt)
            message(STATUS "Found pose algebra plugin: ${dir}")
            add_subdirectory(${dir})
        endif()
    endif()
endforeach()
