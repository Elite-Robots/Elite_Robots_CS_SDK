cmake_minimum_required(VERSION 3.5)

include(GNUInstallDirs)

message(STATUS "Configuring kinematics plugins")

set(ELITE_KINEMATICS_PLUGIN_INSTALL_PATH "${CMAKE_INSTALL_LIBDIR}/elite-cs-series-sdk-plugins/kinematics")
message(STATUS "Kinematics plugins will be installed to: ${ELITE_KINEMATICS_PLUGIN_INSTALL_PATH}")

set(ELITE_KINEMATICS_PLUGIN_BUILD_PATH "${CMAKE_BINARY_DIR}/plugin/kinematics")
message(STATUS "Kinematics plugins will be build to: ${ELITE_KINEMATICS_PLUGIN_BUILD_PATH}")

file(GLOB KIN_PLUGIN_DIRS
    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    *
)

foreach(dir ${KIN_PLUGIN_DIRS})
    if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
        if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir}/CMakeLists.txt)
            message(STATUS "Found kinematics plugin: ${dir}")
            add_subdirectory(${dir})
        endif()
    endif()
endforeach()
