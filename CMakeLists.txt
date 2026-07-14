cmake_minimum_required(VERSION 3.5)

project(elite_cs_series_sdk VERSION 1.4.0)

option(ELITE_COMPILE_TESTS "Compile tests" OFF)
option(ELITE_COMPILE_DOC "Compile documentation" OFF)
option(ELITE_COMPILE_EXAMPLES "Compile examples" OFF)
option(ELITE_COMPILE_KIN_PLUGIN "Compile kinematics plugin" OFF)
option(ELITE_COMPILE_POSE_ALG_PLUGIN "Compile pose algebra plugin" OFF)
option(ELITE_LINK_BOOST_SYSTEM "Link Boost.System for Boost.Asio versions that are not header-only" OFF)

include(cmake/utils.cmake)

if(NOT DEFINED CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_STANDARD EQUAL 17 OR CMAKE_CXX_STANDARD GREATER 17)
    message(STATUS "Selected C++ standard: C++17")
    set (ELITE_SDK_COMPILE_STANDARD 17)
elseif(CMAKE_CXX_STANDARD EQUAL 14)
    message(STATUS "Selected C++ standard: C++14")
    set(ELITE_SDK_COMPILE_STANDARD 14)
else()
    message(FATAL_ERROR  "C++ standard must C++14 or higher")
endif()


if(ELITE_LINK_BOOST_SYSTEM)
    find_package(Boost REQUIRED COMPONENTS system)
    set(ELITE_BOOST_LIBRARIES Boost::boost Boost::system)
    set(ELITE_BOOST_FIND_COMPONENTS system)
else()
    find_package(Boost REQUIRED)
    set(ELITE_BOOST_LIBRARIES Boost::boost)
    set(ELITE_BOOST_FIND_COMPONENTS)
endif()
find_package(libssh)
if(libssh_FOUND)
    set(THIRDPARTY_LIB ${THIRDPARTY_LIB} ssh)
    add_definitions(-DELITE_USE_LIB_SSH)
endif()

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/EliteOptions.hpp.in
    ${PROJECT_BINARY_DIR}/include/Elite/EliteOptions.hpp
)

# Source file list
set(
    ELITE_LIB_SOURCE_FILE
    source/Common/StringUtils.cpp
    source/Common/TcpServer.cpp
    source/Common/EliteException.cpp
    source/Common/SshUtils.cpp
    source/Common/RtUtils.cpp
    source/Common/SharedLibrary.cpp
    source/Primary/PrimaryPort.cpp
    source/Primary/PrimaryPortInterface.cpp
    source/Primary/RobotConfPackage.cpp
    source/Primary/PrimaryStatePackage.cpp
    source/Rtsi/RtsiClient.cpp
    source/Rtsi/RtsiClientInterface.cpp
    source/Rtsi/RtsiRecipeInternal.cpp
    source/Rtsi/RtsiIOInterface.cpp
    source/Dashboard/DashboardClient.cpp
    source/Control/ReverseInterface.cpp
    source/Control/TrajectoryInterface.cpp
    source/Control/ScriptSender.cpp
    source/Control/ScriptCommandInterface.cpp
    source/Elite/VersionInfo.cpp
    source/Elite/EliteDriver.cpp
    source/Elite/Log.cpp
    source/Elite/Logger.cpp
    source/Elite/RemoteUpgrade.cpp
    source/Elite/ControllerLog.cpp
    source/Elite/SerialCommunicationImpl.cpp
    source/ClassLoader/ClassLoader.cpp
    source/ClassLoader/ClassRegistry.cpp
)

set(
    ELITE_LIB_INCLUDE_DIR
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include/Common
    ${PROJECT_SOURCE_DIR}/include/Primary
    ${PROJECT_SOURCE_DIR}/include/Rtsi
    ${PROJECT_SOURCE_DIR}/include/Dashboard
    ${PROJECT_SOURCE_DIR}/include/Elite
    ${PROJECT_SOURCE_DIR}/include/Control
    ${PROJECT_SOURCE_DIR}/include/ClassLoader
    ${PROJECT_SOURCE_DIR}/include/
    ${PROJECT_BINARY_DIR}/include/
)

set(
    ELITE_HEADER_FILES
    Elite/DataType.hpp
    Elite/VersionInfo.hpp
    Dashboard/DashboardClient.hpp
    Rtsi/RtsiClientInterface.hpp
    Rtsi/RtsiIOInterface.hpp
    Rtsi/RtsiRecipe.hpp
    Primary/PrimaryPackage.hpp
    Primary/RobotConfPackage.hpp
    Primary/PrimaryStatePackage.hpp
    Primary/PrimaryPortInterface.hpp
    EliteException.hpp
    Elite/EliteDriver.hpp
    Elite/Log.hpp
    Elite/RemoteUpgrade.hpp
    Elite/ControllerLog.hpp
    Elite/RobotException.hpp
    Elite/SerialCommunication.hpp
    Common/RtUtils.hpp
    Common/SshUtils.hpp
    Common/Utils.hpp
    Common/EndianUtils.hpp
    Common/StringUtils.hpp
    Common/SharedLibrary.hpp
    Common/BoostAsioCompat.hpp
    KinematicsBase/KinematicsBase.hpp
    PoseAlgebraBase/PoseAlgebraBase.hpp
    ClassLoader/ClassRegistry.hpp
    ClassLoader/ClassLoader.hpp
    ClassLoader/ClassRegisterMacro.hpp
)

set(SDK_STATIC_LIB_OUTPUT_NAME "${PROJECT_NAME}")
set(SDK_SHARED_LIB_OUTPUT_NAME "${PROJECT_NAME}")

if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    message(STATUS "SYSTEM: Windows")
    add_definitions(-D_WIN32_WINNT=0x0601)
    set(SDK_STATIC_LIB_OUTPUT_NAME "${PROJECT_NAME}_static")
    set(SDK_SHARED_LIB_OUTPUT_NAME "${PROJECT_NAME}")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(STATUS "SYSTEM: Linux")
    set(SYSTEM_LIB pthread)
    # Keep the historical library names (with '-') stable across builds
    set(SDK_STATIC_LIB_OUTPUT_NAME "elite-cs-series-sdk")
    set(SDK_SHARED_LIB_OUTPUT_NAME "elite-cs-series-sdk")
else()
    message(FATAL_ERROR "Unsupport operating system")
endif()

add_library(${PROJECT_NAME}_SHARED SHARED ${ELITE_LIB_SOURCE_FILE})
add_library(${PROJECT_NAME}_STATIC STATIC ${ELITE_LIB_SOURCE_FILE})

if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    target_compile_definitions(${PROJECT_NAME}_SHARED PRIVATE ELITE_EXPORT_LIBRARY)
    target_compile_definitions(${PROJECT_NAME}_STATIC PUBLIC ELITE_STATIC_LIBRARY)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_compile_definitions(${PROJECT_NAME}_SHARED PUBLIC ELITE_EXPORT_LIBRARY)
    target_compile_definitions(${PROJECT_NAME}_STATIC PUBLIC ELITE_STATIC_LIBRARY)
endif()

define_rel_file_macro(${PROJECT_NAME}_SHARED)
define_rel_file_macro(${PROJECT_NAME}_STATIC)
set_target_properties(
    ${PROJECT_NAME}_SHARED
    PROPERTIES
        OUTPUT_NAME ${SDK_SHARED_LIB_OUTPUT_NAME}
        EXPORT_NAME shared
)
set_target_properties(
    ${PROJECT_NAME}_STATIC
    PROPERTIES
        OUTPUT_NAME ${SDK_STATIC_LIB_OUTPUT_NAME}
        EXPORT_NAME static
)
target_include_directories(${PROJECT_NAME}_SHARED PRIVATE ${ELITE_LIB_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME}_STATIC PRIVATE ${ELITE_LIB_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}_SHARED PRIVATE ${ELITE_BOOST_LIBRARIES} ${THIRDPARTY_LIB})
target_link_libraries(${PROJECT_NAME}_STATIC PRIVATE ${ELITE_BOOST_LIBRARIES} ${THIRDPARTY_LIB})

foreach(header ${ELITE_HEADER_FILES})
    get_filename_component(header_name ${header} NAME)
    configure_file(
        ${PROJECT_SOURCE_DIR}/include/${header}
        ${PROJECT_BINARY_DIR}/include/Elite/${header_name}
        COPYONLY
    )
endforeach()

add_library(${PROJECT_NAME}::shared ALIAS ${PROJECT_NAME}_SHARED)
add_library(elite-cs-series-sdk::shared ALIAS ${PROJECT_NAME}_SHARED)
target_include_directories(${PROJECT_NAME}_SHARED PUBLIC $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include> $<INSTALL_INTERFACE:include>)
add_library(${PROJECT_NAME}::static ALIAS ${PROJECT_NAME}_STATIC)
add_library(elite-cs-series-sdk::static ALIAS ${PROJECT_NAME}_STATIC)
target_include_directories(${PROJECT_NAME}_STATIC PUBLIC $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include> $<INSTALL_INTERFACE:include>)

# Compile examples, tests, documentation, and plugins based on options
if(ELITE_COMPILE_EXAMPLES)
    message(STATUS "Compile the exmaples")
    add_subdirectory(example ${CMAKE_BINARY_DIR}/example/)
endif()
# Compile documentation
if(ELITE_COMPILE_DOC)
    message(STATUS "Compile the documentation.")
    add_subdirectory(doc ${CMAKE_BINARY_DIR}/doc/)
endif()
# Compile tests
if (ELITE_COMPILE_TESTS)
    message(STATUS "Compile the tests")
    add_subdirectory(test ${CMAKE_BINARY_DIR}/test/)
endif()
# Compile kinematics plugin
if(ELITE_COMPILE_KIN_PLUGIN)
    message(STATUS "Compile the kinematics plugin")
    add_subdirectory(plugin/kinematics ${CMAKE_BINARY_DIR}/kin_plugin/)
endif()

# Compile pose algebra plugin
if(ELITE_COMPILE_POSE_ALG_PLUGIN)
    message(STATUS "Compile the pose algebra plugin")
    add_subdirectory(plugin/PoseAlgebra ${CMAKE_BINARY_DIR}/pose_alg_plugin/)
endif()

string(COMPARE EQUAL "${CMAKE_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}" is_top_level)
option(ELITE_INSTALL "Include packaging rules" "${is_top_level}")

if((CMAKE_SYSTEM_NAME STREQUAL "Linux") AND ELITE_INSTALL)
    include(GNUInstallDirs)
    install(
        TARGETS ${PROJECT_NAME}_SHARED ${PROJECT_NAME}_STATIC
        EXPORT ${PROJECT_NAME}Targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
    install(DIRECTORY ${PROJECT_BINARY_DIR}/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/source/resources/ DESTINATION share/Elite)
    include(CMakePackageConfigHelpers)

    # Legacy CMake package name (historical project / library name)
    set(ELITE_LEGACY_CMAKE_PACKAGE_NAME elite-cs-series-sdk)
    configure_package_config_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/${ELITE_LEGACY_CMAKE_PACKAGE_NAME}Config.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/${ELITE_LEGACY_CMAKE_PACKAGE_NAME}Config.cmake"
        INSTALL_DESTINATION lib/cmake/${ELITE_LEGACY_CMAKE_PACKAGE_NAME}
    )
    write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/${ELITE_LEGACY_CMAKE_PACKAGE_NAME}ConfigVersion.cmake"
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion
    )
    install(
        EXPORT ${PROJECT_NAME}Targets
        FILE ${ELITE_LEGACY_CMAKE_PACKAGE_NAME}Targets.cmake
        NAMESPACE ${ELITE_LEGACY_CMAKE_PACKAGE_NAME}::
        DESTINATION lib/cmake/${ELITE_LEGACY_CMAKE_PACKAGE_NAME}
    )
    install(
        FILES
            "${CMAKE_CURRENT_BINARY_DIR}/${ELITE_LEGACY_CMAKE_PACKAGE_NAME}Config.cmake"
            "${CMAKE_CURRENT_BINARY_DIR}/${ELITE_LEGACY_CMAKE_PACKAGE_NAME}ConfigVersion.cmake"
        DESTINATION lib/cmake/${ELITE_LEGACY_CMAKE_PACKAGE_NAME}
    )

    # ROS / ament metadata
    install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" DESTINATION share/${PROJECT_NAME})
endif()

message(STATUS "CMAKE_BUILD_TYPE:                   ${CMAKE_BUILD_TYPE}")
message(STATUS "CMAKE_C_COMPILER:                   ${CMAKE_C_COMPILER} (id ${CMAKE_C_COMPILER_ID}, clang ${CLANG}, GNUC ${GNUC}, version ${CMAKE_C_COMPILER_VERSION})")
message(STATUS "CMAKE_CXX_COMPILER:                 ${CMAKE_CXX_COMPILER} (id ${CMAKE_CXX_COMPILER_ID}, clang ${CLANG}, GNUC ${GNUC}, version ${CMAKE_CXX_COMPILER_VERSION})")
message(STATUS "CMAKE_C_FLAGS:                      ${CMAKE_C_FLAGS}")
message(STATUS "CMAKE_CXX_FLAGS:                    ${CMAKE_CXX_FLAGS}")
message(STATUS "CMAKE_INSTALL_INCLUDEDIR:           ${CMAKE_INSTALL_INCLUDEDIR}")
message(STATUS "CMAKE_INSTALL_LIBDIR:               ${CMAKE_INSTALL_LIBDIR}")
message(STATUS "THIRDPARTY_LIB                      ${THIRDPARTY_LIB}")
message(STATUS "ELITE_LINK_BOOST_SYSTEM             ${ELITE_LINK_BOOST_SYSTEM}")
message(STATUS "ELITE_COMPILE_TESTS                 ${ELITE_COMPILE_TESTS}")
message(STATUS "ELITE_COMPILE_DOC                   ${ELITE_COMPILE_DOC}")
message(STATUS "ELITE_COMPILE_EXAMPLES              ${ELITE_COMPILE_EXAMPLES}")
message(STATUS "ELITE_COMPILE_KIN_PLUGIN            ${ELITE_COMPILE_KIN_PLUGIN}")
message(STATUS "ELITE_COMPILE_POSE_ALG_PLUGIN       ${ELITE_COMPILE_POSE_ALG_PLUGIN}")

find_package(ament_cmake QUIET)
option(ELITE_ROS2_BUILD "Build as a ROS 2 (ament_cmake) package" ${ament_cmake_FOUND})
if(ELITE_ROS2_BUILD)
    find_package(ament_cmake REQUIRED)
    ament_package()
endif()
