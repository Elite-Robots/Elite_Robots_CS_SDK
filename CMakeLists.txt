cmake_minimum_required(VERSION 3.16)

project(elite-cs-series-sdk VERSION 1.3.0)

option(ELITE_COMPILE_TESTS "Compile tests" OFF)
option(ELITE_COMPILE_DOC "Compile documentation" OFF)
option(ELITE_COMPILE_EXAMPLES "Compile examples" OFF)

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


find_package(Boost REQUIRED)
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
    source/Primary/PrimaryPort.cpp
    source/Primary/PrimaryPortInterface.cpp
    source/Primary/RobotConfPackage.cpp
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
)

set(SDK_STATIC_LIB_OUTPUT_NAME "${PROJECT_NAME}")
set(SDK_SHARED_LIB_OUTPUT_NAME "${PROJECT_NAME}")

if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    message(STATUS "SYSTEM: Windows")
    add_definitions(-D_WIN32_WINNT=0x0601)
    set(SDK_STATIC_LIB_OUTPUT_NAME "${PROJECT_NAME}_static")
    set(SDK_SHARED_LIB_OUTPUT_NAME "${PROJECT_NAME}")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" OR APPLE)
    if(APPLE)
        message(STATUS "SYSTEM: macOS")
    else()
        message(STATUS "SYSTEM: Linux")
    endif()
    set(SYSTEM_LIB pthread)
else()
    message(FATAL_ERROR "Unsupport operating system")
endif()

add_library(${PROJECT_NAME}_SHARED SHARED ${ELITE_LIB_SOURCE_FILE})
add_library(${PROJECT_NAME}_STATIC STATIC ${ELITE_LIB_SOURCE_FILE})

if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    target_compile_definitions(${PROJECT_NAME}_SHARED PRIVATE ELITE_EXPORT_LIBRARY)
    target_compile_definitions(${PROJECT_NAME}_STATIC PRIVATE ELITE_EXPORT_LIBRARY)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" OR APPLE)
    target_compile_definitions(${PROJECT_NAME}_SHARED PRIVATE ELITE_EXPORT_LIBRARY)
    target_compile_definitions(${PROJECT_NAME}_STATIC PRIVATE ELITE_EXPORT_LIBRARY)
endif()

define_rel_file_macro(${PROJECT_NAME}_SHARED)
define_rel_file_macro(${PROJECT_NAME}_STATIC)
set_target_properties(${PROJECT_NAME}_SHARED PROPERTIES OUTPUT_NAME ${SDK_SHARED_LIB_OUTPUT_NAME})
set_target_properties(${PROJECT_NAME}_STATIC PROPERTIES OUTPUT_NAME ${SDK_STATIC_LIB_OUTPUT_NAME})
target_include_directories(${PROJECT_NAME}_SHARED PRIVATE ${ELITE_LIB_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME}_STATIC PRIVATE ${ELITE_LIB_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}_SHARED PRIVATE ${THIRDPARTY_LIB})
target_link_libraries(${PROJECT_NAME}_STATIC PRIVATE ${THIRDPARTY_LIB})

foreach(header ${ELITE_HEADER_FILES})
    file(COPY ${PROJECT_SOURCE_DIR}/include/${header} DESTINATION ${PROJECT_BINARY_DIR}/include/Elite)
endforeach()

add_library(${PROJECT_NAME}::shared ALIAS ${PROJECT_NAME}_SHARED)
target_include_directories(${PROJECT_NAME}_SHARED PUBLIC $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include> $<INSTALL_INTERFACE:include>)
add_library(${PROJECT_NAME}::static ALIAS ${PROJECT_NAME}_STATIC)
target_include_directories(${PROJECT_NAME}_STATIC PUBLIC $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include> $<INSTALL_INTERFACE:include>)

if(ELITE_COMPILE_EXAMPLES)
    message(STATUS "Compile the exmaples")
    add_subdirectory(example ${CMAKE_BINARY_DIR}/example/)
endif()
if(ELITE_COMPILE_DOC)
    message(STATUS "Compile the documentation.")
    add_subdirectory(doc ${CMAKE_BINARY_DIR}/doc/)
endif()
if (ELITE_COMPILE_TESTS)
    message(STATUS "Compile the tests")
    add_subdirectory(test ${CMAKE_BINARY_DIR}/test/)
endif()

string(COMPARE EQUAL "${CMAKE_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}" is_top_level)
option(ELITE_INSTALL "Include packaging rules" "${is_top_level}")

if((CMAKE_SYSTEM_NAME STREQUAL "Linux" OR APPLE) AND ELITE_INSTALL)
    include(GNUInstallDirs)
    install(TARGETS ${PROJECT_NAME}_SHARED ${PROJECT_NAME}_STATIC EXPORT ${PROJECT_NAME}Targets LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
    install(DIRECTORY ${PROJECT_BINARY_DIR}/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/source/resources/ DESTINATION share/Elite)
    include(CMakePackageConfigHelpers)
    configure_package_config_file("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" INSTALL_DESTINATION lib/cmake/${PROJECT_NAME})
    write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
    install(EXPORT ${PROJECT_NAME}Targets FILE ${PROJECT_NAME}Targets.cmake DESTINATION lib/cmake/${PROJECT_NAME})
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake" DESTINATION lib/cmake/${PROJECT_NAME})
endif()

message(STATUS "CMAKE_BUILD_TYPE:                   ${CMAKE_BUILD_TYPE}")
message(STATUS "CMAKE_C_COMPILER:                   ${CMAKE_C_COMPILER} (id ${CMAKE_C_COMPILER_ID}, clang ${CLANG}, GNUC ${GNUC}, version ${CMAKE_C_COMPILER_VERSION})")
message(STATUS "CMAKE_CXX_COMPILER:                 ${CMAKE_CXX_COMPILER} (id ${CMAKE_CXX_COMPILER_ID}, clang ${CLANG}, GNUC ${GNUC}, version ${CMAKE_CXX_COMPILER_VERSION})")
message(STATUS "CMAKE_C_FLAGS:                      ${CMAKE_C_FLAGS}")
message(STATUS "CMAKE_CXX_FLAGS:                    ${CMAKE_CXX_FLAGS}")
message(STATUS "CMAKE_INSTALL_INCLUDEDIR:           ${CMAKE_INSTALL_INCLUDEDIR}")
message(STATUS "CMAKE_INSTALL_LIBDIR:               ${CMAKE_INSTALL_LIBDIR}")
message(STATUS "THIRDPARTY_LIB                      ${THIRDPARTY_LIB}")
