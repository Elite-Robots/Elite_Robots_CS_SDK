cmake_minimum_required(VERSION 3.16)

project(elite-cs-series-sdk VERSION 1.1.0)

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

find_package(GTest)
find_package(Boost)
if(NOT BOOST_FOUND)
    message(AUTHOR_WARNING "Not found boost library. Subsequent compilations may fail.")
endif()

configure_file(
    ${CMAKE_SOURCE_DIR}/include/EliteOptions.hpp.in
    ${PROJECT_BINARY_DIR}/include/Elite/EliteOptions.hpp
)

set(
    ELITE_LIB_SOURCE_FILE
    source/Common/Utils.cpp
    source/Common/TcpServer.cpp
    source/Common/EliteException.cpp
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

# Set output library default name
set(SDK_STATIC_LIB_OUTPUT_NAME "${PROJECT_NAME}")
set(SDK_SHARED_LIB_OUTPUT_NAME "${PROJECT_NAME}")

# Configure accordingly based on the operating system.
if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    message(STATUS "SYSTEM: Windows")
    add_definitions(
        -D_WIN32_WINNT=0x0601
        -DELITE_EXPORT_LIBRARY
    )
    set(SDK_STATIC_LIB_OUTPUT_NAME "${PROJECT_NAME}_static")
    set(SDK_SHARED_LIB_OUTPUT_NAME "${PROJECT_NAME}")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(STATUS "SYSTEM: Linux")
    set(SYSTEM_LIB pthread)
else()
    message(FATAL_ERROR "Unsupport operating system")
endif()


# Add library
add_library(
    ${PROJECT_NAME}_SHARED
    SHARED
    ${ELITE_LIB_SOURCE_FILE}
)
add_library(
    ${PROJECT_NAME}_STATIC
    STATIC
    ${ELITE_LIB_SOURCE_FILE}
)
define_rel_file_macro(${PROJECT_NAME}_SHARED)
define_rel_file_macro(${PROJECT_NAME}_STATIC)
set_target_properties(${PROJECT_NAME}_SHARED PROPERTIES OUTPUT_NAME ${SDK_SHARED_LIB_OUTPUT_NAME})
set_target_properties(${PROJECT_NAME}_STATIC PROPERTIES OUTPUT_NAME ${SDK_STATIC_LIB_OUTPUT_NAME})
target_include_directories(
    ${PROJECT_NAME}_SHARED
    PRIVATE
    ${ELITE_LIB_INCLUDE_DIR}
)
target_include_directories(
    ${PROJECT_NAME}_STATIC
    PRIVATE
    ${ELITE_LIB_INCLUDE_DIR}
)

# Copy head file
file(COPY ${PROJECT_SOURCE_DIR}/include/Elite/DataType.hpp DESTINATION ${PROJECT_BINARY_DIR}/include/Elite)
file(COPY ${PROJECT_SOURCE_DIR}/include/Elite/VersionInfo.hpp DESTINATION ${PROJECT_BINARY_DIR}/include/Elite)
file(COPY ${PROJECT_SOURCE_DIR}/include/Dashboard/DashboardClient.hpp DESTINATION ${PROJECT_BINARY_DIR}/include/Elite)
file(COPY ${PROJECT_SOURCE_DIR}/include/Rtsi/RtsiClientInterface.hpp DESTINATION ${PROJECT_BINARY_DIR}/include/Elite)
file(COPY ${PROJECT_SOURCE_DIR}/include/Rtsi/RtsiIOInterface.hpp DESTINATION ${PROJECT_BINARY_DIR}/include/Elite)
file(COPY ${PROJECT_SOURCE_DIR}/include/Rtsi/RtsiRecipe.hpp DESTINATION ${PROJECT_BINARY_DIR}/include/Elite)
file(COPY ${PROJECT_SOURCE_DIR}/include/Primary/PrimaryPackage.hpp DESTINATION ${PROJECT_BINARY_DIR}/include/Elite)
file(COPY ${PROJECT_SOURCE_DIR}/include/Primary/RobotConfPackage.hpp DESTINATION ${PROJECT_BINARY_DIR}/include/Elite)
file(COPY ${PROJECT_SOURCE_DIR}/include/Primary/PrimaryPortInterface.hpp DESTINATION ${PROJECT_BINARY_DIR}/include/Elite)
file(COPY ${PROJECT_SOURCE_DIR}/include/EliteException.hpp DESTINATION ${PROJECT_BINARY_DIR}/include/Elite)
file(COPY ${PROJECT_SOURCE_DIR}/include/Elite/EliteDriver.hpp DESTINATION ${PROJECT_BINARY_DIR}/include/Elite)
file(COPY ${PROJECT_SOURCE_DIR}/include/Elite/Log.hpp DESTINATION ${PROJECT_BINARY_DIR}/include/Elite)

# Is compile examples
if(ELITE_COMPLIE_EXAMPLES)
    message(STATUS "Compile the exmaples")
    add_subdirectory(example ${CMAKE_BINARY_DIR}/example/)
endif()

# If compile documentation
if(ELITE_COMPLIE_DOC)
    message(STATUS "Compile the documentation.")
    add_subdirectory(doc ${CMAKE_BINARY_DIR}/doc/)
endif()

# If googel test has been foune, compile unit test
if (GTEST_FOUND)
    message(STATUS "Compile the tests")
    add_subdirectory(test ${CMAKE_BINARY_DIR}/test/)
endif()

# On the Linux platform, install binary library files, header files, and CMake configuration files.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    include(GNUInstallDirs)
    install(
        TARGETS ${PROJECT_NAME}_SHARED ${PROJECT_NAME}_STATIC
        EXPORT ${PROJECT_NAME}Targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )
    install(
        DIRECTORY  ${PROJECT_BINARY_DIR}/include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )
    install(
        DIRECTORY ${PROJECT_SOURCE_DIR}/source/resources/
        DESTINATION share/Elite
    )

    include(CMakePackageConfigHelpers)
    configure_package_config_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
        INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
    )
    write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake 
        VERSION ${PROJECT_VERSION} 
        COMPATIBILITY SameMajorVersion
    )

    install(
        EXPORT ${PROJECT_NAME}Targets
        FILE ${PROJECT_NAME}Targets.cmake
        DESTINATION lib/cmake/${PROJECT_NAME}
    )
    install(
        FILES
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        DESTINATION lib/cmake/${PROJECT_NAME}
    )
endif()

message(STATUS "CMAKE_BUILD_TYPE:                   ${CMAKE_BUILD_TYPE}")
message(STATUS "CMAKE_C_COMPILER:                   ${CMAKE_C_COMPILER} (id ${CMAKE_C_COMPILER_ID}, clang ${CLANG}, GNUC ${GNUC}, version ${CMAKE_C_COMPILER_VERSION})")
message(STATUS "CMAKE_CXX_COMPILER:                 ${CMAKE_CXX_COMPILER} (id ${CMAKE_CXX_COMPILER_ID}, clang ${CLANG}, GNUC ${GNUC}, version ${CMAKE_CXX_COMPILER_VERSION})")
message(STATUS "CMAKE_C_FLAGS:                      ${CMAKE_C_FLAGS}")
message(STATUS "CMAKE_CXX_FLAGS:                    ${CMAKE_CXX_FLAGS}")
message(STATUS "CMAKE_INSTALL_INCLUDEDIR:           ${CMAKE_INSTALL_INCLUDEDIR}")
message(STATUS "CMAKE_INSTALL_LIBDIR:               ${CMAKE_INSTALL_LIBDIR}")
