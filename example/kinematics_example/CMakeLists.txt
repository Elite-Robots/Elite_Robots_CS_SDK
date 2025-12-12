cmake_minimum_required(VERSION 3.14)
project(elite_kinematics_example CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(EXAMPLE_OUTPUT_DIR ${CMAKE_BINARY_DIR}/example)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${EXAMPLE_OUTPUT_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${EXAMPLE_OUTPUT_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${EXAMPLE_OUTPUT_DIR})

# 依赖 KDL
find_package(orocos_kdl REQUIRED)

add_library(elite_kdl_kinematics_example SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/src/kdl_kinematics_example.cpp
)

# 头文件搜索路径
target_include_directories(elite_kdl_kinematics_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/Kinematics>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}

)

# 依赖库
target_link_libraries(elite_kdl_kinematics_example
  orocos-kdl
)
if(UNIX AND NOT APPLE)
  target_link_libraries(elite_kdl_kinematics_example dl)
endif()

# 示例可执行文件
add_executable(kinematics_example
  ${CMAKE_CURRENT_SOURCE_DIR}/example/example_Kdl_kinematics.cpp
)
target_link_libraries(kinematics_example PRIVATE
  elite_kdl_kinematics_example
  elite-cs-series-sdk::static
  ${SYSTEM_LIB}
)
if(UNIX AND NOT APPLE)
  target_link_libraries(kinematics_example PRIVATE dl)
endif()
target_include_directories(kinematics_example PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/include/Kinematics
)
set_target_properties(kinematics_example PROPERTIES
  BUILD_RPATH "$ORIGIN"
  RUNTIME_OUTPUT_DIRECTORY ${EXAMPLE_OUTPUT_DIR}
)

install(TARGETS elite_kdl_kinematics_example
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)
