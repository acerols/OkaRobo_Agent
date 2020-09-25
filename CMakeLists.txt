cmake_minimum_required(VERSION 3.5)
project(okarobo_agent)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mpu9250 REQUIRED)
find_package(agent_msgs REQUIRED)
find_package(okarobo_msgs REQUIRED)

add_executable(robo_agent
  src/agent_node.cpp
  src/agent.cpp
)

ament_target_dependencies(robo_agent
  rclcpp
  mpu9250
  agent_msgs
  okarobo_msgs
)

target_include_directories(robo_agent
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  robo_agent
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
