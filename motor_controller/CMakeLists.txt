cmake_minimum_required(VERSION 3.8)
project(motor_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

add_executable(motor_controller_node src/motor_controller_node.cpp)
target_include_directories(motor_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(motor_controller_node rclcpp geometry_msgs std_msgs diagnostic_updater)

install(TARGETS motor_controller_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(test_kart_command test/test_kart_command.cpp)
  target_include_directories(test_kart_command PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  add_test(NAME test_kart_command COMMAND test_kart_command)
endif()

ament_package()
