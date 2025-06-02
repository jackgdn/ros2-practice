cmake_minimum_required(VERSION 3.5)
project(cpp_custom)

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
find_package(custom_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

add_executable(coordinate_publisher src/publisher.cpp)
ament_target_dependencies(coordinate_publisher rclcpp custom_interfaces)
add_executable(coordinate_subscriber src/subscriber.cpp)
ament_target_dependencies(coordinate_subscriber rclcpp custom_interfaces)
add_executable(transformation_server src/server.cpp)
ament_target_dependencies(transformation_server rclcpp custom_interfaces)
add_executable(transformation_client src/client.cpp)
ament_target_dependencies(transformation_client rclcpp custom_interfaces)
add_executable(fibonacci_action_server src/action_server.cpp)
ament_target_dependencies(fibonacci_action_server rclcpp custom_interfaces rclcpp_action)
add_executable(fibonacci_action_client src/action_client.cpp)
ament_target_dependencies(fibonacci_action_client rclcpp custom_interfaces rclcpp_action)

install(TARGETS
  coordinate_publisher
  coordinate_subscriber
  transformation_server
  transformation_client
  fibonacci_action_server
  fibonacci_action_client
  DESTINATION lib/${PROJECT_NAME}
)