cmake_minimum_required(VERSION 3.8)
project(ni_daqmx_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Waveform.msg"  # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
include_directories(include ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(DAQmx_ROS2 src/DAQmx_AIN_ROS2_Example.cpp)
target_include_directories(DAQmx_ROS2 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/date)
message(${CMAKE_CURRENT_SOURCE_DIR}/include/date)
target_link_libraries(DAQmx_ROS2 /usr/lib/x86_64-linux-gnu/libnidaqmx.so)


ament_target_dependencies(DAQmx_ROS2 rclcpp std_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

  target_link_libraries(DAQmx_ROS2 "${cpp_typesupport_target}")


install(TARGETS
  DAQmx_ROS2
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
