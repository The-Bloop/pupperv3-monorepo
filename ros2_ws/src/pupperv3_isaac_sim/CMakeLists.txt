cmake_minimum_required(VERSION 3.8)
project(pupper_isaac_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
# Provides actuator_model.hpp + fixed_size_queue.hpp via ament_export_include_directories(lib).
# Reusing the headers rather than copying them is what keeps the 'plugin' torque path
# bit-identical to MuJoCo's motor model.
find_package(pupper_mujoco_sim REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/lib)

add_library(isaac_hardware_interface SHARED lib/isaac_hardware_interface.cpp)
ament_target_dependencies(
  isaac_hardware_interface
  rclcpp
  rclcpp_lifecycle
  std_msgs
  pluginlib
  hardware_interface
  pupper_mujoco_sim)

pluginlib_export_plugin_description_file(hardware_interface
                                         isaac_hardware_interface.xml)

install(TARGETS isaac_hardware_interface DESTINATION lib)
install(DIRECTORY lib/ DESTINATION lib)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(lib)
ament_export_libraries(isaac_hardware_interface)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
