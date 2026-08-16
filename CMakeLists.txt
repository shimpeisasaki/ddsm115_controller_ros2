cmake_minimum_required(VERSION 3.8)
project(ddsm115_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(motor_control src/motor_control.cpp)
target_compile_features(motor_control PUBLIC cxx_std_17)
target_include_directories(motor_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(check_motor_id src/check_motor_id.cpp)
target_link_libraries(check_motor_id motor_control)
ament_target_dependencies(check_motor_id rclcpp)

add_executable(set_motor_id src/set_motor_id.cpp)
target_link_libraries(set_motor_id motor_control)
ament_target_dependencies(set_motor_id rclcpp)

add_executable(velocity_control src/velocity_control.cpp)
target_link_libraries(velocity_control motor_control)
ament_target_dependencies(velocity_control rclcpp std_msgs std_srvs)

add_executable(two_wheels_robot src/two_wheels_robot.cpp)
ament_target_dependencies(
  two_wheels_robot geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs std_srvs tf2 tf2_ros)
target_compile_features(two_wheels_robot PUBLIC cxx_std_17)
target_include_directories(two_wheels_robot PRIVATE include)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS
  motor_control
  check_motor_id
  set_motor_id
  velocity_control
  two_wheels_robot
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY rviz_config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/motor_test_gui DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)

  ament_cpplint(FILTERS "-legal/copyright")
  ament_flake8(launch scripts/motor_test_gui)
  ament_lint_cmake()
  ament_uncrustify()

  ament_add_gtest(test_differential_drive test/test_differential_drive.cpp)
  target_include_directories(test_differential_drive PRIVATE include)
endif()

ament_package()