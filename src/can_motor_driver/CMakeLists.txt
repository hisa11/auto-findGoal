cmake_minimum_required(VERSION 3.8)
project(can_motor_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(main_node 
  src/main_node.cpp
  src/usb_can.cpp
  src/dm_motor.cpp
  src/base_motor.cpp
)

target_include_directories(main_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(main_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(main_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

# ============================================================
# VESC 4輪オムニドライブ ノード
# ============================================================
add_executable(motor_node
  src/motor_node.cpp
)

target_compile_features(motor_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(motor_node
  rclcpp
  std_msgs
  geometry_msgs
)

install(TARGETS
  main_node
  motor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/yolo_goal_detector.py
  src/advanced_yolo_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

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

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
