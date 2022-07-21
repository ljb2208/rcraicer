cmake_minimum_required(VERSION 3.5)
project(rcraicer)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic )
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rcraicer_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(GTSAM REQUIRED)

#add_library(rc_diagnostics src/diagnostics.cpp)
add_executable(imu_mavlink src/imu_mavlink.cpp src/mavlink_serial_port.cpp)
add_executable(arduino_controller src/arduino_controller.cpp src/serial_port.cpp)
add_executable(ar_arduino_controller src/ar_arduino_controller.cpp src/serial_port.cpp)
add_executable(wheel_odometry src/wheel_odometry.cpp)
add_executable(state_estimator src/state_estimator.cpp)
add_executable(runstop src/runstop.cpp src/serial_port.cpp)
add_executable(joystick_control src/joystick_control.cpp)

target_link_libraries(state_estimator gtsam)

ament_python_install_package(scripts/)

#add_executable(external_tracker src/external_tracker.cpp)
#ament_target_dependencies(rc_diagnostics rclcpp diagnostic_msgs diagnostic_updater)
ament_target_dependencies(imu_mavlink rclcpp std_msgs sensor_msgs Eigen3 geometry_msgs tf2 tf2_eigen)
ament_target_dependencies(arduino_controller rclcpp std_msgs sensor_msgs rcraicer_msgs)
ament_target_dependencies(ar_arduino_controller rclcpp std_msgs sensor_msgs rcraicer_msgs)
ament_target_dependencies(wheel_odometry rclcpp std_msgs sensor_msgs nav_msgs visualization_msgs rcraicer_msgs tf2)
ament_target_dependencies(state_estimator rclcpp std_msgs sensor_msgs nav_msgs visualization_msgs geometry_msgs rcraicer_msgs tf2 GTSAM Eigen3 GeographicLib)
ament_target_dependencies(runstop rclcpp std_msgs rcraicer_msgs)
ament_target_dependencies(joystick_control rclcpp std_msgs sensor_msgs rcraicer_msgs)
#ament_target_dependencies(external_tracker rclcpp std_msgs sensor_msgs)


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

install (TARGETS 
 #   rc_diagnostics
    imu_mavlink
    arduino_controller
    ar_arduino_controller
    wheel_odometry
    state_estimator
    runstop
    joystick_control
    #external_tracker
    DESTINATION lib/${PROJECT_NAME})

 install(PROGRAMS
  scripts/system_status.py
  DESTINATION bin
)

  # Install launch files.
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
