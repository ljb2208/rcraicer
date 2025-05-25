cmake_minimum_required(VERSION 3.5)
project(rcraicer_mppi_test)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcraicer_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(sim_data_recorder_node src/sim_data_recorder.cpp)
add_executable(data_recorder src/data_recorder.cpp)

ament_target_dependencies(sim_data_recorder_node rclcpp std_msgs sensor_msgs rcraicer_msgs tf2 tf2_ros)
ament_target_dependencies(data_recorder rclcpp std_msgs sensor_msgs nav_msgs rcraicer_msgs tf2 tf2_ros)


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
    sim_data_recorder_node    
    data_recorder
    DESTINATION lib/${PROJECT_NAME})

ament_package()
