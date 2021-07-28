cmake_minimum_required(VERSION 3.5)
project(rcraicer_mapping)

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

include_directories( 
  /usr/local/include
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rcraicer_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(cv_bridge REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


add_executable(map_generator_node src/map_generator.cpp)
add_executable(map_publisher_node src/map_publisher.cpp)

ament_target_dependencies(map_generator_node rclcpp std_msgs sensor_msgs rcraicer_msgs OpenCV tf2)
target_link_libraries(map_generator_node /usr/local/lib/libcnpy.so /usr/lib/x86_64-linux-gnu/libz.so)

ament_target_dependencies(map_publisher_node rclcpp std_msgs sensor_msgs rcraicer_msgs OpenCV tf2 image_transport cv_bridge)
target_link_libraries(map_publisher_node /usr/local/lib/libcnpy.so /usr/lib/x86_64-linux-gnu/libz.so)


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
    map_generator_node    
    DESTINATION lib/${PROJECT_NAME})



ament_package()
