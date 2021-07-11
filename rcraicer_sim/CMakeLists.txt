cmake_minimum_required(VERSION 3.5)
project(rcraicer_sim)

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

#set(AirLib_DIR /home/lbarnett/development/AirSim/build_release/AirLib)
#set(MavLinkCom_DIR /home/lbarnett/development/AirSim/build_release/MavLinkCom)
set(AIRSIM_ROOT /home/lbarnett/development/AirSim)

add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)

set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.3.0/include")
set(RPC_LIB rpc) # name of .a file with lib prefix
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rcraicer_msgs REQUIRED)
find_package(GeographicLib REQUIRED)


include_directories( 
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils  
  ${RPC_LIB_INCLUDES}  
)

add_executable(sim_node src/sim_node.cpp src/tcp_server.cpp)
add_executable(airsim_node src/airsim_node.cpp)


ament_target_dependencies(sim_node rclcpp std_msgs sensor_msgs rcraicer_msgs Eigen3 tf2 tf2_eigen GeographicLib)
target_link_libraries(airsim_node AirLib)
ament_target_dependencies(airsim_node rclcpp std_msgs sensor_msgs geometry_msgs rcraicer_msgs Eigen3 tf2 tf2_eigen GeographicLib)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
    sim_node    
    airsim_node
    DESTINATION lib/${PROJECT_NAME})


ament_package()
