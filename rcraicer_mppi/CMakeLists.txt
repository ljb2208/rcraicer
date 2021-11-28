cmake_minimum_required(VERSION 3.5)
project(rcraicer_mppi C CXX CUDA)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set (VERBOSE 1)


# set (CUDA_INCLUDE_DIRS "/usr/local/cuda-11.4/include")
# set (CUDA_CUDART_LIBRARY )

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcraicer_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Boost COMPONENTS system thread chrono REQUIRED)

find_package(OpenCV REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)


################################################################################
#First make the helper libraries
################################################################################
add_library(rcraicer_plant src/rcraicer_plant.cpp)
ament_target_dependencies(rcraicer_plant rclcpp std_msgs sensor_msgs rcraicer_msgs nav_msgs Eigen3 OpenCV)

add_library(param_getter src/param_getter.cpp)
ament_target_dependencies(param_getter rclcpp std_msgs)

# add_executable(path_integral_node src/path_integral_node.cpp)
# ament_target_dependencies(path_integral_node rclcpp std_msgs)

find_package(CUDA REQUIRED)

if(CUDA_FOUND)

include(FindCUDA)

set(CUDA_PROPAGATE_HOST_FLAGS OFF)

# set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -std=c++17 -maxrregcount=32 -O3 -arch=sm_75 -w -lineinfo -G -DTHRUST_DEBUG -g)
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -std=c++17 -maxrregcount=32 -arch=sm_35 -w -g -G)


####################################################################################
#Executable for the basis function model
####################################################################################

# cuda_add_executable(path_integral_bf src/path_integral_node.cu )
# target_compile_definitions(path_integral_bf PRIVATE PRIVATE USE_BASIS_FUNC_MODEL__)
# ament_target_dependencies(path_integral_bf rclcpp std_msgs sensor_msgs rcraicer_msgs nav_msgs Eigen3 OpenCV Boost CUDA)
# target_link_libraries(path_integral_bf rcraicer_plant param_getter cublas curand  /usr/local/lib/libcnpy.so)

# ###################################################################################
# #Executable for the neural network model
# ###################################################################################

# add_executable(path_integral_nn src/path_integral_node.cu )
cuda_add_executable(path_integral_nn src/path_integral_node.cu)
target_compile_definitions(path_integral_nn PRIVATE USE_NEURAL_NETWORK_MODEL__; MPPI_NNET_USING_CONSTANT_MEM__)
ament_target_dependencies(path_integral_nn rclcpp std_msgs sensor_msgs rcraicer_msgs nav_msgs Eigen3 OpenCV Boost CUDA)
target_link_libraries(path_integral_nn rcraicer_plant param_getter cublas curand  /usr/local/lib/libcnpy.so)

cuda_add_executable(test_nn src/test.cu)
target_compile_definitions(test_nn PRIVATE USE_NEURAL_NETWORK_MODEL__; MPPI_NNET_USING_CONSTANT_MEM__)
ament_target_dependencies(test_nn rclcpp Eigen3 OpenCV Boost CUDA)
target_link_libraries(test_nn cublas curand rcraicer_plant param_getter /usr/local/lib/libcnpy.so)

endif(CUDA_FOUND)

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
    rcraicer_plant
    param_getter
    # path_integral_bf
    path_integral_nn
    test_nn
    DESTINATION lib/${PROJECT_NAME})

ament_package()
