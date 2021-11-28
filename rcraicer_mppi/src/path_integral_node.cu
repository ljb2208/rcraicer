/*
* Software License Agreement (BSD License)
* Copyright (c) 2013, Georgia Institute of Technology
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**********************************************
 * @file path_integral_main.cpp
 * @author Grady Williams <gradyrw@gmail.com>
 * @date May 24, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief Main file model predictive path integral control.
 *
 ***********************************************/

//Some versions of boost require __CUDACC_VER__, which is no longer defined in CUDA 9. This is
//the old expression for how it was defined, so should work for CUDA 9 and under.
#define __CUDACC_VER__ __CUDACC_VER_MAJOR__ * 10000 + __CUDACC_VER_MINOR__ * 100 + __CUDACC_VER_BUILD__

// #include "../include/rcraicer_mppi/meta_math.h"
// #include "../include/rcraicer_mppi/param_getter.h"
// #include "../include/rcraicer_mppi/rcraicer_plant.h"
// // #include "../include/rcraicer_mppi/PathIntegralParamsConfig.h"
// #include "../include/rcraicer_mppi/path_integral/costs.cuh"

// //Including neural net model
// #ifdef MPPI_NNET_USING_CONSTANT_MEM__
// __device__ __constant__ float NNET_PARAMS[param_counter(6,32,32,4)];
// #endif
// #include "../include/rcraicer_mppi/neural_net_model.cuh"
// #include "../include/rcraicer_mppi/car_bfs.cuh"
// #include "../include/rcraicer_mppi/car_kinematics.cuh"
// #include "../include/rcraicer_mppi/generalized_linear.cuh"
// #include "../include/rcraicer_mppi/mppi_controller.cuh"
// #include "../include/rcraicer_mppi/run_control_loop.cuh"


#include "rclcpp/rclcpp.hpp"
// #include <atomic>

// #include <stdlib.h>
// #include <stdio.h>
// #include <math.h>

#include "../include/rcraicer_mppi/path_integral_node.cuh"

using namespace rcraicer_control;

using std::placeholders::_1;
using namespace std::chrono_literals;


PathIntegralNode::PathIntegralNode() : Node("path_integral_node"), robot(NULL)
{

  setupParams();
  updateParams();
  params.logger_name = "path_integral_node";
  params.use_sim_time = false;
  is_alive = true;

  paramSetCallbackHandler = this->add_on_set_parameters_callback(std::bind(&PathIntegralNode::paramSetCallback, this, std::placeholders::_1));  
  
  //Define the mppi costs  
  costs = new MPPICosts(config);

  //Define the internal dynamics model for mppi
  float2 control_constraints[2] = {make_float2(-.99, .99), make_float2(-.99, params.max_throttle)};
  model = new DynamicsModel(1.0/params.hz, control_constraints);
  model->loadParams(params.model_path, params.logger_name); //Load the model parameters from the launch file specified path  

  //Define the controller
  init_u[0] = (float)params.init_steering;
  init_u[1] = (float)params.init_throttle;

  exploration_std[0] = (float)params.steering_std;
  exploration_std[1] = (float)params.throttle_std;

  mppi = new Controller(model, costs, params.num_timesteps, params.hz, params.gamma, exploration_std, 
                                    init_u, params.num_iters, params.optimization_stride);

  robot = new RCRaicerPlant(params, config);  
  setupSubscribers();
  setupTimers();
  setupPublishers();
  robot->setPublishers(control_pub_, status_pub_, subscribed_pose_pub_, path_pub_, timing_data_pub_);

  // optimizer = std::thread(&runControlLoop<Controller>, mppi, robot, &params, &is_alive);
  optimizer = boost::thread(&runControlLoop<Controller>, mppi, robot, &params, &is_alive);

  RCLCPP_INFO(this->get_logger(), "Node started.");    
}

PathIntegralNode::~PathIntegralNode()
{
  pathTimer_->cancel();
  statusTimer_->cancel();
  debugImgTimer_->cancel();
  timingInfoTimer_->cancel();

  //Shutdown procedure
  is_alive.store(false);
  optimizer.join();
  robot->shutdown();
  mppi->deallocateCudaMem();
  delete robot;
  delete mppi;
  delete costs;
  delete model;
}


void PathIntegralNode::setupSubscribers()
{
  //Initialize the subscribers.
  pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(params.pose_estimate_name, 1, std::bind(&RCRaicerPlant::poseCall, robot, std::placeholders::_1));
  sim_sub_ = this->create_subscription<rcraicer_msgs::msg::SimState>("sim_state", 1, std::bind(&RCRaicerPlant::simStateCall, robot, std::placeholders::_1));
  servo_sub_ = this->create_subscription<rcraicer_msgs::msg::ChassisState>("chassis_state", 1, std::bind(&RCRaicerPlant::servoCall, robot, std::placeholders::_1));
  model_sub_ = this->create_subscription<rcraicer_msgs::msg::NeuralNetModel>("/model_updater/model", 1, std::bind(&RCRaicerPlant::modelCall, robot, std::placeholders::_1));
}

void PathIntegralNode::setupPublishers()
{
    //Initialize the publishers.
    control_pub_ = this->create_publisher<rcraicer_msgs::msg::ChassisCommand>("chassis_cmds", 1);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("nominalPath", 1);
    subscribed_pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("subscribedPose", 1);
    status_pub_ = this->create_publisher<rcraicer_msgs::msg::PathIntegralStatus>("mppiStatus", 1);
    timing_data_pub_ = this->create_publisher<rcraicer_msgs::msg::PathIntegralTiming>("timingInfo", 1);
}

void PathIntegralNode::setupTimers()
{
    //Timer callback for path publisher
    pathTimer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&RCRaicerPlant::pubPath, robot));
    statusTimer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&RCRaicerPlant::pubStatus, robot));
    debugImgTimer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&RCRaicerPlant::displayDebugImage, robot));
    timingInfoTimer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&RCRaicerPlant::pubTimingData, robot));
}

void PathIntegralNode::setupParams()
{
  // <!--Pose topic configuration parameters -->
  this->declare_parameter<std::string>("pose_estimate", "/ground_truth/state");
  this->declare_parameter<std::string>("path_topic", "/mppi_controller/nominal_path");
  
  // <!--Debug/Sim mode parameter -->
  this->declare_parameter<bool>("debug_mode", true);
  
  // <!--Setup parameters-->
  this->declare_parameter<int>("hz", 50);
  this->declare_parameter<int>("num_timesteps", 100);
  this->declare_parameter<double>("x_pos", 0.0);
  this->declare_parameter<double>("y_pos", 0.0);
  this->declare_parameter<double>("heading", 2.35);

  this->declare_parameter<int>("optimization_stride", 1);
  this->declare_parameter<bool>("use_feedback_gains", true);

  // Model parameters
  this->declare_parameter<std::string>("model_path", "");

  // <!-- Control hyper-parameters-->
  this->declare_parameter<double>("gamma", 0.15);
  this->declare_parameter<int>("num_iters", 1);

  // <!-- Control parameters -->
  this->declare_parameter<double>("init_steering", 0.0);
  this->declare_parameter<double>("init_throttle", 0.0);
  this->declare_parameter<double>("steering_std", 0.275);
  this->declare_parameter<double>("throttle_std", 0.3);
  this->declare_parameter<double>("max_throttle", 0.65);

  // <!-- Cost Parameters -->
  this->declare_parameter<bool>("l1_cost", false);
  this->declare_parameter<double>("desired_speed", 6.0);
  this->declare_parameter<double>("speed_coefficient", 4.25);
  this->declare_parameter<double>("track_coefficient", 200.0);
  this->declare_parameter<double>("max_slip_angle", 1.25);
  this->declare_parameter<double>("slip_penalty", 10.0);
  this->declare_parameter<double>("track_slop", 0.0);
  this->declare_parameter<double>("crash_coeff", 10000.0);
  this->declare_parameter<double>("steering_coeff", 0.0);
  this->declare_parameter<double>("throttle_coeff", 0.0);
  this->declare_parameter<double>("boundary_threshold", 0.65);
  this->declare_parameter<double>("discount", 0.1);
  this->declare_parameter<std::string>("map_path", "");
}

void PathIntegralNode::updateParams()
{      
    params.pose_estimate_name = this->get_parameter("pose_estimate").as_string();
    params.path_topic = this->get_parameter("path_topic").as_string();
    params.debug_mode = this->get_parameter("debug_mode").as_bool();
    params.hz = this->get_parameter("hz").as_int();
    params.num_timesteps = this->get_parameter("num_timesteps").as_int();
    params.num_iters = this->get_parameter("num_iters").as_int();
    params.x_pos = this->get_parameter("x_pos").as_double();
    params.y_pos = this->get_parameter("y_pos").as_double();
    params.heading = this->get_parameter("heading").as_double();
    params.gamma = this->get_parameter("gamma").as_double();
    params.init_steering = this->get_parameter("init_steering").as_double();
    params.init_throttle = this->get_parameter("init_throttle").as_double();
    params.steering_std = this->get_parameter("steering_std").as_double();
    params.throttle_std = this->get_parameter("throttle_std").as_double();
    params.max_throttle = this->get_parameter("max_throttle").as_double();
    params.model_path = this->get_parameter("model_path").as_string();    
    params.optimization_stride = this->get_parameter("optimization_stride").as_int();
    params.use_feedback_gains = this->get_parameter("use_feedback_gains").as_bool();


    if (config.desired_speed != this->get_parameter("desired_speed").as_double())
      costParamsUpdated = true;

    if (config.speed_coefficient != this->get_parameter("speed_coefficient").as_double())
        costParamsUpdated = true;

    if (config.track_coefficient != this->get_parameter("track_coefficient").as_double())
      costParamsUpdated = true;

    if (config.max_slip_angle != this->get_parameter("max_slip_angle").as_double())
      costParamsUpdated = true;

    if (config.slip_penalty != this->get_parameter("slip_penalty").as_double())
      costParamsUpdated = true;

    if (config.crash_coefficient != this->get_parameter("crash_coeff").as_double())
      costParamsUpdated = true;

    if (config.track_slop != this->get_parameter("track_slop").as_double())
      costParamsUpdated = true;

    if (config.steering_coeff != this->get_parameter("steering_coeff").as_double())
      costParamsUpdated = true;

    if (config.throttle_coeff != this->get_parameter("throttle_coeff").as_double())
      costParamsUpdated = true;
      
    config.max_throttle = this->get_parameter("max_throttle").as_double();
    config.desired_speed = this->get_parameter("desired_speed").as_double();
    config.speed_coefficient = this->get_parameter("speed_coefficient").as_double();  
    config.track_coefficient = this->get_parameter("track_coefficient").as_double();  
    config.max_slip_angle = this->get_parameter("max_slip_angle").as_double();  
    config.slip_penalty = this->get_parameter("slip_penalty").as_double();  
    config.crash_coefficient = this->get_parameter("crash_coeff").as_double();  
    config.track_slop = this->get_parameter("track_slop").as_double();  
    config.steering_coeff = this->get_parameter("steering_coeff").as_double();  
    config.throttle_coeff = this->get_parameter("throttle_coeff").as_double();  
    config.boundary_threshold = this->get_parameter("boundary_threshold").as_double();  
    config.discount = this->get_parameter("discount").as_double();  
    config.num_timesteps = this->get_parameter("num_timesteps").as_int();  
    config.l1_cost = this->get_parameter("l1_cost").as_bool();  
    config.map_path = this->get_parameter("map_path").as_string();    

    if (costParamsUpdated && robot != NULL)
    {      
      robot->updateConfigParams(config);
    }

    costParamsUpdated = false;
}

rcl_interfaces::msg::SetParametersResult PathIntegralNode::paramSetCallback(const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    updateParams();
    
    return result;
}

int main(int argc, char** argv) {  
  //Ros node initialization
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathIntegralNode>());  
  rclcpp::shutdown();
  return 0;   
}
