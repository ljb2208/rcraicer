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
 * @file run_control_loop.cuh
 * @author Grady Williams <gradyrw@gmail.com>
 * @date May 24, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief Runs the control loop for a given feedback control law
 * parameter server.
 ***********************************************/

#ifndef RUN_CONTROLLER_CUH_
#define RUN_CONTROLLER_CUH_

#include "rcraicer_plant.h"
#include "param_getter.h"

#include "../ddp/ddp_model_wrapper.h"
#include "../ddp/ddp_tracking_costs.h"
#include "../ddp/ddp.h"

#include <opencv2/core/core.hpp>
#include <atomic>

#include <boost/thread/thread.hpp>
#include <unistd.h>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

namespace rcraicer_control {

void outputVector(std::string vecName, std::vector<float> vec)
{
  std::cout << vecName.c_str() << "\n";

  for (size_t i=0; i < vec.size(); i++)
  {
    std::cout << vec[i] << " ";    
  }
  std::cout << "\n";
}

template <class CONTROLLER_T> 
void runControlLoop(CONTROLLER_T* controller, RCRaicerPlant* robot, SystemParams* params, std::atomic<bool>* is_alive)
{  
  //Initial condition of the robot
  Eigen::MatrixXf state(7,1);
  RCRaicerPlant::FullState fs;
  state << params->x_pos, params->y_pos, params->heading, 0, 0, 0, 0;
  
  //Initial control value
  Eigen::MatrixXf u(2,1);
  u << 0, 0;

  std::vector<float> controlSolution;
  std::vector<float> stateSolution;

  //Obstacle and map parameters
  std::vector<int> obstacleDescription;
  std::vector<float> obstacleData;
  std::vector<int> costmapDescription;
  std::vector<float> costmapData;
  std::vector<int> modelDescription;
  std::vector<float> modelData;

  //Counter, timing, and stride variables.
  int num_iter = 0;
  int status = 1;
  int optimization_stride = params->optimization_stride; 
  bool use_feedback_gains = params->use_feedback_gains;
  double avgOptimizationLoopTime = 0; //Average time between pose estimates
  double avgOptimizationTickTime = 0; //Avg. time it takes to get to the sleep at end of loop
  double avgSleepTime = 0; //Average time spent sleeping
  rclcpp::Time last_pose_update = robot->getLastPoseTime();

  std::chrono::nanoseconds ns{(int)(optimization_stride/(1.0*params->hz))};
  rclcpp::Duration optimizationLoopTime(ns);

  //Set the loop rate
  std::chrono::milliseconds ms{(int)(optimization_stride*1000.0/params->hz)};

  if (!params->debug_mode){
    while(last_pose_update == robot->getLastPoseTime() && is_alive->load()){ //Wait until we receive a pose estimate
      usleep(50);
    }
  }

  controller->resetControls();
  controller->computeFeedbackGains(state);  

  //Start the control loop.
  while (is_alive->load()) {
    std::chrono::steady_clock::time_point loop_start = std::chrono::steady_clock::now();
    robot->setTimingInfo(avgOptimizationLoopTime, avgOptimizationTickTime, avgSleepTime);
    num_iter ++;

    // if (params->debug_mode){ //Display the debug window.
     cv::Mat debug_img = controller->costs_->getDebugDisplay(state(0), state(1), state(2));
     robot->setDebugImage(debug_img);
    // }
    //Update the state estimate
    if (last_pose_update != robot->getLastPoseTime()){
      optimizationLoopTime = robot->getLastPoseTime() - last_pose_update;
      last_pose_update = robot->getLastPoseTime();      
      fs = robot->getState(); //Get the new state.
      state << fs.x_pos, fs.y_pos, fs.yaw, fs.roll, fs.u_x, fs.u_y, fs.yaw_mder;

      // std::cout << "Pose State: " << state(0) << "," << state(1) << "," << state(2) << "," << state(3) << "," << state(4) << "," << state(5) << "," << state(6) << "\r\n";
    }
    //Update the cost parameters
    if (robot->hasNewDynRcfg()){
      controller->costs_->updateConfigParams(robot->getDynRcfgParams());
    }
    //Update any obstacles
    if (robot->hasNewObstacles()){
      robot->getObstacles(obstacleDescription, obstacleData);
      controller->costs_->updateObstacles(obstacleDescription, obstacleData);
    }
    //Update the costmap
    if (robot->hasNewCostmap()){
      robot->getCostmap(costmapDescription, costmapData);
      controller->costs_->updateCostmap(costmapDescription, costmapData);
    }
    //Update dynamics model
    if (robot->hasNewModel()){
      robot->getModel(modelDescription, modelData);
      controller->model_->updateModel(modelDescription, modelData);
    }
  
    //Figure out how many controls have been published since we were last here and slide the 
    //control sequence by that much.
    int stride = round(optimizationLoopTime.seconds()*params->hz);
    if (status != 0){
      stride = optimization_stride;
    }
    if (stride >= 0 && stride < params->num_timesteps){
      controller->slideControlSeq(stride);
    }

    //Compute a new control sequence
    controller->computeControl(state); //Compute the control
    if (use_feedback_gains){
      controller->computeFeedbackGains(state);
    }
    controlSolution = controller->getControlSeq();
    stateSolution = controller->getStateSeq();
    auto result = controller->getFeedbackGains();

    // outputVector("controlSolution", controlSolution);
    // outputVector("stateSolution", stateSolution);

    //Set the updated solution for execution
    robot->setSolution(stateSolution, controlSolution, result.feedback_gain, last_pose_update, avgOptimizationLoopTime);
    
    //Check the robots status
    status = robot->checkStatus();

    //Increment the state if debug mode is set to true
    if (status != 0 && params->debug_mode){
      for (int t = 0; t < optimization_stride; t++){
        u << controlSolution[2*t], controlSolution[2*t + 1];
        controller->model_->updateState(state, u); 

        std::cout << "State: " << state(0) << "," << state(1) << "," << state(2) << "," << state(3) << "," << state(4) << "," << state(5) << "," << state(6) << ",,";
        std::cout <<  u(0) << "," << u(1) << "\n";
      }
    }
    
    //Sleep for any leftover time in the control loop
    std::chrono::duration<double, std::milli> fp_ms = std::chrono::steady_clock::now() - loop_start;
    double optimizationTickTime = fp_ms.count();
    int count = 0;
    while(is_alive->load() && (fp_ms < ms || ((robot->getLastPoseTime() - last_pose_update).seconds() < (1.0/params->hz - 0.0025) && status == 0))) {
      usleep(50);
      fp_ms = std::chrono::steady_clock::now() - loop_start;
      count++;
    }
    double sleepTime = fp_ms.count() - optimizationTickTime;

    //Update the average loop time data
    avgOptimizationLoopTime = (num_iter - 1.0)/num_iter*avgOptimizationLoopTime + 1000.0*optimizationLoopTime.seconds()/num_iter; 
    avgOptimizationTickTime = (num_iter - 1.0)/num_iter*avgOptimizationTickTime + optimizationTickTime/num_iter;
    avgSleepTime = (num_iter - 1.0)/num_iter*avgSleepTime + sleepTime/num_iter;
  }
}

}

#endif /*RUN_CONTROLLER_CUH_*/
