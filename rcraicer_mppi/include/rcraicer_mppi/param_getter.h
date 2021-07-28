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
 * @file param_getter.h
 * @author Grady Williams <gradyrw@gmail.com>
 * @date May 24, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief Function for grabbing parameters from the ROS
 * parameter server.
 ***********************************************/

#ifndef PARAM_GETTER_H_
#define PARAM_GETTER_H_

#include <unistd.h>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace rcraicer_control {

typedef struct
{ 
  bool debug_mode;
  bool use_feedback_gains;
  int hz;
  int num_timesteps;
  int num_iters;
  float x_pos;
  float y_pos;
  float heading;
  float gamma;
  float init_steering;
  float init_throttle;
  float steering_std;
  float throttle_std;
  float max_throttle;
  std::string model_path;  
  std::string logger_name;
  std::string pose_estimate_name;
  std::string path_topic;
  int optimization_stride;
  bool use_sim_time;
} SystemParams;


typedef struct
{
  float max_throttle;
  float desired_speed;
  float speed_coefficient;
  float track_coefficient;
  float max_slip_angle;
  float slip_penalty;
  float crash_coefficient;
  float track_slop;
  float steering_coeff;
  float throttle_coeff;  
  float boundary_threshold;
  float discount;  
  int num_timesteps;
  bool l1_cost;  
  std::string map_path;
} PathIntegralParams;

inline bool fileExists (const std::string& name) {
    return ( access( name.c_str(), F_OK ) != -1 );
}

template <typename T>
T getRosParam(std::string paramName, rclcpp::Node::SharedPtr nh)
{
  std::string key;
  T val;

  bool found = nh->has_parameter(paramName);
  
  if (!found){
    RCLCPP_ERROR(nh->get_logger(), "Could not find parameter name '%s' in tree of node '%s'", 
              paramName.c_str(), nh->get_namespace());
  }
  else {
    rclcpp::Parameter param = nh->get_parameter(paramName);
    val = param.get_value<T>();
  }
  return val;
}

void loadParams(SystemParams* params, rclcpp::Node::SharedPtr nh);

}

#endif /*PARAM_GETTER_H_*/


