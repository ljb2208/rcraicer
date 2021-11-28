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

#include "../include/rcraicer_mppi/meta_math.h"


//Including neural net model
#ifdef MPPI_NNET_USING_CONSTANT_MEM__
__device__ __constant__ float NNET_PARAMS[param_counter(6,32,32,4)];
#endif


#include "../include/rcraicer_mppi/neural_net_model.cuh"
#include "../include/rcraicer_mppi/car_bfs.cuh"
#include "../include/rcraicer_mppi/car_kinematics.cuh"
#include "../include/rcraicer_mppi/generalized_linear.cuh"


#include <atomic>

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <chrono>
#include <functional>
#include <cfloat>

using namespace rcraicer_control;

const int MPPI_NUM_ROLLOUTS__ = 1920;
const int BLOCKSIZE_X = 8;
const int BLOCKSIZE_Y = 16;
typedef NeuralNetModel<7,2,3,6,32,32,4> DynamicsModel;

int main(int argc, char** argv) {  
  //Define the internal dynamics model for mppi
  float2 control_constraints[2] = {make_float2(-.99, .99), make_float2(-.99, .99)};
  DynamicsModel* model = new DynamicsModel(1.0/50.0, control_constraints);
  model->loadParams("/home/lbarnett/catkin_ws/src/autorally/autorally_control/src/path_integral/params/models/autorally_nnet_09_12_2018.npz", ""); //Load the model parameters from the launch file specified path  

  DynamicsModel* model2 = new DynamicsModel(1.0/50.0, control_constraints);
  model2->loadParams("/home/lbarnett/ros2_ws/src/rcraicer/rcraicer_mppi_test/models/dynmodel_states4.npz", ""); //Load the model parameters from the launch file specified path  

  Eigen::MatrixXf s(7,1);
  s(0) = 0.0;
  s(1) = 0.0;
  s(2) = 0.0;
  s(3) = 0.0;
  s(4) = 0.0;
  s(5) = 0.0;
  s(6) = 0.0;
  Eigen::MatrixXf u(2,1);
  u(0) = 0.0;
  u(1) = 0.5;

  model->computeDynamics(s, u);

  std::cout << "Model1: ";
  for (int i=3; i < 7; i++)
  {
    std::cout << model->state_der_(i) << " ";
  }

  std::cout << "\nModel2: ";

  model2->computeDynamics(s, u);

  for (int i=3; i < 7; i++)
  {
    std::cout << model2->state_der_(i) << " ";
  }


  delete model;
  delete model2;
  return 0;   
}
