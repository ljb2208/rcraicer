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
 * @file autorally_plant.h
 * @author Grady Williams <gradyrw@gmail.com>
 * @date May 24, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief Class definition for AutorallyPlant class.
 ***********************************************/

#ifndef RCRAICER_PLANT_H_
#define RCRAICER_PLANT_H_

#include "param_getter.h"

#include "../ddp/util.h"
#include "rcraicer_msgs/msg/chassis_command.hpp"
#include "rcraicer_msgs/msg/chassis_state.hpp"
#include "rcraicer_msgs/msg/run_stop.hpp"
#include "rcraicer_msgs/msg/path_integral_status.hpp"
#include "rcraicer_msgs/msg/path_integral_timing.hpp"
#include "rcraicer_msgs/msg/neural_net_model.hpp"
#include "rcraicer_msgs/msg/path_integral_params.hpp"



#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcraicer_msgs/msg/sim_state.hpp"
#include "rcraicer_msgs/msg/chassis_command.hpp"

// #include <dynamic_reconfigure/server.h>


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <Eigen/Dense>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <atomic>

#include <mutex>

namespace rcraicer_control {

/**
* @class AutorallyPlant autorally_plant.h
* @brief Publishers and subscribers for the autorally control system.
*
* This class is treated as the plant for the MPPI controller. When the MPPI
* controller has a control it sends to a function in this class to get
* send to the actuators. Likewise it calls functions in this class to receive
* state feedback. This class also publishes trajectory and spray information
* and status information for both the controller and the OCS.
*/

class RCRaicerPlant //: public Diagnostics
{
public:
  static const int RCRAICER_STATE_DIM = 7;
  static const int RCRAICER_CONTROL_DIM = 2;
  //Struct for holding the autorally pose.
  typedef struct
  {
    //X-Y-Z position
    float x_pos;
    float y_pos;
    float z_pos;
    //1-2-3 Euler angles
    float roll;
    float pitch;
    float yaw;
    //Quaternions
    float q0;
    float q1;
    float q2;
    float q3;
    //X-Y-Z velocity.
    float x_vel;
    float y_vel;
    float z_vel;
    //Body frame velocity
    float u_x;
    float u_y;
    //Euler angle derivatives
    float yaw_mder;
    //Current servo commands
    float steering;
    float throttle;
  } FullState;

  float last_heading_ = 0.0;
  float throttleMax_ = 0.99;
  int heading_multiplier_ = 0;

	boost::mutex access_guard_;
  std::string nodeNamespace_;

  bool new_model_available_;
  cv::Mat debugImg_;

  bool solutionReceived_ = false;
  bool is_nodelet_;
  std::vector<float> controlSequence_;
  std::vector<float> stateSequence_;
  util::EigenAlignedVector<float, 2, 7> feedback_gains_;
  rclcpp::Time solutionTs_;

  int numTimesteps_;
  double deltaT_;

  double optimizationLoopTime_;

  /**
  * @brief Constructor for AutorallyPlant, takes the a ros node handle and initalizes
  * publishers and subscribers.
  * @param mppi_node A ros node handle.
  */
	RCRaicerPlant(SystemParams params, PathIntegralParams costParams);	

  /**
  * @brief Callback for /pose_estimate subscriber.
  */
	void poseCall(nav_msgs::msg::Odometry::SharedPtr pose_msg);

  void processPose(std_msgs::msg::Header header, geometry_msgs::msg::Quaternion orientation, geometry_msgs::msg::Point position, geometry_msgs::msg::Vector3 linear_velocity, geometry_msgs::msg::Vector3 angular_velocity);

  /**
  * @brief Callback for recording the current servo input.
  * 
  */
  void servoCall(rcraicer_msgs::msg::ChassisState::SharedPtr servo_msg);

  void simStateCall(rcraicer_msgs::msg::SimState::SharedPtr state_msg);

  bool hasNewModel();
  virtual void modelCall(rcraicer_msgs::msg::NeuralNetModel::SharedPtr model_msg);
  virtual void getModel(std::vector<int> &description, std::vector<float> &data);

  /**
  * @brief Callback for safe speed subscriber.
  */
	void runstopCall(rcraicer_msgs::msg::RunStop::SharedPtr safe_msg);

  /**
  * @brief Publishes the controller's nominal path.
  */
	void pubPath();

  void setPublishers(rclcpp::Publisher<rcraicer_msgs::msg::ChassisCommand>::SharedPtr control_pub,
                      rclcpp::Publisher<rcraicer_msgs::msg::PathIntegralStatus>::SharedPtr status_pub,
                      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr subscribed_pose_pub,
                      rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub,
                      rclcpp::Publisher<rcraicer_msgs::msg::PathIntegralTiming>::SharedPtr timing_data_pub);

  void setSolution(std::vector<float> traj, std::vector<float> controls,
                   util::EigenAlignedVector<float, 2, 7> gains,
                   rclcpp::Time timestamp, double loop_speed);

  void setDebugImage(cv::Mat img);

  void setTimingInfo(double poseDiff, double tickTime, double sleepTime);

  void pubTimingData();

  /**
  * @brief Publishes a control input.
  * @param steering The steering command to publish.
  * @param throttle The throttle command to publish.
  */
	void pubControl(float steering, float throttle);

  void pubStatus();

  /**
  * @brief Returns the current state of the system.
  */
	virtual RCRaicerPlant::FullState getState();

  /**
  * @brief Returns the current value of safe speed.
  */
  bool getRunstop();

  /**
  * @brief Returns the timestamp of the last pose callback.
  */
  rclcpp::Time getLastPoseTime();

  /**
  * @brief Checks the system status.
  * @return An integer specifying the status. 0 means the system is operating
  * nominally, 1 means something is wrong but no action needs to be taken,
  * 2 means that the vehicle should stop immediately.
  */
  int checkStatus();

  void updateConfigParams(rcraicer_control::PathIntegralParams &config);

  bool hasNewDynRcfg();

  rcraicer_control::PathIntegralParams getDynRcfgParams();

  virtual void displayDebugImage();  

  virtual bool hasNewObstacles(){return false;};
  virtual void getObstacles(std::vector<int> &description, std::vector<float> &data){};

  virtual bool hasNewCostmap(){return false;};
  virtual void getCostmap(std::vector<int> &description, std::vector<float> &data){};

  virtual void shutdown();

protected:
  //SystemParams mppiParams_;
  int poseCount_ = 0;
  bool useFeedbackGains_ = false;
  std::atomic<bool> receivedDebugImg_;
  std::atomic<bool> debugShutdownSignal_;
  std::atomic<bool> debugShutdownSignalAcknowledged_;
  rcraicer_msgs::msg::NeuralNetModel::SharedPtr dynamicsModel_;
  rcraicer_control::PathIntegralParams costParams_;
  bool hasNewCostParams_ = false;

  const double TIMEOUT = 0.5; ///< Time before declaring pose/controls stale.

  FullState full_state_; ///< Full state of the autorally vehicle.

  int hz_; ///< The frequency of the control publisher.

  int status_; ///< System status
  std::string ocs_msg_; ///< Message to send to the ocs.

  bool safe_speed_zero_; ///< Current value of safe speed.
  bool debug_mode_; ///< Whether or not the system is in debug/simulation mode.
  bool activated_; ///< Whether or not we've received an initial pose message.

  rclcpp::Time last_pose_call_; ///< Timestamp of the last pose callback.


  rclcpp::Publisher<rcraicer_msgs::msg::ChassisCommand>::SharedPtr control_pub_; ///< Publisher of autorally_msgs::chassisCommand type on topic servoCommand.
  rclcpp::Publisher<rcraicer_msgs::msg::PathIntegralStatus>::SharedPtr status_pub_; ///< Publishes the status (0 good, 1 neutral, 2 bad) of the controller
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr subscribed_pose_pub_; ///< Publisher of the subscribed pose
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_; ///< Publisher of nav_mags::Path on topic nominalPath.
  rclcpp::Publisher<rcraicer_msgs::msg::PathIntegralTiming>::SharedPtr timing_data_pub_;
  
  // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr  pose_sub_; ///< Subscriber to /pose_estimate.
  // rclcpp::Subscription<rcraicer_msgs::msg::SimState>::SharedPtr  sim_sub_; ///< Subscriber to /pose_estimate.
  // rclcpp::Subscription<rcraicer_msgs::msg::ChassisState>::SharedPtr  servo_sub_;
  // rclcpp::Subscription<rcraicer_msgs::msg::NeuralNetModel>::SharedPtr  model_sub_;

  // rclcpp::TimerBase::SharedPtr pathTimer_;
  // rclcpp::TimerBase::SharedPtr statusTimer_;
  // rclcpp::TimerBase::SharedPtr debugImgTimer_;
  // rclcpp::TimerBase::SharedPtr timingInfoTimer_;

  nav_msgs::msg::Path path_msg_; ///< Path message for publishing the planned path.
  geometry_msgs::msg::Point time_delay_msg_; ///< Point message for publishing the observed delay.
  rcraicer_msgs::msg::PathIntegralStatus status_msg_; ///<pathIntegralStatus message for publishing mppi status
  rcraicer_msgs::msg::PathIntegralTiming timingData_; ///<pathIntegralStatus message for publishing mppi status

  std::string logger_name;
  rclcpp::Clock clock;
};

}

#endif /* RCRAICER_PLANT_H_ */
