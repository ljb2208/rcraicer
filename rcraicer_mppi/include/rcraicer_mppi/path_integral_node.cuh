
#ifndef PATH_INTEGRAL_CUH_
#define PATH_INTEGRAL_CUH_

#include "rclcpp/rclcpp.hpp"
#include "meta_math.h"
#include "param_getter.h"
#include "rcraicer_plant.h"
// #include "../include/rcraicer_mppi/PathIntegralParamsConfig.h"
#include "costs.cuh"

//Including neural net model
#ifdef MPPI_NNET_USING_CONSTANT_MEM__
__device__ __constant__ float NNET_PARAMS[param_counter(6,32,32,4)];
#endif
#include "neural_net_model.cuh"
#include "car_bfs.cuh"
#include "car_kinematics.cuh"
#include "generalized_linear.cuh"
#include "mppi_controller.cuh"
#include "run_control_loop.cuh"

#include <atomic>

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <chrono>
#include <functional>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>



namespace rcraicer_control {

    #ifdef USE_NEURAL_NETWORK_MODEL__ /*Use neural network dynamics model*/
    const int MPPI_NUM_ROLLOUTS__ = 1920;
    const int BLOCKSIZE_X = 8;
    const int BLOCKSIZE_Y = 16;
    typedef NeuralNetModel<7,2,3,6,32,32,4> DynamicsModel;
    #elif USE_BASIS_FUNC_MODEL__ /*Use the basis function model* */
    const int MPPI_NUM_ROLLOUTS__ = 2560;
    const int BLOCKSIZE_X = 16;
    const int BLOCKSIZE_Y = 4;
    typedef GeneralizedLinear<CarBasisFuncs, 7, 2, 25, CarKinematics, 3> DynamicsModel;
    #endif

    //Convenience typedef for the MPPI Controller.
    typedef MPPIController<DynamicsModel, MPPICosts, MPPI_NUM_ROLLOUTS__, BLOCKSIZE_X, BLOCKSIZE_Y> Controller;


    class PathIntegralNode : public rclcpp::Node
    {

        public:
            PathIntegralNode();
            ~PathIntegralNode();

        private:
            RCRaicerPlant* robot;
            Controller* mppi;
            DynamicsModel* model;
            MPPICosts* costs;
            boost::thread optimizer;
            std::atomic<bool> is_alive;

            rcl_interfaces::msg::SetParametersResult paramSetCallback(const std::vector<rclcpp::Parameter>& parameters);
            OnSetParametersCallbackHandle::SharedPtr paramSetCallbackHandler;        

            void setupParams();
            void updateParams();
            void setupSubscribers();
            void setupPublishers();
            void setupTimers();            

            PathIntegralParams config;
            SystemParams params;

            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr  pose_sub_; ///< Subscriber to /pose_estimate.
            rclcpp::Subscription<rcraicer_msgs::msg::SimState>::SharedPtr  sim_sub_; ///< Subscriber to /pose_estimate.
            rclcpp::Subscription<rcraicer_msgs::msg::ChassisState>::SharedPtr  servo_sub_;
            rclcpp::Subscription<rcraicer_msgs::msg::NeuralNetModel>::SharedPtr  model_sub_;

            rclcpp::Publisher<rcraicer_msgs::msg::ChassisCommand>::SharedPtr control_pub_; ///< Publisher of autorally_msgs::chassisCommand type on topic servoCommand.
            rclcpp::Publisher<rcraicer_msgs::msg::PathIntegralStatus>::SharedPtr status_pub_; ///< Publishes the status (0 good, 1 neutral, 2 bad) of the controller
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr subscribed_pose_pub_; ///< Publisher of the subscribed pose
            rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_; ///< Publisher of nav_mags::Path on topic nominalPath.
            rclcpp::Publisher<rcraicer_msgs::msg::PathIntegralTiming>::SharedPtr timing_data_pub_;

            rclcpp::TimerBase::SharedPtr pathTimer_;
            rclcpp::TimerBase::SharedPtr statusTimer_;
            rclcpp::TimerBase::SharedPtr debugImgTimer_;
            rclcpp::TimerBase::SharedPtr timingInfoTimer_;

            bool costParamsUpdated {false};            
          

            void displayDebugImage();

            float init_u[2];
            float exploration_std[2];
    };
}

#endif