#ifndef StateEstimator_H_
#define StateEstimator_H_

#include "rclcpp/rclcpp.hpp"

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/base/timing.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/navigation/GPSFactor.h>

#include "rcraicer_msgs/msg/wheel_speed.hpp"
#include "rcraicer_msgs/msg/state_estimator_status.hpp"
#include "rcraicer_msgs/msg/imu_filter_output.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


#include "visualization_msgs/msg/marker.hpp"
#include "blocking_queue.h"

#include <mutex>
#include <vector>


#define PI 3.14159265358979323846264338

class StateEstimator : public rclcpp::Node 
{
    public:
        StateEstimator();
        ~StateEstimator();

        void GpsCallback(sensor_msgs::msg::NavSatFix::ConstSharedPtr fix);
        void ImuCallback(sensor_msgs::msg::Imu::ConstSharedPtr imu);
        void ImuFilterCallback(rcraicer_msgs::msg::ImuFilterOutput::ConstSharedPtr imuFilter);
        void WheelOdomCallback(nav_msgs::msg::Odometry::ConstSharedPtr odom);        
        void GpsHelper();
        void GpsHelper_1();
        gtsam::BetweenFactor<gtsam::Pose3> integrateWheelOdom(double prevTime, double stopTime, int curFactor);
        void GetAccGyro(sensor_msgs::msg::Imu::ConstSharedPtr imu, gtsam::Vector3 &acc, gtsam::Vector3 &gyro);

        

    private:
        void completeSetup();
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr posePub_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr biasAccPub_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr biasGyroPub_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr timePub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPub_;
        rclcpp::Publisher<rcraicer_msgs::msg::StateEstimatorStatus>::SharedPtr statusPub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gpsPosePub_;

        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gpsSub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
        rclcpp::Subscription<rcraicer_msgs::msg::ImuFilterOutput>::SharedPtr initialPoseSub_;
        

        double lastImuT_, lastImuTgps_;
        unsigned char status_;
        double accelBiasSigma_, gyroBiasSigma_;
        double gpsSigma_;
        int maxQSize_;

        bool debug_;
        int markerId_;

        BlockingQueue<sensor_msgs::msg::NavSatFix::ConstSharedPtr> gpsOptQ_;
        BlockingQueue<sensor_msgs::msg::Imu::ConstSharedPtr> imuOptQ_;
        BlockingQueue<nav_msgs::msg::Odometry::ConstSharedPtr> odomOptQ_;

        std::mutex optimizedStateMutex_;
        gtsam::NavState optimizedState_;
        double optimizedTime_;
        std::shared_ptr<gtsam::PreintegratedImuMeasurements> imuPredictor_;
        double imuDt_;
        gtsam::imuBias::ConstantBias optimizedBias_, previousBias_;
        sensor_msgs::msg::Imu::ConstSharedPtr lastIMU_;
        boost::shared_ptr<gtsam::PreintegrationParams> preintegrationParams_;

        std::list<sensor_msgs::msg::Imu::ConstSharedPtr> imuMeasurements_, imuGrav_;
        rcraicer_msgs::msg::ImuFilterOutput initialPose_;        
        gtsam::Pose3 bodyPSensor_, carENUPcarNED_;
        gtsam::Pose3 imuPgps_;

        bool fixedOrigin_;
        GeographicLib::LocalCartesian enu_;   /// Object to put lat/lon coordinates into local cartesian
        bool gotFirstFix_;
        bool invertx_, inverty_, invertz_;
        bool usingOdom_;
        double maxGPSError_;

        bool gotInitialPose_;

        gtsam::SharedDiagonal priorNoisePose_;
        gtsam::SharedDiagonal priorNoiseVel_;
        gtsam::SharedDiagonal priorNoiseBias_;
        gtsam::Vector noiseModelBetweenBias_sigma_;
        gtsam::ISAM2 *isam_;

      	template<typename T>
			T getsetParameter(std::string paramName, T value);


        nav_msgs::msg::Odometry::ConstSharedPtr lastOdom_;

        std::thread optimizer;

        double gpslat, gpslon, gpsalt, gpsstamp;

        void getRPY(geometry_msgs::msg::Quaternion q, double &roll, double &pitch, double &yaw);

};


#endif /* StateEstimator_H_ */