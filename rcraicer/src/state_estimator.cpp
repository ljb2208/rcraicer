#include "../include/rcraicer/state_estimator.h"
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <exception>
#include <fstream>

using namespace gtsam;
// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;
using symbol_shorthand::G; // GPS pose



StateEstimator::StateEstimator() : Node("state_estimator"),
                                    lastImuT_(0.0),
                                    lastImuTgps_(0.0),
                                    maxQSize_(0),
                                    gpsOptQ_(40),
                                    imuOptQ_(400),
                                    odomOptQ_(100),
                                    gotFirstFix_(false), // should be false 
                                    gotInitialPose_(false),
                                    debug_(false),
                                    markerId_(0)
{

 // temporary variables to retrieve parameters
    double accSigma, gyroSigma, initialVelNoise, initialBiasNoiseAcc, initialBiasNoiseGyro, initialRotationNoise,
        carXAngle, carYAngle, carZAngle, sensorX, sensorY, sensorZ, sensorXAngle, sensorYAngle, sensorZAngle,
        gravityMagnitude;

    initialRotationNoise = getsetParameter<double>("InitialRotationNoise", 1.0);
    initialVelNoise = getsetParameter<double>("InitialVelocityNoise", 0.1);
    initialBiasNoiseAcc = getsetParameter<double>("initialBiasNoiseAcc", 1e-2);
    initialBiasNoiseGyro = getsetParameter<double>("initialBiasNoiseGyro", 1e-2);
    accSigma = getsetParameter<double>("AccelerometerSigma", 6.0e-2);
    gyroSigma = getsetParameter<double>("GyroSigma", 2.0e-2);
    accelBiasSigma_ =  getsetParameter<double>("AccelBiasSigma", 2.0e-4);
    gyroBiasSigma_ = getsetParameter<double>("GyroBiasSigma", 3.0e-5);    
    gpsSigma_ = getsetParameter<double>("GPSSigma", 0.07);
    sensorX = getsetParameter<double>("SensorTransformX", 0.0);
    sensorY = getsetParameter<double>("SensorTransformY", 0.0);
    sensorZ = getsetParameter<double>("SensorTransformZ",  0.0);
    sensorXAngle = getsetParameter<double>("SensorXAngle", 0);
    sensorYAngle = getsetParameter<double>("SensorYAngle", 0);
    sensorZAngle = getsetParameter<double>("SensorZAngle", 0);
    carXAngle = getsetParameter<double>("CarXAngle", 0);
    carYAngle = getsetParameter<double>("CarYAngle", 0);
    carZAngle = getsetParameter<double>("CarZAngle", 0);
    gravityMagnitude =  getsetParameter<double>("Gravity", 9.8);
    invertx_ = getsetParameter<bool>("InvertX", false);
    inverty_ = getsetParameter<bool>("InvertY", false);
    invertz_ = getsetParameter<bool>("InvertZ", false);
    imuDt_ = getsetParameter<double>("Imudt", 1.0/20.0);    // was 200

    debug_ = getsetParameter<bool>("debug", true);


    double gpsx, gpsy, gpsz;
    gpsx = getsetParameter<double>("GPSX", 0);
    gpsy =getsetParameter<double>("GPSY",  0);
    gpsz = getsetParameter<double>("GPSZ", 0);    
    imuPgps_ = Pose3(Rot3(), Point3(gpsx, gpsy, gpsz));
    imuPgps_.print("IMU->GPS");

    bool fixedInitialPose;
    double initialRoll, intialPitch, initialYaw;

    fixedInitialPose = getsetParameter<bool>("FixedInitialPose",  true);
    initialRoll = getsetParameter<double>("initialRoll", 0);
    intialPitch = getsetParameter<double>("intialPitch", 0);
    initialYaw = getsetParameter<double>("initialYaw", 0);

    double latOrigin, lonOrigin, altOrigin;
    fixedOrigin_ = getsetParameter<bool>("FixedOrigin", false);
    latOrigin = getsetParameter<double>("latOrigin", 0);
    lonOrigin = getsetParameter<double>("lonOrigin", 0);
    altOrigin = getsetParameter<double>("altOrigin", 0);    

    usingOdom_ = getsetParameter<bool>("UseOdom", true);
    maxGPSError_ = getsetParameter<double>("MaxGPSError", 10.0);    

    if (fixedOrigin_)
      enu_.Reset(latOrigin, lonOrigin, altOrigin);


    std::cout << "InitialRotationNoise " << initialRotationNoise << "\n"
    << "InitialVelocityNoise " << initialVelNoise << "\n"
    << "InitialBiasNoiseAcc " << initialBiasNoiseAcc << "\n"
    << "InitialBiasNoiseGyro " << initialBiasNoiseGyro << "\n"
    << "AccelerometerSigma " << accSigma << "\n"
    << "GyroSigma " << gyroSigma << "\n"
    << "AccelBiasSigma " << accelBiasSigma_ << "\n"
    << "GyroBiasSigma " << gyroBiasSigma_ << "\n"
    << "GPSSigma " << gpsSigma_ << "\n"
    << "SensorTransformX " << sensorX << "\n"
    << "SensorTransformY " << sensorY << "\n"
    << "SensorTransformZ " << sensorZ << "\n"
    << "SensorXAngle " <<  sensorXAngle << "\n"
    << "SensorYAngle " << sensorYAngle << "\n"
    << "SensorZAngle " <<   sensorZAngle << "\n"
    << "CarXAngle " <<  carXAngle << "\n"
    << "CarYAngle " <<  carYAngle << "\n"
    << "CarZAngle " <<  carZAngle << "\n"
    << "Gravity " <<   gravityMagnitude << "\n";

    // Use an ENU frame
    preintegrationParams_ =  PreintegrationParams::MakeSharedU(gravityMagnitude);
    preintegrationParams_->accelerometerCovariance = accSigma * I_3x3;
    preintegrationParams_->gyroscopeCovariance = gyroSigma * I_3x3;
    preintegrationParams_->integrationCovariance = 1e-5 * I_3x3;

    Vector biases((Vector(6) << 0, 0, 0, 0, 0, 0).finished());
    optimizedBias_ = imuBias::ConstantBias(biases);
    previousBias_ = imuBias::ConstantBias(biases);
    imuPredictor_ = std::make_shared<PreintegratedImuMeasurements>(preintegrationParams_, optimizedBias_);

    optimizedTime_ = 0;

    bodyPSensor_ = Pose3(Rot3::RzRyRx(sensorXAngle, sensorYAngle, sensorZAngle),
        Point3(sensorX, sensorY, sensorZ));
    carENUPcarNED_ = Pose3(Rot3::RzRyRx(carXAngle, carYAngle, carZAngle), Point3());

    bodyPSensor_.print("Body pose\n");
    carENUPcarNED_.print("CarBodyPose\n");    


    ISAM2Params params;
    params.factorization = ISAM2Params::QR;
    isam_ = new ISAM2(params);

    // prior on the first pose
    priorNoisePose_ = noiseModel::Diagonal::Sigmas(
         (Vector(6) << initialRotationNoise, initialRotationNoise, 3*initialRotationNoise,
             gpsSigma_, gpsSigma_, gpsSigma_).finished());

     // Add velocity prior
     priorNoiseVel_ = noiseModel::Diagonal::Sigmas(
         (Vector(3) << initialVelNoise, initialVelNoise, initialVelNoise).finished());

     // Add bias prior
     priorNoiseBias_ = noiseModel::Diagonal::Sigmas(
         (Vector(6) << initialBiasNoiseAcc,
             initialBiasNoiseAcc,
             initialBiasNoiseAcc,
             initialBiasNoiseGyro,
             initialBiasNoiseGyro,
             initialBiasNoiseGyro).finished());

     std::cout<<"checkpoint"<<std::endl;

     Vector sigma_acc_bias_c(3), sigma_gyro_bias_c(3);
     sigma_acc_bias_c << accelBiasSigma_,  accelBiasSigma_,  accelBiasSigma_;
     sigma_gyro_bias_c << gyroBiasSigma_, gyroBiasSigma_, gyroBiasSigma_;
     noiseModelBetweenBias_sigma_ = (Vector(6) << sigma_acc_bias_c, sigma_gyro_bias_c).finished();
    
    if (!fixedInitialPose)
    {
        initialPoseSub_ = this->create_subscription<rcraicer_msgs::msg::ImuFilterOutput>("imu_filter", 1, std::bind(&StateEstimator::ImuFilterCallback, this, std::placeholders::_1));    
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Using fixed initial orientation");
      Rot3 initialRotation = Rot3::Ypr(initialYaw, intialPitch, initialRoll);
      initialPose_.orientation.w = initialRotation.quaternion()[0];
      initialPose_.orientation.x = initialRotation.quaternion()[1];
      initialPose_.orientation.y = initialRotation.quaternion()[2];
      initialPose_.orientation.z = initialRotation.quaternion()[3];
      initialPose_.bias.x = 0;
      initialPose_.bias.y = 0;
      initialPose_.bias.z = 0;

      completeSetup();
    }

    
}


StateEstimator::~StateEstimator()
{
    
}

// called after initial pose received or from constructor if fixedinitialpose is true 
void StateEstimator::completeSetup()
{
    Rot3 initRot(Quaternion(initialPose_.orientation.w, initialPose_.orientation.x, initialPose_.orientation.y,
          initialPose_.orientation.z));

    posePub_ = this->create_publisher<nav_msgs::msg::Odometry>("pose", 1);
    biasAccPub_ = this->create_publisher<geometry_msgs::msg::Point>("bias_acc", 1);
    biasGyroPub_ = this->create_publisher<geometry_msgs::msg::Point>("bias_gyro", 1);
    timePub_ = this->create_publisher<geometry_msgs::msg::Point>("time_delays", 1);
    statusPub_ = this->create_publisher<rcraicer_msgs::msg::StateEstimatorStatus>("status", 1);

    if (debug_)
      markerPub_ = this->create_publisher<visualization_msgs::msg::Marker>("state_estimator_path", 1);


    gpsSub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("rover_navsat_fix", 300, std::bind(&StateEstimator::GpsCallback, this, std::placeholders::_1));
    imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 600, std::bind(&StateEstimator::ImuCallback, this, std::placeholders::_1));
    odomSub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 300, std::bind(&StateEstimator::WheelOdomCallback, this, std::placeholders::_1));
    
    optimizer = std::thread(&StateEstimator::GpsHelper,this);        
}

template<typename T> T StateEstimator::getsetParameter(std::string paramName, T value)
{
    this->declare_parameter<T>(paramName, value);   
    rclcpp::Parameter param = this->get_parameter(paramName);

    return param.get_value<T>();
}

void StateEstimator::ImuFilterCallback(rcraicer_msgs::msg::ImuFilterOutput::ConstSharedPtr imuFilter)
{
    if (!gotInitialPose_)
    {
        initialPose_ = *imuFilter;        
        gotInitialPose_ = true;
        completeSetup();
    }
}

void StateEstimator::GpsCallback(sensor_msgs::msg::NavSatFix::ConstSharedPtr fix)
{
    rclcpp::Time tm = fix->header.stamp;
    std::cout <<"GPS Message recv: " << std::fixed << tm.seconds() << " lat: " << fix->latitude << " lon: " << fix->longitude << " alt: " << fix->altitude << "\r\n";

    if (!gpsOptQ_.pushNonBlocking(fix))
        RCLCPP_WARN(this->get_logger(), "Dropping a GPS measurement due to full queue");
}

void StateEstimator::ImuCallback(sensor_msgs::msg::Imu::ConstSharedPtr imu)
{
    rclcpp::Time ts = imu->header.stamp;

    double roll, pitch, yaw;
    getRPY(imu->orientation, roll, pitch, yaw);

    std::cout <<"IMU Message recv: " << std::fixed << ts.seconds() << " ang vel x: " << imu->angular_velocity.x <<  " y: " << imu->angular_velocity.y << "  z: " << imu->angular_velocity.z;
    std::cout << std::fixed << " lin accel x: " << imu->linear_acceleration.x << " y: " << imu->linear_acceleration.y << " z: " << imu->linear_acceleration.z;
    // std::cout << std::fixed << " orientation x: " << imu->orientation.x << " y: " << imu->orientation.y << " z: " << imu->orientation.z << " w: " << imu->orientation.w << "\r\n";
    std::cout << std::fixed << " roll: " << roll << " pitch: " << pitch << " yaw: " << yaw << "\r\n";

    double dt;
    if (lastImuT_ == 0) dt = 0.005;
    else dt = ts.seconds() - lastImuT_;

    lastImuT_ = ts.seconds();    

    // Push the IMU measurement to the optimization thread
    int qSize = imuOptQ_.size();
    if (qSize > maxQSize_)
      maxQSize_ = qSize;
    if (!imuOptQ_.pushNonBlocking(imu))
      RCLCPP_WARN(this->get_logger(), "Dropping an IMU measurement due to full queue!!");

    // Each time we get an imu measurement, calculate the incremental pose from the last GTSAM pose
    imuMeasurements_.push_back(imu);
    //Grab the most current optimized state
    double optimizedTime;
    NavState optimizedState;
    imuBias::ConstantBias optimizedBias;
    unsigned char status;
    {
      std::scoped_lock guard(optimizedStateMutex_);
      optimizedState = optimizedState_;
      optimizedBias = optimizedBias_;
      optimizedTime = optimizedTime_;
      status = status_;
    }
    if (optimizedTime == 0) return; // haven't optimized first state yet

    bool newMeasurements = false;
    int numImuDiscarded = 0;
    double imuQPrevTime;
    Vector3 acc, gyro;

    if (!imuMeasurements_.empty())
      ts = imuMeasurements_.front()->header.stamp;
      
    while (!imuMeasurements_.empty() && ts.seconds() < optimizedTime)
    {      
      imuQPrevTime = ts.seconds();
      imuMeasurements_.pop_front();
      newMeasurements = true;
      numImuDiscarded++;
    }

    if(newMeasurements)
    {
      //We need to reset integration and iterate through all our IMU measurements
      imuPredictor_->resetIntegration();
      int numMeasurements = 0;
      for (auto it=imuMeasurements_.begin(); it!=imuMeasurements_.end(); ++it)
      {
        ts = (*it)->header.stamp;
        double dt_temp =  ts.seconds() - imuQPrevTime;
        imuQPrevTime = ts.seconds();
        GetAccGyro(*it, acc, gyro);
        imuPredictor_->integrateMeasurement(acc, gyro, dt_temp);
        numMeasurements++;        
      }      
    }
    else
    {
      //Just need to add the newest measurement, no new optimized pose
      GetAccGyro(imu, acc, gyro);
      imuPredictor_->integrateMeasurement(acc, gyro, dt);
      // ROS_INFO("Integrating %f, dt %f", m_lastImuT, dt);
    }

    // predict next state given the imu measurements
    NavState currentPose = imuPredictor_->predict(optimizedState, optimizedBias);
    nav_msgs::msg::Odometry poseNew;
    poseNew.header.stamp = imu->header.stamp;

    Vector4 q = currentPose.quaternion().coeffs();
    poseNew.pose.pose.orientation.x = q[0];
    poseNew.pose.pose.orientation.y = q[1];
    poseNew.pose.pose.orientation.z = q[2];
    poseNew.pose.pose.orientation.w = q[3];

    poseNew.pose.pose.position.x = currentPose.position().x();
    poseNew.pose.pose.position.y = currentPose.position().y();
    poseNew.pose.pose.position.z = currentPose.position().z();

    poseNew.twist.twist.linear.x = currentPose.velocity().x();
    poseNew.twist.twist.linear.y = currentPose.velocity().y();
    poseNew.twist.twist.linear.z = currentPose.velocity().z();
    
    poseNew.twist.twist.angular.x = gyro.x() + optimizedBias.gyroscope().x();
    poseNew.twist.twist.angular.y = gyro.y() + optimizedBias.gyroscope().y();
    poseNew.twist.twist.angular.z = gyro.z() + optimizedBias.gyroscope().z();

    poseNew.child_frame_id = "base_link";
    poseNew.header.frame_id = "odom";

    posePub_->publish(poseNew);

    if (debug_)
    {
        visualization_msgs::msg::Marker dMsg = visualization_msgs::msg::Marker();

        dMsg.header.frame_id = "map";
        dMsg.header.stamp = this->get_clock()->now();
        dMsg.id = markerId_++;
        dMsg.type = visualization_msgs::msg::Marker::SPHERE;
        dMsg.action = visualization_msgs::msg::Marker::ADD;
        dMsg.pose = poseNew.pose.pose;
        dMsg.scale.x = 0.1;
        dMsg.scale.y = 0.1;
        dMsg.scale.z = 0.1;
        dMsg.color.r = 0.0;
        dMsg.color.g = 1.0;
        dMsg.color.b = 0.0;
        dMsg.color.a = 1.0;

        dMsg.frame_locked = false;        

        markerPub_->publish(dMsg);
    }

    //ros::Time after = ros::Time::now();
    geometry_msgs::msg::Point delays;
    ts = imu->header.stamp;
    delays.x = ts.seconds();
    delays.y = (this->get_clock()->now() - imu->header.stamp).seconds();
    delays.z = ts.seconds() - optimizedTime;
    timePub_->publish(delays);

    // publish the status of the estimate - set in the gpsHelper thread
    rcraicer_msgs::msg::StateEstimatorStatus statusMsgs;
    statusMsgs.header.stamp = imu->header.stamp;
    statusMsgs.status = status;
    statusPub_->publish(statusMsgs);
    return;
}

void StateEstimator::WheelOdomCallback(nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
    double roll, pitch, yaw;
    getRPY(odom->pose.pose.orientation, roll, pitch, yaw);

    rclcpp::Time tm = odom->header.stamp;
    std::cout << "ODOM Message recv: " << std::fixed << tm.seconds() << " x: " << odom->pose.pose.position.x << " y: " << odom->pose.pose.position.y << " z: " << odom->pose.pose.position.z;
    std::cout << "roll: " << roll << " pitch: " << pitch << " yaw: " << yaw;
    // std::cout << " Orient x: " << std::fixed << odom->pose.pose.orientation.x << " y: " << odom->pose.pose.orientation.y << " z: " << odom->pose.pose.orientation.z << " w: " << odom->pose.pose.orientation.w;
    std::cout << " Twist lin x: " << std::fixed << odom->twist.twist.linear.x << " ang z: " << odom->twist.twist.angular.z << "\r\n";     

    if (!odomOptQ_.pushNonBlocking(odom))
        RCLCPP_WARN(this->get_logger(), "Dropping a wheel odometry measurement due to full queue");
}

void StateEstimator::getRPY(geometry_msgs::msg::Quaternion q, double &roll, double &pitch, double &yaw)
{
  tf2::Quaternion qrpy(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 mat(qrpy);

  mat.getRPY(roll, pitch, yaw);
}

void StateEstimator::GpsHelper()
{
    rclcpp::Rate loop_rate(10);
    bool gotFirstFix = false;
    double startTime;
    int odomKey = 1;
    int imuKey = 1;
    int latestGPSKey = 0;
    imuBias::ConstantBias prevBias;
    Vector3 prevVel = (Vector(3) << 0.0,0.0,0.0).finished();
    Pose3 prevPose;
    unsigned char status = rcraicer_msgs::msg::StateEstimatorStatus::OK;


    while (rclcpp::ok())
    {
      bool optimize = false;

      if (!gotFirstFix)
      {
        sensor_msgs::msg::NavSatFix::ConstSharedPtr fix = gpsOptQ_.popBlocking();
        rclcpp::Time ts = fix->header.stamp;
        startTime = ts.seconds();
        if(usingOdom_) {
          lastOdom_ = odomOptQ_.popBlocking();
        }

        NonlinearFactorGraph newFactors;
        Values newVariables;
        gotFirstFix = true;

        double E, N, U;
        if (!fixedOrigin_)
        {
          enu_.Reset(fix->latitude, fix->longitude, fix->altitude);
          E = 0; N = 0; U = 0; // we're choosing this as the origin
        }
        else
        {
          // we are given an origin
          enu_.Forward(fix->latitude, fix->longitude, fix->altitude, E, N, U);
        }

        // Add prior factors on pose, vel and bias
        Rot3 initialOrientation = Rot3::Quaternion(initialPose_.orientation.w,
            initialPose_.orientation.x,
            initialPose_.orientation.y,
            initialPose_.orientation.z);
        std::cout << "Initial orientation" << std::endl;
        std::cout << bodyPSensor_.rotation() * initialOrientation * carENUPcarNED_.rotation() << std::endl;
        Pose3 x0(bodyPSensor_.rotation() * initialOrientation * carENUPcarNED_.rotation(),
            Point3(E, N, U));
        prevPose = x0;
        PriorFactor<Pose3> priorPose(X(0), x0, priorNoisePose_);
        newFactors.add(priorPose);
        PriorFactor<Vector3> priorVel(V(0), Vector3(0, 0, 0), priorNoiseVel_);
        newFactors.add(priorVel);
        Vector biases((Vector(6) << 0, 0, 0, initialPose_.bias.x,
            -initialPose_.bias.y, -initialPose_.bias.z).finished());
        prevBias = imuBias::ConstantBias(biases);
        PriorFactor<imuBias::ConstantBias> priorBias(B(0), imuBias::ConstantBias(biases), priorNoiseBias_);
        newFactors.add(priorBias);

        //Factor for imu->gps translation
        BetweenFactor<Pose3> imuPgpsFactor(X(0), G(0), imuPgps_,
            noiseModel::Diagonal::Sigmas((Vector(6) << 0.001,0.001,0.001,0.03,0.03,0.03).finished()));
        newFactors.add(imuPgpsFactor);

        // add prior values on pose, vel and bias
        newVariables.insert(X(0), x0);
        newVariables.insert(V(0), Vector3(0, 0, 0));
        newVariables.insert(B(0), imuBias::ConstantBias(biases));
        newVariables.insert(G(0), x0.compose(imuPgps_));

        isam_->update(newFactors, newVariables);
        //Read IMU measurements up to the first GPS measurement
        lastIMU_ = imuOptQ_.popBlocking();
        //If we only pop one, we need some dt
        rclcpp::Time tsIMU = lastIMU_->header.stamp;
        // lastImuTgps_ = tsIMU.seconds() - 0.005;
        lastImuTgps_ = tsIMU.seconds() - 0.1;

        std::cout.precision(17);
        std::cout << "lastImuTgps_ 1 : " << lastImuTgps_ << " tsIMU: " <<tsIMU.seconds() << "\r\n";
        while(tsIMU.seconds() < ts.seconds())
        {
          lastImuTgps_ = tsIMU.seconds();
          std::cout << "lastImuTgps_ 2 : " << lastImuTgps_ << " tsIMU: " <<tsIMU.seconds() << "\r\n";
          lastIMU_ = imuOptQ_.popBlocking();

          tsIMU = lastIMU_->header.stamp;
        }
        loop_rate.sleep();
      }
      else
      {
        NonlinearFactorGraph newFactors;
        Values newVariables;


        // add IMU measurements
        rclcpp::Time tsIMU;
        
        if (imuOptQ_.size() > 0)
          tsIMU = imuOptQ_.back()->header.stamp;

        while (imuOptQ_.size() > 0 && (tsIMU.seconds() > (startTime + imuKey * 0.1)))
        {
          double curTime = startTime + imuKey * 0.1;
          PreintegratedImuMeasurements pre_int_data(preintegrationParams_, previousBias_);

          rclcpp::Time tsIMU = lastIMU_->header.stamp;
          while(tsIMU.seconds() < curTime)
          {
            Vector3 acc, gyro;
            GetAccGyro(lastIMU_, acc, gyro);
            double imuDT = tsIMU.seconds() - lastImuTgps_;
            lastImuTgps_ = tsIMU.seconds();
            std::cout << "lastImuTgps_ 3 : " << lastImuTgps_ << " tsIMU: " << tsIMU.seconds() << "\r\n";
            std::cout << "Timestep: " << imuDT << "\r\n";
            pre_int_data.integrateMeasurement(acc, gyro, imuDT);
            lastIMU_ = imuOptQ_.popBlocking();
            tsIMU = lastIMU_->header.stamp;
          }
          // adding the integrated IMU measurements to the factor graph
          ImuFactor imuFactor(X(imuKey-1), V(imuKey-1), X(imuKey), V(imuKey), B(imuKey-1), pre_int_data);
          std::cout << "new factors" << "\r\n";
          newFactors.add(imuFactor);
          newFactors.add(BetweenFactor<imuBias::ConstantBias>(B(imuKey-1), B(imuKey), imuBias::ConstantBias(),
              noiseModel::Diagonal::Sigmas( sqrt(pre_int_data.deltaTij()) * noiseModelBetweenBias_sigma_)));
          std::cout << "new factors end" << "\r\n";

          // Predict forward to get an initial estimate for the pose and velocity
          NavState curNavState(prevPose, prevVel);
          std::cout << "predict" << "\r\n";
          NavState nextNavState = pre_int_data.predict(curNavState, prevBias);
          std::cout << "predict end" << "\r\n";
          newVariables.insert(X(imuKey), nextNavState.pose());
          newVariables.insert(V(imuKey), nextNavState.v());
          newVariables.insert(B(imuKey), previousBias_);
          newVariables.insert(G(imuKey), nextNavState.pose().compose(imuPgps_));

          // std::cout << "IMU Key: " << imuKey << " imu ts: " << std::fixed << tsIMU.seconds() << "\r\n";

          std::cout.precision(17);

          std::cout << "IMUKey: " << imuKey << "\r\n";
          std::cout <<  "CNS: " << std::fixed << curNavState << "\r\n";
          std::cout <<  "NNS: " << std::fixed <<nextNavState << "\r\n";


          //  << " Pose: " << nextNavState.pose() << " V: " << nextNavState.v() << "\r\n";
          
          prevPose = nextNavState.pose();
          prevVel = nextNavState.v();
          ++imuKey;
          optimize = true;
        }


        // add GPS measurements that are not ahead of the imu messages
        rclcpp::Time tsGps;

        if (gpsOptQ_.size() > 0)
          tsGps = gpsOptQ_.front()->header.stamp;

        std::cout.precision(17);
        // std::cout << "Optimize: " << optimize << " gps queue size: " << gpsOptQ_.size() << " gps seconds " << std::fixed << tsGps.seconds() << " start time: " << std::fixed << startTime << " imu key: " << imuKey << " calc: " << std::fixed << (startTime + (imuKey-1)*0.1 + 1e-2) << "\r\n";

        while (optimize && gpsOptQ_.size() > 0 && tsGps.seconds() < (startTime + (imuKey-1)*0.1 + 1e-2))
        {
          sensor_msgs::msg::NavSatFix::ConstSharedPtr fix = gpsOptQ_.popBlocking();
          rclcpp::Time tsFix = fix->header.stamp;
          double timeDiff = (tsFix.seconds() - startTime) / 0.1;
          int key = round(timeDiff);

          
          // std::cout << "time diff: " << std::fixed << timeDiff << " key: " << key << " diff: " << std::fixed << std::abs(timeDiff - key) << "\r\n";

          // if (std::abs(timeDiff - key) < 1e-4)          
          if (std::abs(timeDiff - key) < 2e-1 && key < imuKey)          
          {            
            // std::cout << "GPS Message. Last message " << (tsFix.seconds() - gpsstamp) << "\r\n";

            // this is a gps message for a factor
            latestGPSKey = key;
            double E,N,U;
            enu_.Forward(fix->latitude, fix->longitude, fix->altitude, E, N, U);            

            // check if the GPS message is close to our expected position
            Pose3 expectedState;
            if (newVariables.exists(X(key)))
              expectedState = (Pose3) newVariables.at<Pose3>(X(key));
            else                                    
              expectedState = isam_->calculateEstimate<Pose3>(X(key));

            double dist = std::sqrt( std::pow(expectedState.x() - E, 2) + std::pow(expectedState.y() - N, 2) );
            // std::cout <<"GPS Message. Distance: " << std::fixed << dist << "\r\n";
            if (dist < maxGPSError_ || latestGPSKey < imuKey-2)
            {
              SharedDiagonal gpsNoise = noiseModel::Diagonal::Sigmas(Vector3(gpsSigma_, gpsSigma_, 3.0 * gpsSigma_));
              GPSFactor gpsFactor(G(key), Point3(E, N, U), gpsNoise);
              newFactors.add(gpsFactor);
              BetweenFactor<Pose3> imuPgpsFactor(X(key), G(key), imuPgps_,
                  noiseModel::Diagonal::Sigmas((Vector(6) << 0.001,0.001,0.001,0.03,0.03,0.03).finished()));
              newFactors.add(imuPgpsFactor);

              // std::cout << "GPS Key: " << key << " GPS: " << Point3(E, N, U) << "\r\n";

              gpslat = fix->latitude;
              gpslon = fix->longitude;
              gpsalt = fix->altitude;
              gpsstamp = tsFix.seconds();

              if (!usingOdom_)
                odomKey = key+1;
            }
            else
            {
              RCLCPP_WARN(this->get_logger(), "Received bad GPS message. Dist %f IMU Key %i GPS Key %i", dist, imuKey, latestGPSKey);              
              std::cout.precision(17);
              std::cout << "Bad GPS Msg. Lat: " << fix->latitude << " lon: " <<  fix->longitude << " alt: " << fix->altitude << " Stamp: " << tsFix.seconds() << "\r\n";
              std::cout << "Prior GPS Msg. Lat: " << gpslat << " lon: " <<  gpslon << " alt: " << gpsalt << " Stamp: " << gpsstamp << "\r\n";

              GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
              GeographicLib::LocalCartesian proj(gpslat, gpslon, gpsalt, earth);

              double tx,ty,tz;

              proj.Forward(fix->latitude, fix->longitude, fix->altitude, tx, ty, tz);

              double dist = sqrt(tx*tx + ty*ty + tz*tx);

              std::cout << "Distance: " << dist << " x: " << tx << " y: " << ty << " z: " << tz << "\r\n";

              dist = 0;

            }
          }
        }


        // if only using odom with no GPS, then remove old messages from queue
        rclcpp::Time tsOdom;
        
        if (odomOptQ_.size() > 0)
          tsOdom = odomOptQ_.front()->header.stamp;

        while (!usingOdom_ && odomOptQ_.size() > 0 && tsOdom.seconds() < (odomKey*0.1 + startTime))
        {
          lastOdom_ = odomOptQ_.popBlocking();
          tsOdom = lastOdom_->header.stamp;
        }

        // if available, add any odom factors that are not ahead of the imu messages
        if (odomOptQ_.size() > 0)
          tsOdom = odomOptQ_.back()->header.stamp;

        while ((usingOdom_ || latestGPSKey < imuKey-2) && optimize && odomKey < imuKey && odomOptQ_.size() > 0
            && (tsOdom.seconds() > (startTime + odomKey * 0.1)))
        {
          double prevTime = startTime + (odomKey-1) * 0.1;
          newFactors.add(integrateWheelOdom(prevTime, prevTime+0.1, odomKey++));
        }


        // if we processed imu - then we can optimize the state
        if (optimize)
        {
          try
          {
            std::cout << "update" << "\r\n";            
            std::ofstream fg("se.viz", std::ofstream::out);
            newFactors.saveGraph(fg, newVariables);
            fg.close();
            isam_->update(newFactors, newVariables);
            std::cout << "here" << "\r\n";
            Pose3 nextState = isam_->calculateEstimate<Pose3>(X(imuKey-1));

            prevPose = nextState;
            std::cout << "here1" << "\r\n";
            prevVel = isam_->calculateEstimate<Vector3>(V(imuKey-1));
            std::cout << "here2" << "\r\n";
            prevBias = isam_->calculateEstimate<imuBias::ConstantBias>(B(imuKey-1));

            // if we haven't added gps data for 2 message (0.2s) then change status
            if (latestGPSKey + 3 < imuKey)
            {
              status = rcraicer_msgs::msg::StateEstimatorStatus::WARN;              
            }
            else
            {
              status = rcraicer_msgs::msg::StateEstimatorStatus::OK;             
            }

            double curTime = startTime + (imuKey-1) * 0.1;

            {
              std::scoped_lock guard(optimizedStateMutex_);
              optimizedState_ = NavState(prevPose, prevVel);
              optimizedBias_ = prevBias;
              optimizedTime_ = curTime;
              status_ = status;
            }

            nav_msgs::msg::Odometry poseNew;
            poseNew.header.stamp = rclcpp::Time(curTime);

            geometry_msgs::msg::Point ptAcc;
            ptAcc.x = prevBias.vector()[0];
            ptAcc.y = prevBias.vector()[1];
            ptAcc.z = prevBias.vector()[2];

            geometry_msgs::msg::Point ptGyro;
            ptGyro.x = prevBias.vector()[3];
            ptGyro.y = prevBias.vector()[4];
            ptGyro.z = prevBias.vector()[5];

            biasAccPub_->publish(ptAcc);
            biasGyroPub_->publish(ptGyro);
          }
          catch(gtsam::IndeterminantLinearSystemException ex)
          {
            RCLCPP_ERROR(this->get_logger(), "Encountered Indeterminant System Error! %s", ex.what());            
            status = rcraicer_msgs::msg::StateEstimatorStatus::ERROR;
            {
              std::scoped_lock guard(optimizedStateMutex_);
              status_ = status;
            }
          }
        }
        loop_rate.sleep();
      }
    }

    RCLCPP_INFO(this->get_logger(), "Exiting GPS Helper loop");
}

void StateEstimator::GpsHelper_1()
{

}

gtsam::BetweenFactor<gtsam::Pose3> StateEstimator::integrateWheelOdom(double prevTime, double stopTime, int curKey)
{
    double x=0, y=0, theta=0, xVar=0, yVar=0, zVar=0, thetaVariance=0, dt=0, lastTimeUsed=prevTime;

    while (lastTimeUsed != stopTime)
    {
      rclcpp::Time ts = odomOptQ_.front()->header.stamp;
      if (odomOptQ_.size() != 0 && ts.seconds() < stopTime)
      {
        lastOdom_ = odomOptQ_.popBlocking();
        ts = lastOdom_->header.stamp;
        dt = ts.seconds() - lastTimeUsed;
        
        lastTimeUsed = ts.seconds();
      }
      else
      {
        dt = stopTime - lastTimeUsed;
        lastTimeUsed = stopTime;
      }

      // the local frame velocities
      double vx = lastOdom_->twist.twist.linear.x;
      double vy = lastOdom_->twist.twist.linear.y;
      // update the relative position from the initial
      x += vx*dt*cos(theta) - vy*dt*sin(theta);
      y += vx*dt*sin(theta) + vy*dt*cos(theta);
      theta += dt*lastOdom_->twist.twist.angular.z;
      xVar += dt * lastOdom_->twist.covariance[0];
      yVar += dt * lastOdom_->twist.covariance[7];
      zVar += dt * lastOdom_->twist.covariance[14];
      thetaVariance += dt*lastOdom_->twist.covariance[35];
    }

    Pose3 betweenPose = Pose3(Rot3::Rz(theta), Point3(x, y, 0.0));
    return BetweenFactor<Pose3>(X(curKey-1), X(curKey), betweenPose, noiseModel::Diagonal::Sigmas(
          (Vector(6) << thetaVariance*2,thetaVariance*2,thetaVariance,xVar,yVar,zVar).finished()));
}

void StateEstimator::GetAccGyro(sensor_msgs::msg::Imu::ConstSharedPtr imu, gtsam::Vector3 &acc, gtsam::Vector3 &gyro)
{
    double accx, accy, accz;
    if (invertx_) accx = -imu->linear_acceleration.x;
    else accx = imu->linear_acceleration.x;
    if (inverty_) accy = -imu->linear_acceleration.y;
    else accy = imu->linear_acceleration.y;
    if (invertz_) accz = -imu->linear_acceleration.z;
    else accz = imu->linear_acceleration.z;
    acc = Vector3(accx, accy, accz);

    double gx, gy, gz;
    if (invertx_) gx = -imu->angular_velocity.x;
    else gx = imu->angular_velocity.x;
    if (inverty_) gy = -imu->angular_velocity.y;
    else gy = imu->angular_velocity.y;
    if (invertz_) gz = -imu->angular_velocity.z;
    else gz = imu->angular_velocity.z;

    gyro = Vector3(gx, gy, gz);
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateEstimator>());
    rclcpp::shutdown();
    return 0;
}