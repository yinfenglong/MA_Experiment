/*
Yan Li, Uni Stuttgart, Germany
You can contact the author at <yan1.li@web.de>
 */
#include <itm_nonlinear_mpc/sim_only/itm_ukf_observer.h>

namespace quadrotor_control
{
  constexpr int UKFnmpcObserver::kStateSize; //state vector dimension
  constexpr int UKFnmpcObserver::kMeasurementSize;
  constexpr double UKFnmpcObserver::kGravity;
  constexpr int UKFnmpcObserver::kSigmaSize; //Sigma points dimension kSigmaSize= 2 * kStateSize + 1
  // constexpr double UKFnmpcObserver::lambda_;  //Sigma point spreading parameter lambda_=3 - n_x_

UKFnmpcObserver::UKFnmpcObserver(const ros::NodeHandle& nh,
                                 const ros::NodeHandle& private_nh)
        :nh_(nh),
        private_nh_(private_nh),
        observer_nh_(private_nh, "UKF_observer"),
        initialized_(false),
        is_calibrating_(false),
        F_(kStateSize, kStateSize),
        H_(kMeasurementSize, kStateSize),
        calibration_counter_(0)
{
  state_covariance_.setZero();
  process_noise_covariance_.setZero();
  measurement_noise_.setZero();

  // params to angular_acceleration
  I_xx_ = 0.007;
  I_yy_ = 0.007;
  I_zz_ = 0.012;
  I_ww_ = 0.000065;
  b_w_  = 0.051; //friction coefficient
  length_ = 0.17; // robot arm length
  input_moment_coefficient_x_ = 2.533e-05;
  input_moment_coefficient_y_ = 2.533e-05;
  input_moment_coefficient_z_ = 0.001;

  // initialize params to reasonable values
  roll_damping_ = 1.0;
  roll_omega_ = 8.0;
  roll_gain_ = 1.0;

  pitch_damping_ = 1.0;
  pitch_omega_ = 8.0;
  pitch_gain_ = 1.0;

  yaw_damping_ = 1.0;
  yaw_omega_ = 5.0;
  yaw_gain_ = 1.0;


  initialize();

}

// bool UKFnmpcObserver::startCalibrationCallback(std_srvs::Empty::Request& req,
//                                                std_srvs::Empty::Response& res)
// {
//   if (startCalibration())
//   {
//     return true;
//   }
//   ROS_WARN("UKF Calibration Failed...");
//   return false;
// }

bool UKFnmpcObserver::startCalibration()
{
  if(initialized_)
  {
    is_calibrating_ = true;
    forces_offset_.setZero();
    moments_offset_.setZero();
    calibration_counter_ = 0;
    start_calibration_time_ = ros::Time::now();
    return true;
  }
  return false;
}

void UKFnmpcObserver::initialize()
{
  ROS_INFO("start initializing UKF_observer:UKF");
  // service_ = observer_nh_.advertiseService("StartCalibrateUKF",
  //                                          &UKFnmpcObserver::startCalibrationCallback, this);
  // observer_state_pub_ = observer_nh_.advertise<ukf_observer::Observer>(
  // "Observer", 10);

  // dynamic_reconfigure::Server<ma_nmpc_observer::UKFnmpcObserverConfig>::CallbackType f;
  // f = boost::bind(&UKFnmpcObserver::DynConfigCallback, this, _1, _2);// 要绑定的函数的地址,
  // dyn_config_server_.setCallback(f);

  loadROSParams();

  state_.setZero();
  predicted_state_.setZero();
  forces_offset_.setZero();
  moments_offset_.setZero();

  initialized_ = true;
  ROS_INFO("Unscented Kalman Filter Initialized!");

}

void UKFnmpcObserver::loadROSParams()
{
  std::vector<double> temporary_drag;
  std::vector<double> temporary_external_forces_limit, temporary_external_moments_limit;
  std::vector<double> temporary_omega_limit;

  double P0_position, P0_velocity, P0_attitude, P0_angular_velocity, P0_force, P0_torque;
  double q_position, q_velocity, q_attitude, q_angular_velocity, q_force, q_torque;
  double  r_position, r_velocity, r_attitude;

  double calibration_duration;
  if (!observer_nh_.getParam("calibration_duration", calibration_duration))
  {
    ROS_ERROR("calibration_duration in UKF are not loaded from ros parameter server");
    abort();
  }
  calibration_duration_ = ros::Duration(calibration_duration);

  if (!observer_nh_.getParam("drag_coefficients", temporary_drag))
  {
    ROS_ERROR("Drag Coefficients in UKF are not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("roll_omega", roll_omega_))
  {
    ROS_ERROR("roll_omega in UKF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("roll_damping", roll_damping_))
  {
    ROS_ERROR("roll_damping in UKF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("roll_gain", roll_gain_))
  {
    ROS_ERROR("roll_gain in UKF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("pitch_omega", pitch_omega_))
  {
    ROS_ERROR("pitch_omega in UKF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("pitch_damping", pitch_damping_))
  {
    ROS_ERROR("pitch_damping in UKF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("pitch_gain", pitch_gain_))
  {
    ROS_ERROR("pitch_gain in UKF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("yaw_omega", yaw_omega_))
  {
    ROS_ERROR("yaw_omega in UKF is not loaded from ros parameter server");
  }

  if (!observer_nh_.getParam("yaw_damping", yaw_damping_))
  {
    ROS_ERROR("yaw_damping in UKF is not loaded from ros parameter server");
  }

  if (!observer_nh_.getParam("yaw_gain", yaw_gain_))
  {
    ROS_ERROR("yaw_gain in UKF is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("sampling_time", sampling_time_)) {
    ROS_ERROR("sampling_time in UKF is not loaded from ros parameter server");
    abort();
  }

   if (!observer_nh_.getParam("q_position", q_position)) {
    ROS_ERROR("q_position in KF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("q_velocity", q_velocity)) {
    ROS_ERROR("q_velocity in KF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("q_attitude", q_attitude)) {
    ROS_ERROR("q_attitude in KF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("q_angular_velocity", q_angular_velocity)) {
    ROS_ERROR("q_angular_velocity in KF is not loaded from ros parameter server");
    abort();
  }

 if (!observer_nh_.getParam("q_force", q_force)) {
    ROS_ERROR("q_force in KF is not loaded from ros parameter server");
    abort();
  }

 if (!observer_nh_.getParam("q_torque", q_torque)) {
    ROS_ERROR("q_torque in KF is not loaded from ros parameter server");
    abort();
  }

 if (!observer_nh_.getParam("r_position", r_position)) {
    ROS_ERROR("r_position in KF is not loaded from ros parameter server");
    abort();
  }

 if (!observer_nh_.getParam("r_velocity", r_velocity)) {
    ROS_ERROR("r_velocity in KF is not loaded from ros parameter server");
    abort();
  }

 if (!observer_nh_.getParam("r_attitude", r_attitude)) {
    ROS_ERROR("r_attitude  in KF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("external_forces_limit", temporary_external_forces_limit)) {
    ROS_ERROR("external_forces_limit in KF is not loaded from ros parameter server");
    abort();
  }
  if (!observer_nh_.getParam("external_moments_limit", temporary_external_moments_limit)) {
    ROS_ERROR("external_moments_limit in KF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("omega_limit", temporary_omega_limit)) {
    ROS_ERROR("omega_limit in UKF is not loaded from ros parameter server");
    abort();
  }


  // Eigen::Matrix<double, 18, 18>  F_continous_time;
  // F_continous_time.setZero();
  // F_continous_time.block<3, 3>(0, 3) = Eigen::MatrixXd::Identity(3, 3);
  // F_continous_time.block<3, 3>(3, 3) = -1.0
  //     * Eigen::DiagonalMatrix<double, 3>(temporary_drag.at(0), temporary_drag.at(1),
  //                                        temporary_drag.at(2));
  // F_continous_time.block<3, 3>(3, 12) = Eigen::MatrixXd::Identity(3, 3);
  // F_continous_time.block<3, 3>(6, 9) = Eigen::MatrixXd::Identity(3, 3);
  // F_continous_time.block<3, 3>(9, 6) = -1.0
  //     * Eigen::DiagonalMatrix<double, 3>(roll_omega_ * roll_omega_, pitch_omega_ * pitch_omega_,
  //                                        yaw_omega_ * yaw_omega_);
  // F_continous_time.block<3, 3>(9, 9) = -2.0
  //     * Eigen::DiagonalMatrix<double, 3>(roll_omega_ * roll_damping_, pitch_omega_ * pitch_damping_,
  //                                        yaw_omega_ * yaw_damping_);
  // F_continous_time.block<3, 3>(9, 15) = Eigen::MatrixXd::Identity(3, 3);

  // ROS_INFO_STREAM("F_C \n"<<F_continous_time);

  // F_ =  (sampling_time_* F_continous_time).exp().sparseView();

  // ROS_INFO_STREAM("F_ :\n"<<F_);

  // First 9x9 (=measurement size) block is identity, rest is zero.
  H_.reserve(kMeasurementSize);
  for (int i = 0; i < kMeasurementSize; ++i)
  {
    H_.insert(i, i) = 1.0;
  }

  if (!observer_nh_.getParam("P0_position", P0_position)) {
    ROS_ERROR("P0_position in UKF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("P0_velocity", P0_velocity)) {
    ROS_ERROR("P0_velocity in UKF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("P0_attitude", P0_attitude)) {
    ROS_ERROR("P0_attitude in UKF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("P0_angular_velocity", P0_angular_velocity)) {
    ROS_ERROR("P0_angular_velocity in UKF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("P0_force", P0_force)) {
    ROS_ERROR("P0_force in UKF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("P0_torque", P0_torque)) {
    ROS_ERROR("P0_torque in UKF is not loaded from ros parameter server");
    abort();
  }

  for (int i = 0; i < 3; i++)
  {
    initial_state_covariance_(i) = P0_position;
    initial_state_covariance_(i + 3) = P0_velocity;
    initial_state_covariance_(i + 6) = P0_attitude;
    initial_state_covariance_(i + 9) = P0_angular_velocity;
    initial_state_covariance_(i + 12) = P0_force;
    initial_state_covariance_(i + 15) = P0_torque;
  }
  state_covariance_ = initial_state_covariance_.asDiagonal();

  for (int i = 0; i < 3; i++)
  {
    initial_measurement_noise_(i) = r_position;
    initial_measurement_noise_(i + 3) = r_velocity;
    initial_measurement_noise_(i + 6) = r_attitude;
  }
  measurement_noise_ = initial_measurement_noise_.asDiagonal();

  for (size_t i = 0; i < 3; i++)
  {
    initial_process_noise_covariance_(i) = q_position;
    initial_process_noise_covariance_(i + 3) = q_velocity;
    initial_process_noise_covariance_(i + 6) = q_attitude;
    initial_process_noise_covariance_(i + 9) = q_angular_velocity;
    initial_process_noise_covariance_(i + 12) = q_force;
    initial_process_noise_covariance_(i + 15) = q_torque;
  }
  process_noise_covariance_ = initial_process_noise_covariance_.asDiagonal();

  Eigen::Map<Eigen::Vector3d> omega_limit_map(temporary_omega_limit.data(), 3, 1);
  omega_limit_ = omega_limit_map;

  Eigen::Map<Eigen::Vector3d> external_forces_limit_map(temporary_external_forces_limit.data(), 3,
                                                        1);
  external_forces_limit_ = external_forces_limit_map;

  Eigen::Map<Eigen::Vector3d> external_moments_limit_map(temporary_external_moments_limit.data(), 3,
                                                         1);
  external_moments_limit_ = external_moments_limit_map;

  drag_coefficients_matrix_.setZero();
  for (int i = 0; i < 3; i++)
  {
    drag_coefficients_matrix_(i, i) = temporary_drag.at(i);
  }

  // F_.makeCompressed();

  ROS_INFO_ONCE("UKF parameters loaded successfully");
}

void UKFnmpcObserver::feedPositionMeasurement(const Eigen::Vector3d& position)
{
  this->measurements_(0) = position(0);
  this->measurements_(1) = position(1);
  this->measurements_(2) = position(2);
}

void UKFnmpcObserver::feedVelocityMeasurement(const Eigen::Vector3d& velocity)
{
  this->measurements_(3) = velocity(0);
  this->measurements_(4) = velocity(1);
  this->measurements_(5) = velocity(2);
}

void UKFnmpcObserver::feedRotationMatrix(const Eigen::Matrix3d& rotation_matrix)
{
  this->rotation_matrix_ = rotation_matrix;
  this->measurements_(6) = limit_angle(atan2((double) rotation_matrix(2, 1), (double) rotation_matrix(2, 2)));
  this->measurements_(7) = limit_angle(-asin((double) rotation_matrix(2, 0)));
  this->measurements_(8) = limit_angle(atan2((double) rotation_matrix(1, 0), (double) rotation_matrix(0, 0)));
}

/**
 * control inpurs: roll_cmd, pitch_cmd, thrust_cmd
 */
void UKFnmpcObserver::feedAttitudeCommand(const Eigen::Vector4d& roll_pitch_yaw_thrust_cmd)
{
  this->roll_pitch_yaw_thrust_cmd_ = roll_pitch_yaw_thrust_cmd;
}

void UKFnmpcObserver::reset(const Eigen::Vector3d& initial_position,
                            const Eigen::Vector3d& initial_velocity,
                            const Eigen::Vector3d& initial_attitude,
                            const Eigen::Vector3d& initial_angular_rate,
                            const Eigen::Vector3d& initial_external_forces,
                            const Eigen::Vector3d& initial_external_moments)
{

  state_covariance_ = initial_state_covariance_.asDiagonal();

  state_.setZero();
  state_.segment(0, 3) = initial_position;
  state_.segment(3, 3) = initial_velocity;
  state_.segment(6, 3) = initial_attitude;
  state_.segment(9, 3) = initial_angular_rate;
  state_.segment(12, 3) = initial_external_forces;
  state_.segment(15, 3) = initial_external_moments;
}


void UKFnmpcObserver::systemDynamics(double dt)
{
  /* 1. Computing Weights of Sigma Points
   * creat Weights of sigma points weights_ and weights_c*/
  double alpha_weights = 1e-1; // tunable (1e-4 to 1) first state Xsig_col(0)
  double beta_weights = 2;  // tunable
  double k_weights = 3-kStateSize;  // 0/3-n
  lambda_ = alpha_weights * alpha_weights * (kStateSize + k_weights) - kStateSize;
  weights_(0) = lambda_ / (lambda_ + kStateSize);

	for (int i = 1; i < kSigmaSize; i++)
	{
		weights_(i) = .5 / (lambda_ + kStateSize);
	}
  weights_c_ = weights_;
  weights_c_(0) += (1 - alpha_weights*alpha_weights + beta_weights);

  /* 2.Generate sigma points
  * create sigma point matrix Xsig_
  * Eigen::MatrixXd Xsig_ = Eigen::MatrixXd(kStateSize, kSigmaSize);*/
  Eigen::Matrix<double, kStateSize, kStateSize> A_ = state_covariance_.llt().matrixL();

  /*sigma point matrix Xsig_ */
  Xsig_.col(0) = state_;
  for (int i = 0; i < kStateSize; i++)
  {
    // Xsig_.col( i + 1 ) = state_ + sqrt(lambda_ + kStateSize) * A_.col(i);
    // Xsig_.col( i + 1 + kStateSize ) = state_ - sqrt(lambda_ + kStateSize) * A_.col(i);
    Xsig_.col( i + 1 ) = state_ + sqrt(lambda_ + kStateSize) * A_.col(i);
    Xsig_.col( i + 1 + kStateSize ) = state_ - sqrt(lambda_ + kStateSize) * A_.col(i);
  }


  /* 3. Computing Mean and Covariance of the approximate Gaussian
   * Predict Sigma Points*/
    Eigen::Vector3d old_position;// state_.segment(0, 3);
    Eigen::Vector3d old_velocity;// state_.segment(3, 3);
    Eigen::Vector3d old_attitude;// state_.segment(6, 3);
    Eigen::Vector3d old_omega;// state_.segment(9, 3);
    Eigen::Vector3d old_external_forces;// state_.segment(12, 3);
    Eigen::Vector3d old_external_moments;// state_.segment(15, 3);

      for (int i = 0; i< kSigmaSize; i++)
    {
    /* extract values for better readability*/
    old_position(0) = Xsig_(0,i);
    old_position(1) = Xsig_(1,i);
    old_position(2) = Xsig_(2,i);
    old_velocity(0) = Xsig_(3,i);
    old_velocity(1) = Xsig_(4,i);
    old_velocity(2) = Xsig_(5,i);
    old_attitude(0) = Xsig_(6,i);
    old_attitude(1) = Xsig_(7,i);
    old_attitude(2) = Xsig_(8,i);
    old_omega(0)    = Xsig_(9,i);
    old_omega(1)    = Xsig_(10,i);
    old_omega(2)    = Xsig_(11,i);
    old_external_forces(0) = Xsig_(12,i);
    old_external_forces(1) = Xsig_(13,i);
    old_external_forces(2) = Xsig_(14,i);
    old_external_moments(0) = Xsig_(15,i);
    old_external_moments(1) = Xsig_(16,i);
    old_external_moments(2) = Xsig_(17,i);

    /* predicted state values */
    // Eigen::Vector3d new_position, new_velocity, new_attitude, new_omega;

    const Eigen::Vector3d thrust(0.0, 0.0, this->roll_pitch_yaw_thrust_cmd_(3));

    const Eigen::Vector3d acceleration = rotation_matrix_ * thrust + Eigen::Vector3d(0, 0, -kGravity)
    + this->drag_coefficients_matrix_ * old_velocity + old_external_forces;

    const Eigen::Vector3d new_velocity = old_velocity + acceleration * dt;

    const Eigen::Vector3d new_position = old_position + old_velocity * dt
    + 0.5 * acceleration * dt * dt;

    /** input torques */
    // Eigen::Vector3d input_torques_;
    // double kThrust = 8.54858e-06; //motor constant
    // double kTorque = 0.016; //moment constant
    // input_torques_(0) = (kThrust/I_xx_)*length_*(motorspeed_(1) * motorspeed_(1) - motorspeed_(3)*motorspeed_(3));
    // input_torques_(1) = (kThrust/I_yy_)*length_*(motorspeed_(0) * motorspeed_(0) - motorspeed_(2)*motorspeed_(2));
    // input_torques_(2) = (kThrust * kTorque)/(I_zz_) * (motorspeed_(0) * motorspeed_(0)+ motorspeed_(2)*motorspeed_(2)
    //                                         -motorspeed_(1) * motorspeed_(1)-motorspeed_(3)*motorspeed_(3));

    // input_torques_(0) = input_moment_coefficient_x_ * length_ * (motorspeed_(3) * motorspeed_(3) - motorspeed_(1)*motorspeed_(1));
    // input_torques_(1) = input_moment_coefficient_y_ * length_ * (motorspeed_(2) * motorspeed_(2) - motorspeed_(0)*motorspeed_(0));
    // input_torques_(2) = input_moment_coefficient_z_ *(motorspeed_(3) * motorspeed_(3)+ motorspeed_(1)*motorspeed_(1)
    //                                                   -motorspeed_(2) * motorspeed_(2)-motorspeed_(0)*motorspeed_(0));

    /** angular_acceleration part1  */
    // Eigen::Vector3d angular_acceleration_prart1_;
    // // angular_acceleration_prart1_(0) = ((I_yy_ - I_zz_)/I_xx_)*(old_omega(1)*old_omega(2));
    // // angular_acceleration_prart1_(1) = ((I_zz_ - I_xx_)/I_yy_)*(old_omega(0)*old_omega(2));
    // // angular_acceleration_prart1_(2) = ((I_xx_ - I_yy_)/I_zz_)*(old_omega(0)*old_omega(1));
    // angular_acceleration_prart1_(0) = 1/I_xx_ * (input_torques_(0) - old_omega(1) * old_omega(2) * (I_zz_ - I_yy_));
    // angular_acceleration_prart1_(1) = 1/I_yy_ * (input_torques_(1) - old_omega(0) * old_omega(2) * (I_xx_ - I_zz_));
    // angular_acceleration_prart1_(2) = 1/I_zz_ * (input_torques_(2) - old_omega(0) * old_omega(1) * (I_yy_ - I_xx_));

    /** angular_acceleration part2  */
    // Eigen::Vector3d angular_acceleration_prart2_;
    // angular_acceleration_prart2_(0) = -old_omega(1) * (I_ww_ / I_xx_) * (motorspeed_(0) + motorspeed_(1) + motorspeed_(2) + motorspeed_(3));
    // angular_acceleration_prart2_(1) = -old_omega(0) * (I_ww_ / I_yy_) * (motorspeed_(0) + motorspeed_(1) + motorspeed_(2) + motorspeed_(3));
    // angular_acceleration_prart2_(2) = 0 ;
    // /** angular_acceleration part3  */
    // Eigen::Vector3d angular_acceleration_prart3_;
    // angular_acceleration_prart3_(0) = (-b_w_/I_xx_)*(sign(old_omega(0)))*(old_omega(0)*old_omega(0));
    // angular_acceleration_prart3_(1) = (-b_w_/I_yy_)*(sign(old_omega(1)))*(old_omega(1)*old_omega(1));
    // angular_acceleration_prart3_(2) = (-b_w_/I_zz_)*(sign(old_omega(2)))*(old_omega(2)*old_omega(2));


   /**Transformation of rate the rate of change of
    * the Euler angles into the body-referenced angular velocity vector ωB */
  //  Eigen::Matrix<double, 3, 3> rotation_matrix_angular_;
  //  rotation_matrix_angular_(0, 0) = 1;
  //  rotation_matrix_angular_(0, 1) = sin(old_omega(0))*tan(old_omega(1));
  //  rotation_matrix_angular_(0, 2) = cos(old_omega(0))*tan(old_omega(1));
  //  rotation_matrix_angular_(1, 0) = 0;
  //  rotation_matrix_angular_(1, 1) = cos(old_omega(0));
  //  rotation_matrix_angular_(1, 2) = -sin(old_omega(0));
  //  rotation_matrix_angular_(2, 0) = 0;
  //  rotation_matrix_angular_(2, 1) = sin(old_omega(0))*cos(old_omega(1));
  //  rotation_matrix_angular_(2, 2) = cos(old_omega(0))/cos(old_omega(1));

    /** angular_acceleration */
    // Eigen::Vector3d angular_acceleration_;
    // angular_acceleration_ = (rotation_matrix_angular_)*(input_torques_ + angular_acceleration_prart1_)+ old_external_moments;
    // angular_acceleration_ = (rotation_matrix_)*(input_torques_ + angular_acceleration_prart1_)+ old_external_moments;
    // angular_acceleration_ = (rotation_matrix_)*(angular_acceleration_prart1_ + angular_acceleration_prart2_+angular_acceleration_prart3_) + old_external_moments;


    // /** angular_acceleration */
    Eigen::Vector3d angular_acceleration_;
    angular_acceleration_(0) = -2.0 * roll_damping_ * roll_omega_ * old_omega(0)
      - roll_omega_ * roll_omega_ * old_attitude(0)
      + roll_gain_ * roll_omega_ * roll_omega_ * roll_pitch_yaw_thrust_cmd_(0)
      + old_external_moments(0);

    angular_acceleration_(1) = -2.0 * pitch_damping_ * pitch_omega_ * old_omega(1)
        - pitch_omega_ * pitch_omega_ * old_attitude(1)
        + pitch_gain_ * pitch_omega_ * pitch_omega_ * roll_pitch_yaw_thrust_cmd_(1)
        + old_external_moments(1);

    angular_acceleration_(2) = -2.0 * yaw_damping_ * yaw_omega_ * old_omega(2)
        - yaw_omega_ * yaw_omega_ * old_attitude(2)
        + yaw_gain_ * yaw_omega_ * yaw_omega_ * roll_pitch_yaw_thrust_cmd_(2)
       + old_external_moments(2);

    /*publish angular_acceleration*/
    // geometry_msgs::Pose angular_acceleration;
    // angular_acceleration.position.x = angular_acceleration_(0);
    // angular_acceleration.position.y = angular_acceleration_(1);
    // angular_acceleration.position.z = angular_acceleration_(2);
    // angular_acceleration_pub_.publish(angular_acceleration);


    const Eigen::Vector3d new_omega = old_omega+ angular_acceleration_ * dt;
    const Eigen::Vector3d new_attitude = old_attitude + old_omega * dt+ 0.5 * angular_acceleration_ * dt * dt;

    const Eigen::Vector3d new_external_forces = old_external_forces;
    const Eigen::Vector3d new_external_moments = old_external_moments;


    /* write predicted sigma point into right column*/
    Xsig_pred_(0,i) = new_position(0);
    Xsig_pred_(1,i) = new_position(1);
    Xsig_pred_(2,i) = new_position(2);
    Xsig_pred_(3,i) = new_velocity(0);
    Xsig_pred_(4,i) = new_velocity(1);
    Xsig_pred_(5,i) = new_velocity(2);
    Xsig_pred_(6,i) = new_attitude(0);
    Xsig_pred_(7,i) = new_attitude(1);
    Xsig_pred_(8,i) = new_attitude(2);
    Xsig_pred_(9,i) = new_omega(0);
    Xsig_pred_(10,i) = new_omega(1);
    Xsig_pred_(11,i) = new_omega(2);
    Xsig_pred_(12,i) = new_external_forces(0);
    Xsig_pred_(13,i) = new_external_forces(1);
    Xsig_pred_(14,i) = new_external_forces(2);
    Xsig_pred_(15,i) = new_external_moments(0);
    Xsig_pred_(16,i) = new_external_moments(1);
    Xsig_pred_(17,i) = new_external_moments(2);
  }

  /** computing approximated mean and
   * convariance of the postensor sigma points */
  /** approximated mean  state_mean_pred_*/
  state_mean_pred_.fill(0.0);
  for (int i = 0; i < kSigmaSize; i++)
  {
    state_mean_pred_= state_mean_pred_ + weights_(i)* Xsig_pred_.col(i);
  }

  /** approximated mean  state_covariance_*/
  // for (int i = 0; i < kSigmaSize; i++)
  for (int i = 1; i < kSigmaSize; i++)
  {
    Eigen::VectorXd x_diff_ = Xsig_pred_.col(i) - state_mean_pred_;
    state_covariance_ = state_covariance_+ weights_c_(i) * x_diff_ * x_diff_.transpose();
  }
  state_covariance_ = state_covariance_ + process_noise_covariance_; // ??maybe without process_noise_covariance_

  Zsig_ = H_ * Xsig_pred_;
  z_mean_.fill(0.0);
  for (int i = 0; i < kSigmaSize; i++)
  {
    	z_mean_ = z_mean_ + weights_(i) * Zsig_.col(i);
  }


}//systemdynamics()

bool UKFnmpcObserver::updateEstimator()
{
  if (initialized_ == false)
    return false;
  ROS_INFO_ONCE("UKF is updated for first time.");
  static ros::Time t_previous = ros::Time::now();
  static bool do_once = true;
  double dt;

  if (do_once)
  {
    dt = 0.033;
    do_once = false;
  }
  else
  {
    ros::Time t0 = ros::Time::now();
    dt = (t0 - t_previous).toSec();
    t_previous = t0;
  }
  //check that dt is not so different from 0.01
  if (dt > 0.05)
  {
    dt = 0.05;
  }
  if (dt < 0.016)
  {
    dt = 0.016;
  }

  // state_covariance_ = F_ * state_covariance_ * F_.transpose();
  // state_covariance_+= process_noise_covariance_;

  /* predict state, predict state covariance */
  systemDynamics(dt);

  /* 4. update
   * predicted state to the measurement state
   * transform sigma points into measurement space Zsig_
   * Eigen::MatrixXd Zsig_ = Eigen::MatrixXd(kMeasurementSize, kSigmaSize);*/
  /* creat sigma points Xsig_update */
  /*TODO*/
  // Eigen::Matrix<double, kStateSize, kStateSize> A_update_ = state_covariance_.llt().matrixL();
  // Xsig_update_.col(0) = state_mean_pred_;
  // for (int i = 0; i < kStateSize; i++)
  // {
  //   Xsig_update_.col( i + 1 ) = state_mean_pred_ + sqrt(lambda_ + kStateSize) * A_update_.col(i);
  //   Xsig_update_.col( i + 1 + kStateSize ) = state_mean_pred_ - sqrt(lambda_ + kStateSize) * A_update_.col(i);
  // }
  /*TODO*/

  // /** computing eigenvalues of state_covariance_*/
  // Eigen::EigenSolver<Eigen::MatrixXd> eigen_solver(state_covariance_);
  // ROS_INFO_STREAM("eigenvalues state_covariance_: \n" << eigen_solver.eigenvalues());
  /*TODO*/

  /*TODO*/
  // Zsig_ = H_ * Xsig_update_;
  // Zsig_ = H_ * Xsig_pred_;
 /*TODO*/

  /* Mean in measurement space z_mean_
  Eigen::VectorXd z_mean_ = Eigen::VectorXd(kMeasurementSize);*/

  /*TODO*/
  // z_mean_.fill(0.0);
  // for (int i = 0; i < kSigmaSize; i++)
  // {
  //   	z_mean_ = z_mean_ + weights_(i) * Zsig_.col(i);
  // }
/*TODO*/

  /* Covariance in measurement space S_*/
  // Eigen::MatrixXd S_ = Eigen::MatrixXd(kMeasurementSize, kMeasurementSize);
  S_.fill(0.0);
  Eigen::MatrixXd z_diff_;
  // for (int i = 0; i < kSigmaSize; i++)
  for (int i = 1; i < kSigmaSize; i++)
  {
    z_diff_ = Zsig_.col(i) - z_mean_;
    S_ = S_ + weights_c_(i) * z_diff_ * z_diff_.transpose();
  }
  /* add measurement noise covariance matrix*/
  S_ = S_ + measurement_noise_;


  /* 5. Update state
   * create matrix for cross correlation Tc
   Eigen::MatrixXd Tc_ = Eigen::MatrixXd(kStateSize, kMeasurementSize);*/
  Tc_.fill(0.0);
  Eigen::MatrixXd X_diff_;
  // for (int i = 0; i < kSigmaSize; i++)
  for (int i = 1; i < kSigmaSize; i++)
  {
    /*TODO*/
    // X_diff_ = Xsig_update_.col(i) - state_mean_pred_;
    X_diff_ = Xsig_pred_.col(i) - state_mean_pred_;
    z_diff_ = Zsig_.col(i) - z_mean_;

    Tc_ = Tc_ + weights_c_(i) * X_diff_* z_diff_.transpose();
  }

  /* Kalman gain K */
	// K_ = Tc_ * Tc_.inverse();
  K_ = Tc_ * S_.inverse(); //08082019

  /* new measurement */
	Eigen::MatrixXd z_diff = measurements_ - z_mean_;

  /* update state mean and covariance matrix */
  state_ = state_mean_pred_ + K_ * z_diff;
  state_covariance_ = state_covariance_ - K_ * S_ * K_.transpose();
  // ROS_INFO_STREAM("UKF_state_: \n" << state_);

  /* Limits on estimated_disturbances */
  if (state_.allFinite() == false) {
    ROS_ERROR("The estimated state in UKF Disturbance Observer has a non-finite element");
    return false;
  }

  Eigen::Vector3d omega = state_.segment(9, 3);
  omega = omega.cwiseMax(-omega_limit_);
  omega = omega.cwiseMin(omega_limit_);
  state_.segment(9, 3) << omega;

  Eigen::Vector3d external_forces = state_.segment(12, 3);
  Eigen::Vector3d external_moments = state_.segment(15, 3);
  external_forces = external_forces.cwiseMax(-external_forces_limit_);
  external_forces = external_forces.cwiseMin(external_forces_limit_);

  external_moments = external_moments.cwiseMax(-external_moments_limit_);
  external_moments = external_moments.cwiseMin(external_moments_limit_);

  state_.segment(12, 6) << external_forces, external_moments;

  // if (is_calibrating_ == true) {
  //   ROS_INFO_THROTTLE(1.0, "calibrating UKF...");
  //   forces_offset_ += external_forces;
  //   moments_offset_ += external_moments;
  //   calibration_counter_++;

  // if ((ros::Time::now() - start_calibration_time_) > calibration_duration_) {
  //   is_calibrating_ = false;
  //   forces_offset_ = forces_offset_ / calibration_counter_;
  //   moments_offset_ = moments_offset_ / calibration_counter_;
  //   calibration_counter_ = 0;
  //   ROS_INFO("Calibration finished");
  //   }
  // }

  // if (observer_state_pub_.getNumSubscribers() > 0)
  // {
  //   ukf_observer::ObserverPtr msg(new ukf_observer::Observer);
  //   msg->header.stamp = ros::Time::now();
  //   for (int i = 0; i < 3; i++)
  //   {
  //     msg->position[i] = state_(i);
  //     msg->velocity[i] = state_(i + 3);
  //     msg->attitude[i] = state_(i + 6);
  //     msg->angular_velocity[i] = state_(i + 9);
  //     msg->external_forces[i] = state_(i + 12);
  //     msg->external_moments[i] = state_(i + 15);
  //     msg->forces_offset[i] = forces_offset_(i);
  //     msg->moments_offset[i] = moments_offset_(i);
  //   }
  //   observer_state_pub_.publish(msg);
  // }
  return true;
}

void UKFnmpcObserver::getEstimatedState(Eigen::VectorXd* estimated_state) const
{
  assert(estimated_state);
  assert(initialized_);

  estimated_state->resize(kStateSize);
  *estimated_state = this->state_;
  ROS_INFO_STREAM("Estimated state\n"<<state_);
}

int UKFnmpcObserver::sign(double x)
  {
    if (x-0.0 < 1e-5){
      return -1;
    }
    else{
      return 1;
    }
  }

double UKFnmpcObserver::limit_angle(double input)
{
  if (input - M_PI > 1e-5) input-=2.*M_PI;
  else if (1e-5 < -M_PI - input) input+=2.*M_PI;
  else
    return input;
  return input;
}

UKFnmpcObserver::~UKFnmpcObserver()
{
}

// void UKFnmpcObserver::DynConfigCallback(
//     ma_nmpc_observer::UKFObserverConfig &config, uint32_t level)
// {
//   if (config.calibrate == true)
//   {
//     startCalibration();
//     config.calibrate = false;
//   }
  // for (size_t i = 0; i < 3; i++)
  // {
  //   process_noise_covariance_(i) = config.q_position;
  //   process_noise_covariance_(i + 3) = config.q_velocity;
  //   process_noise_covariance_(i + 6) = config.q_attitude;
  //   process_noise_covariance_(i + 9) = config.q_angular_velocity;
  //   // process_noise_covariance_(i + 12) = config.q_force;
  //   // process_noise_covariance_(i + 15) = config.q_torque;

  //   measurement_covariance_(i) = config.r_position;
  //   measurement_covariance_(i + 3) = config.r_velocity;
  //   measurement_covariance_(i + 6) = config.r_attitude;
  // }

//   ROS_INFO("ma_nmpc_observer:UKF dynamic config is called successfully");
// }
}//namespace