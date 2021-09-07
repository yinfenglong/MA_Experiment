/*
name: itm_nonlinear_mpc.h
Yan Li, Uni Stuttgart, Germany
You can contact the author at <yan1.li@web.de>

NMPC: Non Linear Model Predictive Control
 */

#ifndef KFDisturbanceObserver_H_
#define KFDisturbanceObserver_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
// #include <itm_nonlinear_mpc/itm_kf_observer.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>

//dynamic reconfigure
// #include <dynamic_reconfigure/server.h>
// #include <itm_nonlinear_mpc/KFDisturbanceObserverConfig.h>

namespace quadrotor_control {
class KFDisturbanceObserver
{
 public:

  KFDisturbanceObserver(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
  void reset(const Eigen::Vector3d& initial_position, const Eigen::Vector3d& initial_velocity,
             const Eigen::Vector3d& initial_attitude, const Eigen::Vector3d& initial_angular_rate,
             const Eigen::Vector3d& initial_external_forces,
             const Eigen::Vector3d& initial_external_moments);

  //Getters
  Eigen::Vector3d getEstimatedPosition() const
  {
    if (initialized_)
      return state_.segment(0, 3);
    else
      return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d getEstimatedVelocity() const
  {
    if (initialized_)
      return state_.segment(3, 3);
    else
      return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d getEstimatedAttitude() const
  {
    if (initialized_)
      return state_.segment(6, 3);
    else
      return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d getEstimatedAngularVelocity() const
  {
    if (initialized_)
      return state_.segment(9, 3);
    else
      return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d getEstimatedExternalForces() const
  {
    if (initialized_ == true && is_calibrating_ == false)
      return state_.segment(12, 3) - forces_offset_;
    else
      return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d getEstimatedExternalMoments() const
  {
    if (initialized_ && is_calibrating_ == false)
      return state_.segment(15, 3) - moments_offset_;
    else
      return Eigen::Vector3d::Zero();
  }

  void getEstimatedState(Eigen::VectorXd* estimated_state) const;

  //Feeding
  void feedPositionMeasurement(const Eigen::Vector3d& position);
  void feedVelocityMeasurement(const Eigen::Vector3d& velocity);
  void feedRotationMatrix(const Eigen::Matrix3d& rotation_matrix);
  void feedAttitudeCommand(const Eigen::Vector4d& roll_pitch_yaw_thrust_cmd);

  bool updateEstimator();

  virtual ~KFDisturbanceObserver();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  static constexpr int kStateSize = 18;
  static constexpr int kMeasurementSize = 9;
  static constexpr double kGravity = 9.8066;

  typedef Eigen::Matrix<double, kStateSize, 1> StateVector;

  ros::NodeHandle nh_, private_nh_, observer_nh_;
  bool initialized_;
  Eigen::Matrix<double, kStateSize, 1> state_;  // [pos, vel, rpy, omega, external_forces, external_moments]
  Eigen::Matrix<double, kStateSize, 1> predicted_state_;
  Eigen::Matrix<double, kMeasurementSize, 1> measurements_;  // [pos, vel, rpy]
  Eigen::Matrix3d rotation_matrix_;
  Eigen::Vector4d roll_pitch_yaw_thrust_cmd_;
  Eigen::Matrix<double, kStateSize, 1> process_noise_covariance_; // We express it as diag() later.
  Eigen::Matrix<double, kStateSize, kStateSize> state_covariance_;
  Eigen::Matrix<double, kStateSize, 1> initial_state_covariance_; // P0
  Eigen::Matrix<double, kMeasurementSize, 1> measurement_covariance_; // We express it as diag() later.
  Eigen::Matrix3d drag_coefficients_matrix_;

  Eigen::SparseMatrix<double> F_; // System dynamics matrix.
//  Eigen::Matrix<double, kStateSize, kStateSize> F_; // System dynamics matrix.
  Eigen::Matrix<double, kStateSize, kMeasurementSize> K_; // Kalman gain matrix.
  Eigen::SparseMatrix<double> H_; // Measurement matrix.
//  Eigen::Matrix<double, kMeasurementSize, kStateSize> H_; // Measurement matrix.

  Eigen::Vector3d external_forces_limit_;
  Eigen::Vector3d external_moments_limit_;
  Eigen::Vector3d omega_limit_;

  // Parameters
  double roll_damping_;
  double roll_omega_;
  double roll_gain_;

  double pitch_damping_;
  double pitch_omega_;
  double pitch_gain_;

  double yaw_damping_;
  double yaw_omega_;
  double yaw_gain_;

  double sampling_time_;

  ros::ServiceServer service_;
  ros::Publisher observer_state_pub_;
  ros::Publisher KF_pub_;

  bool is_calibrating_;         // true if calibrating
  ros::Time start_calibration_time_;   // t0 calibration
  ros::Duration calibration_duration_;     // calibration duration
  Eigen::Vector3d forces_offset_;
  Eigen::Vector3d moments_offset_;
  int calibration_counter_;
  bool startCalibration();

  void initialize();
  void systemDynamics(double dt);
  bool startCalibrationCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  // dynamic_reconfigure::Server<itm_nonlinear_mpc::KFDisturbanceObserverConfig> dyn_config_server_;

  // void DynConfigCallback(itm_nonlinear_mpc::KFDisturbanceObserverConfig &config, uint32_t level);

  void loadROSParams();

};
}
#endif /* SRC_KFDisturbanceObserver_H_ */
