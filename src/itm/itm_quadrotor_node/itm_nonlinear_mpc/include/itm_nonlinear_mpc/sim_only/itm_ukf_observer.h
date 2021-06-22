/*
Yan Li, Uni Stuttgart, Germany
You can contact the author at <yan1.li@web.de>
 */

#ifndef _UKF_NMPC_OBSERVER_H_
#define _UKF_NMPC_OBSERVER_H_

#include <Eigen/Core>
// #include <eigen3/Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Sparse>

// #include <eigen3/Eigen/Eigenvalues>
#include <ros/ros.h>
#include<itm_nonlinear_mpc/itm_ukf_observer.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <math.h>
#include <geometry_msgs/Pose.h> 
#include <unsupported/Eigen/MatrixFunctions>
// #include <unsupported/Eigen/MatrixFunctions>

//dynamic reconfigure
// #include <dynamic_reconfigure/server.h>
// #include <itm_nonlinear_mpc/UKFObserverConfig.h>

namespace quadrotor_control
{
  class UKFnmpcObserver
  {
    public:

    UKFnmpcObserver(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
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
    void feedMotorSpeedMeasurement(const Eigen::Vector4d& motor_speed_);

    bool updateEstimator();

    virtual ~UKFnmpcObserver();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   
    private:
    static constexpr int kStateSize = 18; //state vector dimension with external forces and moments
    static constexpr int kMeasurementSize = 9;
    static constexpr int kMotorSize = 4; // size of Motor
    static constexpr double kGravity = 9.8066;  //static constexpr
    static constexpr int kSigmaSize = 37; //Sigma points dimension kSigmaSize= 2 * kStateSize + 1 
    // static constexpr double lambda_ = -15.0;  //Sigma point spreading parameter lambda_=3 - n_x_
    

    ros::NodeHandle nh_, private_nh_, observer_nh_;
    bool initialized_;

    Eigen::Matrix<double, kStateSize, 1> state_;  ///12x1 x_ state vector: [position velocity attitude angular_rate]
    Eigen::Matrix<double, kStateSize, 1> predicted_state_;//12x1 x_pred_
    Eigen::Matrix<double, kMeasurementSize, 1> measurements_;  // [pos, vel, rpy]
    Eigen::Matrix<double, kMotorSize, 1> motorspeed_;  // 
    Eigen::Matrix3d rotation_matrix_;
    Eigen::Vector4d roll_pitch_yaw_thrust_cmd_;
    Eigen::Matrix<double, kStateSize, kStateSize> process_noise_covariance_; // We express it as diag() later.
    Eigen::Matrix<double, kStateSize, 1> initial_process_noise_covariance_;
    Eigen::Matrix<double, kStateSize, kStateSize> state_covariance_;//state covariance MatrixXd P_
    Eigen::Matrix<double, kStateSize, 1> initial_state_covariance_; //initial covariance matrix P_0 = MatrixXd(n_x_, n_x_);
    Eigen::Matrix<double, kMeasurementSize, kMeasurementSize> measurement_noise_; // We express it as diag() later.
    Eigen::Matrix<double, kMeasurementSize, 1> initial_measurement_noise_; // We express it as diag() later.
    Eigen::Matrix3d drag_coefficients_matrix_;
    // 1 Generate sigma points
    Eigen::Matrix<double, kStateSize, kSigmaSize> Xsig_; // sigma points matrix
    Eigen::Matrix<double, kStateSize, kStateSize> A_; 
    // 2. Computing Weights of Sigma Points
    Eigen::Matrix<double, kSigmaSize, 1> weights_; //Weights of sigma points
     Eigen::Matrix<double, kSigmaSize, 1> weights_c_;
    // 3. Computing Mean and Covariance of the approximate Gaussian
    Eigen::Matrix<double, kStateSize, kSigmaSize> Xsig_pred_; //predicted sigma points matrix 
    Eigen::Matrix<double, kStateSize, 1> state_mean_pred_; // predicted state mean 
    // Eigen::Matrix<double, kStateSize, kStateSize> covariance_matrix_pred_; //predicted state covariance matrix
    // Eigen::Matrix<double, kStateSize, 1> x_diff_;   //state difference predicted_state_
    //4. predicted state to the measurement state 
    Eigen::Matrix<double, kStateSize, kSigmaSize> Xsig_update_; // sigma points matrix
    Eigen::Matrix<double, kMeasurementSize, kSigmaSize> Zsig_;  //transform sigma points into measurement space Zsig_
    Eigen::Matrix<double, kMeasurementSize, kMeasurementSize> S_; //Covariance in measurement space S_
    //5. Update state
    Eigen::Matrix<double, kStateSize, kMeasurementSize> Tc_; //matrix for cross correlation Tc
    Eigen::Matrix<double, kStateSize, kMeasurementSize> K_; //Kalman gain K;
    Eigen::Matrix<double, kMeasurementSize, 1> z_mean_; // Mean in measurement space z_mean_
    // Eigen::Matrix<double, kStateSize, 1> X_diff_;
    Eigen::Matrix<double, kMeasurementSize, 1> z_diff_;
    Eigen::Matrix<double, kMeasurementSize, 1> z_diff; // new measurement
    
    Eigen::SparseMatrix<double> F_; // System dynamics matrix.
    // Eigen::Matrix<double, kStateSize, kStateSize> F_; // System dynamics matrix. Eigen::SparseMatrix<double> F_;
    // Eigen::Matrix<double, kMeasurementSize, n_x_> H_; // Measurement matrix.Eigen::SparseMatrix<double> H_;
    Eigen::SparseMatrix<double> H_;// Measurement matrix.Eigen::SparseMatrix<double> H_;

    Eigen::Vector3d omega_limit_;
    Eigen::Vector3d external_forces_limit_;
    Eigen::Vector3d external_moments_limit_;

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
    double dt; 

    // Parameters to angular_acceleration
    double I_xx_; //inertia moment in the X axis
    double I_yy_; //inertia moment in the Y axis
    double I_zz_; //inertia moment in the Z axis
    double I_ww_; //inertia moment of the rotor
    double b_w_;
    double length_; 
    double input_moment_coefficient_x_;
    double input_moment_coefficient_y_;
    double input_moment_coefficient_z_;
    double lambda_;



    ros::ServiceServer service_;
    ros::Publisher observer_state_pub_;
    ros::Publisher angular_acceleration_pub_;
    // ros::Publisher thrust_pub_;

    bool is_calibrating_;         // true if calibrating
    ros::Time start_calibration_time_;   // t0 calibration
    ros::Duration calibration_duration_;     // calibration duration
    Eigen::Vector3d forces_offset_;
    Eigen::Vector3d moments_offset_;
    int calibration_counter_;
    bool startCalibration();

    void initialize();
    void systemDynamics(double dt); //predict state
    // bool startCalibrationCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    // dynamic_reconfigure::Server<ma_nmpc_observer::UKFObserverConfig> dyn_config_server_;

    // void DynConfigCallback(ma_nmpc_observer::UKFObserverConfig &config, uint32_t level);

    void loadROSParams();

    double limit_angle(double input);

    int sign(double x);

  }; //class UKFnmpcOberserver

}//namespace 

#endif /* SRC_UKFnmpcObserver_H_ */