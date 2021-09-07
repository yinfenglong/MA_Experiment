/*
name: itm_nonlinear_mpc.h
Yan Li, Uni Stuttgart, Germany
You can contact the author at <yan1.li@web.de>

NMPC: Non Linear Model Predictive Control
 */

#ifndef _ITM_NONLINEAR_MPC_H_
#define _ITM_NONLINEAR_MPC_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
/*msg */
/*mavros*/
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
/*msg of itm_nonlinear_mpc*/
#include <itm_nonlinear_mpc/mpc_command_rpyt.h>
/*others*/
#include <stdio.h>
#include <std_srvs/Empty.h>
#include <lapacke.h>
#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Quaternion.h>
/*acado*/
#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#include <itm_nonlinear_mpc/sim_only/itm_nonlinear_mpc_queue.h>
/*observer*/
#include <itm_nonlinear_mpc/sim_only/itm_kf_observer.h>
#include <itm_nonlinear_mpc/sim_only/itm_ukf_observer.h>
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

namespace quadrotor_control
{
    lapack_logical select_lhp(const double *real, const double *imag)
    {
        return *real < 0.0;
    }
class NMPCQuadrotor
{
    public:
    NMPCQuadrotor(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~NMPCQuadrotor();

    /*Dynamic parameters*/
    void setPositionPenality(const Eigen::Vector3d& q_position){q_position_ = q_position;}
    void setVelocityPenality(const Eigen::Vector3d& q_velocity){q_velocity_ = q_velocity;}
    void setAttitudePenality(const Eigen::Vector2d& q_attitude){q_attitude_ = q_attitude;}
    void setCommandPenality(const Eigen::Vector3d& r_command){r_command_ = r_command;}
    void setYawGain(double K_yaw){K_yaw_ = K_yaw;}
    void setAltitudeIntratorGain(double Ki_altitude){Ki_altitude_ = Ki_altitude;}
    void setXYIntratorGain(double Ki_xy){Ki_xy_ = Ki_xy;}
    void setEnableOffsetFree(bool enable_offset_free){enable_offset_free_ = enable_offset_free;}
    void setEnableIntegrator(bool enable_integrator){enable_integrator_ = enable_integrator;}
    void setControlLimits(const Eigen::VectorXd& control_limits)
    {
        /*roll_max, pitch_max, yaw_rate_max, thrust_min and thrust_max*/
        roll_limit_ = control_limits(0);
        pitch_limit_ = control_limits(1);
        yaw_rate_limit_ = control_limits(2);
        thrust_min_ = control_limits(3);
        thrust_max_ = control_limits(4);
    }
    void applyParameters();

    // /*TODO*/
     /*ros::Subscriber current states*/
    bool odometry_pose_Callback_flag_;
    ros::Subscriber odometry_pose_subscriber_;
    void OdometryPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void OdometryPoseCallback(const nav_msgs::Odometry::ConstPtr& msg);
    /*get reference and predicted state*/
    bool getforCurrentReference(mav_msgs::EigenTrajectoryPoint* reference) const;
    bool getforCurrentReference(mav_msgs::EigenTrajectoryPointDeque* reference) const;
    bool getforPredictedState(mav_msgs::EigenTrajectoryPointDeque* predicted_state) const;

    /*set Odometry and reference*/
    void setforOdometry(const mav_msgs::EigenOdometry& odometry);
    void setforCommandTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory);
    void setforCommandTrajectory(const mav_msgs::EigenTrajectoryPointDeque& command_trajectory);

    /*set desired position*/
    void setDesiredPosition(double desired_x, double desired_y, double desired_z)
    {
        position_reference_<<desired_x, desired_y, desired_z;
    }

    /*calculate RollPitchYawrateThrust Command*/
    bool calculateRollPitchYawrateThrustCommand(Eigen::Vector4d* ref_attitude_thrust);

    /*Initialize*/
    void initializeParameters();
    bool initialized_parameters_;

    private:/**/
    /*constants*/
    static constexpr double kGravity = 9.8066;
    static constexpr int kDisturbanceSize = 3;

    /*ros node handles*/
    ros::NodeHandle nh_, private_nh_;

    /*Service serve: reset integrators*/
    ros::ServiceServer reset_integrator_service_server_quadrotor_;
    bool resetIntegratorServiceQuadrotorCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    /*most recent odometry information*/
    mav_msgs::EigenOdometry odometry_;
    bool received_first_odometry_;
    /*from command position */
    Eigen::Vector3d position_reference_;
    Eigen::Vector3d velocity_reference_;
    Eigen::Vector3d acceleration_reference_;
    std::deque<double> yaw_reference_, yaw_rate_reference_;
    /*reference queue*/
    MPCQueue mpc_queue_;
    Vector3dDeque position_ref_, velocity_ref_, acceleration_ref_;
    std::deque<double> yaw_ref_, yaw_rate_ref_;

    /*initilize solvers*/
    void initializeforAcadoSolver(Eigen::VectorXd x0);

    /*Parameters*/
    /*sampling time and system model parameters*/
    double sampling_time_;
    double prediction_sampling_time_;
    double queue_dt_; //MPCQueue
    int minimum_queue_size_;//MPCQueue
    double mass_;
    double roll_time_constant_;
    double roll_gain_;
    double pitch_time_constant_;
    double pitch_gain_;
    Eigen::Vector3d drag_coefficients_;
    double position_error_integration_limit_;
    double antiwindup_ball_;
    /*controller parameters*/
    Eigen::Vector3d q_position_;
    Eigen::Vector3d q_velocity_;
    Eigen::Vector2d q_attitude_;
    Eigen::Vector3d r_command_;
    double K_yaw_;
    /*control input limits*/
    double roll_limit_;
    double pitch_limit_;
    double yaw_rate_limit_;
    double thrust_min_;
    double thrust_max_;
    /*commands*/
    Eigen::Vector4d command_roll_pitch_yaw_thrust_;

    /*ACADO Solver*/
    /*ACADO solver matrices*/
    Eigen::Matrix<double, ACADO_NY, ACADO_NY> W_;
    Eigen::Matrix<double, ACADO_NYN, ACADO_NYN> WN_;
    Eigen::Matrix<double, ACADO_N + 1, ACADO_NX> state_;
    Eigen::Matrix<double, ACADO_N, ACADO_NU> input_;
    Eigen::Matrix<double, ACADO_N, ACADO_NY> reference_;
    Eigen::Matrix<double, 1, ACADO_NYN> referenceN_;
    Eigen::Matrix<double, ACADO_N + 1, ACADO_NOD> acado_online_data_;

    /*solve continuous time Riccati equation*/
    Eigen::MatrixXd solveCARE(Eigen::MatrixXd Q, Eigen::MatrixXd R);

    /*debug info*/
    bool verbose_;
    double solve_time_average_;

    /*error integrator*/
    bool enable_integrator_;
    double Ki_altitude_;
    double Ki_xy_;
    Eigen::Vector3d position_error_integration_;

    /*Ros publisher, subscribe, service*/
    ros::Publisher mpc_command_rpyt_pub_;
    ros::Publisher thrust_pub_;

    // some quaternion functions
    void quaternion_cross(Eigen::Quaterniond q1, Eigen::Quaterniond q2, Eigen::Quaterniond &q_r);
    void quaternion2vector(Eigen::Quaterniond q, Eigen::Vector3d &v_r);
    void vector2quaternion(Eigen::Vector3d v, Eigen::Quaterniond &q_r);
    void quaternion_difference(Eigen::Quaterniond q1, Eigen::Quaterniond q2, Eigen::Quaterniond &q_r);
    void quaternion_difference(Eigen::Quaterniond q1, Eigen::Quaterniond q2, Eigen::Vector3d &v_r);
    void sum_quaternionAndvector(Eigen::Quaterniond q, Eigen::Vector3d v, Eigen::Quaterniond &q_r);

    /*disturbance observer*/
    bool enable_offset_free_;
    UKFnmpcObserver UKF_observer_;
    KFDisturbanceObserver KF_observer_;
    /* python UKF result */
    Eigen::Vector3d ukf_result;
    ros::Subscriber sub_ukf;
    bool ukf_available;
    void pythonUKFCallback(const geometry_msgs::Point::ConstPtr& msg);

};//class NMPCQuadrotor

}//namespace quadrotor_control

#endif