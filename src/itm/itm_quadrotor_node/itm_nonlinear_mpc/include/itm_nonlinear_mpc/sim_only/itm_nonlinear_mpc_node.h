/*
Yan Li, Uni Stuttgart, Germany
You can contact the author at <yan1.li@web.de>

NMPC: Non Linear Model Predictive Control
 */
#ifndef _ITM_NONLINEAR_MPC_NODE_H_
#define _ITM_NONLINEAR_MPC_NODE_H_
// #include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>
/*ROS*/
#include <ros/ros.h>
#include <ros/callback_queue.h>
/*msg of itm_nonlinear_mpc*/
//#include <itm_nonlinear_mpc/mpc_command_rpyt.h>
#include <itm_nonlinear_mpc/mpc_set_point_pos.h>
/*ROS msgs*/
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/Status.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_msgs/Status.h>
/*dynamic reconfiguration*/
#include <dynamic_reconfigure/server.h>
#include <itm_nonlinear_mpc/NonLinearMPCConfig.h>
/*mavros*/
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
// #include <itm_nonlinear_mpc/NonLinearMPCConfig.h>
/*itm_nonlinear_mpc*/
#include <itm_nonlinear_mpc/mpc_set_point_pos.h>
#include <itm_nonlinear_mpc/mpc_command_rpyt.h>

#include <itm_nonlinear_mpc/sim_only/itm_nonlinear_mpc.h>
#include <tf/transform_datatypes.h>

#include <boost/bind.hpp>

// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>

namespace quadrotor_control
{
class NMPCQuadrotorNode //: public mav_control_interface::PositionControllerInterface
{
public:
  NMPCQuadrotorNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
  ~NMPCQuadrotorNode();

  void InitializeParams();
  bool got_first_position_command;
  bool got_Roll_Pitch_Yaw_Thrust_command;
  bool calculateRollPitchYawrateThrustCommand(mav_msgs::EigenRollPitchYawrateThrust* attitude_thrust_command);
  /*Ros publisher*/
  ros::Publisher command_attitude_thrust_pub_;
  NMPCQuadrotor nonlinear_mpc_;

  /*msg from RollPitchYawrateThrust*/
  void msgFromRollPitchYawrateThrust(
    const mav_msgs::EigenRollPitchYawrateThrust& roll_pitch_yawrate_thrust, ros::Time time_stamp, mavros_msgs::AttitudeTarget* msg);
  void msgFromRollPitchYawrateThrust(
    const mav_msgs::EigenRollPitchYawrateThrust& roll_pitch_yawrate_thrust, ros::Time time_stamp, geometry_msgs::Pose* msg);


private:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ros::NodeHandle nh_;

  /*Controller dynamic_reconfigure*/
  /*ros service Server*/
  dynamic_reconfigure::Server<itm_nonlinear_mpc::NonLinearMPCConfig> controller_dyn_config_server_;
  /**/
  void ControllerDynConfigCallback(itm_nonlinear_mpc::NonLinearMPCConfig &config, uint32_t level);

 /* ros::ServiceServer command_position_server_;*/
  ros::ServiceServer command_position_server_;
  bool commandPositionCallback(itm_nonlinear_mpc::mpc_set_point_pos::Request &req,
                                itm_nonlinear_mpc::mpc_set_point_pos::Response &res);
  // mav_msgs::EigenTrajectoryPoint reference;
  // mav_msgs::EigenTrajectoryPointDeque references;
  void referencesUpdate(const mav_msgs::EigenTrajectoryPointDeque& _reference_array);

  /* Subscriber trajectory */
  ros::Subscriber command_trajectory_array_subscriber_;

  /*set Reference, ReferenceArray, Odometry*/
  bool setReference(const mav_msgs::EigenTrajectoryPoint& reference);
  bool setReferenceArray(const mav_msgs::EigenTrajectoryPointDeque& reference_array);
  bool setOdometry(const mav_msgs::EigenOdometry& odometry);
  // virtual bool setOdometry(const geometry_msgs::PoseStamped& odometry);

  /*calculate RollPitchYawrate and Thrust Command*/
  bool calculateAttitudeThrustCommand(mav_msgs::EigenAttitudeThrust* attitude_thrust_command);

  /*get Current Reference*/
  /*TODO*/
  mav_msgs::EigenTrajectoryPointDeque reference_array_;
  bool getCurrentReference(mav_msgs::EigenTrajectoryPoint* reference) const;
  bool getCurrentReference(mav_msgs::EigenTrajectoryPointDeque* reference) const;

  /*get Predicted State*/
  bool getPredictedState(mav_msgs::EigenTrajectoryPointDeque* predicted_state) const;

}; //class NMPCQuadrotorNode

}//namespace quadrotor_control

#endif