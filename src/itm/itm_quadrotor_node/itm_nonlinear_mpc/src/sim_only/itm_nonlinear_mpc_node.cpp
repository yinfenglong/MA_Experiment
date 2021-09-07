/*
Yan Li, Uni Stuttgart, Germany
You can contact the author at <yan1.li@web.de>

NMPC: Non Linear Model Predictive Control
 */

#include <itm_nonlinear_mpc/sim_only/itm_nonlinear_mpc_node.h>

namespace quadrotor_control
{
  NMPCQuadrotorNode::NMPCQuadrotorNode(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh)
      : nonlinear_mpc_(nh, private_nh),
        nh_(nh),
        got_first_position_command(false),
        got_Roll_Pitch_Yaw_Thrust_command(false),
        controller_dyn_config_server_(private_nh)
{
  /*initialize Parameters*/
  nonlinear_mpc_.initializeParameters();

  /* Service serve: get set point command*/
  command_position_server_ = nh_.advertiseService("/itm_quadrotor_control/mpc_set_point_pos", &NMPCQuadrotorNode::commandPositionCallback, this);

  //TODO
  // /* Subscriber trajectory */
  // command_trajectory_array_subscriber_ = nh_.subscribe("/itm_quadrotor_control/command_trajectory", 1, &NMPCQuadrotorNode::CommandTrajectoryCallback, this);

  dynamic_reconfigure::Server<itm_nonlinear_mpc::NonLinearMPCConfig>::CallbackType f_controller;
  f_controller = boost::bind(&NMPCQuadrotorNode::ControllerDynConfigCallback, this, _1, _2);
  controller_dyn_config_server_.setCallback(f_controller);
}

NMPCQuadrotorNode::~NMPCQuadrotorNode()
{
}

bool NMPCQuadrotorNode::commandPositionCallback(itm_nonlinear_mpc::mpc_set_point_pos::Request &req,
                                                itm_nonlinear_mpc::mpc_set_point_pos::Response &res )
{
  geometry_msgs::Pose command_position_msg;
  command_position_msg.position.x = req.x;
  command_position_msg.position.y = req.y;
  command_position_msg.position.z = req.z;
  mav_msgs::EigenTrajectoryPoint command_position_reference;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(command_position_msg, &command_position_reference);
  if(!got_first_position_command) got_first_position_command = true;
  if(got_first_position_command)
  {
    res.success = true;
  }else{
    res.success = false;
  }
  ROS_INFO_ONCE("Nonlinear MPC: command position success");
  setReference(command_position_reference);
  // mav_msgs::EigenTrajectoryPointDeque references;
  // references.push_back(reference);
  // setReference(reference);
  // setReferenceArray(references);
  // referencesUpdate(references);
  return true;
}

//TODO
// void NMPCQuadrotorNode::CommandTrajectoryCallback(
//     const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg)
// {
//   int array_size = msg->points.size();
//   if (array_size == 0)
//     return;

//   mav_msgs::EigenTrajectoryPointDeque references;
//   mav_msgs::eigenTrajectoryPointDequeFromMsg(*msg, &references);

//   state_machine_->process_event(state_machine::ReferenceUpdate(references));
// }

/*todo is this right?*/
// void NMPCQuadrotorNode::referencesUpdate(const mav_msgs::EigenTrajectoryPointDeque&_reference_array)
// {
//   reference_array_ = _reference_array;
// }

void NMPCQuadrotorNode::ControllerDynConfigCallback(itm_nonlinear_mpc::NonLinearMPCConfig &config, uint32_t level)
{
  Eigen::Vector3d q_position;
  Eigen::Vector3d q_velocity;
  Eigen::Vector2d q_attitude;
  Eigen::Vector3d r_command;
  Eigen::VectorXd control_limits(5);
  q_position << config.q_x, config.q_y, config.q_z;
  q_velocity << config.q_vx, config.q_vy, config.q_vz;
  q_attitude << config.q_roll, config.q_pitch;
  r_command << config.r_roll, config.r_pitch, config.r_thrust;
  control_limits << config.roll_max, config.pitch_max, config.yaw_rate_max, config.thrust_min, config.thrust_max;

  nonlinear_mpc_.setPositionPenality(q_position);
  nonlinear_mpc_.setVelocityPenality(q_velocity);
  nonlinear_mpc_.setAttitudePenality(q_attitude);
  nonlinear_mpc_.setCommandPenality(r_command);
  nonlinear_mpc_.setYawGain(config.K_yaw);
  nonlinear_mpc_.setControlLimits(control_limits);

  nonlinear_mpc_.setAltitudeIntratorGain(config.Ki_altitude);
  nonlinear_mpc_.setXYIntratorGain(config.Ki_xy);

  nonlinear_mpc_.setEnableIntegrator(config.enable_integrator);
  //nonlinear_mpc_.setEnableOffsetFree(config.enable_offset_free);

  nonlinear_mpc_.applyParameters();
}

bool NMPCQuadrotorNode::setReference(const mav_msgs::EigenTrajectoryPoint &reference)
{
  // reference_array_.clear();
  // reference_array_.push_back(reference);
  // ROS_INFO_STREAM("got reference: position=" << reference.position_W.transpose());
  // NMPCQuadrotorNode::setReferenceArray(reference_array_);
  nonlinear_mpc_.setforCommandTrajectoryPoint(reference);
  return true;
}

/*TODO*/
bool NMPCQuadrotorNode::setReferenceArray(const mav_msgs::EigenTrajectoryPointDeque &reference_array)
{
  reference_array_ = reference_array;
  nonlinear_mpc_.setforCommandTrajectory(reference_array_);
  return true;
}

bool NMPCQuadrotorNode::setOdometry(const mav_msgs::EigenOdometry &odometry)
{
  nonlinear_mpc_.setforOdometry(odometry);
  return true;
}

bool NMPCQuadrotorNode::calculateAttitudeThrustCommand(mav_msgs::EigenAttitudeThrust *attitude_thrust_command)
{
  ROS_WARN("calculateAttitudeThrustCommand not implemented");
  return false;
}

bool NMPCQuadrotorNode::calculateRollPitchYawrateThrustCommand(mav_msgs::EigenRollPitchYawrateThrust *attitude_thrust_command)
{
  Eigen::Vector4d rpy_thrust;
  nonlinear_mpc_.calculateRollPitchYawrateThrustCommand(&rpy_thrust);
  attitude_thrust_command->roll = rpy_thrust(0);
  attitude_thrust_command->pitch = rpy_thrust(1);
  attitude_thrust_command->yaw_rate = rpy_thrust(2);
  attitude_thrust_command->thrust.z() = rpy_thrust(3);
  if (!got_Roll_Pitch_Yaw_Thrust_command) got_Roll_Pitch_Yaw_Thrust_command = true;
  return true;
} //calculateRollPitchYawrateThrustCommand

void msgFromRollPitchYawrateThrust(
    const mav_msgs::EigenRollPitchYawrateThrust& roll_pitch_yawrate_thrust, ros::Time time_stamp, geometry_msgs::Pose* msg)
{
  assert(msg != NULL);
  Eigen::Vector3d command_rpy;
  command_rpy.setZero();
  command_rpy[0] = roll_pitch_yawrate_thrust.roll;
  command_rpy[1] = roll_pitch_yawrate_thrust.pitch;
  geometry_msgs::Vector3 command_body_rate;
  command_body_rate.x = 0.0;
  command_body_rate.y = 0.0; //roll_pitch_yawrate_thrust.yaw_rate;
  command_body_rate.z = roll_pitch_yawrate_thrust.yaw_rate; //0.0;
  double thrust = std::max(0.0, roll_pitch_yawrate_thrust.thrust.z());/// 9.8006 - 0.5;

  geometry_msgs::Quaternion quaternion_command_rpy;
  quaternion_command_rpy = tf::createQuaternionMsgFromRollPitchYaw(command_rpy[0], command_rpy[1], command_rpy[2]);

  /*output*/
  msg->position.x = 0.0;
  msg->position.y = 0.0;
  msg->position.z = float(thrust)/ 9.8006 - 0.5;
  msg->orientation = quaternion_command_rpy;
}

void NMPCQuadrotorNode::msgFromRollPitchYawrateThrust(
    const mav_msgs::EigenRollPitchYawrateThrust& roll_pitch_yawrate_thrust, ros::Time time_stamp, mavros_msgs::AttitudeTarget* msg)
{
  assert(msg != NULL);
  Eigen::Vector3d command_rpy;
  command_rpy.setZero();
  command_rpy[0] = roll_pitch_yawrate_thrust.roll;
  command_rpy[1] = roll_pitch_yawrate_thrust.pitch;
  geometry_msgs::Vector3 command_body_rate;
  command_body_rate.x = 0.0;
  command_body_rate.y = roll_pitch_yawrate_thrust.yaw_rate;
  command_body_rate.z = 0.0;
  double thrust = std::max(0.0, roll_pitch_yawrate_thrust.thrust.z());

  geometry_msgs::Quaternion quaternion_command_rpy;
  quaternion_command_rpy = tf::createQuaternionMsgFromRollPitchYaw(command_rpy[0], command_rpy[1], command_rpy[2]);

  /*output*/
  msg->orientation = quaternion_command_rpy;
  msg->body_rate= command_body_rate;
  msg->thrust = float(thrust)/ 9.8006 - 0.5;

}


bool NMPCQuadrotorNode::getCurrentReference(mav_msgs::EigenTrajectoryPoint *reference) const
{
  assert(reference != nullptr);
  if(reference_array_.empty())
      return false;

    *reference = reference_array_.front();
    return true;
  // return nonlinear_mpc_.getforCurrentReference(reference);
}

bool NMPCQuadrotorNode::getCurrentReference(mav_msgs::EigenTrajectoryPointDeque *reference) const
{
  assert(reference != nullptr);

    if(reference_array_.empty())
      return false;

    *reference = reference_array_;
      return true;
    // return nonlinear_mpc_.getforCurrentReference(reference);
}

bool NMPCQuadrotorNode::getPredictedState(mav_msgs::EigenTrajectoryPointDeque *predicted_state) const
{
  assert(predicted_state != nullptr);
  return nonlinear_mpc_.getforPredictedState(predicted_state);
}

}; // namespace quadrotor_control

int main(int argc, char **argv)
{
  ros::init(argc, argv, "NMPCQuadrotorNode");

  ros::NodeHandle nh_node, private_nh_node("~");
  quadrotor_control::NMPCQuadrotorNode NMPC_controller_node(nh_node, private_nh_node);
  ros::Rate rate(100.0);
  /* Publisher: publisch command_roll_pitch_yaw_thrust_*/
  ros::Publisher command_attitude_thrust_pub_ = nh_node.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 100);

  mavros_msgs::AttitudeTargetPtr attitude_msg(new mavros_msgs::AttitudeTarget);

  while(ros::ok())
  {
    if(NMPC_controller_node.got_first_position_command)
    {
      mav_msgs::EigenRollPitchYawrateThrust cmd_rpy_thrust;
      NMPC_controller_node.calculateRollPitchYawrateThrustCommand(&cmd_rpy_thrust);

      // geometry_msgs::PosePtr msg(new geometry_msgs::Pose);

      // msg->header.stamp = ros::Time::now();  // TODO(acmarkus): get from msg
      NMPC_controller_node.msgFromRollPitchYawrateThrust(cmd_rpy_thrust,ros::Time::now(),attitude_msg.get());
      command_attitude_thrust_pub_.publish(attitude_msg);
    }
    ros::spinOnce();
    rate.sleep();
  }//while

    return 0;
}