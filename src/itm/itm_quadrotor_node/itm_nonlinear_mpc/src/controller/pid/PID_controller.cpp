/*
 * @Author: Wei Luo
 * @Date: 2021-03-21 21:07:20
 * @LastEditors: Wei Luo
 * @LastEditTime: 2021-06-07 13:08:42
 * @Note: Basic PID controller
 */

#include <itm_nonlinear_mpc/controller/pid/PID_controller.hpp>

PIDController::PIDController(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh) : nh_(nh),
                                                                                             private_nh_(private_nh),
                                                                                             is_trajectory_provided_(false),
                                                                                             get_landing_last_pose_(false)
{
    /* get parameters */
    private_nh_.getParam("takeoff_height", takeoff_height_);
    private_nh_.getParam("sim_only", is_only_sim);
    /* subscribe topics */
    if (is_only_sim)
        robot_pose_sub_ = nh_.subscribe<nav_msgs::Odometry>("/real_pose", 10, &PIDController::current_odom_callback, this);
    else
        robot_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/real_pose", 10, &PIDController::robot_pose_callback, this);
    trajectory_sub_ = nh_.subscribe<itm_mav_msgs::itm_trajectory_msg>("/robot_trajectory", 10, &PIDController::trajectory_callback, this);
    /* server */
    controller_server_ = nh_.advertiseService("/itm_quadrotor_control/get_controller_state", &PIDController::controller_server_response_callback, this);
    /* client */
    /* Publisher: publish command_roll_pitch_yaw_thrust_*/
    // ros::Publisher command_attitude_thrust_pub_ = nh_node.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 100);
    command_position_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    command_position_target_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    /* init parameters */
    command_id = 0;
    is_current_pose_sub_ = false;
    is_robot_client_connected_ = false;
    integral_error_.setZero(3);

    ROS_INFO("Initialize PID controller");
}

PIDController::~PIDController()
{
}

void PIDController::idle()
{
    // stay the quadrotor on the ground
}

void PIDController::landing()
{
    if (!get_landing_last_pose_)
    {
        get_landing_last_pose_ = true;
        last_pose_ = current_pose_;

        last_quaternion_.w() = last_pose_.pose.orientation.w;
        last_quaternion_.x() = last_pose_.pose.orientation.x;
        last_quaternion_.y() = last_pose_.pose.orientation.y;
        last_quaternion_.z() = last_pose_.pose.orientation.z;
        last_position_error_.setZero(3);
    }

    mavros_msgs::PositionTarget msg;
    Eigen::Vector3d pose_error;
    Eigen::Vector3d delta_error;
    Eigen::Vector3d vec_cmd;
    pose_error(0) = last_pose_.pose.position.x - current_pose_.pose.position.x;
    pose_error(1) = last_pose_.pose.position.y - current_pose_.pose.position.y;
    pose_error(2) = initial_pose_.pose.position.z - current_pose_.pose.position.z;

    delta_error = pose_error - last_position_error_;
    vec_cmd = 0.35 * pose_error + delta_error * 0.001;

    msg.header.stamp = ros::Time::now();
    msg.type_mask = 0b100111000111;
    msg.coordinate_frame = 1;
    msg.velocity.x = vec_cmd(0);
    msg.velocity.y = vec_cmd(1);
    msg.velocity.z = vec_cmd(2);
    Eigen::Vector3d angle;
    quaternion_to_rpy(last_quaternion_, angle);
    msg.yaw = angle(2);

    command_position_target_pub_.publish(msg);
    last_position_error_ = pose_error;
}

void PIDController::takeoff_target()
{
    mavros_msgs::PositionTarget msg;
    msg.header.stamp = ros::Time::now();
    msg.type_mask = 0b100111111000;
    msg.coordinate_frame = 1;

    msg.position.x = initial_pose_.pose.position.x;
    msg.position.y = initial_pose_.pose.position.y;
    msg.position.z = initial_pose_.pose.position.z + takeoff_height_;

    msg.yaw = 0.0;
    command_position_target_pub_.publish(msg);
}

bool PIDController::is_initialized()
{
    if (is_robot_client_connected_ && is_current_pose_sub_)
        return true;
    else
        return false;
}

/* callback functions */
void PIDController::robot_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose_ = *msg;
    if (!is_current_pose_sub_)
    {
        is_current_pose_sub_ = true;
        initial_pose_ = *msg;
    }
}

void PIDController::trajectory_callback(const itm_mav_msgs::itm_trajectory_msg::ConstPtr &msg)
{
    if (!is_trajectory_provided_)
        is_trajectory_provided_ = true;
    {
        boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexTrajectoryCallback_);
        trajectory_ref_point_ = std::make_shared<itm_mav_msgs::itm_trajectory_msg>(*msg);
    }
}

void PIDController::current_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_pose_.pose.position.x = msg->pose.pose.position.x;
    current_pose_.pose.position.y = msg->pose.pose.position.y;
    current_pose_.pose.position.z = msg->pose.pose.position.z;
    current_pose_.pose.orientation.x = msg->pose.pose.orientation.x;
    current_pose_.pose.orientation.y = msg->pose.pose.orientation.y;
    current_pose_.pose.orientation.z = msg->pose.pose.orientation.z;
    current_pose_.pose.orientation.w = msg->pose.pose.orientation.w;
    if (!is_current_pose_sub_)
    {
        is_current_pose_sub_ = true;
        initial_odom_ = *msg;
    }
}

bool PIDController::controller_server_response_callback(itm_mav_srvs::GetControllerState::Request &req,
                                                        itm_mav_srvs::GetControllerState::Response &res)
{
    command_id = req.command_id;
    // robot_name_ = req.robot_name;
    if (is_current_pose_sub_)
    {
        res.connected = true;
        if (!is_robot_client_connected_)
            is_robot_client_connected_ = true;
        return true;
    }
    else
    {
        res.connected = false;
        return false;
    }
}

void PIDController::mission_loop()
{
    // mavros_msgs::PositionTarget msg;
    // msg.header.stamp = ros::Time::now();
    // // bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate.
    // // bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude
    // msg.type_mask = 0b100111111000;
    // msg.coordinate_frame = 1;
    // /* check if the trajectory command is avaliable */
    // if (is_trajectory_provided_)
    // {
    //     boost::shared_lock<boost::shared_mutex> mutexTrajectoryCallback(mutexTrajectoryCallback_);
    //     if (trajectory_ref_point_->size > 1)
    //     {
    //         ROS_INFO_ONCE("PID only takes the first trajectory waypoint!");
    //     }
    //     msg.position.x = trajectory_ref_point_->traj[0].x;
    //     msg.position.y = trajectory_ref_point_->traj[0].y;
    //     msg.position.z = trajectory_ref_point_->traj[0].z;

    //     if (trajectory_ref_point_->traj[0].quaternion_given)
    //     {
    //         Eigen::Vector3d rpy_des;
    //         Eigen::Quaterniond q_des;
    //         q_des.w() = trajectory_ref_point_->traj[0].q[0];
    //         q_des.x() = trajectory_ref_point_->traj[0].q[1];
    //         q_des.y() = trajectory_ref_point_->traj[0].q[2];
    //         q_des.z() = trajectory_ref_point_->traj[0].q[3];

    //         quaternion_to_rpy(q_des, rpy_des);
    //         msg.yaw = rpy_des[2];
    //     }
    //     else
    //     {
    //         msg.yaw = trajectory_ref_point_->traj[0].yaw;
    //     }
    // }
    // else
    // {
    //     /* stay with the take off position */
    //     msg.position.x = initial_pose_.pose.position.x;
    //     msg.position.y = initial_pose_.pose.position.y;
    //     msg.position.z = initial_pose_.pose.position.z + takeoff_height_;

    //     msg.yaw = 0.0;
    // }

    if (is_trajectory_provided_)
    {
        Eigen::Vector3d pose_error;
        Eigen::Vector3d delta_error;
        Eigen::Vector3d vec_cmd;
        pose_error(0) = trajectory_ref_point_->traj[0].x - current_pose_.pose.position.x;
        pose_error(1) = trajectory_ref_point_->traj[0].y - current_pose_.pose.position.y;
        pose_error(2) = trajectory_ref_point_->traj[0].z - current_pose_.pose.position.z;

        delta_error = pose_error - last_position_error_;
        integral_error_ += pose_error;

        output_constrains(integral_error_, 1.0);
        vec_cmd = 0.45 * pose_error + delta_error * 0.01 + integral_error_ * 0.02;

        output_constrains(vec_cmd, 0.5);

        mavros_msgs::PositionTarget msg;
        msg.header.stamp = ros::Time::now();
        msg.type_mask = 0b100111000111;
        msg.coordinate_frame = 1;
        msg.velocity.x = vec_cmd(0);
        msg.velocity.y = vec_cmd(1);
        msg.velocity.z = vec_cmd(2);
        msg.yaw = trajectory_ref_point_->traj[0].yaw;
        last_position_error_ = pose_error;
        command_position_target_pub_.publish(msg);
    }
    else
    {
        /* stay with the take off position */
        mavros_msgs::PositionTarget msg;
        msg.header.stamp = ros::Time::now();
        msg.type_mask = 0b100111111000;
        msg.coordinate_frame = 1;

        msg.position.x = initial_pose_.pose.position.x;
        msg.position.y = initial_pose_.pose.position.y;
        msg.position.z = initial_pose_.pose.position.z + takeoff_height_;

        msg.yaw = 0.0;
        command_position_target_pub_.publish(msg);
    }
}

void PIDController::rpy_to_quaternion(Eigen::Vector3d rpy, Eigen::Quaterniond &q)
{
    double cy = std::cos(rpy[2] * 0.5);
    double sy = std::sin(rpy[2] * 0.5);
    double cp = std::cos(rpy[1] * 0.5);
    double sp = std::sin(rpy[1] * 0.5);
    double cr = std::cos(rpy[0] * 0.5);
    double sr = std::sin(rpy[0] * 0.5);

    q.w() = cr * cp * cy + sr * sp * sy;
    q.x() = sr * cp * cy - cr * sp * sy;
    q.y() = cr * sp * cy + sr * cp * sy;
    q.z() = cr * cp * sy - sr * sp * cy;
}

void PIDController::quaternion_to_rpy(Eigen::Quaterniond q, Eigen::Vector3d &rpy)
{
    auto q0 = q.w();
    auto q1 = q.x();
    auto q2 = q.y();
    auto q3 = q.z();

    rpy(0) = std::atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (std::pow(q1, 2) + std::pow(q2, 2)));
    rpy(1) = std::asin(2 * (q0 * q2 - q3 * q1));
    rpy(2) = std::atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (std::pow(q2, 2) + std::pow(q3, 2)));
}

void PIDController::output_constrains(Eigen::Vector3d &cmd, double max_cmd)
{
    cmd(0) = std::max(std::min(cmd(0), max_cmd), -max_cmd);
    cmd(1) = std::max(std::min(cmd(1), max_cmd), -max_cmd);
    cmd(2) = std::max(std::min(cmd(2), max_cmd), -max_cmd);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "QuadrotorPIDNode");

    ros::NodeHandle nh_node, private_nh_node("~");
    PIDController controller_node(nh_node, private_nh_node);
    ros::Rate rate(50.0);

    // mavros_msgs::AttitudeTargetPtr attitude_msg(new mavros_msgs::AttitudeTarget);
    // geometry_msgs::PoseStampedPtr position_msg(new geometry_msgs::PoseStamped);

    while (ros::ok())
    {
        if (controller_node.is_initialized())
        {
            switch (controller_node.command_id)
            {
            case 0: // idle
                controller_node.idle();
                break;
            case 1: // take_off
            {
                controller_node.takeoff_target();
                ROS_INFO_ONCE("Take off");
                break;
            }
            case 2: // landing
            {
                controller_node.landing();
                // command_position_pub_.publish(position_msg);
                break;
            }
            case 3: // go to position
            {
                // controller_node.mission_loop(ros::Time::now(), position_msg.get());
                // command_position_pub_.publish(position_msg);
                controller_node.mission_loop();
                ROS_INFO_ONCE("Go to Position Mode.");
                break;
            }
            case 4: // emergency landing
            {

                ROS_INFO_ONCE("Emergency Mode.");
                break;
            }
            default:
            {
                ROS_INFO("Please check the command id.");
            }
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("PID controller is closed");
}