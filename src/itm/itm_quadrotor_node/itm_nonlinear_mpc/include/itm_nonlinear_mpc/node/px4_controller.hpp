#ifndef PX4_CONTROLLER_HPP
#define PX4_CONTROLLER_HPP
/*
 * @Author: Wei Luo
 * @Date: 2021-03-14 23:40:44
 * @LastEditors: Wei Luo
 * @LastEditTime: 2021-05-27 21:56:38
 * @Note: Note
 */
#include <ros/ros.h>
// server msg
#include <itm_mav_srvs/GetControllerState.h>
#include <itm_mav_srvs/SetMode.h>
// topic msg
#include <geometry_msgs/PoseStamped.h>
#include <itm_mav_msgs/SetMission.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Dense>
#include <chrono>

mavros_msgs::State mavros_state;
itm_mav_srvs::SetMode uav_mode;
itm_mav_srvs::GetControllerState controller_state;

bool current_pos_flag;
int controller_type;
geometry_msgs::PoseStamped current_pos;
geometry_msgs::PoseStamped initial_pos;
itm_mav_msgs::SetMission user_command;
itm_mav_msgs::SetMission last_user_command;

mavros_msgs::SetMode offboard_set_mode;
mavros_msgs::CommandBool arm_cmd;
ros::Time last_request;

bool is_controller_working;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
geometry_msgs::PoseStamped command_pose;

ros::Publisher robot_command;
bool is_only_sim;

// for emergency landing
geometry_msgs::PoseStamped landing_target;
bool got_landing_target;

// fence
bool got_fence;
std::vector<double> fence_def;

ros::Publisher flight_mission_mode_pub_;

// subscribe mavros
void mavros_state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    if (mavros_state.armed != msg->armed)
    {
        std::cout << "UAV: armed state changes from " << static_cast<int16_t>(mavros_state.armed) << "to" << static_cast<int16_t>(msg->armed) << std::endl;
    }

    if (mavros_state.connected != msg->connected)
    {
        std::cout << "UAV: connection changes from " << static_cast<int16_t>(mavros_state.connected) << "to" << static_cast<int16_t>(msg->connected) << std::endl;
    }

    if (mavros_state.mode != msg->mode)
    {
        std::cout << "UAV: mode changes from " << mavros_state.mode << "to" << msg->mode << std::endl;
    }

    if (mavros_state.system_status != msg->system_status)
    {
    }

    mavros_state = *msg;
} // mavros_state_cb

void current_pos_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    assert(msg != nullptr);
    if (!current_pos_flag)
    {
        current_pos_flag = true;
        initial_pos = *msg;
        ROS_INFO("Inertial Position is %f, %f, %f", msg->pose.position.x,
                 msg->pose.position.y, msg->pose.position.z);
    }
    // ROS_INFO("Inertial Piiiosition is %f, %f, %f", msg->pose.position.x,
    //          msg->pose.position.y, msg->pose.position.z);
    current_pos = *msg;

    ROS_INFO_ONCE("mpc_current_pos_flag is true");
} //current_pos_callback

void current_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    assert(msg != nullptr);
    if (!current_pos_flag)
    {
        initial_pos.pose.position.x = msg->pose.pose.position.x;
        initial_pos.pose.position.y = msg->pose.pose.position.y;
        initial_pos.pose.position.z = msg->pose.pose.position.z;
        initial_pos.pose.orientation.x = msg->pose.pose.orientation.x;
        initial_pos.pose.orientation.y = msg->pose.pose.orientation.y;
        initial_pos.pose.orientation.z = msg->pose.pose.orientation.z;
        initial_pos.pose.orientation.w = msg->pose.pose.orientation.w;
        ROS_INFO("Inertial Position is %f, %f, %f", initial_pos.pose.position.x,
                 initial_pos.pose.position.y, initial_pos.pose.position.z);
        current_pos_flag = true;
    }

    current_pos.pose.position.x = msg->pose.pose.position.x;
    current_pos.pose.position.y = msg->pose.pose.position.y;
    current_pos.pose.position.z = msg->pose.pose.position.z;
    current_pos.pose.orientation.x = msg->pose.pose.orientation.x;
    current_pos.pose.orientation.y = msg->pose.pose.orientation.y;
    current_pos.pose.orientation.z = msg->pose.pose.orientation.z;
    current_pos.pose.orientation.w = msg->pose.pose.orientation.w;
}

void user_command_callback(const itm_mav_msgs::SetMission::ConstPtr &msg)
{
    user_command.mission_mode = msg->mission_mode;
    user_command.command_idx += 1;
}

void landing_command(geometry_msgs::PoseStamped target)
{
    /* let the quadrotor fly back to the inertial position */
    command_pose.pose.position.x = target.pose.position.x;
    command_pose.pose.position.y = target.pose.position.y;
    command_pose.pose.position.z = target.pose.position.z;
    command_pose.pose.orientation.w = 1.0;
    command_pose.pose.orientation.x = 0.0;
    command_pose.pose.orientation.y = 0.0;
    command_pose.pose.orientation.z = 0.0;
    robot_command.publish(command_pose);
    ROS_INFO_ONCE("landing");
}

bool safety_fence_check(int fence_type)
{
    if (fence_type == 0)
    {
        // square
        geometry_msgs::PoseStamped temp_pose_;
        temp_pose_ = current_pos;

        if (temp_pose_.pose.position.x <= fence_def[1] && temp_pose_.pose.position.x >= fence_def[0] && temp_pose_.pose.position.y <= fence_def[3] && temp_pose_.pose.position.y >= fence_def[2] && temp_pose_.pose.position.z <= fence_def[4])
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        ROS_INFO_ONCE("No safety_fence type definition");
        return true; // always safe if no type is defined.
    }
}

void position_to_vector(const geometry_msgs::PoseStamped pose, Eigen::Vector3d &output)
{
    output(0) = pose.pose.position.x;
    output(1) = pose.pose.position.y;
    output(2) = pose.pose.position.z;
}

void output_constrains(Eigen::Vector3d &cmd, double max_cmd)
{
    cmd(0) = std::max(std::min(cmd(0), max_cmd), -max_cmd);
    cmd(1) = std::max(std::min(cmd(1), max_cmd), -max_cmd);
    cmd(2) = std::max(std::min(cmd(2), max_cmd), -max_cmd);
}

#endif /* PX4_CONTROLLER_HPP */