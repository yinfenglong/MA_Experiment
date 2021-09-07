#ifndef _ITM_NONLINEAR_ARUCO_MPC_OFFBOARD_NODE_HPP_
#define _ITM_NONLINEAR_ARUCO_MPC_OFFBOARD_NODE_HPP_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

// mavros 
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

// multi threads
#include <boost/thread.hpp>


mavros_msgs::State mavros_state;
geometry_msgs::PoseStamped set_point_pos_;
bool mpc_set_point_pos_flag;
bool received_uav_gt_;
bool received_aruco_pose_;
nav_msgs::Odometry gps_pose;
geometry_msgs::PoseStamped aruco_pose;

bool mpc_avaliable_;

#endif // _ITM_NONLINEAR_ARUCO_M