/*
 * @Author: Wei Luo
 * @Date: 2021-07-05 12:41:02
 * @LastEditors: Wei Luo
 * @LastEditTime: 2021-07-10 21:38:24
 * @Note: Note
 */

#ifndef _SE3_SUPPORT_HPP_
#define _SE3_SUPPORT_HPP_
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

double getHeading(geometry_msgs::Quaternion q);
double getHeadingRate(const geometry_msgs::Quaternion q, const Eigen::Vector3d yaw_array);
Eigen::Matrix3d geom_q_to_rotation(const geometry_msgs::Quaternion q);
double getYawRateIntrinsic(const geometry_msgs::Quaternion q, const double &heading_rate);
double forceToThrust(const double motor_parameter_a, const double motor_parameter_b, const double force, const int num_motors);
Eigen::Matrix3d setHeadingByYaw(double heading, Eigen::Matrix3d rd);
double angleBetween(const Eigen::Vector3d &vec1, const Eigen::Vector3d &vec2);
geometry_msgs::Quaternion getQuaternionfromMatrix(const Eigen::Matrix3d rd);

tf2::Quaternion tf2_quaternion_;

#endif //_SE3_SUPPORT_HPP_