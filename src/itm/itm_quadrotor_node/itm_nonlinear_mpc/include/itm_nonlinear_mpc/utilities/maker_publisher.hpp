/*
 * @Author: Wei Luo
 * @Date: 2021-04-16 10:54:28
 * @LastEditors: Wei Luo
 * @LastEditTime: 2021-04-16 15:19:19
 * @Note: Note
 */
#ifndef __MARKER_PUBLISHER_HPP__
#define __MARKER_PUBLISHER_HPP__

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <itm_mav_msgs/itm_trajectory_msg.h>

#include <Eigen/Dense>
#include <cmath>

bool is_publish_path;
bool is_publish_markers;
bool is_sim_only;

int max_traj_length;

geometry_msgs::PoseStamped current_pose;
nav_msgs::Odometry current_odometry;
nav_msgs::Path robot_path;
visualization_msgs::Marker traj_points;
int num_path;
ros::Publisher path_pub;
ros::Publisher request_trajectory_pub;
ros::Publisher past_trajectory_pub;
ros::Subscriber robot_sub;
ros::Subscriber trajectory_sub;
#endif