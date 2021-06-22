#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP
/*
 * @Author: Wei Luo
 * @Date: 2021-03-22 14:05:09
 * @LastEditors: Wei Luo
 * @LastEditTime: 2021-05-24 18:00:41
 * @Note: Note
 */

#include <geometry_msgs/PoseStamped.h>
#include <itm_mav_srvs/GetControllerState.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
// #include <itm_nonlinear_mpc/itm_trajectory_srv.h>
#include <itm_mav_msgs/itm_trajectory_msg.h>

// std vector
#include <vector>
// Eigen
#include <Eigen/Dense>
// boost
#include <boost/thread.hpp>

class PIDController
{
public:
    PIDController(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh);
    ~PIDController();
    void landing();
    void takeoff(ros::Time time_stamp, geometry_msgs::PoseStamped *msg);
    void takeoff_target();
    void mission_loop();
    void idle();
    bool is_initialized();
    int command_id;

private:
    ros::NodeHandle nh_, private_nh_;
    ros::Publisher command_position_pub_;
    ros::Publisher command_position_target_pub_;
    double takeoff_height_;
    Eigen::Vector3d robot_current_pose;
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped initial_pose_;
    nav_msgs::Odometry initial_odom_;
    geometry_msgs::PoseStamped takeoff_pose_;
    ros::Subscriber robot_pose_sub_;
    ros::Subscriber trajectory_sub_;
    ros::ServiceServer controller_server_;
    // ros::Publisher robot_loc_pose_pub_;
    std::string robot_name_;
    bool is_current_pose_sub_;
    bool is_robot_client_connected_;
    bool is_trajectory_provided_;
    bool is_only_sim;
    // mutex
    boost::shared_mutex mutexTrajectoryCallback_;
    // landing
    bool get_landing_last_pose_;
    geometry_msgs::PoseStamped last_pose_;
    Eigen::Quaterniond last_quaternion_;
    Eigen::Vector3d last_position_error_;
    Eigen::Vector3d integral_error_;

    std::shared_ptr<itm_mav_msgs::itm_trajectory_msg> trajectory_ref_point_;

    void robot_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void trajectory_callback(const itm_mav_msgs::itm_trajectory_msg::ConstPtr &msg);
    bool controller_server_response_callback(itm_mav_srvs::GetControllerState::Request &req, itm_mav_srvs::GetControllerState::Response &res);

    void current_odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
    void rpy_to_quaternion(Eigen::Vector3d rpy, Eigen::Quaterniond &q);
    void quaternion_to_rpy(Eigen::Quaterniond q, Eigen::Vector3d &rpy);

    void output_constrains(Eigen::Vector3d &cmd, double max_cmd);
};

#endif /* PID_CONTROLLER_HPP */