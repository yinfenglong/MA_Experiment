/*
 * @Author: Wei Luo
 * @Date: 2021-04-04 00:13:31
 * @LastEditors: Wei Luo
 * @LastEditTime: 2021-07-09 11:06:11
 * @Note: Note
 */
#ifndef _MPC_ACADOS_SE3_HPP_
#define _MPC_ACADOS_SE3_HPP_

// ros
#include <geometry_msgs/PoseStamped.h>
#include <itm_mav_msgs/AttitudeCommand.h>
#include <itm_mav_msgs/PositionCommand.h>
#include <itm_mav_msgs/UavState.h>
#include <itm_mav_msgs/itm_trajectory_msg.h>
#include <itm_mav_srvs/GetControllerState.h>
#include <itm_nonlinear_mpc/controller/utilitis/se3_support.hpp>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
// Eigen
#include <Eigen/Dense>
// boost
#include <boost/thread.hpp>
// acados
#include <acados/utils/math.h>
#include <acados_c/ocp_nlp_interface.h>
#include <acados_sim_solver_quadrotor.h>
#include <acados_solver_quadrotor.h>

namespace acados_quadrotor
{
    class MPCAcadosController
    {
    public:
        MPCAcadosController(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh);
        ~MPCAcadosController();
        bool is_initialized();
        void takeoff();
        void landing();
        void mission_loop();
        void safe_landing();
        int command_id;

    private:
        /* ROS related */
        ros::NodeHandle nh_, private_nh_;
        ros::Subscriber robot_pose_sub_;
        ros::Subscriber trajectory_sub_;
        ros::ServiceServer controller_server_;
        ros::Publisher command_attitude_thrust_pub_;
        /* state check */
        bool is_current_pose_sub_;
        bool is_robot_client_connected_;
        bool is_trajectory_provided_;
        bool is_only_sim;
        /* states */
        nav_msgs::Odometry initial_odom_;
        double takeoff_height_;
        bool got_landing_pose;
        double landing_state[9];
        /* mutex */
        boost::shared_mutex mutexTrajectoryCallback_;
        boost::shared_mutex mutexMPCCallback_;
        std::shared_ptr<itm_mav_msgs::itm_trajectory_msg> trajectory_ref_point_;
        /* ACADOS */
        nlp_solver_capsule *acados_ocp_capsule;
        int acados_status;
        ocp_nlp_config *nlp_config;
        ocp_nlp_dims *nlp_dims;
        ocp_nlp_in *nlp_in;
        ocp_nlp_out *nlp_out;
        int time_horizon;
        int num_states;
        int num_controls;
        double robot_current_state[9]; // x, y, z, vx, vy, vz, roll, pitch, yaw
        double robot_init_state[9];
        double robot_command[3]; // roll_d, pitch_d, thrust
        double thrust_cmd;       //thrust
        Eigen::MatrixXd trajectory_reference;
        double control_acc_offset;

        // se3
        bool is_first_iteration_;
        itm_mav_msgs::UavState uav_state;
        ros::Time last_update_time_;
        itm_mav_msgs::AttitudeCommand activation_attitude_cmd_;
        itm_mav_msgs::AttitudeCommand::ConstPtr last_attitude_cmd_;
        double hover_thrust_;
        double uav_mass_;
        double uav_mass_difference_;
        double motor_parameter_a;
        double motor_parameter_b;
        Eigen::Vector2d Ib_b_; // body error integral in the body frame
        Eigen::Vector2d Iw_w_; // world error integral in the world frame
        bool rampup_active_;
        bool integral_terms_enabled_;

        /* only for safe landing with PID */
        Eigen::Vector3d last_position_error_;
        Eigen::Quaterniond last_quaternion_;
        ros::Publisher command_position_target_pub_;

        /* callback functions */
        void robot_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void trajectory_callback(const itm_mav_msgs::itm_trajectory_msg::ConstPtr &msg);
        bool controller_server_response_callback(itm_mav_srvs::GetControllerState::Request &req, itm_mav_srvs::GetControllerState::Response &res);
        void current_odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
        /* math functions */
        void rpy_to_quaternion(Eigen::Vector3d rpy, Eigen::Quaterniond &q);
        void load_trajectory(Eigen::MatrixXd &trajectory);
        void output_constrains(Eigen::Vector3d &cmd, double max_cmd);
        /* ACADOS calculation */
        void solvingACADOS(Eigen::MatrixXd ref); //, double *estimated_control
        void quaternion_to_rpy(Eigen::Quaterniond q, Eigen::Vector3d &rpy);

        /* SE3 */
        const itm_mav_msgs::AttitudeCommand::ConstPtr update(const itm_mav_msgs::PositionCommand::ConstPtr &pos_command);
        /* multi thread */
        boost::thread mpc_thread;
        void mpc_thread_function();
    };
}

#endif /* _MPC_ACADOS_HPP_ */