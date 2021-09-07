/*
 * @Author: Wei Luo
 * @Date: 2021-04-04 00:12:43
 * @LastEditors: Wei Luo
 * @LastEditTime: 2021-08-27 15:41:49
 * @Note: Note
 */
#include <itm_nonlinear_mpc/controller/mpc_acados/mpc_q_acados_complex.hpp>
namespace acados_quadrotor
{
    MPCAcadosController::MPCAcadosController(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh) : nh_(nh),
                                                                                                             private_nh_(private_nh),
                                                                                                             is_trajectory_provided_(false),
                                                                                                             is_current_pose_sub_(false),
                                                                                                             is_robot_client_connected_(false),
                                                                                                             got_landing_pose(false),
                                                                                                             got_pid_thrust_(false),
                                                                                                             thrust_counter_(0),
                                                                                                             is_velocity_progress_init_(false),
                                                                                                             has_landing_target(false)
    {
        /* get some parameters */
        private_nh_.getParam("takeoff_height", takeoff_height_);
        private_nh_.getParam("sim_only", is_only_sim);
        if (private_nh_.hasParam("control_offset"))
            private_nh_.getParam("control_offset", control_acc_offset);
        else
            control_acc_offset = 0.5;
        private_nh_.getParam("takeoff_height", takeoff_height_);
        if (private_nh_.hasParam("landing_target_topic"))
        {
            has_landing_target = true;
            private_nh_.getParam("landing_target_topic", landing_target_pose_topic);
        }
        /* publisher */
        command_attitude_thrust_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 100);
        command_position_target_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
        mission_mode_pub_ = nh_.advertise<itm_mav_msgs::SetMission>("/itm_quadrotor_control/user_command", 10);
        /* subscribe topics */
        if (is_only_sim)
            robot_pose_sub_ = nh_.subscribe<nav_msgs::Odometry>("/real_pose", 10, &MPCAcadosController::current_odom_callback, this);
        else
        {
            robot_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/real_pose", 10, &MPCAcadosController::robot_pose_callback, this);
        }
        trajectory_sub_ = nh_.subscribe<itm_mav_msgs::itm_trajectory_msg>("/robot_trajectory", 10, &MPCAcadosController::trajectory_callback, this);
        pid_target_control_sub_ = nh_.subscribe<mavros_msgs::ActuatorControl>("/mavros/target_actuator_control", 10, &MPCAcadosController::target_actual_control_callback, this);
        if (has_landing_target)
        {
            landing_target_2D_pose = Eigen::Vector3d::Zero();
            std::cout << "topic :" << landing_target_pose_topic << std::endl;
            if (is_only_sim)
                landing_target_pose_sub_ = nh_.subscribe<nav_msgs::Odometry>(landing_target_pose_topic, 10, &MPCAcadosController::moving_target_odom_callback, this);
            else
                landing_target_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(landing_target_pose_topic, 10, &MPCAcadosController::moving_target_pose_callback, this);
        }
        /* server */
        controller_server_ = nh_.advertiseService("/itm_quadrotor_control/get_controller_state", &MPCAcadosController::controller_server_response_callback, this);
        /* init ACADOS */
        acados_ocp_capsule = quadrotor_q_acados_create_capsule();
        acados_status = quadrotor_q_acados_create(acados_ocp_capsule);
        if (acados_status)
        {
            ROS_ERROR("Cannot create the ACADOS solver!");
        }
        nlp_config = quadrotor_q_acados_get_nlp_config(acados_ocp_capsule);
        nlp_dims = quadrotor_q_acados_get_nlp_dims(acados_ocp_capsule);
        nlp_in = quadrotor_q_acados_get_nlp_in(acados_ocp_capsule);
        nlp_out = quadrotor_q_acados_get_nlp_out(acados_ocp_capsule);

        time_horizon = nlp_dims->N;
        num_states = *nlp_dims->nx;
        num_controls = *nlp_dims->nu;

        trajectory_reference = Eigen::MatrixXd::Zero(time_horizon + 1, num_states + num_controls);
        robot_des_quaternion[0] = 1.0;
        robot_des_quaternion[1] = 0.0;
        robot_des_quaternion[2] = 0.0;
        robot_des_quaternion[3] = 0.0;
        // create a thread for MPC
        mpc_thread = boost::thread(boost::bind(&MPCAcadosController::mpc_thread_function, this));

        ROS_INFO_STREAM("Time horizon is " << time_horizon << ", with "
                                           << num_states << " states, and " << num_controls << " controls");
    }

    MPCAcadosController::~MPCAcadosController()
    {
        mpc_thread.join();
    }

    void MPCAcadosController::mpc_thread_function()
    {
        // check if MPC is ready
        const auto wait_duration = boost::chrono::milliseconds(2000);
        while (!is_initialized())
            boost::this_thread::sleep_for(wait_duration);
        ros::Rate mpc_rate(100);
        while (ros::ok())
        {
            if (command_id > 0 and command_id != 4) // not idle or emergency landing mode
            {
                {
                    boost::unique_lock<boost::shared_mutex> mpc_lk(mutexMPCCallback_);
                    solvingACADOS(trajectory_reference);
                }
            }
            mpc_rate.sleep();
        }
    }

    void MPCAcadosController::takeoff()
    {
        ROS_INFO_ONCE("Got offset %.2f", control_acc_offset);
        for (int i = 0; i < time_horizon + 1; i++)
        {
            trajectory_reference(i, 0) = robot_init_state[0];
            trajectory_reference(i, 1) = robot_init_state[1];
            trajectory_reference(i, 2) = takeoff_height_;
            trajectory_reference(i, 3) = 1.0;
            trajectory_reference(i, 4) = 0.0;
            trajectory_reference(i, 5) = 0.0;
            trajectory_reference(i, 6) = 0.0;
            trajectory_reference(i, 7) = 0.0;
            trajectory_reference(i, 8) = 0.0;
            trajectory_reference(i, 9) = 0.0;
            trajectory_reference(i, 10) = 0.0;
            trajectory_reference(i, 11) = 0.0;
            trajectory_reference(i, 12) = 0.0;
            trajectory_reference(i, 13) = 9.8066;
        }

        mavros_msgs::AttitudeTarget msg;
        msg.header.stamp = ros::Time::now();
        msg.type_mask = 128;

        {
            boost::shared_lock<boost::shared_mutex> mpc_lk(mutexMPCCallback_);
            double computed_thrust;
            computed_thrust = robot_command[3] / 9.8066 * control_acc_offset;
            // max acceleration based on the constraint defined in MPC. One can edit it accordingly
            if (computed_thrust >= 1.05 * control_acc_offset)
                computed_thrust = 1.05 * control_acc_offset;
            msg.thrust = computed_thrust;

            msg.body_rate.x = robot_command[0];
            msg.body_rate.y = robot_command[1];
            msg.body_rate.z = robot_command[2];
            command_attitude_thrust_pub_.publish(msg);
        }
    }

    void MPCAcadosController::safe_takeoff()
    {
        mavros_msgs::PositionTarget msg;
        msg.header.stamp = ros::Time::now();
        msg.type_mask = 0b100111111000;
        msg.coordinate_frame = 1;

        msg.position.x = robot_init_state[0];
        msg.position.y = robot_init_state[1];
        msg.position.z = takeoff_height_; // robot_init_state[2] +
        // std::cout << msg.position.x << " " << msg.position.y << " " << msg.position.z << std::endl;

        msg.yaw = 0.0;
        command_position_target_pub_.publish(msg);

        // prepare also for MPC
        for (int i = 0; i < time_horizon + 1; i++)
        {
            trajectory_reference(i, 0) = robot_init_state[0];
            trajectory_reference(i, 1) = robot_init_state[1];
            trajectory_reference(i, 2) = takeoff_height_;
            trajectory_reference(i, 3) = 1.0;
            trajectory_reference(i, 4) = 0.0;
            trajectory_reference(i, 5) = 0.0;
            trajectory_reference(i, 6) = 0.0;
            trajectory_reference(i, 7) = 0.0;
            trajectory_reference(i, 8) = 0.0;
            trajectory_reference(i, 9) = 0.0;
            trajectory_reference(i, 10) = 0.0;
            trajectory_reference(i, 11) = 0.0;
            trajectory_reference(i, 12) = 0.0;
            trajectory_reference(i, 13) = 9.8066;
        }
    }

    void MPCAcadosController::landing()
    {
        if (!got_landing_pose)
        {
            std::copy(std::begin(robot_current_state), std::end(robot_current_state), std::begin(landing_state));
            got_landing_pose = true;
            // landing_state[2] = robot_init_state[2] + 0.1;
            landing_state[7] = 0.0;
            landing_state[8] = 0.0;
            landing_state[9] = 0.0;
            for (int i = 0; i < num_states; ++i)
                std::cout << landing_state[i] << std::endl;
            std::cout << "got landing state" << std::endl;

            if (has_landing_target)
            {
                double target_error_ = std::sqrt(std::pow(robot_current_state[0] - landing_target_2D_pose[0], 2) + std::pow(robot_current_state[1] - landing_target_2D_pose[1], 2));
                if (target_error_ <= 0.2)
                {
                    // above the landing platform
                    for (int i = 0; i < 3; i++)
                        landing_state[i] = landing_target_2D_pose[i];
                }
                else
                {
                    landing_state[2] = robot_init_state[2];
                }
            }
            else
            {
                // setup the landing target is the initial takeoff height
                landing_state[2] = robot_init_state[2];
            }
        }

        for (int i = 0; i < time_horizon + 1; i++)
        {
            trajectory_reference(i, 0) = robot_current_state[0] + i * (landing_state[0] - robot_current_state[0]) / time_horizon;
            trajectory_reference(i, 1) = robot_current_state[1] + i * (landing_state[1] - robot_current_state[1]) / time_horizon;
            trajectory_reference(i, 2) = robot_current_state[2] + i * (landing_state[2] - robot_current_state[2]) / time_horizon;
            trajectory_reference(i, 3) = landing_state[3];
            trajectory_reference(i, 4) = landing_state[4];
            trajectory_reference(i, 5) = landing_state[5];
            trajectory_reference(i, 6) = landing_state[6];
            trajectory_reference(i, 7) = landing_state[7];
            trajectory_reference(i, 8) = landing_state[8];
            trajectory_reference(i, 9) = landing_state[9];
            trajectory_reference(i, 10) = 0.0;
            trajectory_reference(i, 11) = 0.0;
            trajectory_reference(i, 12) = 0.0;
            trajectory_reference(i, 13) = 9.8066;
        }

        mavros_msgs::AttitudeTarget msg;
        msg.header.stamp = ros::Time::now();
        msg.type_mask = 128;
        msg.thrust = robot_command[3] / 9.8066 * control_acc_offset;

        // if one specifics a landing platform
        if (has_landing_target)
        {
            double target_error_ = std::sqrt(std::pow(robot_current_state[0] - landing_target_2D_pose[0], 2) + std::pow(robot_current_state[1] - landing_target_2D_pose[1], 2));
            // ! currently the diameter of the landing platform is 42 cm
            if (target_error_ < 0.35)
            {                                                                                           // landing should be on the landing platform
                if (target_error_ < 0.05 && robot_current_state[2] - landing_target_2D_pose[2] <= 0.10) // ! height difference between robot and platform
                {
                    ROS_INFO_ONCE("set thrust 0.1");
                    msg.thrust = 0.1;
                }
                // else
                // {
                //     std::cout << target_error_ << std::endl;
                //     std::cout << robot_current_state[2] - landing_target_2D_pose[2] << std::endl;
                // }
            }
            else
            {
                if (robot_current_state[2] - robot_init_state[2] <= 0.05) // 5 cm above the initial height, ready for shouting down the motors
                {
                    ROS_INFO_ONCE("set thrust 0.1");
                    msg.thrust = 0.1;
                }
            }
        }
        else
        {
            //so if one has significant position difference, one should consider it cannot landing on the landing platform.
            if (robot_current_state[2] - robot_init_state[2] <= 0.05) // 5 cm above the initial height, ready for shouting down the motors
            {
                ROS_INFO_ONCE("set thrust 0.1");
                msg.thrust = 0.1;
            }
        }

        msg.body_rate.x = robot_command[0];
        msg.body_rate.y = robot_command[1];
        msg.body_rate.z = robot_command[2];

        command_attitude_thrust_pub_.publish(msg);
    }

    void MPCAcadosController::safe_landing()
    {

        if (!got_landing_pose)
        {
            got_landing_pose = true;
            std::copy(std::begin(robot_current_state), std::end(robot_current_state), std::begin(landing_state));
            landing_state[7] = 0.0;
            landing_state[8] = 0.0;
            landing_state[9] = 0.0;
            last_quaternion_.w() = landing_state[3];
            last_quaternion_.x() = landing_state[4];
            last_quaternion_.y() = landing_state[5];
            last_quaternion_.z() = landing_state[6];
            last_position_error_.setZero(3);
        }

        mavros_msgs::PositionTarget msg;
        Eigen::Vector3d pose_error;
        Eigen::Vector3d delta_error;
        Eigen::Vector3d vec_cmd;
        pose_error(0) = landing_state[0] - robot_current_state[0];
        pose_error(1) = landing_state[1] - robot_current_state[1];
        pose_error(2) = robot_init_state[2] - robot_current_state[2];

        delta_error = pose_error - last_position_error_;
        vec_cmd = 0.35 * pose_error + delta_error * 0.001;
        output_constrains(vec_cmd, 0.4);
        vec_cmd(2) = pose_error(2);

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

    void MPCAcadosController::mission_loop()
    {
        if (is_trajectory_provided_)
        {
            load_trajectory(trajectory_reference);
            ROS_INFO_ONCE("Got trajectory");
        }
        else
        {
            for (int i = 0; i < time_horizon + 1; i++)
            {
                trajectory_reference(i, 0) = robot_init_state[0];
                trajectory_reference(i, 1) = robot_init_state[1];
                trajectory_reference(i, 2) = takeoff_height_;
                trajectory_reference(i, 3) = 1.0;
                trajectory_reference(i, 4) = 0.0;
                trajectory_reference(i, 5) = 0.0;
                trajectory_reference(i, 6) = 0.0;
                trajectory_reference(i, 7) = 0.0;
                trajectory_reference(i, 8) = 0.0;
                trajectory_reference(i, 9) = 0.0;
                trajectory_reference(i, 10) = 0.0;
                trajectory_reference(i, 11) = 0.0;
                trajectory_reference(i, 12) = 0.0;
                trajectory_reference(i, 13) = 9.8066;
            }
            ROS_INFO_ONCE("Waiting for trajectory");
        }
        mavros_msgs::AttitudeTarget msg;
        msg.header.stamp = ros::Time::now();
        msg.type_mask = 128;
        msg.thrust = robot_command[3] / 9.8066 * control_acc_offset;
        msg.body_rate.x = robot_command[0];
        msg.body_rate.y = robot_command[1];
        msg.body_rate.z = robot_command[2];

        command_attitude_thrust_pub_.publish(msg);

        // if one specifics a landing target
        if (has_landing_target)
        {
            std::cout << robot_current_state[0] << robot_current_state[1] << std::endl;
            std::cout << landing_target_2D_pose << std::endl;
            double target_error_ = std::sqrt(std::pow(robot_current_state[0] - landing_target_2D_pose[0], 2) + std::pow(robot_current_state[1] - landing_target_2D_pose[1], 2));
            std::cout << "target error: " << target_error_ << std::endl;
            std::cout << "height error: " << robot_current_state[2] - landing_target_2D_pose[2] << std::endl;

            if (target_error_ < 0.08 && robot_current_state[2] > landing_target_2D_pose[2])
            {
                ROS_INFO("start landing");
                itm_mav_msgs::SetMission mode_message_;
                mode_message_.header.stamp = ros::Time::now();
                mode_message_.mission_mode = 2;
                mission_mode_pub_.publish(mode_message_);
            }
        }
    }

    void MPCAcadosController::load_trajectory(Eigen::MatrixXd &trajectory)
    {
        boost::shared_lock<boost::shared_mutex> mutexTrajectoryCallback(mutexTrajectoryCallback_);

        int num_waypoints = trajectory_ref_point_->size;

        if (num_waypoints > 1)
        {
            int num_diff = 0;
            if (num_waypoints != time_horizon + 1)
            {
                num_diff = (time_horizon + 1 - num_waypoints > 0) ? (time_horizon + 1 - num_waypoints) : (-time_horizon - 1 + num_waypoints);
            }
            int num_min_waypoints = std::min(num_waypoints, time_horizon);

            // std::cout << "num_min_waypoints" << num_min_waypoints << std::endl;

            for (int i = 0; i < num_min_waypoints + 1; i++)
            {
                trajectory(i, 0) = trajectory_ref_point_->traj[i].x;
                trajectory(i, 1) = trajectory_ref_point_->traj[i].y;
                trajectory(i, 2) = trajectory_ref_point_->traj[i].z;
                if (trajectory_ref_point_->traj[i].quaternion_given)
                {
                    trajectory(i, 3) = trajectory_ref_point_->traj[i].q[0];
                    trajectory(i, 4) = trajectory_ref_point_->traj[i].q[1];
                    trajectory(i, 5) = trajectory_ref_point_->traj[i].q[2];
                    trajectory(i, 6) = trajectory_ref_point_->traj[i].q[3];
                }
                else
                {
                    /* convert rpy to quaternion */
                    Eigen::Vector3d rpy = {trajectory_ref_point_->traj[i].roll, trajectory_ref_point_->traj[i].pitch, trajectory_ref_point_->traj[i].yaw};
                    Eigen::Quaterniond quat;
                    rpy_to_quaternion(rpy, quat);
                    trajectory(i, 3) = quat.w();
                    trajectory(i, 4) = quat.x();
                    trajectory(i, 5) = quat.y();
                    trajectory(i, 6) = quat.z();
                }
                trajectory(i, 7) = trajectory_ref_point_->traj[i].vx;
                trajectory(i, 8) = trajectory_ref_point_->traj[i].vy;
                trajectory(i, 9) = trajectory_ref_point_->traj[i].vz;
                if (trajectory_ref_point_->traj[i].input_given)
                {
                    trajectory(i, 10) = trajectory_ref_point_->traj[i].roll_rate_des;
                    trajectory(i, 11) = trajectory_ref_point_->traj[i].pitch_rate_des;
                    trajectory(i, 12) = trajectory_ref_point_->traj[i].yaw_rate_des;
                    trajectory(i, 13) = trajectory_ref_point_->traj[i].thrust_des;
                }
                else
                {

                    trajectory(i, 10) = 0.0;
                    trajectory(i, 11) = 0.0;
                    trajectory(i, 12) = 0.0;
                    trajectory(i, 13) = 9.8066;
                }
            }

            if (num_diff > 0)
            {
                for (int i = num_min_waypoints; i < num_min_waypoints + num_diff; i++)
                {
                    /* repeat last required waypoint */
                    trajectory(i) = trajectory(num_min_waypoints - 1);
                }
            }
        }
        else
        {
            /* only a fixed point is provided */
            ROS_INFO_ONCE("Single point");
            for (int i = 0; i < time_horizon + 1; i++)
            {
                trajectory(i, 0) = trajectory_ref_point_->traj[0].x;
                trajectory(i, 1) = trajectory_ref_point_->traj[0].y;
                trajectory(i, 2) = trajectory_ref_point_->traj[0].z;
                if (trajectory_ref_point_->traj[0].quaternion_given)
                {
                    trajectory(i, 3) = trajectory_ref_point_->traj[0].q[0];
                    trajectory(i, 4) = trajectory_ref_point_->traj[0].q[1];
                    trajectory(i, 5) = trajectory_ref_point_->traj[0].q[2];
                    trajectory(i, 6) = trajectory_ref_point_->traj[0].q[3];
                }
                else
                {
                    /* convert rpy to quaternion */
                    Eigen::Vector3d rpy = {trajectory_ref_point_->traj[0].roll, trajectory_ref_point_->traj[0].pitch, trajectory_ref_point_->traj[0].yaw};
                    Eigen::Quaterniond quat;
                    rpy_to_quaternion(rpy, quat);
                    trajectory(i, 3) = quat.w();
                    trajectory(i, 4) = quat.x();
                    trajectory(i, 5) = quat.y();
                    trajectory(i, 6) = quat.z();
                }
                trajectory(i, 7) = trajectory_ref_point_->traj[0].vx;
                trajectory(i, 8) = trajectory_ref_point_->traj[0].vy;
                trajectory(i, 9) = trajectory_ref_point_->traj[0].vz;
                trajectory(i, 10) = 0.0;
                trajectory(i, 11) = 0.0;
                trajectory(i, 12) = 0.0;
                trajectory(i, 13) = 9.8066;
            }
        }
    }

    void MPCAcadosController::solvingACADOS(Eigen::MatrixXd ref)
    {
        std::vector<double> end_term_ref;
        for (int i = 0; i < 3; i++)
        {
            end_term_ref.push_back(ref(time_horizon, i));
        }
        for (int i = 7; i < num_states; i++)
        {
            end_term_ref.push_back(ref(time_horizon, i));
        }
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, time_horizon, "yref", &end_term_ref[0]);

        for (int i = 0; i < time_horizon; i++)
        {
            std::vector<double> y_ref;
            for (int j = 0; j < num_controls + num_states; j++)
            {
                y_ref.push_back(ref(i, j));
            }
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", &y_ref[0]);
        }
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", robot_current_state);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", robot_current_state);

        // solve MPC
        acados_status = quadrotor_q_acados_solve(acados_ocp_capsule);
        if (acados_status)
        {
            robot_command[0] = 0.0;
            robot_command[1] = 0.0;
            robot_command[2] = 0.0;
            robot_command[3] = 9.8066;
            ROS_INFO("Cannot solve the MPC");
        }
        else
        {
            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", &robot_command);
        }
    }

    /* callback functions */
    void MPCAcadosController::robot_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        robot_current_state[0] = msg->pose.position.x;
        robot_current_state[1] = msg->pose.position.y;
        robot_current_state[2] = msg->pose.position.z;

        if (msg->pose.orientation.w < 0.0)
        {
            robot_current_state[3] = -msg->pose.orientation.w;
            robot_current_state[4] = -msg->pose.orientation.x;
            robot_current_state[5] = -msg->pose.orientation.y;
            robot_current_state[6] = -msg->pose.orientation.z;
        }
        else
        {
            robot_current_state[3] = msg->pose.orientation.w;
            robot_current_state[4] = msg->pose.orientation.x;
            robot_current_state[5] = msg->pose.orientation.y;
            robot_current_state[6] = msg->pose.orientation.z;
        }
        // need a state observer ? KF?
        Eigen::Vector3d velocity_uav;
        current_time = msg->header.stamp.toSec();
        current_position_ << robot_current_state[0], robot_current_state[1], robot_current_state[2];
        velocity_estimation(velocity_uav);
        robot_current_state[7] = velocity_uav[0];
        robot_current_state[8] = velocity_uav[1];
        robot_current_state[9] = velocity_uav[2];

        if (!is_current_pose_sub_)
        {
            is_current_pose_sub_ = true;
            for (int i = 0; i < num_states; i++)
                robot_init_state[i] = robot_current_state[i];
        }
    }

    void MPCAcadosController::trajectory_callback(const itm_mav_msgs::itm_trajectory_msg::ConstPtr &msg)
    {
        if (!is_trajectory_provided_)
            is_trajectory_provided_ = true;
        {
            boost::unique_lock<boost::shared_mutex> lockTrajectoryCallback(mutexTrajectoryCallback_);
            trajectory_ref_point_ = std::make_shared<itm_mav_msgs::itm_trajectory_msg>(*msg);
        }
    }

    void MPCAcadosController::current_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        robot_current_state[0] = msg->pose.pose.position.x;
        robot_current_state[1] = msg->pose.pose.position.y;
        robot_current_state[2] = msg->pose.pose.position.z;
        robot_current_state[3] = msg->pose.pose.orientation.w;
        robot_current_state[4] = msg->pose.pose.orientation.x;
        robot_current_state[5] = msg->pose.pose.orientation.y;
        robot_current_state[6] = msg->pose.pose.orientation.z;
        // robot_current_state[7] = msg->twist.twist.linear.x;
        // robot_current_state[8] = msg->twist.twist.linear.y;
        // robot_current_state[9] = msg->twist.twist.linear.z;
        Eigen::Vector3d velocity_uav;
        current_time = msg->header.stamp.toSec();
        current_position_ << robot_current_state[0], robot_current_state[1], robot_current_state[2];
        velocity_estimation(velocity_uav);
        robot_current_state[7] = velocity_uav[0];
        robot_current_state[8] = velocity_uav[1];
        robot_current_state[9] = velocity_uav[2];
        if (!is_current_pose_sub_)
        {
            is_current_pose_sub_ = true;
            for (int i = 0; i < num_states; i++)
            {
                robot_init_state[i] = robot_current_state[i];
                // std::cout << robot_init_state[i] << std::endl;
            }
        }
    }

    void MPCAcadosController::target_actual_control_callback(const mavros_msgs::ActuatorControlConstPtr &msg)
    {
        if (command_id == 1)
        {
            if (!got_pid_thrust_)
                got_pid_thrust_ = true;
            thrust_counter_ += 1;
            control_acc_offset = control_acc_offset * 0.3 + msg->controls[3] * 0.7;
        }
    }

    void MPCAcadosController::moving_target_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (has_landing_target)
            landing_target_2D_pose << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        else
        {
            Eigen::Quaterniond q_;
            q_.w() = msg->pose.orientation.w;
            q_.x() = msg->pose.orientation.x;
            q_.y() = msg->pose.orientation.y;
            q_.z() = msg->pose.orientation.z;
            Eigen::Vector3d v_;
            quaternion_to_rpy(q_, v_);
            //TODO: target_2D_pose << msg->pose.position.x, msg->pose.position.y, v_[2];
        }
    }

    void MPCAcadosController::moving_target_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        if (has_landing_target)
            landing_target_2D_pose << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z + 0.45;
        else
        {
            Eigen::Quaterniond q_;
            q_.w() = msg->pose.pose.orientation.w;
            q_.x() = msg->pose.pose.orientation.x;
            q_.y() = msg->pose.pose.orientation.y;
            q_.z() = msg->pose.pose.orientation.z;
            Eigen::Vector3d v_;
            quaternion_to_rpy(q_, v_);
            // TODO: target_2D_pose << msg->pose.pose.position.x, msg->pose.pose.position.y, v_[2];
        }
    }

    bool MPCAcadosController::controller_server_response_callback(itm_mav_srvs::GetControllerState::Request &req,
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

    bool MPCAcadosController::is_initialized()
    {
        if (is_robot_client_connected_ && is_current_pose_sub_)
        // if (is_current_pose_sub_) //MPC test!!!
        {
            // ROS_INFO_STREAM("MPC got robot current pose" << robot_init_state);
            return true;
        }
        else
            return false;
    }

    void MPCAcadosController::rpy_to_quaternion(Eigen::Vector3d rpy, Eigen::Quaterniond &q)
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

        q.normalize();
    }

    void MPCAcadosController::quaternion_to_rpy(Eigen::Quaterniond q, Eigen::Vector3d &rpy)
    {
        auto q0 = q.w();
        auto q1 = q.x();
        auto q2 = q.y();
        auto q3 = q.z();

        rpy(0) = std::atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (std::pow(q1, 2) + std::pow(q2, 2)));
        rpy(1) = std::asin(2 * (q0 * q2 - q3 * q1));
        rpy(2) = std::atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (std::pow(q2, 2) + std::pow(q3, 2)));
    }

    void MPCAcadosController::output_constrains(Eigen::Vector3d &cmd, double max_cmd)
    {
        cmd(0) = std::max(std::min(cmd(0), max_cmd), -max_cmd);
        cmd(1) = std::max(std::min(cmd(1), max_cmd), -max_cmd);
        cmd(2) = std::max(std::min(cmd(2), max_cmd), -max_cmd);
    }

    void MPCAcadosController::velocity_estimation(Eigen::Vector3d &vel)
    {
        if (!is_velocity_progress_init_)
        {
            is_velocity_progress_init_ = true;
            last_position_ = current_position_;
            previous_time = current_time;
            last_velocity_ << 0.0, 0.0, 0.0;
        }
        else
        {
            double dt = current_time - previous_time;
            if (dt >= 0.01)
            {
                vel = (current_position_ - last_position_) / (1e-5 + dt);
                vel = 0.8 * vel + 0.2 * last_velocity_;

                last_velocity_ = vel;
                previous_time = current_time;
                last_position_ = current_position_;
            }
            else
            {
                // ROS_WARN("dt is too short %.2f", dt);
            }
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "QuadrotorMPCNode");
    ros::NodeHandle nh_node, private_nh_node("~");
    acados_quadrotor::MPCAcadosController controller_node(nh_node, private_nh_node);
    ros::Rate rate(100);

    /* publisher */
    ROS_INFO("Begin the loop.");
    while (ros::ok())
    {
        if (controller_node.is_initialized())
        {
            switch (controller_node.command_id)
            {
            case 0: // idle
                // controller_node.idle();
                break;
            case 1: // takeoff
            {
                ROS_INFO_ONCE("Take off");
                // controller_node.takeoff();
                controller_node.safe_takeoff();
                break;
            }
            case 2: // landing
            {
                controller_node.landing();
                // controller_node.safe_landing();
                ROS_INFO_ONCE("Landing");
                break;
            }
            case 3: // go to position
            {
                controller_node.mission_loop();
                ROS_INFO_ONCE("Go to Position Mode");
                break;
            }
            case 4: // Emergency landing
            {
                ROS_INFO_ONCE("MPC got Emergency Landing command, STOP!");
                break;
            }
            case 5: // mpc takeoff
            {
                ROS_INFO_ONCE("MPC takes over the Quadrotor");
                controller_node.takeoff();
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

    ROS_INFO("MPC controller is closed");
    return 0;
}