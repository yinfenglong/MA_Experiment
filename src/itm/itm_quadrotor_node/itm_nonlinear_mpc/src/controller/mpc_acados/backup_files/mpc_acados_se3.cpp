/*
 * @Author: Wei Luo
 * @Date: 2021-07-07 00:12:43
 * @LastEditors: Wei Luo
 * @LastEditTime: 2021-07-10 22:04:43
 * @Note: Note
 */
#include <itm_nonlinear_mpc/controller/mpc_acados/mpc_acados_se3.hpp>
namespace acados_quadrotor
{
    MPCAcadosController::MPCAcadosController(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh) : nh_(nh),
                                                                                                             private_nh_(private_nh),
                                                                                                             is_trajectory_provided_(false),
                                                                                                             is_current_pose_sub_(false),
                                                                                                             is_robot_client_connected_(false),
                                                                                                             got_landing_pose(false),
                                                                                                             is_first_iteration_(true),
                                                                                                             motor_parameter_a(0.544),
                                                                                                             motor_parameter_b(-0.467)
    {
        /* get some parameters */
        private_nh_.getParam("takeoff_height", takeoff_height_);
        private_nh_.getParam("sim_only", is_only_sim);
        if (private_nh_.hasParam("control_offset"))
            private_nh_.getParam("control_offset", control_acc_offset);
        else
            control_acc_offset = 0.5;
        private_nh_.getParam("uav_mass", uav_mass_);
        private_nh_.getParam("takeoff_height", takeoff_height_);
        /* publisher */
        command_attitude_thrust_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 100);
        command_position_target_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
        /* subscribe topics */
        if (is_only_sim)
            robot_pose_sub_ = nh_.subscribe<nav_msgs::Odometry>("/real_pose", 10, &MPCAcadosController::current_odom_callback, this);
        else
        {
            robot_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/real_pose", 10, &MPCAcadosController::robot_pose_callback, this);
        }
        trajectory_sub_ = nh_.subscribe<itm_mav_msgs::itm_trajectory_msg>("/robot_trajectory", 10, &MPCAcadosController::trajectory_callback, this);
        /* server */
        controller_server_ = nh_.advertiseService("/itm_quadrotor_control/get_controller_state", &MPCAcadosController::controller_server_response_callback, this);
        /* init ACADOS */
        acados_ocp_capsule = quadrotor_acados_create_capsule();
        acados_status = quadrotor_acados_create(acados_ocp_capsule);
        if (acados_status)
        {
            ROS_ERROR("Cannot create the ACADOS solver");
        }
        nlp_config = quadrotor_acados_get_nlp_config(acados_ocp_capsule);
        nlp_dims = quadrotor_acados_get_nlp_dims(acados_ocp_capsule);
        nlp_in = quadrotor_acados_get_nlp_in(acados_ocp_capsule);
        nlp_out = quadrotor_acados_get_nlp_out(acados_ocp_capsule);

        time_horizon = nlp_dims->N;
        num_states = *nlp_dims->nx;
        num_controls = *nlp_dims->nu;

        trajectory_reference = Eigen::MatrixXd::Zero(time_horizon + 1, num_states + num_controls);

        // create a thread for MPC
        mpc_thread = boost::thread(boost::bind(&MPCAcadosController::mpc_thread_function, this));

        ROS_INFO_STREAM("Time horizon is " << time_horizon << ", with "
                                           << num_states << " states, and " << num_controls << " controls");

        // only for MPC test!!!
        // command_id = 1;
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
            if (command_id > 0 and command_id != 4) // not idle or emergency
            {
                {
                    boost::unique_lock<boost::shared_mutex> mpc_lk(mutexMPCCallback_);
                    // auto t_start = std::chrono::high_resolution_clock::now();
                    solvingACADOS(trajectory_reference);
                    // auto t_end = std::chrono::high_resolution_clock::now();
                    // double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
                    // std::cout << "time for MPC" << elapsed_time_ms << std::endl;
                }
            }
            mpc_rate.sleep();
        }
    }

    void MPCAcadosController::takeoff()
    {
        for (int i = 0; i < time_horizon + 1; i++)
        {
            trajectory_reference(i, 0) = robot_init_state[0];
            trajectory_reference(i, 1) = robot_init_state[1];
            trajectory_reference(i, 2) = robot_init_state[2] + takeoff_height_;
            trajectory_reference(i, 3) = 0.0;
            trajectory_reference(i, 4) = 0.0;
            trajectory_reference(i, 5) = 0.0;
            trajectory_reference(i, 6) = 0.0;
            trajectory_reference(i, 7) = 0.0;
            trajectory_reference(i, 8) = 0.0;
            trajectory_reference(i, 9) = 0.0;
            trajectory_reference(i, 10) = 0.0;
            trajectory_reference(i, 11) = 9.8066;
        }

        mavros_msgs::AttitudeTarget msg;
        msg.header.stamp = ros::Time::now();
        msg.type_mask = 7;

        {
            boost::shared_lock<boost::shared_mutex> mpc_lk(mutexMPCCallback_);
            // double computed_thrust;
            thrust_cmd = thrust_cmd * 0.3 + 0.7 * robot_command[2] / 9.8066 * control_acc_offset;
            // max acceleration based on the constraint defined in MPC. One can edit it accordingly
            if (thrust_cmd >= 1.5 * control_acc_offset)
                thrust_cmd = 1.5 * control_acc_offset;
            // std::cout << "final output : " << computed_thrust << std::endl;
            msg.thrust = thrust_cmd;
            // msg.thrust = robot_command[3] / 9.8066 - (1 - control_acc_offset);

            // tf2::Quaternion q_local_;
            // q_local_.setRPY(robot_command[0], robot_command[1], 0.0);
            // q_local_.normalize();
            // msg.orientation.x = q_local_[0];
            // msg.orientation.y = q_local_[1];
            // msg.orientation.z = q_local_[2];
            // msg.orientation.w = q_local_[3];
            Eigen::Quaterniond q_local_;
            Eigen::Vector3d rpy_local_;
            rpy_local_ << robot_command[0], robot_command[1], 0.0;
            // std::cout << "command " << robot_command[0] << robot_command[1] << std::endl;
            rpy_to_quaternion(rpy_local_, q_local_);

            msg.orientation.x = q_local_.x();
            msg.orientation.y = q_local_.y();
            msg.orientation.z = q_local_.z();
            msg.orientation.w = q_local_.w();

            // msg.body_rate.z = 0.0;
        }

        command_attitude_thrust_pub_.publish(msg);
    }

    void MPCAcadosController::landing()
    {
        // if (!got_landing_pose)
        // {
        //     std::copy(std::begin(robot_current_state), std::end(robot_current_state), std::begin(landing_state));
        //     got_landing_pose = true;
        //     // landing_state[2] = robot_init_state[2] + 0.1;
        //     landing_state[7] = 0.0;
        //     landing_state[8] = 0.0;
        //     landing_state[9] = 0.0;
        //     for (int i = 0; i < num_states; ++i)
        //         std::cout << landing_state[i] << std::endl;
        // }
        // Eigen::MatrixXd landing_trajectory(time_horizon + 1, 14);
        // mavros_msgs::AttitudeTarget msg;
        // for (int i = 0; i < time_horizon + 1; i++)
        // {
        //     landing_trajectory(i, 0) = landing_state[0];
        //     landing_trajectory(i, 1) = landing_state[1];
        //     landing_trajectory(i, 2) = robot_current_state[2] + (i + 1) * (robot_init_state[2] + 0.1 - robot_current_state[2]) / (time_horizon);
        //     landing_trajectory(i, 3) = landing_state[3];
        //     landing_trajectory(i, 4) = landing_state[4];
        //     landing_trajectory(i, 5) = landing_state[5];
        //     landing_trajectory(i, 6) = landing_state[6];
        //     landing_trajectory(i, 7) = landing_state[7];
        //     landing_trajectory(i, 8) = landing_state[8];
        //     landing_trajectory(i, 9) = landing_state[9];
        //     landing_trajectory(i, 10) = 0.0;
        //     landing_trajectory(i, 11) = 0.0;
        //     landing_trajectory(i, 12) = 0.0;
        //     landing_trajectory(i, 13) = 9.8066;
        // }
        // solvingACADOS(landing_trajectory);
        // msg.header.stamp = ros::Time::now();
        // msg.type_mask = 128;
        // msg.thrust = robot_command[3] / 9.8066 * control_acc_offset;

        // // ROS_INFO_STREAM("command: " << robot_command[3] << "thrust:" << msg.thrust);
        // msg.body_rate.x = robot_command[0];
        // msg.body_rate.y = robot_command[1];
        // msg.body_rate.z = robot_command[2];

        // command_attitude_thrust_pub_.publish(msg);
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
        output_constrains(vec_cmd, 0.25);

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
        // mavros_msgs::AttitudeTarget msg;
        if (is_trajectory_provided_)
        {
            load_trajectory(trajectory_reference);
            ROS_INFO_ONCE("Got trajectory");
            // solvingACADOS(trajectory_reference);
        }
        else
        {
            for (int i = 0; i < time_horizon + 1; i++)
            {
                trajectory_reference(i, 0) = robot_init_state[0];
                trajectory_reference(i, 1) = robot_init_state[1];
                trajectory_reference(i, 2) = robot_init_state[2] + takeoff_height_;
                trajectory_reference(i, 3) = 0.0;
                trajectory_reference(i, 4) = 0.0;
                trajectory_reference(i, 5) = 0.0;
                trajectory_reference(i, 6) = 0.0;
                trajectory_reference(i, 7) = 0.0;
                trajectory_reference(i, 8) = 0.0;
                trajectory_reference(i, 9) = 0.0;
                trajectory_reference(i, 10) = 0.0;
                trajectory_reference(i, 11) = 9.8066;
            }
            ROS_INFO_ONCE("Waiting for trajectory");
        }
        mavros_msgs::AttitudeTarget msg;
        msg.header.stamp = ros::Time::now();
        msg.type_mask = 3;

        {
            boost::shared_lock<boost::shared_mutex> mpc_lk(mutexMPCCallback_);
            double computed_thrust;
            computed_thrust = robot_command[2] / 9.8066 * control_acc_offset;
            // max acceleration based on the constraint defined in MPC. One can edit it accordingly
            if (computed_thrust >= 1.5 * control_acc_offset)
                computed_thrust = 1.5 * control_acc_offset;
            // std::cout << "final output : " << computed_thrust << std::endl;
            msg.thrust = computed_thrust;
            // msg.thrust = robot_command[3] / 9.8066 - (1 - control_acc_offset);

            // tf2::Quaternion q_local_;
            // q_local_.setRPY(robot_command[0], robot_command[1], 0.0);
            // q_local_.normalize();
            // msg.orientation.x = q_local_[0];
            // msg.orientation.y = q_local_[1];
            // msg.orientation.z = q_local_[2];
            // msg.orientation.w = q_local_[3];
            Eigen::Quaterniond q_local_;
            Eigen::Vector3d rpy_local_;
            rpy_local_ << robot_command[0], robot_command[1], 0.0;
            // std::cout << "command " << robot_command[0] << robot_command[1] << std::endl;
            rpy_to_quaternion(rpy_local_, q_local_);

            msg.orientation.x = q_local_.x();
            msg.orientation.y = q_local_.y();
            msg.orientation.z = q_local_.z();
            msg.orientation.w = q_local_.w();

            msg.body_rate.z = 0.0;
        }

        // ROS_INFO_STREAM("msg output :" << msg);

        command_attitude_thrust_pub_.publish(msg);
    }

    void MPCAcadosController::load_trajectory(Eigen::MatrixXd &trajectory)
    {
        boost::shared_lock<boost::shared_mutex> mutexTrajectoryCallback(mutexTrajectoryCallback_);

        int num_waypoints = trajectory_ref_point_->size;

        if (num_waypoints > 1)
        {
            int num_diff = 0;
            if (num_waypoints != time_horizon)
            {
                num_diff = (time_horizon - num_waypoints > 0) ? (time_horizon - num_waypoints) : (-time_horizon + num_waypoints);
            }
            int num_min_waypoints = std::min(num_waypoints, time_horizon);

            for (int i = 0; i < num_min_waypoints; i++)
            {
                trajectory(i, 0) = trajectory_ref_point_->traj[i].x;
                trajectory(i, 1) = trajectory_ref_point_->traj[i].y;
                trajectory(i, 2) = trajectory_ref_point_->traj[i].z;
                if (trajectory_ref_point_->traj[i].quaternion_given)
                {
                    /* convert quaternion to rpy */
                    Eigen::Quaterniond q_(trajectory_ref_point_->traj[i].q[0], trajectory_ref_point_->traj[i].q[1], trajectory_ref_point_->traj[i].q[2], trajectory_ref_point_->traj[i].q[3]);
                    Eigen::Vector3d rpy_;
                    quaternion_to_rpy(q_, rpy_);
                    trajectory(i, 3) = rpy_[0];
                    trajectory(i, 4) = rpy_[1];
                    trajectory(i, 5) = rpy_[2];
                }
                else
                {
                    trajectory(i, 3) = trajectory_ref_point_->traj[i].roll;
                    trajectory(i, 4) = trajectory_ref_point_->traj[i].pitch;
                    trajectory(i, 5) = trajectory_ref_point_->traj[i].yaw;
                }
                trajectory(i, 6) = trajectory_ref_point_->traj[i].vx;
                trajectory(i, 7) = trajectory_ref_point_->traj[i].vy;
                trajectory(i, 8) = trajectory_ref_point_->traj[i].vz;
                trajectory(i, 9) = 0.0;
                trajectory(i, 10) = 0.0;
                trajectory(i, 11) = 9.8066;
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
                // if (trajectory_ref_point_->traj[0].quaternion_given)
                // {
                //     /* convert quaternion to rpy */
                //     Eigen::Quaterniond q_(trajectory_ref_point_->traj[i].q[0], trajectory_ref_point_->traj[i].q[1], trajectory_ref_point_->traj[i].q[2], trajectory_ref_point_->traj[i].q[3]);
                //     Eigen::Vector3d rpy_;
                //     quaternion_to_rpy(q_, rpy_);
                //     trajectory(i, 3) = rpy_[0];
                //     trajectory(i, 4) = rpy_[1];
                //     trajectory(i, 5) = rpy_[2];
                //     std::cout << rpy_[0] << rpy_[1] << rpy_[2] << std::endl;
                // }
                // else
                // {
                ROS_INFO_ONCE("directly rpy");
                trajectory(i, 3) = trajectory_ref_point_->traj[0].vx;
                trajectory(i, 4) = trajectory_ref_point_->traj[0].vy;
                trajectory(i, 5) = trajectory_ref_point_->traj[0].vz;
                trajectory(i, 5) = trajectory_ref_point_->traj[0].roll;
                trajectory(i, 6) = trajectory_ref_point_->traj[0].pitch;
                trajectory(i, 7) = trajectory_ref_point_->traj[0].yaw;
                trajectory(i, 9) = 0.0;
                trajectory(i, 10) = 0.0;
                trajectory(i, 11) = 9.8066;
            }
            ROS_INFO_ONCE("Single point set");
        }
    }

    void MPCAcadosController::solvingACADOS(Eigen::MatrixXd ref)
    {
        std::vector<double> end_term_ref;
        for (int i = 0; i < 6; i++)
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
        acados_status = quadrotor_acados_solve(acados_ocp_capsule);
        if (acados_status)
        {
            robot_command[0] = 0.0;
            robot_command[1] = 0.0;
            robot_command[2] = 9.8066;
            ROS_INFO("Cannot solve the MPC");
        }
        else
            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", &robot_command);
    }

    const itm_mav_msgs::AttitudeCommand::ConstPtr MPCAcadosController::update(const itm_mav_msgs::PositionCommand::ConstPtr &pos_command)
    {
        // dt checks
        double dt;
        if (is_first_iteration_)
        {
            last_update_time_ = uav_state.header.stamp;
            is_first_iteration_ = false;
            return itm_mav_msgs::AttitudeCommand::ConstPtr(new itm_mav_msgs::AttitudeCommand(activation_attitude_cmd_));
        }
        else
        {
            dt = (uav_state.header.stamp - last_update_time_).toSec();
            last_update_time_ = uav_state.header.stamp;
        }

        if (fabs(dt) <= 0.001)
        {
            ROS_INFO("update too often");
            if (last_attitude_cmd_ != itm_mav_msgs::AttitudeCommand::Ptr())
                return last_attitude_cmd_;
            else
                return itm_mav_msgs::AttitudeCommand::ConstPtr(new itm_mav_msgs::AttitudeCommand(activation_attitude_cmd_));
        }

        // heading
        double uav_heading = 0.0;
        try
        {
            uav_heading = getHeading(uav_state.pose.orientation);
        }
        catch (...)
        {
            ROS_ERROR("cannot calculate heading");
        }

        // --------------------------------------------------------------
        // |          load the control reference and estimates          |
        // --------------------------------------------------------------

        // Rp - position reference in global frame
        // Rv - velocity reference in global frame
        // Ra - acceleration reference in global frame
        // Rw - angular velocity reference
        Eigen::Vector3d Rp, Rv, Ra, Rw;
        Rp << pos_command->position.x, pos_command->position.y, pos_command->position.z;
        Rv << pos_command->velocity.x, pos_command->velocity.y, pos_command->velocity.z;

        if (pos_command->use_heading_rate)
        {
            double desired_yaw_rate = 0.0;
            try
            {
                desired_yaw_rate = (uav_state.pose.orientation, pos_command->heading_rate);
            }
            catch (...)
            {
                ROS_ERROR("Yaw rating error");
            }
            Rw << 0, 0, desired_yaw_rate;
        }
        else
        {
            Rw << 0, 0, 0;
        }

        // Op - position in global frame
        // Ov - velocity in global frame
        Eigen::Vector3d Op(uav_state.pose.position.x, uav_state.pose.position.y, uav_state.pose.position.z);
        Eigen::Vector3d Ov(uav_state.velocity.linear.x, uav_state.velocity.linear.y, uav_state.velocity.linear.z); // how to estimate this from PoseStamped data?

        // R - current uav attitude
        Eigen::Matrix3d R = geom_q_to_rotation(uav_state.pose.orientation);

        // Ow - UAV angular rate
        Eigen::Vector3d Ow(uav_state.velocity.angular.x, uav_state.velocity.angular.y, uav_state.velocity.angular.z);

        // | --------------- calculate the control erros -------------- |

        Eigen::Vector3d Ep = Op - Rp;
        Eigen::Vector3d Ev = Ov - Rv;

        /* gain filter ? */

        /* gains */
        Eigen::Vector3d Ka;
        Eigen::Array3d Kq;
        Ka << 5.0, 5.0, 5.0;
        Kq << 8.0, 8.0, 5.0;

        /* recalculate the hover thrust */
        hover_thrust_ = forceToThrust(motor_parameter_a, motor_parameter_b, (uav_mass_ + uav_mass_difference_) * 9.81, 4);

        /* desired orientation matrix and force */
        // get body integral error to the world frame
        Eigen::Vector2d Ib_w = Eigen::Vector2d::Zero();
        Ib_w = R.block<2, 2>(0, 0) * Ib_b_;

        if (pos_command->use_acceleration)
        {
            Ra << pos_command->acceleration.x, pos_command->acceleration.y, pos_command->acceleration.z + robot_command[2];
        }
        else
        {
            Ra << 0.0, 0.0, robot_command[2];
        }

        double total_mass = uav_mass_ + uav_mass_difference_;
        Eigen::Vector3d feed_forward = total_mass * Ra;

        Eigen::Vector3d integral_feedback;
        integral_feedback << Ib_w[0] + Iw_w_[0], Ib_w[1] + Iw_w_[1], 0.0;

        Eigen::Vector3d f = integral_feedback + feed_forward;

        /* limiting the downwards acceleration */
        if (f[2] < 0)
        {
            ROS_WARN("downward acceleration is estimated!");
            f << 0.0, 0.0, 1.0;
        }

        /* limiting the tilt angle */
        Eigen::Vector3d f_norm = f.normalized();

        double theta = acos(f_norm[2]);
        double phi = atan2(f_norm[1], f_norm[0]);

        if (!std::isfinite(theta))
        {
            return itm_mav_msgs::AttitudeCommand::ConstPtr();
        }

        if (theta > 75 / 180.0 * 3.14159) // angle over 75 degree
        {
            return itm_mav_msgs::AttitudeCommand::ConstPtr();
        }

        /* saturate the angle based on theta ? */

        /* rotation matrix */
        Eigen::Matrix3d Rd;
        if (pos_command->use_orientation)
        {
            Rd = geom_q_to_rotation(pos_command->orientation);
            if (pos_command->use_heading)
            {
                Rd = setHeadingByYaw(pos_command->heading, Rd);
            }
        }
        else
        {
            Eigen::Vector3d bxd;
            if (pos_command->use_heading)
                bxd << cos(pos_command->heading), sin(pos_command->heading), 0.0;
            else
            {
                bxd << cos(uav_heading), sin(uav_heading), 0.0;
            }

            // fill in the desired orientation based on the state feedback

            // | ------------------------- body z ------------------------- |
            Rd.col(2) = f_norm;

            // | ------------------------- body x ------------------------- |

            // construct the oblique projection
            Eigen::Matrix3d projector_body_z_compl = (Eigen::Matrix3d::Identity(3, 3) - f_norm * f_norm.transpose());

            // create a basis of the body-z complement subspace
            Eigen::MatrixXd A = Eigen::MatrixXd(3, 2);
            A.col(0) = projector_body_z_compl.col(0);
            A.col(1) = projector_body_z_compl.col(1);

            // create the basis of the projection null-space complement
            Eigen::MatrixXd B = Eigen::MatrixXd(3, 2);
            B.col(0) = Eigen::Vector3d(1, 0, 0);
            B.col(1) = Eigen::Vector3d(0, 1, 0);

            // oblique projector to <range_basis>
            Eigen::MatrixXd Bt_A = B.transpose() * A;
            Eigen::MatrixXd Bt_A_pseudoinverse = ((Bt_A.transpose() * Bt_A).inverse()) * Bt_A.transpose();
            Eigen::MatrixXd oblique_projector = A * Bt_A_pseudoinverse * B.transpose();

            Rd.col(0) = oblique_projector * bxd;
            Rd.col(0).normalize();

            // | ------------------------- body y ------------------------- |

            Rd.col(1) = Rd.col(2).cross(Rd.col(0));
            Rd.col(1).normalize();
        }

        // | --------------------orientation error-- ----------------- |

        Eigen::Matrix3d E = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);

        Eigen::Vector3d Eq;

        // clang-format off
        Eq << (E(2, 1) - E(1, 2)) / 2.0,
        (E(0, 2) - E(2, 0)) / 2.0,
        (E(1, 0) - E(0, 1)) / 2.0;
        // clang-format on

        // | ------------------- angular rate error ------------------- |

        double thrust_force = f.dot(R.col(2));
        double thrust = 0;

        if (thrust_force >= 0)
        {
            thrust = forceToThrust(motor_parameter_a, motor_parameter_b, thrust_force, 4);
        }
        else
        {
            ROS_WARN_THROTTLE(1.0, "just so you know, the desired thrust force is negative (%.2f)", thrust_force);
        }

        // saturate the thrust
        if (!std::isfinite(thrust))
        {

            thrust = 0;
            ROS_ERROR("NaN detected in variable 'thrust', setting it to 0 and returning!!!");
        }
        else if (thrust > 1.5 * control_acc_offset)
        {

            thrust = 1.5 * control_acc_offset;
            ROS_WARN_THROTTLE(1.0, "saturating thrust to %.2f", 1.5 * control_acc_offset);
            ROS_WARN_THROTTLE(0.1, "[Se3Controller]: ---------------------------");
            ROS_WARN_THROTTLE(0.1, "[Se3Controller]: desired state: pos [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", pos_command->position.x,
                              pos_command->position.y, pos_command->position.z, pos_command->heading);
            ROS_WARN_THROTTLE(0.1, "[Se3Controller]: desired state: vel [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", pos_command->velocity.x,
                              pos_command->velocity.y, pos_command->velocity.z, pos_command->heading_rate);
            ROS_WARN_THROTTLE(0.1, "[Se3Controller]: desired state: acc [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", pos_command->acceleration.x,
                              pos_command->acceleration.y, pos_command->acceleration.z, pos_command->heading_acceleration);
            ROS_WARN_THROTTLE(0.1, "[Se3Controller]: desired state: jerk [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", pos_command->jerk.x, pos_command->jerk.y,
                              pos_command->jerk.z, pos_command->heading_jerk);
            ROS_WARN_THROTTLE(0.1, "[Se3Controller]: ---------------------------");
            ROS_WARN_THROTTLE(0.1, "[Se3Controller]: current state: pos [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", uav_state.pose.position.x, uav_state.pose.position.y,
                              uav_state.pose.position.z, uav_heading);
            ROS_WARN_THROTTLE(0.1, "[Se3Controller]: current state: vel [x: %.2f, y: %.2f, z: %.2f, yaw rate: %.2f]", uav_state.velocity.linear.x,
                              uav_state.velocity.linear.y, uav_state.velocity.linear.z, uav_state.velocity.angular.z);
            ROS_WARN_THROTTLE(0.1, "[Se3Controller]: ---------------------------");
        }
        else if (thrust < 0.0)
        {

            thrust = 0.0;
            ROS_WARN_THROTTLE(1.0, "saturating thrust to %.2f", 0.0);
            ROS_WARN_THROTTLE(0.1, "[Se3Controller]: ---------------------------");
            ROS_WARN_THROTTLE(0.1, "[Se3Controller]: desired state: pos [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", pos_command->position.x,
                              pos_command->position.y, pos_command->position.z, pos_command->heading);
            ROS_WARN_THROTTLE(0.1, "[Se3Controller]: desired state: vel [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", pos_command->velocity.x,
                              pos_command->velocity.y, pos_command->velocity.z, pos_command->heading_rate);
            ROS_WARN_THROTTLE(0.1, "[Se3Controller]: desired state: acc [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", pos_command->acceleration.x,
                              pos_command->acceleration.y, pos_command->acceleration.z, pos_command->heading_acceleration);
            ROS_WARN_THROTTLE(0.1, "[Se3Controller]: desired state: jerk [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", pos_command->jerk.x, pos_command->jerk.y,
                              pos_command->jerk.z, pos_command->heading_jerk);
            ROS_WARN_THROTTLE(0.1, "[Se3Controller]: ---------------------------");
            ROS_WARN_THROTTLE(0.1, "[Se3Controller]: current state: pos [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", uav_state.pose.position.x, uav_state.pose.position.y,
                              uav_state.pose.position.z, uav_heading);
            ROS_WARN_THROTTLE(0.1, "[Se3Controller]: current state: vel [x: %.2f, y: %.2f, z: %.2f, yaw rate: %.2f]", uav_state.velocity.linear.x,
                              uav_state.velocity.linear.y, uav_state.velocity.linear.z, uav_state.velocity.angular.z);
            ROS_WARN_THROTTLE(0.1, "[Se3Controller]: ---------------------------");
        }

        // prepare the attitude feedback
        Eigen::Vector3d q_feedback = -Kq * Eq.array();

        // feedforward angular acceleration
        Eigen::Vector3d q_feedforward = Eigen::Vector3d(0, 0, 0);

        Eigen::Matrix3d I;
        I << 0, 1, 0, -1, 0, 0, 0, 0, 0;
        Eigen::Vector3d desired_jerk = Eigen::Vector3d(pos_command->jerk.x, pos_command->jerk.y, pos_command->jerk.z);
        q_feedforward = (I.transpose() * Rd.transpose() * desired_jerk) / (thrust_force / total_mass);

        // angular feedback + angular rate feedforward
        Eigen::Vector3d t = q_feedback + Rw + q_feedforward;

        // compensate for the parasitic heading rate created by the desired pitch and roll rate
        Eigen::Vector3d rp_heading_rate_compensation = Eigen::Vector3d(0, 0, 0);

        Eigen::Vector3d q_feedback_yawless = t;
        q_feedback_yawless(2) = 0; // nullyfy the effect of the original yaw feedback

        double parasitic_heading_rate = 0;
        parasitic_heading_rate = getHeadingRate(uav_state.pose.orientation, q_feedback_yawless);

        rp_heading_rate_compensation(2) = getYawRateIntrinsic(uav_state.pose.orientation, -parasitic_heading_rate);
        t += rp_heading_rate_compensation;
        // --------------------------------------------------------------
        // |                 integrators and estimators                 |
        // --------------------------------------------------------------

        /* body error integrator //{ */

        {
            Eigen::Vector2d Ep_fcu_untilted = Eigen::Vector2d(0, 0); // position error in the untilted frame of the UAV
            Eigen::Vector2d Ev_fcu_untilted = Eigen::Vector2d(0, 0); // velocity error in the untilted frame of the UAV

            // get the position control error in the fcu_untilted frame
            {

                geometry_msgs::Vector3Stamped Ep_stamped;

                Ep_stamped.header.stamp = ros::Time::now();
                Ep_stamped.header.frame_id = uav_state.header.frame_id;
                Ep_stamped.vector.x = Ep(0);
                Ep_stamped.vector.y = Ep(1);
                Ep_stamped.vector.z = Ep(2);
                Ep_fcu_untilted = R.block<2, 2>(0, 0) * Ep.head<2>();

                // auto res = common_handlers_->transformer->transformSingle("fcu_untilted", Ep_stamped);

                // if (res)
                // {
                //     Ep_fcu_untilted[0] = res.value().vector.x;
                //     Ep_fcu_untilted[1] = res.value().vector.y;
                // }
                // else
                // {
                //     ROS_ERROR_THROTTLE(1.0, "[%s]: could not transform the position error to fcu_untilted", name_.c_str());
                // }
            }

            // get the velocity control error in the fcu_untilted frame
            {
                geometry_msgs::Vector3Stamped Ev_stamped;

                Ev_stamped.header.stamp = ros::Time::now();
                Ev_stamped.header.frame_id = uav_state.header.frame_id;
                Ev_stamped.vector.x = Ev(0);
                Ev_stamped.vector.y = Ev(1);
                Ev_stamped.vector.z = Ev(2);

                Ev_fcu_untilted = R.block<2, 2>(0, 0) * Ev.head<2>();
                // auto res = common_handlers_->transformer->transformSingle("fcu_untilted", Ev_stamped);

                // if (res)
                // {
                //     Ev_fcu_untilted[0] = res.value().vector.x;
                //     Ev_fcu_untilted[1] = res.value().vector.x;
                // }
                // else
                // {
                //     ROS_ERROR_THROTTLE(1.0, "[%s]: could not transform the velocity error to fcu_untilted", name_.c_str());
                // }
            }

            // integrate the body error

            // antiwindup
            double temp_gain = 0.1;
            //kibxy_;
            if (!pos_command->disable_antiwindups)
            {
                if (rampup_active_ || sqrt(pow(uav_state.velocity.linear.x, 2) + pow(uav_state.velocity.linear.y, 2)) > 0.3)
                {
                    temp_gain = 0;
                    ROS_INFO_THROTTLE(1.0, "anti-windup for body integral kicks in");
                }
            }

            if (integral_terms_enabled_)
            {
                if (pos_command->use_position_horizontal)
                {
                    Ib_b_ -= temp_gain * Ep_fcu_untilted * dt;
                }
                else if (pos_command->use_velocity_horizontal)
                {
                    Ib_b_ -= temp_gain * Ev_fcu_untilted * dt;
                }
            }

            // saturate the body X
            bool body_integral_saturated = false;
            double kibxy_lim_ = 10.0;
            if (!std::isfinite(Ib_b_[0]))
            {
                Ib_b_[0] = 0;
                ROS_ERROR_THROTTLE(1.0, "NaN detected in variable 'Ib_b_[0]', setting it to 0!!!");
            }
            else if (Ib_b_[0] > kibxy_lim_)
            {
                Ib_b_[0] = kibxy_lim_;
                body_integral_saturated = true;
            }
            else if (Ib_b_[0] < -kibxy_lim_)
            {
                Ib_b_[0] = -kibxy_lim_;
                body_integral_saturated = true;
            }

            if (kibxy_lim_ > 0 && body_integral_saturated)
            {
                ROS_WARN_THROTTLE(1.0, "MPC's body pitch integral is being saturated!");
            }

            // saturate the body
            body_integral_saturated = false;
            if (!std::isfinite(Ib_b_[1]))
            {
                Ib_b_[1] = 0;
                ROS_ERROR_THROTTLE(1.0, "NaN detected in variable 'Ib_b_[1]', setting it to 0!!!");
            }
            else if (Ib_b_[1] > kibxy_lim_)
            {
                Ib_b_[1] = kibxy_lim_;
                body_integral_saturated = true;
            }
            else if (Ib_b_[1] < -kibxy_lim_)
            {
                Ib_b_[1] = -kibxy_lim_;
                body_integral_saturated = true;
            }

            if (kibxy_lim_ > 0 && body_integral_saturated)
            {
                ROS_WARN_THROTTLE(1.0, "MPC's body roll integral is being saturated!");
            }
        }

        /*world error integrator //{ */

        // --------------------------------------------------------------
        // |                  integrate the world error                 |
        // --------------------------------------------------------------

        {

            Eigen::Vector3d integration_switch(1, 1, 0);
            double kiwxy_lim_ = 10.0;
            double kiwxy_ = 0.5;

            // integrate the world error

            // antiwindup
            double temp_gain = kiwxy_;
            if (!pos_command->disable_antiwindups)
            {
                if (rampup_active_ || sqrt(pow(uav_state.velocity.linear.x, 2) + pow(uav_state.velocity.linear.y, 2)) > 0.3)
                {
                    temp_gain = 0;
                    ROS_INFO_THROTTLE(1.0, "anti-windup for world integral kicks in");
                }
            }

            if (integral_terms_enabled_)
            {
                if (pos_command->use_position_horizontal)
                {
                    Iw_w_ -= temp_gain * Ep.head(2) * dt;
                }
                else if (pos_command->use_velocity_horizontal)
                {
                    Iw_w_ -= temp_gain * Ev.head(2) * dt;
                }
            }

            // saturate the world X
            bool world_integral_saturated = false;
            if (!std::isfinite(Iw_w_[0]))
            {
                Iw_w_[0] = 0;
                ROS_ERROR_THROTTLE(1.0, "NaN detected in variable 'Iw_w_[0]', setting it to 0!!!");
            }
            else if (Iw_w_[0] > kiwxy_lim_)
            {
                Iw_w_[0] = kiwxy_lim_;
                world_integral_saturated = true;
            }
            else if (Iw_w_[0] < -kiwxy_lim_)
            {
                Iw_w_[0] = -kiwxy_lim_;
                world_integral_saturated = true;
            }

            if (kiwxy_lim_ >= 0 && world_integral_saturated)
            {
                ROS_WARN_THROTTLE(1.0, "MPC's world X integral is being saturated!");
            }

            // saturate the world Y
            world_integral_saturated = false;
            if (!std::isfinite(Iw_w_[1]))
            {
                Iw_w_[1] = 0;
                ROS_ERROR_THROTTLE(1.0, "NaN detected in variable 'Iw_w_[1]', setting it to 0!!!");
            }
            else if (Iw_w_[1] > kiwxy_lim_)
            {
                Iw_w_[1] = kiwxy_lim_;
                world_integral_saturated = true;
            }
            else if (Iw_w_[1] < -kiwxy_lim_)
            {
                Iw_w_[1] = -kiwxy_lim_;
                world_integral_saturated = true;
            }

            if (kiwxy_lim_ >= 0 && world_integral_saturated)
            {
                ROS_WARN_THROTTLE(1.0, "MPC's world Y integral is being saturated!");
            }
        }

        /* mass estimatior //{ */

        // --------------------------------------------------------------
        // |                integrate the mass difference               |
        // --------------------------------------------------------------

        {
            // antiwindup
            double temp_gain = 1.0;
            double km_lim_ = 6.0;
            //km_;
            if (rampup_active_ ||
                (fabs(uav_state.velocity.linear.z) > 0.2 && ((Ep[2] < 0 && uav_state.velocity.linear.z > 0) || (Ep[2] > 0 && uav_state.velocity.linear.z < 0))))
            {
                temp_gain = 0;
                ROS_INFO_THROTTLE(1.0, "anti-windup for the mass kicks in");
            }

            if (pos_command->use_position_vertical)
            {
                uav_mass_difference_ -= temp_gain * Ep[2] * dt;
            }

            // saturate the mass estimator
            bool uav_mass_saturated = false;
            if (!std::isfinite(uav_mass_difference_))
            {
                uav_mass_difference_ = 0;
                ROS_WARN_THROTTLE(1.0, "NaN detected in variable 'uav_mass_difference_', setting it to 0 and returning!!!");
            }
            else if (uav_mass_difference_ > km_lim_)
            {
                uav_mass_difference_ = km_lim_;
                uav_mass_saturated = true;
            }
            else if (uav_mass_difference_ < -km_lim_)
            {
                uav_mass_difference_ = -km_lim_;
                uav_mass_saturated = true;
            }

            if (uav_mass_saturated)
            {
                ROS_WARN_THROTTLE(1.0, "The UAV mass difference is being saturated to %.2f!", uav_mass_difference_);
            }
        }

        // --------------------------------------------------------------
        // |                 produce the control output                 |
        // --------------------------------------------------------------

        itm_mav_msgs::AttitudeCommand::Ptr output_command(new itm_mav_msgs::AttitudeCommand);
        output_command->header.stamp = ros::Time::now();

        // | --------------- saturate the attitude rate --------------- |

        // if (got_constraints_)
        // {

        //     if (t[0] > constraints.roll_rate)
        //     {
        //         t[0] = constraints.roll_rate;
        //     }
        //     else if (t[0] < -constraints.roll_rate)
        //     {
        //         t[0] = -constraints.roll_rate;
        //     }

        //     if (t[1] > constraints.pitch_rate)
        //     {
        //         t[1] = constraints.pitch_rate;
        //     }
        //     else if (t[1] < -constraints.pitch_rate)
        //     {
        //         t[1] = -constraints.pitch_rate;
        //     }

        //     if (t[2] > constraints.yaw_rate)
        //     {
        //         t[2] = constraints.yaw_rate;
        //     }
        //     else if (t[2] < -constraints.yaw_rate)
        //     {
        //         t[2] = -constraints.yaw_rate;
        //     }
        // }
        // else
        // {
        //     ROS_WARN_THROTTLE(1.0, "missing dynamics constraints");
        // }

        // | ------------ compensated desired acceleration ------------ |

        double desired_x_accel = 0;
        double desired_y_accel = 0;
        double desired_z_accel = 0;

        {
            Eigen::Matrix3d des_orientation = Rd;
            Eigen::Vector3d thrust_vector = thrust_force * des_orientation.col(2);

            double world_accel_x = (thrust_vector[0] / total_mass) - (Iw_w_[0] / total_mass) - (Ib_w[0] / total_mass);
            double world_accel_y = (thrust_vector[1] / total_mass) - (Iw_w_[1] / total_mass) - (Ib_w[1] / total_mass);
            /* double world_accel_z = (thrust_vector[2] / total_mass) - common_handlers_->g; */
            double world_accel_z = pos_command->acceleration.z;

            desired_x_accel = world_accel_x; //
            desired_y_accel = world_accel_y;
            desired_z_accel = world_accel_z;

            // geometry_msgs::Vector3Stamped world_accel;
            // world_accel.header.stamp = ros::Time::now();
            // world_accel.header.frame_id = uav_state.header.frame_id;
            // world_accel.vector.x = world_accel_x;
            // world_accel.vector.y = world_accel_y;
            // world_accel.vector.z = world_accel_z;

            // auto res = common_handlers_->transformer->transformSingle("fcu", world_accel);

            // if (res)
            // {

            //     desired_x_accel = res.value().vector.x;
            //     desired_y_accel = res.value().vector.y;
            //     desired_z_accel = res.value().vector.z;
            // }
        }

        // | --------------- fill the resulting command --------------- |

        // fill the attitude anyway, since we know it
        output_command->attitude = getQuaternionfromMatrix(Rd);
    }

    /* callback functions */
    void MPCAcadosController::robot_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        // current_pose_ = *msg;
        robot_current_state[0] = msg->pose.position.x;
        robot_current_state[1] = msg->pose.position.y;
        robot_current_state[2] = msg->pose.position.z;
        // need a state observer ?
        robot_current_state[3] = 0.0;
        robot_current_state[4] = 0.0;
        robot_current_state[5] = 0.0;
        Eigen::Quaterniond q_(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
        Eigen::Vector3d rpy_;
        quaternion_to_rpy(q_, rpy_);
        robot_current_state[6] = rpy_[0];
        robot_current_state[7] = rpy_[1];
        robot_current_state[8] = rpy_[2];

        if (!is_current_pose_sub_)
        {
            is_current_pose_sub_ = true;
            // initial_pose_ = *msg;
            for (int i = 0; i < num_states; i++)
                robot_init_state[i] = robot_current_state[i];

            // se3
            uav_state.header = msg->header;
            uav_state.pose = msg->pose;
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
        robot_current_state[3] = msg->twist.twist.linear.x;
        robot_current_state[4] = msg->twist.twist.linear.y;
        robot_current_state[5] = msg->twist.twist.linear.z;
        Eigen::Quaterniond q_(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        Eigen::Vector3d rpy_;
        quaternion_to_rpy(q_, rpy_);
        robot_current_state[6] = rpy_[0];
        robot_current_state[7] = rpy_[1];
        robot_current_state[8] = rpy_[2];
        if (!is_current_pose_sub_)
        {
            is_current_pose_sub_ = true;
            for (int i = 0; i < num_states; i++)
                robot_init_state[i] = robot_current_state[i];
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
            uav_mass_difference_ = 0.0;
            Iw_w_ = Eigen::Vector2d::Zero();
            Ib_b_ = Eigen::Vector2d::Zero();
            rampup_active_ = true;
            integral_terms_enabled_ = true;
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

        rpy(0) = std::atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (std::pow(q1, 2) + std::pow(q2, 2)));
        double sinp = 2.0 * (q0 * q2 - q3 * q1);
        if (std::fabs(sinp) >= 1)
            rpy(1) = std::copysign(3.1415926 / 2.0, sinp);
        else
            rpy(1) = std::asin(sinp);
        rpy(2) = std::atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (std::pow(q2, 2) + std::pow(q3, 2)));
    }

    void MPCAcadosController::output_constrains(Eigen::Vector3d &cmd, double max_cmd)
    {
        cmd(0) = std::max(std::min(cmd(0), max_cmd), -max_cmd);
        cmd(1) = std::max(std::min(cmd(1), max_cmd), -max_cmd);
        cmd(2) = std::max(std::min(cmd(2), max_cmd), -max_cmd);
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
                controller_node.takeoff();
                break;
            }
            case 2: // landing
            {
                // controller_node.landing();
                controller_node.safe_landing();
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