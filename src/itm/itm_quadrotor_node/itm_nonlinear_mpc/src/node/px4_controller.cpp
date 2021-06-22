

/*
 * @Author: Wei Luo
 * @Date: 2021-03-14 23:23:46
 * @LastEditors: Wei Luo
 * @LastEditTime: 2021-06-04 17:19:22
 * @Note: To control an experiment/simulation quadrotor with different
 *      controllers.
 */

// include header files
#include <itm_nonlinear_mpc/node/px4_controller.hpp>

void normal_landing(double dt)
{
    ROS_INFO_ONCE("Start Landing at current position");
    if (abs(current_pos.pose.position.z - initial_pos.pose.position.z) <= 0.1)
    {
        offboard_set_mode.request.custom_mode = "MANUAL";
        if (set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent)
        {
            ROS_INFO_ONCE("Landing detected and set to landing");
        }

        else if (mavros_state.mode == "AUTO.RTL" || mavros_state.mode == "AUTO.LOITER")
        {
            offboard_set_mode.request.custom_mode = "MANUAL";
            if (set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent)
            {
                ROS_INFO_ONCE("Landing and set to MANUAL");
            }
        }
        else if (mavros_state.mode == "MANUAL" || mavros_state.mode == "AUTO.RTL" || mavros_state.mode == "AUTO.LOITER")
        {
            if (mavros_state.armed)
            {
                arm_cmd.request.value = false;
                arming_client.call(arm_cmd);
            }
            if (arm_cmd.response.success)
            {
                ROS_INFO_ONCE("Safely Disarm");
            }
            else
            {
                offboard_set_mode.request.custom_mode = "MANUAL";
                if (set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent)
                {
                    ROS_INFO_ONCE("Landing detected and set to Manual");
                }
            }
        }
        else
        {
            std::cout << mavros_state.mode << std::endl;
            ROS_ERROR("Unknown state");
        }
        if (mavros_state.armed)
        {
            arm_cmd.request.value = false;
            arming_client.call(arm_cmd);
        }
        // if (arm_cmd.response.success)
        // {
        //     ROS_INFO_ONCE("Safely Disarm");
        // }
        // else
        // {
        //     offboard_set_mode.request.custom_mode = "MANUAL";
        //     if (set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent)
        //     {
        //         ROS_INFO_ONCE("Landing detected and set to Manual");
        //     }
        // }
    }
    else
    {
        // printf("distance %f \n", abs(current_pos.pose.position.z-initial_pos.pose.position.z));
        // printf("current height %f, initial height %f \n",current_pos.pose.position.z, initial_pos.pose.position.z);
    }
    // if (abs(current_pos.pose.position.z - initial_pos.pose.position.z) <= 0.1)
    // {
    //     if (mavros_state.mode == "OFFBOARD")
    //     {
    //         offboard_set_mode.request.custom_mode = "MANUAL";
    //         if (set_mode_client.call(offboard_set_mode))
    //         {
    //             ROS_INFO_ONCE("Landing detected and set to landing");
    //         }
    //     }

    //     if (mavros_state.armed)
    //     {
    //         arm_cmd.request.value = false;
    //         arming_client.call(arm_cmd);
    //     }

    //     if (arm_cmd.response.success)
    //     {
    //         ROS_INFO_ONCE("Disarm successfully!");
    //     }
    // }
}

void emergency_landing()
{
    ROS_INFO_ONCE("Emergency Landing");
    if (!got_landing_target)
    {
        landing_target = current_pos;
        landing_target.pose.position.z = initial_pos.pose.position.z;
        got_landing_target = true;
        ROS_INFO_STREAM("landing target" << landing_target);
    }
    landing_command(landing_target);
    if (abs(current_pos.pose.position.z - initial_pos.pose.position.z) <= 0.05)
    {
        offboard_set_mode.request.custom_mode = "MANUAL";
        if (set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent)
        {
            ROS_INFO_ONCE("Landing detected and set to landing");
        }

        else if (mavros_state.mode == "AUTO.RTL" || mavros_state.mode == "AUTO.LOITER")
        {
            offboard_set_mode.request.custom_mode = "MANUAL";
            if (set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent)
            {
                ROS_INFO_ONCE("Landing and set to MANUAL");
            }
        }
        else if (mavros_state.mode == "MANUAL" || mavros_state.mode == "AUTO.RTL" || mavros_state.mode == "AUTO.LOITER")
        {
            if (mavros_state.armed)
            {
                arm_cmd.request.value = false;
                arming_client.call(arm_cmd);
            }
            if (arm_cmd.response.success)
            {
                ROS_INFO_ONCE("Safely Disarm");
            }
            else
            {
                offboard_set_mode.request.custom_mode = "MANUAL";
                if (set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent)
                {
                    ROS_INFO_ONCE("Landing detected and set to POSITION");
                }
            }
        }
        else
        {
            std::cout << mavros_state.mode << std::endl;
            ROS_ERROR("Unknown state");
        }
    }
    else
    {
        // printf("distance %f \n", abs(current_pos.pose.position.z-initial_pos.pose.position.z));
        // printf("current height %f, initial height %f \n",current_pos.pose.position.z, initial_pos.pose.position.z);
    }
}

/* -- main function -- */
int main(int argc, char **argv)
{
    // init ros node
    ros::init(argc, argv, "px4_controller");
    ros::NodeHandle nh_node, private_nh_node("~");

    // init variables
    /* user command mode
        0 -- idle
        1 -- take off
        2 -- landing at the initial position
        3 -- mission
        4 -- (emergency) landing
    */
    user_command.mission_mode = 0;
    /* default controller_state is false */
    controller_state.response.connected = false;

    uav_mode.request.mode = 0;
    current_pos_flag = false;
    is_controller_working = false;
    is_only_sim = true;

    /* get some parameters */
    private_nh_node.getParam("sim_only", is_only_sim);
    if (private_nh_node.hasParam("fence_def"))
    {
        got_fence = true;
        private_nh_node.getParam("fence_def", fence_def);
    }
    else
    {
        got_fence = false;
    }

    /*Subscribers*/
    ros::Subscriber state_sub = nh_node.subscribe<mavros_msgs::State>("/mavros/state", 10, mavros_state_cb);
    ros::Subscriber user_command_sub = nh_node.subscribe<itm_mav_msgs::SetMission>("itm_quadrotor_control/user_command", 10, user_command_callback);
    ros::Subscriber uav_state_sub_;
    if (is_only_sim)
        uav_state_sub_ = nh_node.subscribe<nav_msgs::Odometry>("/real_pose", 10, current_odom_callback);
    else
        uav_state_sub_ = nh_node.subscribe<geometry_msgs::PoseStamped>("/real_pose", 10, current_pos_callback);

    /*ServiceClient*/
    arming_client = nh_node.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client = nh_node.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::ServiceClient set_mission_mode = nh_node.serviceClient<itm_mav_srvs::SetMode>("/itm_quadrotor_control/setmode");
    ros::ServiceClient get_controller_state = nh_node.serviceClient<itm_mav_srvs::GetControllerState>("/itm_quadrotor_control/get_controller_state");
    /*Publisher*/
    robot_command = nh_node.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    flight_mission_mode_pub_ = nh_node.advertise<itm_mav_msgs::SetMission>("itm_quadrotor_control/user_command", 10);

    /*the setpoint publishing rate MUST be faster than 2Hz*/
    ros::Rate rate(20.0);

    /* get the controller type */
    if (private_nh_node.hasParam("controller"))
    {
        private_nh_node.getParam("controller", controller_type);
    }
    else
    {
        ROS_ERROR("NO controller specified, please provide a controller server");
    }

    /*wait for FCU connection */
    while (ros::ok() && (!mavros_state.connected || !current_pos_flag))
    {
        // if get state from ROS and also the localization of the quadrotor
        ros::spinOnce();
        rate.sleep();
    }

    /*inertial takeoff height*/
    command_pose.pose.position.x = 0;
    command_pose.pose.position.y = 0;
    command_pose.pose.position.z = 0.3;

    //send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        robot_command.publish(command_pose);
        ros::spinOnce();
        rate.sleep();
    }

    /* check the controller state */
    while (ros::ok() && !controller_state.response.connected)
    {
        // check the controller state through server
        ROS_INFO("Connect to Controller Server.");
        controller_state.request.robot_name = "ITM_Q250";
        controller_state.request.command_id = 0;
        get_controller_state.call(controller_state);
        ros::spinOnce();
        rate.sleep();
    }
    is_controller_working = true;
    ROS_INFO("Get connection with Controller Server");

    /*set offboard and armed*/
    offboard_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;

    last_request = ros::Time::now();
    ROS_INFO("Begin the loop");
    auto t_start = std::chrono::high_resolution_clock::now();
    // main loop
    // while (ros::ok())
    // {
    //     if (mavros_state.mode != "OFFBOARD" &&
    //         (ros::Time::now() - last_request > ros::Duration(5.0)) &&
    //         user_command.mission_mode != 2 && user_command.mission_mode != 4)
    //     {
    //         if (set_mode_client.call(offboard_set_mode) &&
    //             offboard_set_mode.response.mode_sent)
    //         {
    //             ROS_INFO("Offboard enabled");
    //         }
    //         last_request = ros::Time::now();
    //     }
    //     else
    //     {
    //         if (!mavros_state.armed &&
    //             (ros::Time::now() - last_request > ros::Duration(5.0)) &&
    //             user_command.mission_mode != 2 && user_command.mission_mode != 4)
    //         {
    //             if (arming_client.call(arm_cmd) &&
    //                 arm_cmd.response.success)
    //             {
    //                 ROS_INFO_ONCE("Vehicle armed");
    //             }
    //             else
    //             {
    //                 ROS_INFO("cannot arm");
    //             }
    //             last_request = ros::Time::now();
    //         }
    //     }

    //     // once the vehicle is ready to fly
    //     if (mavros_state.armed)
    //     {
    //         controller_state.request.command_id = user_command.mission_mode;
    //         get_controller_state.call(controller_state);
    //         if (!controller_state.response.connected)
    //         {
    //             // controller no response
    //             is_controller_working = false;
    //         }
    //         ROS_INFO_ONCE("Begin the Mission");
    //     }

    //     if (user_command.mission_mode == 2)
    //     {
    //         auto t_end = std::chrono::high_resolution_clock::now();
    //         double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    //         normal_landing(elapsed_time_ms / 1000.0);
    //         t_start = std::chrono::high_resolution_clock::now();
    //     }
    //     else if (user_command.mission_mode == 4 || !is_controller_working)
    //     {
    //         emergency_landing();
    //     }

    //     if (got_fence)
    //     {
    //         // safety checks
    //         if (!safety_fence_check(0))
    //         {
    //             emergency_landing();
    //             user_command.mission_mode = 4;
    //             // publish user command mode to tell controller to stop publish control commands
    //             itm_mav_msgs::SetMission set_mission_mode;
    //             set_mission_mode.mission_mode = 4;
    //             flight_mission_mode_pub_.publish(set_mission_mode);
    //         }
    //     }

    //     // last_user_command = user_command;
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    while (ros::ok())
    {
        if (mavros_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)) &&
            (user_command.mission_mode != 2) && (user_command.mission_mode != 4))
        {
            if (set_mode_client.call(offboard_set_mode) &&
                offboard_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!mavros_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)) && (user_command.mission_mode != 2) && (user_command.mission_mode != 4))
            {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        controller_state.request.command_id = user_command.mission_mode;
        get_controller_state.call(controller_state);
        if (!controller_state.response.connected)
        {
            // controller no response
            is_controller_working = false;
            ROS_ERROR("Controller is lost");
        }

        if (user_command.mission_mode == 2)
        {
            auto t_end = std::chrono::high_resolution_clock::now();
            double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            normal_landing(elapsed_time_ms / 1000.0);
            t_start = std::chrono::high_resolution_clock::now();
        }
        else if (user_command.mission_mode == 4 || !is_controller_working)
        {
            emergency_landing();
        }

        if (got_fence)
        {
            // safety checks
            if (!safety_fence_check(0))
            {
                emergency_landing();
                user_command.mission_mode = 4;
                // publish user command mode to tell controller to stop publish control commands
                itm_mav_msgs::SetMission set_mission_mode;
                set_mission_mode.mission_mode = 4;
                flight_mission_mode_pub_.publish(set_mission_mode);
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
}
