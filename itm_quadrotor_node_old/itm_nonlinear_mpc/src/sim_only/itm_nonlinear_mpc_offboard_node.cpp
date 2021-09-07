/**
 * @file offboard_nmpc_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
/*mavros*/
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
/*mav_msgs*/
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
/*itm_nonlinear_mpc*/
#include <itm_nonlinear_mpc/mpc_set_point_pos.h>
/*others*/
#include <Eigen/Eigen>
#include <tf/transform_datatypes.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}// state_cb

/* callback function of current_pos*/
geometry_msgs::PoseStamped current_pos;
itm_nonlinear_mpc::mpc_set_point_pos mpc_current_pos_srv;
bool mpc_current_pos_flag = false;//false
void current_pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(!mpc_current_pos_flag) mpc_current_pos_flag = true;

    current_pos = *msg;

    ROS_INFO_ONCE("mpc_current_pos_flag is true");
}//current_pos_callback

/* callback function of set_point_pos*/
geometry_msgs::PoseStamped set_point_pos_;
itm_nonlinear_mpc::mpc_set_point_pos mpc_set_point_pos_srv;
bool mpc_set_point_pos_flag = false;
void set_point_pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (!mpc_set_point_pos_flag) mpc_set_point_pos_flag = true;

    set_point_pos_ = *msg;

    ROS_INFO("mpc_set_point_pos_flag is true");
}// set_point_pos_callback


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;

    /*Subscribers*/
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    ros::Subscriber set_point_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/itm_quadrotor_control/set_point_pos",50, set_point_pos_callback);
    ros::Subscriber current_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",50, current_pos_callback);

    /*Publisher*/
    // ros::Publisher command_attitude_thrust_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10); //for test
    // ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10); //for test

    /*ServiceClient*/
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::ServiceClient set_mpc_client = nh.serviceClient<itm_nonlinear_mpc::mpc_set_point_pos>("/itm_quadrotor_control/mpc_set_point_pos");

    /*the setpoint publishing rate MUST be faster than 2Hz*/
    ros::Rate rate(50.0);


    /*wait for FCU connection */
    while(ros::ok() && !current_state.connected && !mpc_current_pos_flag)
    // if get state from ROS and also the localization of the quadrotor
    {
        ros::spinOnce();
        rate.sleep();
    }

    for(int i=250; ros::ok() && i>0; --i)
    {
        ROS_INFO_ONCE("publish current position to MPC");
        if(mpc_current_pos_flag == true)
        {
            mpc_current_pos_srv.request.x = current_pos.pose.position.x;
            mpc_current_pos_srv.request.y = current_pos.pose.position.y;
            mpc_current_pos_srv.request.z = current_pos.pose.position.z;
            set_mpc_client.call(mpc_current_pos_srv);
            if(set_mpc_client.call(mpc_current_pos_srv) && mpc_current_pos_flag) break;
            // if mpc says that it got the msg, break this loop and save the time.
        }

        ros::spinOnce();
        rate.sleep();
    }

    if (!mpc_current_pos_srv.response.success)
    {
        ROS_ERROR("MPC does not response the initial position request after 5s, please start the MPC node first or check its heath.");
        return 0;
    }
    // else MPC is ready now.

    /*set offboard and armed*/
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    /*set mpc client:mpc set point*/
    itm_nonlinear_mpc::mpc_set_point_pos mpc_set_point_pos_srv;

    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO_ONCE("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO_ONCE("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // once the vehicle is ready to fly
        if (current_state.mode == "OFFBOARD" && current_state.armed)
        {
            if(mpc_set_point_pos_flag == true)
            {
                mpc_set_point_pos_srv.request.x = set_point_pos_.pose.position.x;
                mpc_set_point_pos_srv.request.y = set_point_pos_.pose.position.y;
                mpc_set_point_pos_srv.request.z = set_point_pos_.pose.position.z;
                set_mpc_client.call(mpc_set_point_pos_srv);

                if(!set_mpc_client.call(mpc_set_point_pos_srv))
                {
                    ROS_INFO("publish set point to MPC falled");
                }


            // last_request = ros::Time::now();
            }else{
                ROS_INFO("No setpoint is provid");
            }
        }

        // local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }//while

    return 0;
}//fcn main