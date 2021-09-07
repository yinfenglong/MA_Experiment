#include <itm_nonlinear_mpc/sim_only/itm_nonlinear_aruco_mpc_offboard_node.hpp>

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    mavros_state = *msg;
}

void set_point_pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (!mpc_set_point_pos_flag) mpc_set_point_pos_flag = true;

    set_point_pos_ = *msg;

    ROS_INFO_ONCE("mpc_set_point_pos_flag is true");
}// set_point_pos_callback

void uav_gt_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (!received_uav_gt_) received_uav_gt_ = true;
    gps_pose = *msg;

}

void aruco_loc_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (!received_aruco_pose_) received_aruco_pose_ = true;
    aruco_pose = *msg;
}

void mpc_thread()
{
    while(ros::ok)
        ROS_INFO_STREAM("nothing but test");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_mpc_offboard_node");
    ros::NodeHandle nh;
    mpc_set_point_pos_flag = false;
    received_uav_gt_ = false;
    received_aruco_pose_ = false;
    mpc_avaliable_ = false;

    /*topic publish*/
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10); //for setting PX4 into position mode.

    /* topic subscribe */
    // subscribe the state of the mavros
    ros::Subscriber mavros_state_sb = nh.
    subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    // trajectory command from user.
    ros::Subscriber set_point_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/itm_quadrotor_control/set_point_pos",50, set_point_pos_callback);
    // subscribe fake GPS info
    ros::Subscriber uav_gps_sub = nh.subscribe<nav_msgs::Odometry>("/uav/ground_truth/odometry", 50, uav_gt_cb);
    // subscribe aruco marker positioning
    ros::Subscriber aruco_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/aruco_pose", 50, aruco_loc_cb);

    /*Service*/
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    /*the setpoint publishing rate MUST be faster than 2Hz*/
    ros::Rate rate(50.0);

    /*wait for FCU connection */
    while(!ros::ok() || !mavros_state.connected||!received_uav_gt_) // if ros or mavros is not ok and UAV position is still unknown, we need to wait for them
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO_STREAM("adb");
    }

    // temp pose, but is it necessary? since without publish this position, the mavros is still in AUTO.LOITER mode (need to check later)
    geometry_msgs::PoseStamped temp_pose;
    temp_pose.pose.position.x = 0;
    temp_pose.pose.position.y = 0;
    temp_pose.pose.position.z = 1.3;

    // send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(temp_pose);
        ros::spinOnce();
        rate.sleep();
    }

    // multi thread
    boost::thread mpc_overwatch_thread(mpc_thread);

    // normal ros thread
    /*set offboard and armed*/
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    int temp_num = 1000;

    // main thread loop begins
    while(ros::ok()){
        if(mavros_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO_ONCE("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !mavros_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO_ONCE("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        } // check and maintain the px4 in off-board mode


        //
        if (!received_aruco_pose_)
        {
            // aruco is not ready (MPC also is not ready yet)
            local_pos_pub.publish(temp_pose);
        }
        else
        {

        }



        // maintain the frequency of the main loop
        ros::spinOnce();
        rate.sleep();

        // landing condiction
        if(temp_num>=0)
        {
            temp_num = temp_num - 1;
        }
        else
            break;
    }



    // landing
    offb_set_mode.request.custom_mode = "AUTO.LAND";
    while(ros::ok())
    {
        if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO_ONCE("Land enabled");
            }
            last_request = ros::Time::now();
        // maintain the frequency of the main loop
        ros::spinOnce();
        rate.sleep();
    }

    mpc_overwatch_thread.detach(); // close the mpc overwatch thread, and make it easier to shutdown the whole process.

}
