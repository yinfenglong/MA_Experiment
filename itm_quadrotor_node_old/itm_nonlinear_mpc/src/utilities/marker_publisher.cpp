/*
 * @Author: Wei Luo
 * @Date: 2021-04-16 10:43:33
 * @LastEditors: Wei Luo
 * @LastEditTime: 2021-04-16 15:23:44
 * @Note: Note
 */
#include <itm_nonlinear_mpc/utilities/maker_publisher.hpp>


void robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_odometry = *msg;
    current_pose.pose = current_odometry.pose.pose;
    if (robot_path.poses.size() > num_path)
        robot_path.poses.erase(robot_path.poses.begin());
    robot_path.poses.push_back(current_pose);
    robot_path.header.stamp = ros::Time::now();
}

void robot_pose_callback(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
    current_pose = *msg;
    if (robot_path.poses.size()>num_path)
        robot_path.poses.erase(robot_path.poses.begin());
    robot_path.poses.push_back(current_pose);
    robot_path.header.stamp = ros::Time::now();
}

void trajectory_callback(const itm_mav_msgs::itm_trajectory_msg::ConstPtr &msg)
{
    int len_traj;
    len_traj = msg->size;
    traj_points.header.stamp = ros::Time::now();
    for (int i=0; i<len_traj; i++)
    {
        geometry_msgs::Point p_;
        p_.x = msg->traj[i].x;
        p_.y = msg->traj[i].y;
        p_.z = msg->traj[i].z;
        traj_points.points.push_back(p_);
    }

    if (traj_points.points.size()>max_traj_length)
        traj_points.points.erase(traj_points.points.begin());

}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "marker_publisher");
    ros::NodeHandle nh, private_nh("~");

    /* get parameters */
    private_nh.getParam("sim_only", is_sim_only);
    private_nh.getParam("num_path", num_path);
    private_nh.getParam("publish_path", is_publish_path);
    private_nh.getParam("publish_markers", is_publish_markers);
    ros::Rate loop_rate(1);

    /* publisher */
    if (is_publish_path)
    {
        path_pub = nh.advertise<nav_msgs::Path>("/path_trajectory", 10);
        if (is_sim_only)
            robot_sub = nh.subscribe<nav_msgs::Odometry>("/uav_odom", 10, robot_odom_callback);
        else
            robot_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav_pose", 10, robot_pose_callback);
        robot_path.header.frame_id = "map";
    }

    if (is_publish_markers)
    {
        max_traj_length = 100;
        request_trajectory_pub = nh.advertise<visualization_msgs::Marker>("/mark_trajectory", 10);
        trajectory_sub = nh.subscribe<itm_mav_msgs::itm_trajectory_msg>("/desired_trajectory", 10, trajectory_callback);
        traj_points.header.frame_id = "map";
        traj_points.action = visualization_msgs::Marker::ADD;
        traj_points.id = 0;
        traj_points.type = visualization_msgs::Marker::POINTS;// LINE_STRIP;
        traj_points.scale.x = 0.1;
        traj_points.scale.y = 0.1;
        traj_points.color.r = 1.0;
        traj_points.color.a = 1.0;
    }

    if (!is_publish_path && !is_publish_markers)
    {
        ROS_ERROR("No work to do!");
        return 2;
    }


    while (ros::ok())
    {
        if (is_publish_path)
        {
            path_pub.publish(robot_path);
        }
        if (is_publish_markers)
        {
            request_trajectory_pub.publish(traj_points);
        }
        loop_rate.sleep();
        ros::spinOnce();
    }

    ROS_INFO("RVIZ trajectory publisher is closed.");
}
