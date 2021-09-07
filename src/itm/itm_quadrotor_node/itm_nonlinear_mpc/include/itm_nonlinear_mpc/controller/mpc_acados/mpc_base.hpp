/*
 * @Author: Wei Luo
 * @Date: 2021-08-27 12:51:58
 * @LastEditors: Wei Luo
 * @LastEditTime: 2021-08-27 12:54:44
 * @Note: Note
 */
#ifndef _MPC_BASE_HPP_
#define _MPC_BASE_HPP_

#include <ros/ros.h>

namespace acados_quadrotor
{
    class MPCBase
    {
    public:
        MPCBase(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh);
        ~MPCBase();
    };
}
#endif