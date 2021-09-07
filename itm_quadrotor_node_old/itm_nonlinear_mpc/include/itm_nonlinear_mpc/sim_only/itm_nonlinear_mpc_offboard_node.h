/*
Yan Li, Uni Stuttgart, Germany
You can contact the author at <yan1.li@web.de>

NMPC: Non Linear Model Predictive Control
 */

#ifndef _ITM_NONLINEAR_MPC_OFFBOARD_NODE_H_
#define _ITM_NONLINEAR_MPC_OFFBOARD_NODE_H_
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <itm_nonlinear_mpc/mpc_command_rpyt.h>