/*
 * KeyboardController.cpp
 *
 *  Created on: Apr 17, 2015
 *      Author: mfiore
 */




#include "opaque-pub.h"
#include "mp-pub.h"
#include "ros/ros.h"
#include <iostream>
#include "std_msgs/String.h"
#include "boost/thread.hpp"
#include "boost/lexical_cast.hpp"
#include "string.h"
#include <vector>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <moveit_msgs/GetKinematicSolverInfo.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include "trajectory_msgs/JointTrajectory.h"
#include <sensor_msgs/JointState.h>
#include "ros/spinner.h"
#include <map>
#include <pr2_teleop_general/include/pr2_teleop_general/pr2_teleop_general_commander.h>





int main(int argc, char** argv){
	ros::init(argc, argv, "arm_controller");

	ros::NodeHandle n_;
	GeneralCommander gc(false,
	false,
	true,
	false,
	false,
	"arm_controller");

	while (ros::ok) {

	gc.sendArmVelCommands(0.0,0.0,0,0.0,-0.10,0.0,
	0.0, 0.0,0.0,0.0,0.0,0.0,
	20.0);
	}
	ros::shutdown();


}
