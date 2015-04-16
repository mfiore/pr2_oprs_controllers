/*
 * VimanController.cpp
 *
 *  Created on: Apr 15, 2015
 *      Author: mfiore
 */

#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include "opaque-pub.h"
#include "mp-pub.h"

using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "viman_controller");

  ros::ServiceClient client = n.serviceClient<viman_bridge::QueryDatabase>("QueryDatabase");



  //Initialize the client for the Action interface to the gripper controller
  //and tell the action client that we want to spin a thread by default
  gripper_client_right = new GripperClient("r_gripper_controller/gripper_action", true);
  gripper_client_left = new GripperClient("l_gripper_controller/gripper_action", true);

  //wait for the gripper action server to come up
  while(!gripper_client_right->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");

  }
  while(!gripper_client_left->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the l_gripper_controller/gripper_action action server to come up");

  }

  mpSocket = external_register_to_the_mp_prot("gripper_controller", 3300, STRINGS_PT);

      if (mpSocket != -1) {
      	boost::thread oprsThread(oprsLoop);

      	ros::waitForShutdown();
  }

  return 0;
}

