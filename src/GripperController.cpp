/*
 * GripperController.cpp
 *
 *  Created on: Apr 30, 2015
 *      Author: mfiore
 */


#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>
#include "opaque-pub.h"
#include "mp-pub.h"
#include "pr2_oprs_controllers/UseGripper.h"
#include <boost/thread.hpp>

using namespace std;

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;



int mpSocket;
string oprsSup = "OPRS_SUP";

GripperClient* gripper_client_;


string getNext(char *message, int *i) {
	string ret;
    while (message[*i]!=' ' && message[*i]!=')'){
    	ret=ret+message[*i];
    	(*i)++;
    }
    (*i)++;
    return ret;
}


  //Open the gripper
  void open(){
    pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command.position = 0.08;
    open.command.max_effort = -1.0;  // Do not limit effort (negative)

    ROS_INFO("Sending open goal");
    gripper_client_->sendGoal(open);
    gripper_client_->waitForResult();
    string status=gripper_client_->getState().getText();
    ROS_INFO("%s",status.c_str());
    sleep(5);
    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The gripper opened!");
    else
      ROS_INFO("The gripper failed to open.");
  }



  //Close the gripper
  void close(){
    pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
    squeeze.command.position = 0.0;
    squeeze.command.max_effort = 100.0;  // Close gently

    ROS_INFO("Sending squeeze goal");
    gripper_client_->sendGoal(squeeze);
    gripper_client_->waitForResult();
    sleep(5);
    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The gripper closed!");
    else
      ROS_INFO("The gripper failed to close.");
  }


void oprsLoop() {
	bool sendAnswer=false;

	while (ros::ok()) {
		//read the openprs message
		int length;

		char *sender = read_string_from_socket(mpSocket, &length);
		char *message = read_string_from_socket(mpSocket, &length);
		ROS_INFO("%s\n", message);

		int i=0;
		while (message[i]!=' ') {
			i++;
		}
		i++;

		string command=getNext(message,&i);
		string gripper=getNext(message,&i);
		if (command=="open") {
			open();
			sendAnswer=true;
		}
		else if (command=="close") {
			close();
			sendAnswer=true;
		}
		if (sendAnswer) {
			sendAnswer=false;
			string strMessage=("(gripper_controller.result OK)");
			char returnMessage[100];
		     strcpy(returnMessage, strMessage.c_str());
		     ROS_INFO("Return message %s", returnMessage);
		     send_message_string(returnMessage, oprsSup.c_str());
		}

	}

}

bool UseGripperSrv(
		pr2_oprs_controllers::UseGripper::Request  &req,
        pr2_oprs_controllers::UseGripper::Response &res) {
	ROS_INFO("Received request");
	if (req.mode=="open") {
		ROS_INFO("calling open");
		open();
	}
	else {
		close();
	}
	res.result="ok";
	return true;

}



int main(int argc, char** argv){
  ros::init(argc, argv, "simple_gripper");
  ros::NodeHandle n;

  ros::ServiceServer serviceGripper = n.advertiseService("pr2_oprs_controllers/use_gripper", UseGripperSrv);
  gripper_client_ = new GripperClient("r_gripper_controller/gripper_action", true);

  while(!gripper_client_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");
  }


  mpSocket = external_register_to_the_mp_prot("gripper_controller", 3300, STRINGS_PT);

      if (mpSocket != -1) {
      	boost::thread oprsThread(oprsLoop);

      	ros::spin();
      	ros::waitForShutdown();
  }

  return 0;
}

