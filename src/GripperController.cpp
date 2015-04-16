#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include "opaque-pub.h"
#include "mp-pub.h"

using namespace std;

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;
GripperClient* gripper_client_right;
GripperClient* gripper_client_left;

int mpSocket;
string oprsSup = "OPRS_SUP";



//Open the gripper
void open(string gripper){
  pr2_controllers_msgs::Pr2GripperCommandGoal open;
  open.command.position = 0.06;
  open.command.max_effort = 100;  // Do not limit effort (negative)

  GripperClient* gripper_client;

  if (gripper=="RIGHT") {

	  ROS_INFO("Sending open goal");
	  gripper_client_right->sendGoal(open);

  }
  else {
	  ROS_INFO("Sending open goal");
	  gripper_client_left->sendGoal(open);
  }
	ros::Duration(3).sleep();
}

//Close the gripper
void close(string gripper){
	pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
	squeeze.command.position = 0.0;
	squeeze.command.max_effort = 50.0;  // Close gently


	if (gripper=="RIGHT") {
		ROS_INFO("Sending squeeze goal");
		gripper_client_right->sendGoal(squeeze);
	}
	else {
		ROS_INFO("Sending squeeze goal");
		gripper_client_left->sendGoal(squeeze);
	}
	ros::Duration(3).sleep();
}




string getNext(char *message, int *i) {
	string ret;
    while (message[*i]!=' ' && message[*i]!=')'){
    	ret=ret+message[*i];
    	(*i)++;
    }
    (*i)++;
    return ret;
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
			open(gripper);
			sendAnswer=true;
		}
		else if (command=="close") {
			close(gripper);
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




int main(int argc, char** argv){
  ros::init(argc, argv, "gripper_controller");


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
