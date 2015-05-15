/*
 * VimanController.cpp


 *
 *  Created on: Apr 15, 2015
 *      Author: mfiore
 *
 *
 *      Controller for VIMAN from OPRS.
 *      Syntax is (viman_controller.request switchCamera @mode)
 *      mode is WIDE or NARROW
 *
 *      return message is (viman_controller.report @report)
 *      @report= OK for now
 */

#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include "opaque-pub.h"
#include "mp-pub.h"
#include "viman_bridge/SwitchCameras.h"

using namespace std;

int mpSocket;
ros::ServiceClient client;
string oprsSup="OPRS_SUP";

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
	   //read the openprs message

	while (ros::ok) {
	    int length;
	    char *sender = read_string_from_socket(mpSocket, &length);
	    char *message = read_string_from_socket(mpSocket, &length);
	    ROS_INFO("%s\n", message);

	    viman_bridge::SwitchCameras srv;


	    int i=0;
	    getNext(message,&i);
	    string command=getNext(message,&i);
	    string mode=getNext(message,&i);
	    if (mode=="WIDE") {
	    	srv.request.cameras="wide_stereo";
	    }
	    else {
	    	srv.request.cameras="narrow_stereo";
	    }
	    client.call(srv);

	    char returnMessage[100];
	    string strMessage="(viman_controller.report OK)";
	    strcpy(returnMessage,strMessage.c_str());
	    send_message_string(returnMessage,oprsSup.c_str());

	}
}

int main(int argc, char** argv){
  ros::init(argc, argv, "viman_controller");

  ros::NodeHandle n;

  client = n.serviceClient<viman_bridge::SwitchCameras>("SwitchCameras");


  mpSocket = external_register_to_the_mp_prot("viman_controller", 3300, STRINGS_PT);

      if (mpSocket != -1) {
      	boost::thread oprsThread(oprsLoop);

      	ros::waitForShutdown();
  }

  return 0;
}

