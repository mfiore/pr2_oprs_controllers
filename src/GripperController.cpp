#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_gripper_sensor_msgs/PR2GripperGrabAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperReleaseAction.h>
#include "opaque-pub.h"
#include "mp-pub.h"

#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperFindContactAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperSlipServoAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperEventDetectorAction.h>

using namespace std;

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperGrabAction> GrabClient;
// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperReleaseAction> ReleaseClient;

typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperSlipServoAction> SlipClient;
// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperFindContactAction> ContactClient;
// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperEventDetectorAction> EventDetectorClient;
// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;


int mpSocket;
string oprsSup = "OPRS_SUP";

GrabClient* r_grab_client;
  ReleaseClient* r_release_client;
SlipClient* r_slip_client;
ContactClient* r_contact_client;
EventDetectorClient* r_event_detector_client;
GripperClient* r_gripper_client;







string getNext(char *message, int *i) {
	string ret;
    while (message[*i]!=' ' && message[*i]!=')'){
    	ret=ret+message[*i];
    	(*i)++;
    }
    (*i)++;
    return ret;
}

//Open the gripper, find contact on both fingers, and go into slip-servo control mode
  void grab(){
    pr2_gripper_sensor_msgs::PR2GripperGrabGoal grip;
    grip.command.hardness_gain = 0.03;

    ROS_INFO("Sending grab goal");
    r_grab_client->sendGoal(grip);
    r_grab_client->waitForResult(ros::Duration(20.0));
    if(r_grab_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Successfully completed Grab");
    else
      ROS_INFO("Grab Failed");
  }

  // Look for side impact, finerpad slip, or contact acceleration signals and release the object once these occur
  void release(){
    pr2_gripper_sensor_msgs::PR2GripperReleaseGoal place;
    // set the robot to release on a figner-side impact, fingerpad slip, or acceleration impact with hand/arm
    place.command.event.trigger_conditions = place.command.event.FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC;
    // set the acceleration impact to trigger on to 5 m/s^2
    place.command.event.acceleration_trigger_magnitude = 5.0;
    // set our slip-gain to release on to .005
    place.command.event.slip_trigger_magnitude = .01;


    ROS_INFO("Waiting for object placement contact...");
    r_release_client->sendGoal(place);
    r_release_client->waitForResult();
    if(r_release_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Release Success");
    else
      ROS_INFO("Place Failure");

  }

  //move into event_detector mode to detect object contact
  void grabOnContact(){
    pr2_gripper_sensor_msgs::PR2GripperEventDetectorGoal place_goal;
    place_goal.command.trigger_conditions = place_goal.command.FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC;
    place_goal.command.acceleration_trigger_magnitude = 4.0;  // set the contact acceleration to n m/s^2
    place_goal.command.slip_trigger_magnitude = .005;

    ROS_INFO("Waiting for object placement contact...");
    r_event_detector_client->sendGoal(place_goal);
    r_event_detector_client->waitForResult();
    if(r_event_detector_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Contact found");
      grab();
    }
    else
      ROS_INFO("Place Failure");
  }

  void open() {
	   pr2_controllers_msgs::Pr2GripperCommandGoal open;
	    open.command.position = 0.09;    // position open (9 cm)
	    open.command.max_effort = -1.0;  // unlimited motor effort

	    ROS_INFO("Sending open goal");
	    r_gripper_client->sendGoal(open);
	    r_gripper_client->waitForResult();
	    if(r_gripper_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	      ROS_INFO("The gripper opened!");
	    else
	      ROS_INFO("The gripper failed to open.");
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
			grab();
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
   r_grab_client= new GrabClient("r_gripper_sensor_controller/grab", true);
   r_release_client = new ReleaseClient("l_gripper_sensor_controller/release", true);
   r_gripper_client = new GripperClient("r_gripper_sensor_controller/gripper_action", true);
   r_contact_client  = new ContactClient("r_gripper_sensor_controller/find_contact",true);
   r_slip_client  = new SlipClient("r_gripper_sensor_controller/slip_servo",true);
   r_event_detector_client = new EventDetectorClient("r_gripper_sensor_controller/event_detector",true);




  //wait for the gripper action server to come up
  while(!r_grab_client->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");

  }
  while(!r_release_client->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the l_gripper_controller/gripper_action action server to come up");

  }

  //wait for the gripper action server to come up
  while(!r_gripper_client->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the r_gripper_sensor_controller/gripper_action action server to come up");
  }

  while(!r_contact_client->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the r_gripper_sensor_controller/find_contact action server to come up");
  }

  while(!r_slip_client->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the r_gripper_sensor_controller/slip_servo action server to come up");
  }

  while(!r_event_detector_client->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the r_gripper_sensor_controller/event_detector action server to come up");
  }




  mpSocket = external_register_to_the_mp_prot("gripper_controller", 3300, STRINGS_PT);

      if (mpSocket != -1) {
      	boost::thread oprsThread(oprsLoop);

      	ros::waitForShutdown();
  }

  return 0;
}
