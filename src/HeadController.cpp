/**
 * OPRS head interface
 * syntax is (HeadController.action @actionName @actionParameters)
 * @actionName= moveHead, lookAt, moveDir, track, cancel
 * @params:
 * moveHead pan tilt
 * lookAt x y z
 * moveDir dir
 * track frame
 *
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
#include <pr2_controllers_msgs/PointHeadAction.h>
#include "trajectory_msgs/JointTrajectory.h"
#include <sensor_msgs/JointState.h>
#include "ros/spinner.h"
#include <map>


using namespace std;

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

string oprsSup = "OPRS_SUP";
double maxPan=2.7;
double maxTilt=1.4;
double minTilt=-0.4;


bool cancelTrack=false;
bool tracking=false;


int mpSocket;
PointHeadClient* point_head_client_;
ros::Publisher head_pub_;
ros::Subscriber joint_state_sub_;
std::map<std::string, double> joint_state_position_map_;

string getNext(char *message, int *i) {
	string ret;
    while (message[*i]!=' ' && message[*i]!=')'){
    	ret=ret+message[*i];
    	(*i)++;
    }
    (*i)++;
    return ret;
}

void jointStateCallback(const sensor_msgs::JointStateConstPtr &jointState)
{
	for(unsigned int i = 0; i < jointState->name.size(); i++) {
		joint_state_position_map_[jointState->name[i]] = jointState->position[i];

	}
}




//! Points the high-def camera frame at a point in a given frame
void lookAt(std::string frame_id, double x, double y, double z)
{
  //the goal message we will be sending
  pr2_controllers_msgs::PointHeadGoal goal;
  //the target point, expressed in the requested frame

  geometry_msgs::PointStamped point;
  point.header.frame_id = frame_id;
  point.point.x = x; point.point.y = y; point.point.z = z;
  goal.target = point;

  //we are pointing the high-def camera frame
  //(pointing_axis defaults to X-axis)
  goal.pointing_frame = "high_def_frame";

  //take at least 0.5 seconds to get there
  goal.min_duration = ros::Duration(0.2);

  //and go no faster than 1 rad/s
  goal.max_velocity = 1.0;

  //send the goal
  point_head_client_->sendGoal(goal);

  //wait for it to get there (abort after 2 secs to prevent getting stuck)
  point_head_client_->waitForResult(ros::Duration(2));
}


//track
void track(std::string frame_id) {
	tracking=true;
	while (ros::ok() && !cancelTrack)
	{
		lookAt("r_gripper_tool_frame", 0, 0, 0);
		ros::Duration(0.5).sleep();
	}
	tracking=false;
}

void moveHead(double pan, double tilt) {

	pan=std::min(pan,maxPan);
	pan=std::max(pan,-maxPan);

	tilt=std::min(tilt,maxTilt);
	tilt=std::max(tilt,minTilt);


	trajectory_msgs::JointTrajectory traj;
	traj.header.stamp = ros::Time::now() + ros::Duration(0.01);
	traj.joint_names.push_back("head_pan_joint");
	traj.joint_names.push_back("head_tilt_joint");
	traj.points.resize(1);
	traj.points[0].positions.push_back(pan);
	traj.points[0].velocities.push_back(0.0);//req_pan_vel);
	traj.points[0].positions.push_back(tilt);
	traj.points[0].velocities.push_back(0.0);//req_tilt_vel);
	traj.points[0].time_from_start = ros::Duration(0.1);
	//ROS_INFO_STREAM("Publishing " << req_pan << " " << req_tilt);
	head_pub_.publish(traj);

}

void moveDir(char dir) {
	 double pan;
	 double tilt;
	 pan=joint_state_position_map_["head_pan_joint"];
	 tilt=joint_state_position_map_["head_tilt_joint"];

	 ROS_INFO("Moving head toward %c",dir);
      if (dir=='u') {
    	  tilt=tilt-0.2;
      }
      else if (dir=='l') {
    	  pan=pan-0.2;
      }
      else if (dir=='r') {
    	  pan=pan+0.2;
      }
      else if (dir=='d') {
    	  tilt=tilt+0.2;
      }
 	 ROS_INFO("Final pan and tilt are %f %f",pan,tilt);

      moveHead(pan,tilt);
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
    if (command=="lookAt") {
        string x=getNext(message,&i);
        string y=getNext(message,&i);
        string z=getNext(message,&i);
        lookAt("base_link",boost::lexical_cast<double>(x),boost::lexical_cast<double>(y),boost::lexical_cast<double>(z));
        sendAnswer=true;
    }
    else if (command=="track") {
    	string frame=getNext(message,&i);
    	track(frame);
    	sendAnswer=true;
    }
    else if (command=="cancel") {
    	if (tracking==true) {
    		cancelTrack=true;
    	}
    }
    else if (command=="moveHead") {
    	string pan=getNext(message,&i);
    	string tilt=getNext(message,&i);
    	moveHead(boost::lexical_cast<double>(pan),boost::lexical_cast<double>(tilt));
    	sendAnswer=true;
    }
    else if (command=="moveDir") {
    	string dir=getNext(message,&i);
    	moveDir(dir[0]);
    	sendAnswer=true;
    }
	if (sendAnswer) {
			sendAnswer=false;
			string strMessage=("(pr2_oprs_head_controller.report OK)");
			char returnMessage[100];
		     strcpy(returnMessage, strMessage.c_str());
		     ROS_INFO("Return message %s", returnMessage);
		     send_message_string(returnMessage, oprsSup.c_str());
		}
	}

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pr2_oprs_head_controller");
    ros::NodeHandle n_;
    ros::AsyncSpinner spinner(1);

    head_pub_ = n_.advertise<trajectory_msgs::JointTrajectory>("head_traj_controller/command", 1);

    joint_state_sub_ = n_.subscribe("joint_states", 1, &jointStateCallback);

    //Initialize the client for the Action interface to the head controller
        point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);

        //wait for head controller action server to come up
        while(!point_head_client_->waitForServer(ros::Duration(5.0))){
          ROS_INFO("Waiting for the point_head_action server to come up");
        }

        mpSocket = external_register_to_the_mp_prot("pr2_oprs_head_controller", 3300, STRINGS_PT);

        if (mpSocket != -1) {
        	spinner.start();
        	boost::thread oprsThread(oprsLoop);

        	ros::waitForShutdown();
    }
}

