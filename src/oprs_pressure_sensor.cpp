#include <ros/ros.h>
#include <opaque-pub.h>
#include <mp-pub.h>

#include <pr2_msgs/PressureState.h>


int mp_socket;
string oprs_db="OPRS_DB";
bool hadPression=false;

void rightCallback(const pr2_msgs::PressureState::ConstPtr& msg) {
	vector<int> l_finger_tip=msg->l_finger_tip;
	vector<int> r_finger_tip=msg->r_finger_tip;

	double sum_l=0;
	double sum_r=0;
	for (int i=0; i<l_finger_tip.size();i++) {
		sum_l=sum_l+l_finger_tip[i];
		sum_r=sum_r+r_finger_tip[i];
	}
	sum_l=sum_l/l_finger_tip.size();
	sum_r=sum_r/r_finger_tip.size();

	double avg=(sum_l+sum_r) /2;

	if (avg>3500 && !hadPression) {
		send_message_string("(AGENT-STATEMENT PR2_ROBOT R_GRIPPER detectPression TRUE)",oprs_db.c_str());
		hadPression=true;
	}
	else if (hadPression){
		send_message_string("(AGENT-STATEMENT PR2_ROBOT R_GRIPPER detectPression FALSE)",oprs_db.c_str());
		hadPression=false;
	}

}

int main(int argc, char ** argv) {
	ros::init(argc,argv,"oprs_pressure_sensor");
	ros::NodeHandle node_handle;

	ROS_INFO("Started oprs_pressure_sensor node");

	// ros::Subscriber left_subscriber=node_handle.subscribe("/pressure/l_gripper_motor",100,leftCallback);
	ros::Subscriber right_subscriber=node_handle.subscribe("/pressure/r_gripper_motor",100,rightCallback);

	ROS_INFO("Reading pressure sensor info on right gripper");

	mp_socket=external_register_to_the_mp_host_prot("oprs_pressure_sensor","localhost", 3300, STRINGS_PT);
	if (mp_socket!=-1) {
		ROS_INFO("Connected to the oprs");
		ros::spin();	
	}	
	else {
		ROS_INFO("Couldn't connect to oprs");
	}
	return 0;
}