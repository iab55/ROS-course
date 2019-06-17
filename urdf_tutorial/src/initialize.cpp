#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <urdf_tutorial/changescale.h>
#include <urdf_tutorial/changecontrolledjoints.h>
#include <algorithm>
#include <std_msgs/Float64.h>
#include <stdlib.h>
// Function for setting scale of
bool ctn=true;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "initialize");
  ros::NodeHandle n;

	ros::Publisher pub1 = n.advertise<std_msgs::Float64>(
		"seven_dof_arm/joint1_position_controller/command", 1000);
		ros::Publisher pub2 = n.advertise<std_msgs::Float64>(
			"seven_dof_arm/joint2_position_controller/command", 1000);
			ros::Publisher pub3 = n.advertise<std_msgs::Float64>(
				"seven_dof_arm/joint3_position_controller/command", 1000);
				ros::Publisher pub4 = n.advertise<std_msgs::Float64>(
					"seven_dof_arm/joint4_position_controller/command", 1000);
					ros::Publisher pub5 = n.advertise<std_msgs::Float64>(
						"seven_dof_arm/joint5_position_controller/command", 1000);
						ros::Publisher pub6 = n.advertise<std_msgs::Float64>(
							"seven_dof_arm/joint6_position_controller/command", 1000);
							ros::Publisher pub7 = n.advertise<std_msgs::Float64>(
								"seven_dof_arm/joint7_position_controller/command", 1000);
								ros::Publisher pub8 = n.advertise<std_msgs::Float64>(
									"seven_dof_arm/joint8_position_controller/command", 1000);
									ros::Publisher pub9 = n.advertise<std_msgs::Float64>(
										"seven_dof_arm/joint9_position_controller/command", 1000);

  // message declarations
  std_msgs::Float64 msg1, msg2, msg3, msg4, msg5, msg6, msg7, msg8, msg9;

	ros::Rate loop_rate(30);
	while (ros::ok()){

	if(ctn){
	msg1.data = 0;
	msg2.data = 0;
	msg3.data = 0;
	msg4.data = 0;
	msg5.data = 0;
	msg6.data = M_PI/2;
	msg7.data = 0;

	pub1.publish(msg1);
	pub2.publish(msg2);
	pub3.publish(msg3);
	pub4.publish(msg4);
	pub5.publish(msg5);
	pub6.publish(msg6);
	pub7.publish(msg7);
	ROS_INFO_STREAM("PUBLISHING");
	sleep(5);
		ctn=false;
	}
	loop_rate.sleep();
	}
  return 0;
}
