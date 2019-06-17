#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <urdf_tutorial/changescale.h>
#include <urdf_tutorial/changecontrolledjoints.h>
#include <algorithm>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <math.h>

const double degree2rad = M_PI/180;
double copied_x;
double copied_th;
double scale = 1;
int joint1=0;
int joint2=1;
double speed_0, speed_1, speed_2, speed_3, speed_4, speed_5, speed_6, speed_7, speed_8;
double pos0, pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8;
double delta0, delta1, delta2, delta3, delta4, delta5, delta6, delta7, delta8;
double aruco_x, aruco_y, aruco_z, aruco_qx, aruco_qy, aruco_qz, aruco_qw;
double roll_aruco, pitch_aruco, yaw_aruco, roll_camera, pitch_camera, yaw_camera;
double rotzy_aruco_11, rotzy_aruco_12, rotzy_aruco_13, rotzy_aruco_21, rotzy_aruco_22, rotzy_aruco_23, rotzy_aruco_31, rotzy_aruco_32, rotzy_aruco_33;
double rotzyx_aruco_11, rotzyx_aruco_12, rotzyx_aruco_13, rotzyx_aruco_21, rotzyx_aruco_22, rotzyx_aruco_23, rotzyx_aruco_31, rotzyx_aruco_32, rotzyx_aruco_33;

double rotzy_camera_11, rotzy_camera_12, rotzy_camera_13, rotzy_camera_21, rotzy_camera_22, rotzy_camera_23, rotzy_camera_31, rotzy_camera_32, rotzy_camera_33;
double rotzyx_camera_11, rotzyx_camera_12, rotzyx_camera_13, rotzyx_camera_21, rotzyx_camera_22, rotzyx_camera_23, rotzyx_camera_31, rotzyx_camera_32, rotzyx_camera_33;
double rot_cube2base_11, rot_cube2base_12, rot_cube2base_13, rot_cube2base_21, rot_cube2base_22, rot_cube2base_23, rot_cube2base_31, rot_cube2base_32, rot_cube2base_33, rot_cube2base_14, rot_cube2base_24, rot_cube2base_34;
double pitch_grasp, roll_grasp, yaw_grasp,rotX,rotY,rotZ;
// Function for setting scale of
bool setScale(
	urdf_tutorial::changescale::Request &req,
	urdf_tutorial::changescale::Response &resp){
        scale = req.s;
        ROS_INFO_STREAM("Scale is "<<(scale)<< " now.");
	return true;
}
// Function for choosing joints to operate
/* bool chooseJoints(
	urdf_tutorial::changecontrolledjoints::Request &req,
	urdf_tutorial::changecontrolledjoints::Response &resp){
        joint1 = req.c1;
        joint2 = req.c2;
        ROS_INFO_STREAM("The joints "<<(joint1)<< " and "<<(joint2)<<" are being operated.");
	return true;
} */

// subscribing to velocity of operating (call-back function)
void poseMessageReceived(const sensor_msgs::JointState& msg) {
		speed_0=msg.velocity[0];
		speed_1=msg.velocity[1];
		speed_2=msg.velocity[2];
		speed_3=msg.velocity[3];
		speed_4=msg.velocity[4];
		speed_5=msg.velocity[5];
		speed_6=msg.velocity[6];
		speed_7=msg.velocity[7];
		speed_8=msg.velocity[8];
}

void poseMessageReceived2(const geometry_msgs::TransformStamped& msg) {
		/*aruco_x=msg.pose.position.x;
		aruco_y=msg.pose.position.y;
		aruco_z=msg.pose.position.z;
		aruco_qx=msg.pose.rotation.x;
		aruco_qy=msg.pose.rotation.y;
		aruco_qz=msg.pose.rotation.z;
		aruco_qw=msg.pose.rotation.w;

		double sinr = +2.0 * (msg.pose.rotation.w * msg.pose.rotation.x + msg.pose.rotation.y * msg.pose.rotation.z);
		double cosr = +1.0 - 2.0 * (msg.pose.rotation.x * msg.pose.rotation.x + msg.pose.rotation.y*msg.pose.rotation.y);
		pitch_aruco = atan2(sinr, cosr); //x

		// pitch (y-axis rotation)
		double sinp = +2.0 * (msg.pose.rotation.w * msg.pose.rotation.y - msg.pose.rotation.z * msg.pose.rotation.x);
		if (fabs(sinp) >= 1)
			roll_aruco = copysign(M_PI / 2, sinp); // use 90 degrees if out of range y
		else
			roll_aruco = asin(sinp); //y

		// yaw (z-axis rotation)
		double siny = +2.0 * (msg.pose.rotation.w * msg.pose.rotation.z + msg.pose.rotation.x * msg.pose.rotation.y);
		double cosy = +1.0 - 2.0 * (msg.pose.rotation.y * msg.pose.rotation.y + msg.pose.rotation.z * msg.pose.rotation.z);
		yaw_aruco = atan2(siny, cosy);//z*/

//ROS_INFO_STREAM("rot x " << (pitch_aruco));
//ROS_INFO_STREAM("rot y " << (roll_aruco));
//ROS_INFO_STREAM("rot z " << (yaw_aruco));
		/*rotzy_aruco_11 = cos(yaw_aruco)*cos(pitch_aruco);
		rotzy_aruco_12 = -sin(yaw_aruco);
		rotzy_aruco_13 = cos(yaw_aruco)*sin(pitch_aruco);
		rotzy_aruco_21 = sin(yaw_aruco)*cos(pitch_aruco);
		rotzy_aruco_22 = cos(yaw_aruco);
		rotzy_aruco_23 = sin(yaw_aruco)*sin(pitch_aruco);
		rotzy_aruco_31 = -sin(yaw_aruco);
		rotzy_aruco_32 = 0;
		rotzy_aruco_33 = cos(yaw_aruco);

		rotzyx_aruco_11 = rotzy_aruco_11;
		rotzyx_aruco_12 = rotzy_aruco_12*cos(roll_aruco)+rotzy_aruco_13*sin(roll_aruco);
		rotzyx_aruco_13 = -rotzy_aruco_12*sin(roll_aruco)+rotzy_aruco_13*cos(roll_aruco);
		rotzyx_aruco_21 = rotzy_aruco_21;
		rotzyx_aruco_22 = rotzy_aruco_22*cos(roll_aruco)+rotzy_aruco_23*sin(roll_aruco);
		rotzyx_aruco_23 = -rotzy_aruco_22*sin(roll_aruco)+rotzy_aruco_23*cos(roll_aruco);
		rotzyx_aruco_31 = rotzy_aruco_31;
		rotzyx_aruco_32 = -rotzy_aruco_32*sin(roll_aruco)+rotzy_aruco_33*cos(roll_aruco);
		rotzyx_aruco_33 = -rotzy_aruco_22*sin(roll_aruco)+rotzy_aruco_23*cos(roll_aruco);*/

		//YZX
		/*rotzy_aruco_11 = cos(yaw_aruco);
		rotzy_aruco_12 = -sin(yaw_aruco)*cos(pitch_aruco);
		rotzy_aruco_13 = sin(yaw_aruco)*sin(pitch_aruco);
		rotzy_aruco_21 = sin(yaw_aruco);
		rotzy_aruco_22 = cos(yaw_aruco)*cos(pitch_aruco);
		rotzy_aruco_23 = -cos(yaw_aruco)*sin(pitch_aruco);
		rotzy_aruco_31 = 0;
		rotzy_aruco_32 = sin(pitch_aruco);
		rotzy_aruco_33 = cos(pitch_aruco);

		rotzyx_aruco_11 = cos(roll_aruco)*rotzy_aruco_11;
		rotzyx_aruco_12 = cos(roll_aruco)*rotzy_aruco_12+sin(roll_aruco)*rotzy_aruco_32;
		rotzyx_aruco_13 = cos(roll_aruco)*rotzy_aruco_13+sin(roll_aruco)*rotzy_aruco_33;
		rotzyx_aruco_21 = rotzy_aruco_21;
		rotzyx_aruco_22 = rotzy_aruco_22;
		rotzyx_aruco_23 = rotzy_aruco_23;
		rotzyx_aruco_31 = -sin(roll_aruco)*rotzy_aruco_11;
		rotzyx_aruco_32 = -rotzy_aruco_12*sin(roll_aruco)+cos(roll_aruco)*rotzy_aruco_32;
		rotzyx_aruco_33 = -rotzy_aruco_13*sin(roll_aruco)+cos(roll_aruco)*rotzy_aruco_33;*/

		//ZYX
		/*rotzy_aruco_11 = cos(roll_aruco);
		rotzy_aruco_12 = 0;
		rotzy_aruco_13 = sin(roll_aruco);
		rotzy_aruco_21 = sin(roll_aruco)*sin(pitch_aruco);
		rotzy_aruco_22 = cos(pitch_aruco);
		rotzy_aruco_23 = -sin(pitch_aruco)*cos(roll_aruco);
		rotzy_aruco_31 = -sin(roll_aruco)*cos(pitch_aruco);
		rotzy_aruco_32 = sin(pitch_aruco);
		rotzy_aruco_33 = cos(pitch_aruco)*cos(roll_aruco);

		rotzyx_aruco_12 = rotzy_aruco_11*cos(yaw_aruco)-rotzy_aruco_12*sin(yaw_aruco);
		rotzyx_aruco_11 = -sin(yaw_aruco)*rotzy_aruco_22;
		rotzyx_aruco_13 = rotzy_aruco_13*cos(yaw_aruco)-rotzy_aruco_23*sin(yaw_aruco);
		rotzyx_aruco_21 = rotzy_aruco_11*sin(yaw_aruco)+rotzy_aruco_12*cos(yaw_aruco);
		rotzyx_aruco_22 = cos(yaw_aruco)*rotzy_aruco_22;
		rotzyx_aruco_23 = rotzy_aruco_13*sin(yaw_aruco)+rotzy_aruco_23*cos(yaw_aruco);
		rotzyx_aruco_31 = rotzy_aruco_31;
		rotzyx_aruco_32 = rotzy_aruco_32;
		rotzyx_aruco_33 = rotzy_aruco_33;*/

    /*ROS_INFO_STREAM("roll " << roll_aruco);
		ROS_INFO_STREAM("yaw " << yaw_aruco);
		ROS_INFO_STREAM("pitch " << pitch_aruco);*/
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "final_project_robot");
  ros::NodeHandle n;

  // Advertising services of scale setting and choice of operating joints
  ros::ServiceServer server = n.advertiseService("set_scale",&setScale);
  // Advertising states of individual joints
  //ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
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

  // Subscribing to velocity of operating
  ros::Subscriber sub = n.subscribe("turtle1/cmd_vel", 1, &poseMessageReceived);
	ros::Subscriber sub1 = n.subscribe("aruco_single/transform", 1, &poseMessageReceived);
  // Adding transform listener object
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // message declarations
  std_msgs::Float64 msg1, msg2, msg3, msg4, msg5, msg6, msg7, msg8, msg9;




  ros::Rate loop_rate(30);
  while (ros::ok())
  {
      // Getting transformation between base link and grasping frame
      geometry_msgs::TransformStamped transformStamped;
			geometry_msgs::TransformStamped transformStamped2;

      try {
         transformStamped = tfBuffer.lookupTransform("base_link","grasping_frame",
                                  ros::Time(0));
				 transformStamped2 = tfBuffer.lookupTransform( "base_link","aruco_marker_frame",
										          ros::Time(0));
       }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }

			tf::Transform transform;
			transform.setOrigin( tf::Vector3(aruco_x, aruco_y, aruco_z) );
			transform.setRotation(tf::Quaternion(aruco_qx, aruco_qy, aruco_qz, aruco_qw));


      // Saving obtained z-coordinate (grasping frame related to base link)
			double grasping_x_coord = transformStamped.transform.translation.x;
			double grasping_y_coord = transformStamped.transform.translation.y;
      double grasping_z_coord = transformStamped.transform.translation.z;
			double grasping_qx = transformStamped.transform.rotation.x;
			double grasping_qy = transformStamped.transform.rotation.y;
			double grasping_qz = transformStamped.transform.rotation.z;
			double grasping_qw = transformStamped.transform.rotation.w;

			double sinr = +2.0 * (transformStamped.transform.rotation.w * transformStamped.transform.rotation.x + transformStamped.transform.rotation.y * transformStamped.transform.rotation.z);
			double cosr = +1.0 - 2.0 * (transformStamped.transform.rotation.x * transformStamped.transform.rotation.x + transformStamped.transform.rotation.y * transformStamped.transform.rotation.y);
			pitch_grasp = atan2(sinr, cosr);// rot x

			// pitch (y-axis rotation)
			double sinp = +2.0 * (transformStamped.transform.rotation.w * transformStamped.transform.rotation.y - transformStamped.transform.rotation.z * transformStamped.transform.rotation.x);
			if (fabs(sinp) >= 1)
				roll_grasp = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
			else
				roll_grasp = asin(sinp); // rot y

			// yaw (z-axis rotation)
			double siny = +2.0 * (transformStamped.transform.rotation.w * transformStamped.transform.rotation.z + transformStamped.transform.rotation.x * transformStamped.transform.rotation.y);
			double cosy = +1.0 - 2.0 * (transformStamped.transform.rotation.y * transformStamped.transform.rotation.y + transformStamped.transform.rotation.z * transformStamped.transform.rotation.z);
			yaw_grasp = atan2(siny, cosy);// rot z


			double camera_x_coord = transformStamped2.transform.translation.x;
			double camera_y_coord = transformStamped2.transform.translation.y;
      double camera_z_coord = transformStamped2.transform.translation.z;

			double camera_qx = transformStamped2.transform.rotation.x;
			double camera_qy = transformStamped2.transform.rotation.y;
			double camera_qz = transformStamped2.transform.rotation.z;
			double camera_qw = transformStamped2.transform.rotation.w;

			sinr = +2.0 * (transformStamped2.transform.rotation.w * transformStamped2.transform.rotation.x + transformStamped2.transform.rotation.y * transformStamped2.transform.rotation.z);
			cosr = +1.0 - 2.0 * (transformStamped2.transform.rotation.x * transformStamped2.transform.rotation.x + transformStamped2.transform.rotation.y * transformStamped2.transform.rotation.y);
			pitch_camera = atan2(sinr, cosr);// rot x

			// pitch (y-axis rotation)
			sinp = +2.0 * (transformStamped2.transform.rotation.w * transformStamped2.transform.rotation.y - transformStamped2.transform.rotation.z * transformStamped2.transform.rotation.x);
			if (fabs(sinp) >= 1)
				roll_camera = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
			else
				roll_camera = asin(sinp); // rot y

			// yaw (z-axis rotation)
			siny = +2.0 * (transformStamped2.transform.rotation.w * transformStamped2.transform.rotation.z + transformStamped2.transform.rotation.x * transformStamped2.transform.rotation.y);
			cosy = +1.0 - 2.0 * (transformStamped2.transform.rotation.y * transformStamped2.transform.rotation.y + transformStamped2.transform.rotation.z * transformStamped2.transform.rotation.z);
			yaw_camera = atan2(siny, cosy);// rot z

			/*rotzy_camera_11 = cos(yaw_camera)*cos(pitch_camera);
			rotzy_camera_12 = -sin(yaw_camera);
			rotzy_camera_13 = cos(yaw_camera)*sin(pitch_camera);
			rotzy_camera_21 = sin(yaw_camera)*cos(pitch_camera);
			rotzy_camera_22 = cos(yaw_camera);
			rotzy_camera_23 = sin(yaw_camera)*sin(pitch_camera);
			rotzy_camera_31 = -sin(yaw_camera);
			rotzy_camera_32 = 0;
			rotzy_camera_33 = cos(yaw_camera);

			rotzyx_camera_11 = rotzy_camera_11;
			rotzyx_camera_12 = rotzy_camera_12*cos(roll_camera)+rotzy_camera_13*sin(roll_camera);
			rotzyx_camera_13 = -rotzy_camera_12*sin(roll_camera)+rotzy_camera_13*cos(roll_camera);
			rotzyx_camera_21 = rotzy_camera_21;
			rotzyx_camera_22 = rotzy_camera_22*cos(roll_camera)+rotzy_camera_23*sin(roll_camera);
			rotzyx_camera_23 = -rotzy_camera_22*sin(roll_camera)+rotzy_camera_23*cos(roll_camera);
			rotzyx_camera_31 = rotzy_camera_31;
			rotzyx_camera_32 = -rotzy_camera_32*sin(roll_camera)+rotzy_camera_33*cos(roll_camera);
			rotzyx_camera_33 = -rotzy_camera_22*sin(roll_camera)+rotzy_camera_23*cos(roll_camera);

			rot_cube2base_11 = rotzyx_camera_11*rotzyx_aruco_11+rotzyx_camera_12*rotzyx_aruco_21+rotzyx_camera_13*rotzyx_aruco_31;
			rot_cube2base_12 = rotzyx_camera_11*rotzyx_aruco_12+rotzyx_camera_12*rotzyx_aruco_22+rotzyx_camera_13*rotzyx_aruco_32;
			rot_cube2base_13 = rotzyx_camera_11*rotzyx_aruco_13+rotzyx_camera_12*rotzyx_aruco_23+rotzyx_camera_13*rotzyx_aruco_33;
			rot_cube2base_14 = rotzyx_camera_11*rotzyx_aruco_x+rotzyx_camera_12*rotzyx_aruco_y+rotzyx_camera_13*rotzyx_aruco_z+camera_x_coord;

			rot_cube2base_21 = rotzyx_camera_21*rotzyx_aruco_11+rotzyx_camera_22*rotzyx_aruco_21+rotzyx_camera_23*rotzyx_aruco_31;
			rot_cube2base_22 = rotzyx_camera_21*rotzyx_aruco_12+rotzyx_camera_22*rotzyx_aruco_22+rotzyx_camera_23*rotzyx_aruco_32;
			rot_cube2base_23 = rotzyx_camera_21*rotzyx_aruco_13+rotzyx_camera_22*rotzyx_aruco_23+rotzyx_camera_23*rotzyx_aruco_33;
			rot_cube2base_24 = rotzyx_camera_21*rotzyx_aruco_x+rotzyx_camera_22*rotzyx_aruco_y+rotzyx_camera_23*rotzyx_aruco_z+camera_y_coord;

			rot_cube2base_31 = rotzyx_camera_31*rotzyx_aruco_11+rotzyx_camera_32*rotzyx_aruco_21+rotzyx_camera_33*rotzyx_aruco_31;
			rot_cube2base_32 = rotzyx_camera_31*rotzyx_aruco_12+rotzyx_camera_32*rotzyx_aruco_22+rotzyx_camera_33*rotzyx_aruco_32;
			rot_cube2base_33 = rotzyx_camera_31*rotzyx_aruco_13+rotzyx_camera_32*rotzyx_aruco_23+rotzyx_camera_33*rotzyx_aruco_33;
			rot_cube2base_34 = rotzyx_camera_31*rotzyx_aruco_x+rotzyx_camera_32*rotzyx_aruco_y+rotzyx_camera_33*rotzyx_aruco_z+camera_z_coord;*/

					rotzy_camera_11 = cos(yaw_camera);
					rotzy_camera_12 = -sin(yaw_camera)*cos(pitch_camera);
					rotzy_camera_13 = sin(yaw_camera)*sin(pitch_camera);
					rotzy_camera_21 = sin(yaw_camera);
					rotzy_camera_22 = cos(yaw_camera)*cos(pitch_camera);
					rotzy_camera_23 = -cos(yaw_camera)*sin(pitch_camera);
					rotzy_camera_31 = 0;
					rotzy_camera_32 = sin(pitch_camera);
					rotzy_camera_33 = cos(pitch_camera);

					rotzyx_camera_11 = cos(roll_camera)*rotzy_camera_11;
					rotzyx_camera_12 = cos(roll_camera)*rotzy_camera_12+sin(roll_camera)*rotzy_camera_32;
					rotzyx_camera_13 = cos(roll_camera)*rotzy_camera_13+sin(roll_camera)*rotzy_camera_33;
					rotzyx_camera_21 = rotzy_camera_21;
					rotzyx_camera_22 = rotzy_camera_22;
					rotzyx_camera_23 = rotzy_camera_23;
					rotzyx_camera_31 = -sin(roll_camera)*rotzy_camera_11;
					rotzyx_camera_32 = -rotzy_camera_12*sin(roll_camera)+cos(roll_camera)*rotzy_camera_32;
					rotzyx_camera_33 = -rotzy_camera_13*sin(roll_camera)+cos(roll_camera)*rotzy_camera_33;


					rot_cube2base_11 = rotzyx_camera_11*rotzyx_aruco_11+rotzyx_camera_12*rotzyx_aruco_21+rotzyx_camera_13*rotzyx_aruco_31;
					rot_cube2base_12 = rotzyx_camera_11*rotzyx_aruco_12+rotzyx_camera_12*rotzyx_aruco_22+rotzyx_camera_13*rotzyx_aruco_32;
					rot_cube2base_13 = rotzyx_camera_11*rotzyx_aruco_13+rotzyx_camera_12*rotzyx_aruco_23+rotzyx_camera_13*rotzyx_aruco_33;
					rot_cube2base_14 = rotzyx_camera_11*aruco_x+rotzyx_camera_12*aruco_y+rotzyx_camera_13*aruco_z+camera_x_coord;

					rot_cube2base_21 = rotzyx_camera_21*rotzyx_aruco_11+rotzyx_camera_22*rotzyx_aruco_21+rotzyx_camera_23*rotzyx_aruco_31;
					rot_cube2base_22 = rotzyx_camera_21*rotzyx_aruco_12+rotzyx_camera_22*rotzyx_aruco_22+rotzyx_camera_23*rotzyx_aruco_32;
					rot_cube2base_23 = rotzyx_camera_21*rotzyx_aruco_13+rotzyx_camera_22*rotzyx_aruco_23+rotzyx_camera_23*rotzyx_aruco_33;
					rot_cube2base_24 = rotzyx_camera_21*aruco_x+rotzyx_camera_22*aruco_y+rotzyx_camera_23*aruco_z+camera_y_coord;

					rot_cube2base_31 = rotzyx_camera_31*rotzyx_aruco_11+rotzyx_camera_32*rotzyx_aruco_21+rotzyx_camera_33*rotzyx_aruco_31;
					rot_cube2base_32 = rotzyx_camera_31*rotzyx_aruco_12+rotzyx_camera_32*rotzyx_aruco_22+rotzyx_camera_33*rotzyx_aruco_32;
					rot_cube2base_33 = rotzyx_camera_31*rotzyx_aruco_13+rotzyx_camera_32*rotzyx_aruco_23+rotzyx_camera_33*rotzyx_aruco_33;
					rot_cube2base_34 = rotzyx_camera_31*aruco_x+rotzyx_camera_32*aruco_y+rotzyx_camera_33*aruco_z+camera_z_coord;


					rotZ = asin(rot_cube2base_12);
					rotX = atan2(-rot_cube2base_23,rot_cube2base_22);
					rotY = atan2(-rot_cube2base_31,rot_cube2base_11);
					/*ROS_INFO_STREAM("rotzyx_camera_11 " << (rotzyx_camera_11));
					ROS_INFO_STREAM("aruco_x " << (aruco_x));
					ROS_INFO_STREAM("rotzyx_camera_12 " << (rotzyx_camera_12));
					ROS_INFO_STREAM("aruco_y " << (aruco_y));
					ROS_INFO_STREAM("rotzyx_camera_13 " << (rotzyx_camera_13));
					ROS_INFO_STREAM("aruco_z " << (aruco_z));*/
					/*ROS_INFO_STREAM("A Pose: XYZ: " << (rot_cube2base_14) << ", " << (rot_cube2base_24) << ", " << (rot_cube2base_34) << "Rots " << (rotX) << ", " << (rotY) << ", " << (rotZ));
				  ROS_INFO_STREAM("G Pose: XYZ: " << (grasping_x_coord) << ", " << (grasping_y_coord) << ", " << (grasping_z_coord) << "Rots " << (pitch_grasp) << ", " << (roll_grasp) << ", " << (yaw_grasp));
					ROS_INFO_STREAM("E Pose: XYZ: " << (rot_cube2base_14-grasping_x_coord) << ", " << (rot_cube2base_24-grasping_y_coord) << ", " << (rot_cube2base_34-grasping_z_coord) << "Rots " << (rotX-pitch_grasp) << ", " << (rotY-roll_grasp) << ", " << (rotZ-yaw_grasp));*/
					ROS_INFO_STREAM("A Pose: XYZ: " << (camera_x_coord) << ", " << (camera_y_coord) << ", " << (camera_z_coord) << "Rots " << (pitch_camera) << ", " << (roll_camera) << ", " << (yaw_camera));
				  ROS_INFO_STREAM("G Pose: XYZ: " << (grasping_x_coord) << ", " << (grasping_y_coord) << ", " << (grasping_z_coord) << "Rots " << (pitch_grasp) << ", " << (roll_grasp) << ", " << (yaw_grasp));
					ROS_INFO_STREAM("E Pose: XYZ: " << (camera_x_coord-grasping_x_coord) << ", " << (camera_y_coord-grasping_y_coord) << ", " << (camera_z_coord-grasping_z_coord) << "Rots " << (camera_qx-grasping_qx) << ", " << (camera_qy-grasping_qy) << ", " << (camera_qz-grasping_qz));

      // Movement in one iteration
      delta0 = degree2rad*speed_0*scale;
      delta1 = degree2rad*speed_1*scale;
			delta2 = degree2rad*speed_2*scale;
      delta3 = degree2rad*speed_3*scale;
			delta4 = degree2rad*speed_4*scale;
			delta5 = degree2rad*speed_5*scale;
			delta6 = degree2rad*speed_6*scale;
			delta7 = degree2rad*speed_7*scale;
			delta8 = degree2rad*speed_8*scale;

      //update joint_state


      // Getting old position of choosen joint and moving it for desired delta




      pos0 = msg1.data + delta0;
      pos1 = msg2.data + delta1;
			pos2 = msg3.data + delta2;
			pos3 = msg4.data + delta3;
			pos4 = msg5.data + delta4;
			pos5 = msg6.data + delta5;
			pos6 = msg7.data + delta6;
			pos7 = msg8.data + delta7*0.001;
			pos8 = msg9.data + delta8*0.001;



      // Applying angle and translation limits
      msg1.data = std::min(std::max(pos0,-150*degree2rad),114*degree2rad);
      msg2.data = std::min(std::max(pos1,-67*degree2rad),109*degree2rad);
      msg3.data = std::min(std::max(pos2,-150*degree2rad),41*degree2rad);
      msg4.data = std::min(std::max(pos3,-92*degree2rad),110*degree2rad);
      msg5.data = std::min(std::max(pos4,-150*degree2rad),150*degree2rad);
      msg6.data = std::min(std::max(pos5,92*degree2rad),113*degree2rad);
      msg7.data = std::min(std::max(pos6,-150*degree2rad),150*degree2rad);
      msg8.data = std::min(std::max(pos7,0.0),0.03);
      msg9.data = std::min(std::max(pos8,0.0),0.03);

      //ROS_INFO_STREAM ("Z coordinate related to base xyz:" << z_coord);

      //send the joint state
			pub1.publish(msg1);
      pub2.publish(msg2);
			pub3.publish(msg3);
			pub4.publish(msg4);
			pub5.publish(msg5);
			pub6.publish(msg6);
			pub7.publish(msg7);
			pub8.publish(msg8);
			pub9.publish(msg9);
      ros::spinOnce();
      loop_rate.sleep();
  }
  return 0;
}
