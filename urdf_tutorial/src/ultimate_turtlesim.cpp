#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_Q 0x71
#define KEYCODE_A 0x61
#define KEYCODE_W 0x77
#define KEYCODE_S 0x73
#define KEYCODE_E 0x65
#define KEYCODE_D 0x64
#define KEYCODE_R 0x72
#define KEYCODE_F 0x66
#define KEYCODE_T 0x74
#define KEYCODE_G 0x67
#define KEYCODE_Y 0x79
#define KEYCODE_H 0x68
#define KEYCODE_U 0x75
#define KEYCODE_J 0x6A
#define KEYCODE_I 0x69
#define KEYCODE_K 0x6B
#define KEYCODE_O 0x6F
#define KEYCODE_L 0x6C
#define KEYCODE_SPACE 0x20
  float angular_0,angular_1,angular_2,angular_3,angular_4,angular_5,angular_6,angular_7,angular_8;
double a_scale_=0.1;




int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
    ros::NodeHandle nh_;

  ros::Publisher twist_pub_ = nh_.advertise<sensor_msgs::JointState>("turtle1/cmd_vel", 1);

  signal(SIGINT,quit);

  char c;


  sensor_msgs::JointState twist;
  twist.name.resize(9);
  twist.velocity.resize(9);

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
  ros::Rate loop_rate(30);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("[1] [2] [3] [4] [5] [6] [7] [8] [9] - Joint number.");
  puts("[Q] [W] [E] [R] [T] [Y] [U] [I] [O] - Increase Vel.");
  puts("[A] [S] [D] [F] [G] [H] [J] [K] [L] - Decrease Vel.");
  puts("Press [SPACE] to reset all velocities to 0.");

  for(;;)
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    //linear_=angular_=0;
    ROS_INFO_STREAM(c);

    switch(c)
    {
      case KEYCODE_Q:
        ROS_INFO_STREAM("JOINT 1 +VEL");
        angular_0 = angular_0 + 1.0;
        break;
      case KEYCODE_A:
        ROS_INFO_STREAM("JOINT 1 -VEL");
        angular_0 = angular_0 - 1.0;
        break;
        case KEYCODE_W:
          ROS_INFO_STREAM("JOINT 2 +VEL");
          angular_1 = angular_1 + 1.0;
          break;
        case KEYCODE_S:
          ROS_INFO_STREAM("JOINT 2 -VEL");
          angular_1 = angular_1 - 1.0;
          break;
          case KEYCODE_E:
            ROS_INFO_STREAM("JOINT 3 +VEL");
            angular_2 = angular_2 + 1.0;
            break;
          case KEYCODE_D:
            ROS_INFO_STREAM("JOINT 3 -VEL");
            angular_2 = angular_2 - 1.0;
            break;
            case KEYCODE_R:
              ROS_INFO_STREAM("JOINT 4 +VEL");
              angular_3 = angular_3 + 1.0;
              break;
            case KEYCODE_F:
              ROS_INFO_STREAM("JOINT 4 -VEL");
              angular_3 = angular_3 - 1.0;
              break;
              case KEYCODE_T:
                ROS_INFO_STREAM("JOINT 5 +VEL");
                angular_4 = angular_4 + 1.0;
                break;
              case KEYCODE_G:
                ROS_INFO_STREAM("JOINT 5 -VEL");
                angular_4 = angular_4 - 1.0;
                break;
                case KEYCODE_Y:
                  ROS_INFO_STREAM("JOINT 6 +VEL");
                  angular_5 = angular_5 + 1.0;
                  break;
                case KEYCODE_H:
                  ROS_INFO_STREAM("JOINT 6 -VEL");
                  angular_5 = angular_5 - 1.0;
                  break;
                  case KEYCODE_U:
                    ROS_INFO_STREAM("JOINT 7 +VEL");
                    angular_6 = angular_6 + 1.0;
                    break;
                  case KEYCODE_J:
                    ROS_INFO_STREAM("JOINT 7 -VEL");
                    angular_6 = angular_6 - 1.0;
                    break;
                    case KEYCODE_I:
                      ROS_INFO_STREAM("JOINT 8 +VEL");
                      angular_7 = angular_7 + 1.0;
                      break;
                    case KEYCODE_K:
                      ROS_INFO_STREAM("JOINT 8 -VEL");
                      angular_7 = angular_7 - 1.0;
                      break;
                      case KEYCODE_O:
                        ROS_INFO_STREAM("JOINT 9 +VEL");
                        angular_8 = angular_8 + 1.0;
                        break;
                      case KEYCODE_L:
                        ROS_INFO_STREAM("JOINT 9 -VEL");
                        angular_8 = angular_8 - 1.0;
                        break;
                        case KEYCODE_SPACE:
                          ROS_INFO_STREAM("EMERGENCY STOP");
                          angular_0  = 0;
                          angular_1 = 0;
                          angular_2 = 0;
                          angular_3 = 0;
                          angular_4 = 0;
                          angular_5 = 0;
                          angular_6 = 0;
                          angular_7 = 0;
                          angular_8 = 0;


                          break;
    }

    // Setting the names of joints

    twist.name[0] ="shoulder_pan_joint";
    twist.name[1] ="shoulder_pitch_joint";
    twist.name[2] ="elbow_roll_joint";
    twist.name[3] ="elbow_pitch_joint";
    twist.name[4] ="wrist_roll_joint";
    twist.name[5] ="wrist_pitch_joint";
    twist.name[6] ="gripper_roll_joint";
    twist.name[7] ="finger_joint1";
    twist.name[8] ="finger_joint2";

    //geometry_msgs::Twist twist;
    twist.velocity[0] = a_scale_*angular_0;
    twist.velocity[1] = a_scale_*angular_1;
    twist.velocity[2] = a_scale_*angular_2;
    twist.velocity[3] = a_scale_*angular_3;
    twist.velocity[4] = a_scale_*angular_4;
    twist.velocity[5] = a_scale_*angular_5;
    twist.velocity[6] = a_scale_*angular_6;
    twist.velocity[7] = a_scale_*angular_7;
    twist.velocity[8] = a_scale_*angular_8;
ROS_INFO_STREAM(twist.velocity[0]);
ROS_INFO_STREAM(twist.velocity[1]);
ROS_INFO_STREAM(twist.velocity[2]);
ROS_INFO_STREAM(twist.velocity[3]);
ROS_INFO_STREAM(twist.velocity[4]);
ROS_INFO_STREAM(twist.velocity[5]);
ROS_INFO_STREAM(twist.velocity[6]);
ROS_INFO_STREAM(twist.velocity[7]);
ROS_INFO_STREAM(twist.velocity[8]);

      twist_pub_.publish(twist);
      ros::spinOnce();
      loop_rate.sleep();

  }
  return(0);
}
