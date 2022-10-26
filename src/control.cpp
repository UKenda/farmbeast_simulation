#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"


geometry_msgs::Twist twist_msg;

void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  twist_msg=*msg;
  /*
  std::cout<<"Twist msg:"<<std::endl;
  std::cout<<"  Linear:"<<std::endl;
  std::cout<<"    "<<msg->linear.x<<std::endl;
  std::cout<<"    "<<msg->linear.y<<std::endl;
  std::cout<<"    "<<msg->linear.z<<std::endl;
  std::cout<<"  Angular:"<<std::endl;
  std::cout<<"    "<<msg->angular.x<<std::endl;
  std::cout<<"    "<<msg->angular.y<<std::endl;
  std::cout<<"    "<<msg->angular.z<<std::endl;
  */
}

int main(int argc, char **argv)
{



  ros::init(argc, argv, "cmd_vel_control");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("cmd_vel", 1000, cmd_velCallback);
  ros::Publisher front_left_wheel_pub = n.advertise<std_msgs::Float64>("farmbeast/front/left/wheel/command", 1000);
  ros::Publisher front_right_wheel_pub = n.advertise<std_msgs::Float64>("farmbeast/front/right/wheel/command", 1000);
  ros::Publisher back_left_wheel_pub = n.advertise<std_msgs::Float64>("farmbeast/back/left/wheel/command", 1000);
  ros::Publisher back_right_wheel_pub = n.advertise<std_msgs::Float64>("farmbeast/back/right/wheel/command", 1000);
  ros::Publisher front_left_z_axis_pub = n.advertise<std_msgs::Float64>("farmbeast/front/left/z_axis/command", 1000);
  ros::Publisher front_right_z_axis_pub = n.advertise<std_msgs::Float64>("farmbeast/front/right/z_axis/command", 1000);
  ros::Publisher back_left_z_axis_pub = n.advertise<std_msgs::Float64>("farmbeast/back/left/z_axis/command", 1000);
  ros::Publisher back_right_z_axis_pub = n.advertise<std_msgs::Float64>("farmbeast/back/right/z_axis/command", 1000);

  ros::Rate r(10);

  while(ros::ok()){
    std_msgs::Float64 speed, speed_left, speed_right;
    std_msgs::Float64 turn_front, turn_back;
    speed.data=(twist_msg.linear.x)*10; 
    turn_front.data=twist_msg.angular.z*0.5;
    turn_back.data=-twist_msg.angular.z*0.5;

    if(twist_msg.linear.x == 0 && twist_msg.angular.z != 0){

      turn_front.data=0.5;
      turn_back.data=-0.5;
      speed_left.data = -twist_msg.angular.z*5;
      speed_right.data = twist_msg.angular.z*5;
      front_left_z_axis_pub.publish(turn_back);
      front_right_z_axis_pub.publish(turn_front);
      back_left_z_axis_pub.publish(turn_front);
      back_right_z_axis_pub.publish(turn_back);
      front_left_wheel_pub.publish(speed_left);
      front_right_wheel_pub.publish(speed_right);
      back_left_wheel_pub.publish(speed_left);
      back_right_wheel_pub.publish(speed_right);
    }
    else{
      front_left_z_axis_pub.publish(turn_front);
      front_right_z_axis_pub.publish(turn_front);
      back_left_z_axis_pub.publish(turn_back);
      back_right_z_axis_pub.publish(turn_back);

      front_left_wheel_pub.publish(speed);
      front_right_wheel_pub.publish(speed);
      back_left_wheel_pub.publish(speed);
      back_right_wheel_pub.publish(speed);
    }
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}