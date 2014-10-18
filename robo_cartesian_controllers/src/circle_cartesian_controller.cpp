#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include <ras_arduino_msgs/PWM.h>
#include <geometry_msgs/Twist.h>
#include <sstream>

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "circle_cartesian_controller");
  ros::NodeHandle n;

  ros::Publisher twist = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1000);

  ros::Rate loop_rate(10);

  geometry_msgs::Twist msg;

  float pi = 3.1415926535;
  float r = 0.5; // radius
  float f = 0.1; // frequency (1 rev in 10s)
  float v = 2* pi * f * r; // linear velocity
  float w = v / r; // angular velocity

  msg.linear.x = v; // 0.31415926535 [m/s]
  msg.angular.z = w; // 0.62831853071 [rad/s]


  while (ros::ok())
  {

    twist.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}
