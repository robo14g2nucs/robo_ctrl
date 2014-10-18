#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include <ras_arduino_msgs/PWM.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <geometry_msgs/Twist.h>
#include <sstream>



std::vector<float> dist_ = std::vector<float>(2, 0);

void adcCallback(const ras_arduino_msgs::ADConverter::ConstPtr &msg)
{
    dist_[0] = (float) (msg->ch1);
    dist_[1] = (float) (msg->ch2);
    ROS_INFO("ch1: [%f]", dist_[0]);
    ROS_INFO("ch2: [%f]", dist_[1]);
}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "wall_following_controller");
  ros::NodeHandle n;
  ros::Subscriber ds_ = n.subscribe("/kobuki/adc",1, adcCallback);

  ros::Publisher twist = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1000);

  ros::Rate loop_rate(10);

  geometry_msgs::Twist msg;


  float alpha = 0.005; // P-controller gain
  float v = 0.2; // linear velocity constant
  float w = 0; // angular_vel

  while (ros::ok())
  {

    w = alpha * (dist_[1] - dist_[0]); // angular_vel = alpha*( distance_sensor1 - distance_sensor2)
    ROS_INFO("w: [%f]", w);
    msg.linear.x = v; // [m/s]
    msg.angular.z = w; // [rad/s]

    twist.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}
