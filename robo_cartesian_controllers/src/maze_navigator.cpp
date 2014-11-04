
#include "ros/ros.h"
#include <ir_reader/distance_readings.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <robo_globals.h>


#define MINDIST 9	//9 centimeters
#define STOPDIST 12	//12 centimeters

#define IR_SHORT_LIMIT 25


//Controller modes
#define STILL 0
#define LEFT_WALL_FOLLOW 1
#define RIGHT_WALL_FOLLOW 2
#define LEFT_ROTATE 3
#define RIGHT_ROTATE 4
#define STRAIGHT_FORWARD 5


static const char* MODE_NAMES[6] = {"STILL", "LEFT_WALL_FOLLOW", "RIGHT_WALL_FOLLOW",
"LEFT_ROTATE", "RIGHT_ROTATE", "STRAIGHT_FORWARD"};


class maze_navigator_node
{
private:

	int mode;
	double alpha; // P gain {left, right}	//DON'T CHANGE THIS DURING RUNTIME
	double angle;	//IMU angle
	double v; // linear velocity constant
	double w; // angular_vel
	double refAngle;


	geometry_msgs::Twist out_twist;
	ir_reader::distance_readings in_ir;

public:

	ros::NodeHandle n_;
	ros::Subscriber ir_reader_subscriber_, imu_subscriber_;
	ros::Publisher twist_publisher_;

	maze_navigator_node() : alpha(0.0275), v(0.15), w(0), mode(RIGHT_WALL_FOLLOW)
	{
		n_ = ros::NodeHandle("~");
		ir_reader_subscriber_ = n_.subscribe("/ir_reader_node/cdistance", 1, &maze_navigator_node::irCallback, this);
		imu_subscriber_ = n_.subscribe("/imu_angle", 1, &maze_navigator_node::imuAngleCallback, this);
		twist_publisher_ = n_.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1000);
	}

	void irCallback(const ir_reader::distance_readings::ConstPtr &msg)
	{
		in_ir = *msg;

		ROS_INFO("IR front_left: [%lf]", in_ir.front_left);
		ROS_INFO("IR back_left: [%lf]", in_ir.back_left);
		ROS_INFO("IR front_right: [%lf]", in_ir.front_right);
		ROS_INFO("IR back_right: [%lf]", in_ir.back_right);
		ROS_INFO("IR front_center: [%lf]", in_ir.front_center);
	}

	void imuAngleCallback(const std_msgs::Float64::ConstPtr &msg) {
		angle = msg->data;
		ROS_INFO("The current angle is %lf", angle);
	}

	void publish()
	{

		switch (mode) {

			case STILL:
				//TODO
				break;

			case LEFT_WALL_FOLLOW:
				if (in_ir.front_center<STOPDIST) {
					//Stop and determine how to rotate
					if (in_ir.front_left > IR_SHORT_LIMIT && in_ir.back_left > IR_SHORT_LIMIT) {
						mode = LEFT_ROTATE;
						refAngle = angle;
					} else if (in_ir.front_right > IR_SHORT_LIMIT && in_ir.back_right > IR_SHORT_LIMIT) {
						mode = RIGHT_ROTATE;
						refAngle = angle;
					} else {
						//TODO go back
						mode = LEFT_ROTATE;
					}
					break;
				}

				if (in_ir.front_left > IR_SHORT_LIMIT && in_ir.back_left > IR_SHORT_LIMIT) {
					mode = STRAIGHT_FORWARD;
					break;
				}

				out_twist.angular.z = alpha * (in_ir.front_left - in_ir.back_left);// [m/s]
				out_twist.linear.x = v;
				//w = alpha * (MINDIST-0.5(in_ir.front_left+in_ir.back_left) + 2*(ir[0]-ir[1]));
				break;

			case RIGHT_WALL_FOLLOW:
				if (in_ir.front_center<STOPDIST) {
					//Stop and determine how to rotate
					if (in_ir.front_left > IR_SHORT_LIMIT && in_ir.back_left > IR_SHORT_LIMIT) {
						mode = LEFT_ROTATE;
						refAngle = angle;
					} else if (in_ir.front_right > IR_SHORT_LIMIT && in_ir.back_right > IR_SHORT_LIMIT) {
						mode = RIGHT_ROTATE;
						refAngle = angle;
					} else {
						//TODO go back
						mode = RIGHT_ROTATE;
					}
					break;
				}

				if (in_ir.front_right > IR_SHORT_LIMIT && in_ir.back_right > IR_SHORT_LIMIT) {
					mode = STRAIGHT_FORWARD;
					break;
				}

				out_twist.angular.z = alpha * (in_ir.front_right - in_ir.back_right);// [m/s]
				out_twist.linear.x = v;
				//w = alpha * (MINDIST-0.5(in_ir.front_right+in_ir.back_right) + 2*(ir[2]-ir[3]));
				break;

			case LEFT_ROTATE:
				//TODO
				if (fabs(angle-refAngle) >= 90.0) {
					//Stop rotating
					ROS_INFO("first if");
					mode = RIGHT_WALL_FOLLOW;
					break;
				}

				//Only for stationary demo
//				if (in_ir.front_right < IR_SHORT_LIMIT && in_ir.back_right <
//				IR_SHORT_LIMIT && fabs(in_ir.front_right - in_ir.back_right) < 2) {
//					mode = RIGHT_WALL_FOLLOW;
//					break;
//				}

				out_twist.angular.z = 1.0;
				out_twist.linear.x = 0.0;

				break;

			case RIGHT_ROTATE:
				//TODO
				if (fabs(angle-refAngle) >= 90.0) {
					//Stop rotating
					mode = LEFT_WALL_FOLLOW;
					break;
				}

				//Only for stationary demo
//				if (in_ir.front_left < IR_SHORT_LIMIT && in_ir.back_left <
//				IR_SHORT_LIMIT && fabs(in_ir.front_left - in_ir.back_left) < 2) {
//					mode = LEFT_WALL_FOLLOW;
//					break;
//				}




				out_twist.angular.z = -1.0;
				out_twist.linear.x = 0.0;

				break;

			case STRAIGHT_FORWARD:
				//TODO

				if (in_ir.front_left < IR_SHORT_LIMIT && in_ir.back_left < IR_SHORT_LIMIT) {
					mode = LEFT_WALL_FOLLOW;
					break;
				}
				if (in_ir.front_right < IR_SHORT_LIMIT && in_ir.back_right < IR_SHORT_LIMIT) {
					mode = RIGHT_WALL_FOLLOW;
					break;
				}

				if (in_ir.front_center<STOPDIST) {
					//Stop and determine how to rotate
					if (in_ir.front_left > IR_SHORT_LIMIT && in_ir.back_left > IR_SHORT_LIMIT) {
						mode = LEFT_ROTATE;
						refAngle = angle;
					} else if (in_ir.front_right > IR_SHORT_LIMIT && in_ir.back_right > IR_SHORT_LIMIT) {
						mode = RIGHT_ROTATE;
						refAngle = angle;
					} else {
						//TODO go back
						mode = STILL;
					}
					break;
				}

				out_twist.linear.x = v;
				out_twist.angular.z = 0.0;

				break;
		}

		twist_publisher_.publish(out_twist);


		ROS_INFO("The current mode is %s", MODE_NAMES[mode]);
	}
};


int main(int argc, char **argv)
{

	ros::init(argc, argv, "maze_navigator");
	maze_navigator_node mnnode;
	ros::Rate loop_rate(CTRL_FREQ);

	while (ros::ok())
	{
		mnnode.publish();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
