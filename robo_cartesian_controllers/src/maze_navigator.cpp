#include "ros/ros.h"
#include <ras_arduino_msgs/Encoders.h>
#include <ir_reader/distance_readings.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <robo_globals.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <cmath>

#define MINDIST 9	//9 centimeters
#define STOPDIST 18	//12 centimeters

#define IR_SHORT_LIMIT 25


//Controller modes
#define STILL 0
#define LEFT_WALL_FOLLOW 1
#define RIGHT_WALL_FOLLOW 2
#define LEFT_ROTATE 3
#define RIGHT_ROTATE 4
#define STRAIGHT_FORWARD 5
#define RIGHT_WALL_ALIGN 6
#define LEFT_WALL_ALIGN 7
#define CONFUSED 8


static const char* MODE_NAMES[9] = {"STILL", "LEFT_WALL_FOLLOW", "RIGHT_WALL_FOLLOW",
"LEFT_ROTATE", "RIGHT_ROTATE", "STRAIGHT_FORWARD", "RIGHT_WALL_ALIGN", "LEFT_WALL_ALIGN", "CONFUSED"};


class maze_navigator_node
{
private:

	int mode;
	int prevmode; // Rohit: to know the previous mode
	double alpha; // P gain {left, right}	//DON'T CHANGE THIS DURING RUNTIME
	double alpha_align;
	double angle;	//IMU angle
	double v; // linear velocity constant
	double w; // angular_vel
	double refAngle;
	double delta_enc[2];


	geometry_msgs::Twist out_twist;
	ir_reader::distance_readings in_ir;
	//std_msgs::String out_mode;
		std_msgs::Int16 out_mode;
		std_msgs::Int16 out_previous_mode;

public:

	bool hasIR;

	ros::NodeHandle n_;
    ros::Subscriber ir_reader_subscriber_, imu_subscriber_, encoders_subscriber_, odometry_subscriber_;
	ros::Publisher twist_publisher_, mode_publisher_, prev_mode_publisher_;

	maze_navigator_node() : alpha(0.0175), alpha_align(0.0195), v(0.1), w(0), mode(STRAIGHT_FORWARD), prevmode(STRAIGHT_FORWARD)
	{
		hasIR = false;
		n_ = ros::NodeHandle("~");
		ir_reader_subscriber_ = n_.subscribe("/ir_reader_node/cdistance", 1, &maze_navigator_node::irCallback, this);
		encoders_subscriber_ = n_.subscribe("/arduino/encoders", 1, &maze_navigator_node::encodersCallback, this);
		imu_subscriber_ = n_.subscribe("/imu_angle", 1, &maze_navigator_node::imuAngleCallback, this);
        odometry_subscriber_ = n_.subscribe("/posori/Twist", 1, &maze_navigator_node::odometryCallback, this);
		twist_publisher_ = n_.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1000);
	//Rohit: publish mode
	//	mode_publisher_ = n_.advertise<std_msgs::String>("/maze_navigator/mode", 1000);
			mode_publisher_ = n_.advertise<std_msgs::Int16>("/maze_navigator/mode", 1000);
			prev_mode_publisher_ = n_.advertise<std_msgs::Int16>("/maze_navigator/prevmode", 1000);
	}

	void encodersCallback(const ras_arduino_msgs::Encoders::ConstPtr &msg)
	{
		delta_enc[0] = msg->delta_encoder1;
		delta_enc[1] = msg->delta_encoder2;
	}

	void irCallback(const ir_reader::distance_readings::ConstPtr &msg)
	{
		in_ir = *msg;
		hasIR = true;
	}

	void imuAngleCallback(const std_msgs::Float64::ConstPtr &msg) {
//		angle = msg->data;
		
	}

    void odometryCallback(const geometry_msgs::Twist::ConstPtr &msg) {
        angle = msg->angular.z;
    }

	void publish()
	{
		ROS_INFO("The current angle is %lf", angle);
		ROS_INFO("The reference angle is %lf", refAngle);
		ROS_INFO("IR front_left: [%lf]", in_ir.front_left);
		ROS_INFO("IR back_left: [%lf]", in_ir.back_left);
		ROS_INFO("IR front_right: [%lf]", in_ir.front_right);
		ROS_INFO("IR back_right: [%lf]", in_ir.back_right);
		ROS_INFO("IR front_center: [%lf]", in_ir.front_center);
		switch (mode) {
					
			case STILL:
//Rohit: When in still, decide upon the next mode based on the previous mode
				
								
				out_twist.linear.x = 0.0;
				out_twist.angular.z = 0.0;			
				if (delta_enc[0] != 0 || delta_enc[1] != 0) {
				}

				//Rohit: check the previous mode and accordingly decide the next mode   
				else if (prevmode==RIGHT_WALL_ALIGN){ 
					
		//			if (fabs(in_ir.front_right - in_ir.back_right) < 2){			
						prevmode=mode;
						mode = RIGHT_WALL_FOLLOW;	
						
			//		}
			//	else{
			//			prevmode=mode;
			//			mode = RIGHT_WALL_ALIGN;
			//		}								
					break;
				}
				
				else if (prevmode==LEFT_WALL_ALIGN){
			//		if (fabs(in_ir.front_left - in_ir.back_left) < 2){			
						prevmode=mode;
						mode = LEFT_WALL_FOLLOW;	
			//		}
			//		else{
			//			prevmode=mode;
			//			mode = LEFT_WALL_ALIGN;
			//		}								
					break;
				}

				else if(prevmode==LEFT_ROTATE){
					mode = RIGHT_WALL_ALIGN;
				}

				else if(prevmode==RIGHT_ROTATE){
					mode = LEFT_WALL_ALIGN;
				}
				
				else if (in_ir.front_left > IR_SHORT_LIMIT || in_ir.back_left > IR_SHORT_LIMIT) {
					prevmode=mode;
					mode = LEFT_ROTATE;
					refAngle = angle;
					ROS_INFO("Left rotate:First if");
				} else if (in_ir.front_right > IR_SHORT_LIMIT || in_ir.back_right > IR_SHORT_LIMIT) {
					prevmode=mode;
					mode = RIGHT_ROTATE;
					refAngle = angle;
				} else {
					//TODO go back
					prevmode=mode;
					mode = LEFT_ROTATE;
					refAngle = angle;
					ROS_INFO("Left rotate:Second if");
				}
				break;

			case LEFT_WALL_FOLLOW:
				if (in_ir.front_center<STOPDIST) {
					prevmode=mode;
					mode = STILL;
					//Stop and determine how to rotate
					ROS_INFO("IR front_center inside switch : [%lf]", in_ir.front_center);
					break;
				}

				if (in_ir.front_left > IR_SHORT_LIMIT || in_ir.back_left > IR_SHORT_LIMIT) {
					prevmode=mode;
					mode = STRAIGHT_FORWARD;
					break;
				}
				
				//Rohit: allign first and then give linear velocity
				
//				if (prevmode == STILL){
//				while ((in_ir.front_left - in_ir.back_left) > 2){
//				out_twist.angular.z = alpha * (in_ir.front_left - in_ir.back_left);
//				}
				
//				while ((in_ir.back_left - in_ir.front_left) > 2){
//				out_twist.angular.z = -alpha * (in_ir.back_left - in_ir.front_left);
//				}
//				}
				prevmode == LEFT_WALL_FOLLOW;
				out_twist.angular.z = alpha * (in_ir.front_left - in_ir.back_left);// [m/s]
				out_twist.linear.x = v;
				
				//w = alpha * (MINDIST-0.5(in_ir.front_left+in_ir.back_left) + 2*(ir[0]-ir[1]));
				break;

			case RIGHT_WALL_FOLLOW:
				if (in_ir.front_center<STOPDIST) {
					prevmode=mode;
					mode = STILL;
					ROS_INFO("IR front_center inside switch : [%lf]", in_ir.front_center);
					break;
				}
				if (in_ir.front_right > IR_SHORT_LIMIT || in_ir.back_right > IR_SHORT_LIMIT) {
					prevmode=mode;
					mode = STRAIGHT_FORWARD;
					break;
				}		
				//Rohit: added the negative sign here
				out_twist.angular.z = -alpha * (in_ir.front_right - in_ir.back_right);// [m/s]
				out_twist.linear.x = v;
				//w = alpha * (MINDIST-0.5(in_ir.front_right+in_ir.back_right) + 2*(ir[2]-ir[3]));
				break;

			case LEFT_ROTATE:
				//TODO
			//Rohit: kept angle smaller since it takes a while to judge whether it has turned 
            if (fabs(angle-refAngle) >= 1.3){ 

					//Align to the right wall
					prevmode = mode;
					mode = STILL;
					}				
				

				//Only for stationary demo
//			if (in_ir.front_right < IR_SHORT_LIMIT && in_ir.back_right <
//			IR_SHORT_LIMIT && fabs(in_ir.front_right - in_ir.back_right) < 2) {
//				mode = RIGHT_WALL_FOLLOW;
//				break;
//			}

//			out_twist.angular.z = 1.3;
				out_twist.angular.z = (M_PI_2-fabs(angle-refAngle))*0.7;
				out_twist.linear.x = 0.0;

				break;

			case RIGHT_ROTATE:
				//TODO
//				if (in_ir.front_left < IR_SHORT_LIMIT && in_ir.back_left <
	//			IR_SHORT_LIMIT && fabs(in_ir.front_left - in_ir.back_left) < 2)
                if (fabs(angle-refAngle) >= 1.3){
					//Align to the left wall
					prevmode = mode;
					mode = STILL;				
				}

				//Only for stationary demo
//			if (in_ir.front_left < IR_SHORT_LIMIT && in_ir.back_left <
//			IR_SHORT_LIMIT && fabs(in_ir.front_left - in_ir.back_left) < 2) {
//				mode = LEFT_WALL_FOLLOW;
//				break;
//			}
				//out_twist.angular.z = -1.3;
				//out_twist.angular.z = -0.7;
				out_twist.angular.z = -(M_PI_2-fabs(angle-refAngle))*0.7;
				out_twist.linear.x = 0.0;
				break;

			case STRAIGHT_FORWARD:
				//TODO
				if (in_ir.front_center < STOPDIST) {
					prevmode=mode;
					mode = STILL;	//Stop
				} else if (in_ir.front_left < IR_SHORT_LIMIT && in_ir.back_left < IR_SHORT_LIMIT) {
					prevmode=mode;
					mode = LEFT_WALL_FOLLOW;
				} else if (in_ir.front_right < IR_SHORT_LIMIT && in_ir.back_right < IR_SHORT_LIMIT) {
					prevmode=mode;
					mode = RIGHT_WALL_FOLLOW;
				} else {
					//Keep Straight
					out_twist.linear.x = .08;
					out_twist.angular.z = 0.0;
				}
				break;
				
				case RIGHT_WALL_ALIGN:
				
					if (fabs(in_ir.front_right - in_ir.back_right) > 2) {
						out_twist.angular.z = -alpha_align*2.5* (in_ir.front_right - in_ir.back_right);
					} else {
							prevmode = mode;
							mode = STILL;
					}
				break;
				case LEFT_WALL_ALIGN:
				
					if (fabs(in_ir.front_left - in_ir.back_left) > 2) {
						out_twist.angular.z = alpha_align*2.5* (in_ir.front_left - in_ir.back_left);
					} else {
							prevmode = mode;
							mode = STILL;
					}	
					break;
		}
		out_mode.data=mode;
		out_previous_mode.data=prevmode;
		twist_publisher_.publish(out_twist);
		mode_publisher_.publish(out_mode);
		prev_mode_publisher_.publish(out_previous_mode);
		ROS_INFO("The current mode is %s", MODE_NAMES[mode]);
		ROS_INFO("The previous mode is %s", MODE_NAMES[prevmode]);
	}
};


int main(int argc, char **argv)
{

	ros::init(argc, argv, "maze_navigator");
	maze_navigator_node mnnode;
	ros::Rate loop_rate(CTRL_FREQ);
	
	for (int i = 0; i < 20; ++i) {
		loop_rate.sleep();
	}

	while (ros::ok())
	{
		if (mnnode.hasIR) {
			mnnode.publish();
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
