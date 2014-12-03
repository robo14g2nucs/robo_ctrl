#include "ros/ros.h"
#include <ras_arduino_msgs/Encoders.h>
#include <ir_reader/distance_readings.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <robo_globals.h>
#include <cmath>
#include <mapper/WallInFront.h>

#define MINDIST 13	//13 centimeters
#define STOPDIST 17	//17 centimeters

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
	double alpha, alpha1; // P gain {left, right}	//DON'T CHANGE THIS DURING RUNTIME
    double alpha_align;
	double angle;	//angular orientation
    double targetAngle;	//The angle to aim for when rotating
	double v; // linear velocity constant
	double w; // angular_vel
	double delta_enc[2];	//Read delta encoder values
	double irdiff; //Difference between IR_sensors
	double irav; //IR_sensors average
    ros::ServiceClient client; //client for map service
    int updateCounter;

	geometry_msgs::Twist out_twist;
	geometry_msgs::Twist curPosOri;	//Current position and orientation according to the odometry
	ir_reader::distance_readings in_ir;
	//std_msgs::String out_mode;
	std_msgs::Int16 out_mode;
	std_msgs::Int16 out_previous_mode;

public:

    bool hasIR;

	ros::NodeHandle n_;
	ros::Subscriber ir_reader_subscriber_, imu_subscriber_, encoders_subscriber_, odometry_subscriber_;
	ros::Publisher twist_publisher_, mode_publisher_, prev_mode_publisher_, node_creation_publisher_;
                                //alpha(0.0177) for wall-following without reference distance
                                //alpha(0.0245), alpha1(2.53586),
	maze_navigator_node() : alpha(0.0175), alpha1(0.01), alpha_align(0.0195), v(.17), w(0), mode(STRAIGHT_FORWARD), prevmode(STRAIGHT_FORWARD)
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
			node_creation_publisher_ = n_.advertise<geometry_msgs::Point>("/node_creation", 100);

            //service client
            client = n_.serviceClient<mapper::WallInFront>("wall_in_front");
            updateCounter = 0;
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
		//angle = msg->data;
	}

    void odometryCallback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
		curPosOri = msg->twist;
        angle = curPosOri.angular.z;
	}
	
	//From still, decide what the next mode should be
	int decideNextMode() {
		out_twist.linear.x = 0.0;
		out_twist.angular.z = 0.0;			
		if (delta_enc[0] != 0 || delta_enc[1] != 0) {
			//Do nothing
			return STILL;
		}
		
		//Rohit: check the previous mode and accordingly decide the next mode   
		if (prevmode==RIGHT_WALL_ALIGN){ 	
			prevmode=mode;
			mode = RIGHT_WALL_FOLLOW;
		}
		
		else if (prevmode==LEFT_WALL_ALIGN){	
			prevmode=mode;
			mode = LEFT_WALL_FOLLOW;						
		}

		else if(prevmode==LEFT_ROTATE){
			prevmode=mode;
			mode = RIGHT_WALL_ALIGN;
		}

		else if(prevmode==RIGHT_ROTATE){
			prevmode=mode;
			mode = LEFT_WALL_ALIGN;
		}
		
		else if (in_ir.front_left > IR_SHORT_LIMIT || in_ir.back_left > IR_SHORT_LIMIT) {
			prevmode=mode;
			mode = LEFT_ROTATE;
			targetAngle = floor((angle/(PI/2.0))+0.5) * PI/2.0 + PI/2.0;
	//		ROS_INFO("target Angle = %lf", (targetAngle*360)/TWOPI);
	//	ROS_INFO("Left rotate:First if");
		} else if (in_ir.front_right > IR_SHORT_LIMIT || in_ir.back_right > IR_SHORT_LIMIT) {
			prevmode=mode;
			mode = RIGHT_ROTATE;
			targetAngle = floor((angle/(PI/2.0))+0.5) * PI/2.0 - PI/2.0;
	//		ROS_INFO("target Angle = %lf", (targetAngle*360)/TWOPI);
		} else {
			//TODO go back
			prevmode=mode;
			mode = LEFT_ROTATE;
			targetAngle = floor((angle/(PI/2.0))+0.5) * PI/2.0 + PI/2.0;
//			ROS_INFO("target Angle = %lf", (targetAngle*360)/TWOPI);
	//		ROS_INFO("Left rotate:Second if");
		}

		return mode;
	}
	
	void publish()
	{
		//ROS_INFO("The current angle is %lf", (angle*180)/M_PI);	//degrees
		ROS_INFO("The current angle is %lf", angle);	//radians
		ROS_INFO("The target angle is %lf", targetAngle);	//radians
//		ROS_INFO("IR front_left: [%lf]", in_ir.front_left);
//		ROS_INFO("IR back_left: [%lf]", in_ir.back_left);
//		ROS_INFO("IR front_right: [%lf]", in_ir.front_right);
//		ROS_INFO("IR back_right: [%lf]", in_ir.back_right);
//		ROS_INFO("IR front_center: [%lf]", in_ir.front_center);

        //get wall in front from map
        bool wallInFront = false;
        //divide frequency
        if(updateCounter >= 5){
            updateCounter = 0;
            mapper::WallInFront srv;
            srv.request.position.x = curPosOri.linear.x;
            srv.request.position.y = curPosOri.linear.y;
            srv.request.angle = angle;
            if(client.call(srv)){
                wallInFront = srv.response.wallInFront;
            }
        }
        else{
            updateCounter++;
        }
		switch (mode) {
					
			case STILL:
				//Rohit: When in still, decide upon the next mode based on the previous mode
				decideNextMode();
				break;


			case LEFT_WALL_FOLLOW:
                if (in_ir.front_center<STOPDIST || wallInFront) {
					prevmode=mode;
					mode = STILL;
					//Stop and determine how to rotate
					ROS_INFO("IR front_center inside switch : [%lf]", in_ir.front_center);
					
					//Advertise node creation request
					geometry_msgs::Point p;
					p.x = curPosOri.linear.x;
					p.y = curPosOri.linear.y;
					node_creation_publisher_.publish(p);
					
					break;
				}

				if (in_ir.front_left > IR_SHORT_LIMIT || in_ir.back_left > IR_SHORT_LIMIT) {
					prevmode=mode;
					mode = STRAIGHT_FORWARD;
					break;
				}
				
				//prevmode == LEFT_WALL_FOLLOW;
				
				irav = 0.5*(in_ir.front_left + in_ir.back_left);
				irdiff = in_ir.front_left - in_ir.back_left;
				
				out_twist.angular.z = alpha * (irdiff);// [m/s]
				if(fabs(MINDIST-irav) > 1)		
					out_twist.angular.z += alpha1 * (irav - MINDIST);
					
				//out_twist.angular.z = alpha * (irav - MINDIST + (alpha1 * irdiff)); // [m/s]
				out_twist.linear.x = v;
				
				//w = alpha * (MINDIST-0.5(in_ir.front_left+in_ir.back_left) + 2*(ir[0]-ir[1]));
				break;


			case RIGHT_WALL_FOLLOW:
                if (in_ir.front_center<STOPDIST || wallInFront) {
					prevmode=mode;
					mode = STILL;
					ROS_INFO("IR front_center inside switch : [%lf]", in_ir.front_center);
					
					//Advertise node creation request
					geometry_msgs::Point p;
					p.x = curPosOri.linear.x;
					p.y = curPosOri.linear.y;
					node_creation_publisher_.publish(p);
					
					break;
				}
				if (in_ir.front_right > IR_SHORT_LIMIT || in_ir.back_right > IR_SHORT_LIMIT) {
					prevmode=mode;
					mode = STRAIGHT_FORWARD;
					break;
				}		
			  
			  
				irav = 0.5*(in_ir.front_right + in_ir.back_right);
				irdiff = in_ir.front_right - in_ir.back_right;
				
				out_twist.angular.z = - alpha * (irdiff);// [m/s]
				if(fabs(MINDIST-irav) > 1)		
					out_twist.angular.z += alpha1 * (-irav + MINDIST);
				//out_twist.angular.z = alpha * ( MINDIST - (irav)  - (alpha1 * irdiff)); // [m/s]
				out_twist.linear.x = v;
				//w = alpha * (MINDIST-0.5(in_ir.front_right+in_ir.back_right) + 2*(ir[2]-ir[3]));
				break;


			case LEFT_ROTATE:
	//			ROS_INFO("angle & targetAngle are %lf & %lf.", angle, targetAngle);
				//if (fabs(angle - refAngle) >=(angcomp+M_PI_2-angle)){
				if (angle >= (targetAngle-0.2)) {
					//ROS_INFO("Difference rotation = %lf", (1/M_PI)*(angcomp+M_PI_2)*180 );
					//Align to the right wall
					prevmode = mode;
					mode = STILL;
				}	
				out_twist.angular.z = 0.8;	
	//			out_twist.angular.z = ((M_PI_2-fabs(angle-refAngle))*1.5) + 0.6;			
				out_twist.linear.x = 0.0;
				break;


			case RIGHT_ROTATE:
	//			ROS_INFO("angle & targetAngle are %lf & %lf.", angle, targetAngle);
				//if (fabs(angle - refAngle) >=(angcomp+M_PI_2-angle)){
				//if (fabs(angle-targetAngle) < 0.1) {
				if (angle <= (targetAngle+0.2)) {
					//ROS_INFO("Difference rotation = %lf", (1/M_PI)*(angcomp+M_PI_2)*180 );
					//Align to the left wall
					prevmode = mode;
					mode = STILL;
				}
//					out_twist.angular.z = -((M_PI_2-fabs(angle-refAngle))*1.5) - 0.6;
				out_twist.angular.z = -0.8;//-(M_PI_2-fabs(angle-refAngle))*.8;
				out_twist.linear.x = 0.0;
				break;


			case STRAIGHT_FORWARD:
				//TODO
                if (in_ir.front_center < STOPDIST || wallInFront) {
					prevmode=mode;
					mode = STILL;	//Stop
					
					//Advertise node creation request
					geometry_msgs::Point p;
					p.x = curPosOri.linear.x;
					p.y = curPosOri.linear.y;
					node_creation_publisher_.publish(p);
					
				} else if (in_ir.front_left < IR_SHORT_LIMIT && in_ir.back_left < IR_SHORT_LIMIT) {
					prevmode=mode;
					mode = LEFT_WALL_FOLLOW;
				} else if (in_ir.front_right < IR_SHORT_LIMIT && in_ir.back_right < IR_SHORT_LIMIT) {
					prevmode=mode;
					mode = RIGHT_WALL_FOLLOW;
				} else {
					//Keep Straight
					out_twist.linear.x = .15;
					out_twist.angular.z = 0.0;
				}
				break;

				
			case RIGHT_WALL_ALIGN:
				if (in_ir.front_right > IR_SHORT_LIMIT || in_ir.back_right > IR_SHORT_LIMIT) {
					prevmode=mode;
					mode = STRAIGHT_FORWARD;
					break;
				}	
				if (fabs(in_ir.front_right - in_ir.back_right) > 3.5) {
					out_twist.angular.z = -alpha_align*3* (in_ir.front_right - in_ir.back_right);
				} else {
						prevmode = mode;
						mode = STILL;
				}
				break;

				
			case LEFT_WALL_ALIGN:
				if (in_ir.front_left > IR_SHORT_LIMIT || in_ir.back_left > IR_SHORT_LIMIT) {
					prevmode=mode;
					mode = STRAIGHT_FORWARD;
					break;
				}
				if (fabs(in_ir.front_left - in_ir.back_left) > 3.5) {
					out_twist.angular.z = alpha_align*3* (in_ir.front_left - in_ir.back_left);
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
		ROS_INFO("the twist is = %lf", out_twist.angular.z);
		ROS_INFO("The current mode is %s", MODE_NAMES[mode]);
		ROS_INFO("The previous mode is %s", MODE_NAMES[prevmode]);
	}
};


int main(int argc, char **argv)
{

	ros::init(argc, argv, "maze_navigator");
	maze_navigator_node mnnode;
	ros::Rate loop_rate(CTRL_FREQ*5);
	
	//for (int i = 0; i < 20; ++i) {
	//	loop_rate.sleep();
	//}

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
