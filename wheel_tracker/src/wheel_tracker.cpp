#include <ros/ros.h>
#include <ras_arduino_msgs/Encoders.h>
#include <geometry_msgs/TwistStamped.h>
#include <robo_globals.h>
#include <ir_reader/distance_readings.h>
#include <cmath>

class wheel_tracker_node
{

private:

	double curAngle;
	double curX, curY;
	double denc1, denc2;
	const double DEGTOM; 
//	double leftIrRelAngle;
//	double rightIrRelAngle;

public:

	ros::NodeHandle n_;
	ros::Subscriber encoders_subscriber_;
	ros::Subscriber irSub;
	ros::Publisher posori_publisher_;
 
	wheel_tracker_node() : curAngle(0.0), curX(1.4), curY(2.25), DEGTOM( WHEEL_RADIUS*TWOPI/TICKSPR )
	{
		n_ = ros::NodeHandle("~");
		encoders_subscriber_ = n_.subscribe("/arduino/encoders", 1, &wheel_tracker_node::update, this);
		irSub = n_.subscribe("/ir_reader_node/cdistance", 1, &wheel_tracker_node::irCallback, this);
		posori_publisher_ = n_.advertise<geometry_msgs::TwistStamped>("/posori/Twist", 1);
	}

	~wheel_tracker_node() {}

	void update(const ras_arduino_msgs::Encoders::ConstPtr &msg)
	{
		//Read the values
		denc1 = (double)msg->delta_encoder1;
		denc2 = (double)msg->delta_encoder2;
		
		//Compute new angle and position
		double ad = (denc1-denc2)*DEGTOM/WHEEL_BASE;
		double ld = (denc1+denc2)*DEGTOM/2.0;
		
		if (fabs(ad) <= 0.0) {
		//if (true) {
			curAngle += ad;
			curX -= (ld*cos(curAngle));
			curY -= (ld*sin(curAngle));
		} else {
			double r = ld/ad;
			curX -= (r*(sin(curAngle)-sin(curAngle+ad)));
			curY -= (r*(cos(curAngle+ad)-cos(curAngle)));
			curAngle += ad;	
		}
		
		//ROS_ERROR("1curAngle is %lf", curAngle);
		
//		ROS_ERROR("curAngle is %lf, ad is %lf, ld is %lf, DEGTOM is %lf, denc1 is %lf, denc2 is %lf",
//		curAngle, ad, ld, DEGTOM, denc1, denc2);
		
		//curAngle = fmod(curAngle + TWOPI, TWOPI);
		
		//ROS_ERROR("2curAngle is %lf", curAngle);
		
		//Publish the new orientation and position
		geometry_msgs::TwistStamped pomsg;
		pomsg.header.stamp = ros::Time::now();
		pomsg.twist.linear.x = curX;
		pomsg.twist.linear.y = curY;
		pomsg.twist.linear.z = 0.0;
		pomsg.twist.angular.x = 0.0;
		pomsg.twist.angular.y = 0.0;
		pomsg.twist.angular.z = curAngle;
		posori_publisher_.publish(pomsg);
		
		//ROS_ERROR("curAngle is %lf", curAngle);
	}

	void irCallback(const ir_reader::distance_readings::ConstPtr &msg){
/*
		double leftIrRelAngle = 100;
		double rightIrRelAngle = 100;

		if(msg->front_left < 25 && msg->back_left < 25){
			//compute angle to left wall
			double diff = msg->front_left - msg->back_left;
			leftIrRelAngle = M_PI_2 - std::atan2(diff, IR_SIDE_SENSOR_DISTANCE*100);
		}
		if(msg->front_right < 25 && msg->back_right < 25){
			double diff = msg->front_right - msg->back_right;
			rightIrRelAngle = std::atan2(diff, IR_SIDE_SENSOR_DISTANCE*100);
		}

		ROS_INFO("old odomtry angle: %f", curAngle);
		double odomRelAngle = fmod(curAngle, M_PI_2);
		double offset = curAngle - odomRelAngle;
		ROS_INFO("relative odometry: %f", odomRelAngle);
			if(fabs(odomRelAngle - leftIrRelAngle) < 0.2 && fabs(odomRelAngle - leftIrRelAngle) < fabs(odomRelAngle - rightIrRelAngle)){
			curAngle = offset + (9*odomRelAngle + leftIrRelAngle) / 10.0;
		}
			else if(fabs(odomRelAngle - rightIrRelAngle) < 0.2 && fabs(odomRelAngle - rightIrRelAngle) < fabs(odomRelAngle - leftIrRelAngle)){
			curAngle = offset + (9*odomRelAngle + rightIrRelAngle) / 10.0;
		}

		ROS_INFO("angle from IR: left: %f, right: %f", leftIrRelAngle, rightIrRelAngle);
		ROS_INFO("new odometry angle: %f", curAngle);
*/
	}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "wheel_tracker_node");
	wheel_tracker_node wtn;
	ros::Rate loop_rate(CTRL_FREQ*5);
	while(wtn.n_.ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

