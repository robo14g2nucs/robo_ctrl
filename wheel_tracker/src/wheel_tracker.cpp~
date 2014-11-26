#include <ros/ros.h>
#include <ras_arduino_msgs/Encoders.h>
#include <geometry_msgs/Twist.h>
#include <robo_globals.h>

class wheel_tracker_node
{

private:

	double curAngle;
	double curX, curY;
	double denc1, denc2;
	const double DEGTOM; 

public:

	ros::NodeHandle n_;
	ros::Subscriber encoders_subscriber_;
	ros::Publisher posori_publisher_;

	wheel_tracker_node() : curAngle(0.0), curX(1.4), curY(2.25), DEGTOM( WHEEL_RADIUS*TWOPI/TICKSPR )
	{
		n_ = ros::NodeHandle("~");
		encoders_subscriber_ = n_.subscribe("/arduino/encoders", 1, &wheel_tracker_node::update, this);
		posori_publisher_ = n_.advertise<geometry_msgs::Twist>("/posori/Twist", 1);
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
		
		if (fabs(ad) < 0.15) {
			curAngle += ad;
			curX -= (ld*cos(curAngle));
			curY -= (ld*sin(curAngle));
		} else {
			double r = ld/ad;
			curX -= (r*(sin(curAngle)-sin(curAngle+ad)));
			curY -= (r*(cos(curAngle+ad)-cos(curAngle)));
			curAngle += ad;
		}	
		
		//Publish the new orientation and position
		geometry_msgs::Twist pomsg;
		pomsg.linear.x = curX;
		pomsg.linear.y = curY;
		pomsg.linear.z = 0.0;
		pomsg.angular.x = 0.0;
		pomsg.angular.y = 0.0;
		pomsg.angular.z = curAngle;
		posori_publisher_.publish(pomsg);
	}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "wheel_tracker_node");
	wheel_tracker_node wtn;
	ros::Rate loop_rate(CTRL_FREQ);
	while(wtn.n_.ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

