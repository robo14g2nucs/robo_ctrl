#include <ros/ros.h>
#include <ras_arduino_msgs/Encoders.h>
#include <geometry_msgs/Twist.h>
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
//    double leftIrRelAngle;
//    double rightIrRelAngle;

public:

	ros::NodeHandle n_;
	ros::Subscriber encoders_subscriber_;
    ros::Subscriber irSub;
	ros::Publisher posori_publisher_;
    double alphama; // weight for exponential weighted moving average filter
    double Ffront_left;
    double Ffront_right;
    double Fback_left;
    double Fback_right;
    bool Fboolean;


    wheel_tracker_node() : curAngle(0.0), curX(1.4), curY(2.25), DEGTOM( WHEEL_RADIUS*TWOPI/TICKSPR ), alphama(0.3), Fboolean(0)
	{
		n_ = ros::NodeHandle("~");
		encoders_subscriber_ = n_.subscribe("/arduino/encoders", 1, &wheel_tracker_node::update, this);
        irSub = n_.subscribe("/ir_reader_node/cdistance", 1, &wheel_tracker_node::irCallback, this);
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
		
        if (fabs(ad) < 0.05) {
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

    void irCallback(const ir_reader::distance_readings::ConstPtr &msg){

        double leftIrRelAngle = 100;
        double rightIrRelAngle = 100;

        if (!Fboolean){
            Ffront_left = msg->front_left;
            Ffront_right = msg->front_right;
            Fback_left = msg->back_left;
            Fback_right = msg->back_right;
            Fboolean = 1;
        }

        Ffront_left = Ffront_left * (1-alphama) + msg->front_left * (alphama);
        Ffront_right = Ffront_right * (1-alphama) + msg->front_right * (alphama);
        Fback_left = Fback_left * (1-alphama) + msg->back_left * (alphama);
        Fback_right = Fback_right * (1-alphama) + msg->back_right * (alphama);

        if(msg->front_left < 25 && msg->back_left < 25){
            //compute angle to left wall
            double diff = msg->front_left - msg->back_left;
            leftIrRelAngle = - std::atan2(diff, IR_SIDE_SENSOR_DISTANCE*100);
        }
        if(msg->front_right < 25 && msg->back_right < 25){
            double diff = msg->front_right - msg->back_right;
            rightIrRelAngle = std::atan2(diff, IR_SIDE_SENSOR_DISTANCE*100);
        }

        ROS_ERROR("old odomtry angle: %f", curAngle);
        double odomRelAngle = fmod(curAngle, M_PI_4);
        double offset = curAngle - odomRelAngle;
        ROS_ERROR("relative odometry: %f", odomRelAngle);

        if(odomRelAngle < 0){
            if(leftIrRelAngle > 0){
                leftIrRelAngle -= M_PI_4;
            }
            if(rightIrRelAngle > 0){
                rightIrRelAngle -= M_PI_4;
            }
        }
        else if(odomRelAngle > 0){
            if(leftIrRelAngle < 0){
                leftIrRelAngle += M_PI_4;
            }
            if(rightIrRelAngle < 0){
                rightIrRelAngle += M_PI_4;
            }
        }
        if(fabs(odomRelAngle - leftIrRelAngle) < 0.2 && fabs(odomRelAngle - leftIrRelAngle) < fabs(odomRelAngle - rightIrRelAngle)){
            curAngle = offset + (19*odomRelAngle + leftIrRelAngle) / 20.0d;
        }
        else if(fabs(odomRelAngle - rightIrRelAngle) < 0.2 && fabs(odomRelAngle - rightIrRelAngle) < fabs(odomRelAngle - leftIrRelAngle)){
            curAngle = offset + (19*odomRelAngle + rightIrRelAngle) / 20.0d;
        }

        ROS_ERROR("angle from IR: left: %f, right: %f", leftIrRelAngle, rightIrRelAngle);
        ROS_ERROR("new odometry angle: %f", curAngle);

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

