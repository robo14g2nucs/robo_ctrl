#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <ras_arduino_msgs/ADConverter.h>

#define SHORT_RANGE 0
#define LONG_RANGE 1
#define TYPE LONG_RANGE
//#define TYPE SHORT_RANGE

class ir_verifier_node
{

private:

	double srcoeffs[8];
	double lrcoeffs[8];

	//Exponential terms
//	const	long double sa, sb, sc, sd;
//	const long double la, lb, lc, ld;

public:

	ros::NodeHandle n_;
	ros::Subscriber distance_subscriber_;
	ros::Publisher cdistance_publisher_;

	ir_verifier_node()
//	sa(2963.0), sb(-0.047), sc(29.98), sd(-0.004048),
//	//sa(37770), sb(-0.07152), sc(36.71), sd(-0.004758),
//	la(249.7), lb(-0.01656), lc(31.15), ld(-0.002155)
	{
		n_ = ros::NodeHandle("~");
		srcoeffs[0] = 83.14;
		srcoeffs[1] = -0.9919;
		srcoeffs[2] = 0.005643;
		srcoeffs[3] = -1.461e-05;
		srcoeffs[4] = 6.107e-09;
		srcoeffs[5] = 4.797e-11;
		srcoeffs[6] = -9.312e-14;
		srcoeffs[7] = 5.219e-17;

		//		lrcoeffs[0] = 139.1;
		//		lrcoeffs[1] = -31.02;
		//		lrcoeffs[2] = 45.47;
		//		lrcoeffs[3] = -62;
		//		lrcoeffs[4] = -12.81;
		//		lrcoeffs[5] = 31.07;
		//		lrcoeffs[6] = 5.61;
		//		lrcoeffs[7] = -6.913;

		lrcoeffs[0] = 302.3;
		lrcoeffs[1] = -5.234;
		lrcoeffs[2] = 0.04667;
		lrcoeffs[3] = -0.0002424;
		lrcoeffs[4] = 7.581e-07;
		lrcoeffs[5] = -1.403e-09;
		lrcoeffs[6] = 1.412e-12;
		lrcoeffs[7] = -5.945e-16;
	}

	~ir_verifier_node() {}

	void init()
	{
		distance_subscriber_ = n_.subscribe("/arduino/adc", 1, &ir_verifier_node::topicCallbackDistance, this);
		cdistance_publisher_ = n_.advertise<std_msgs::Float64>("cdistance", 1);
	}

	void topicCallbackDistance(const ras_arduino_msgs::ADConverter::ConstPtr &msg)
	{
		unsigned int adc1 = msg->ch1;


		//ROS_INFO("adc1 is %u\n", adc1);

//		long double res = 0.0;
//		//Evaluate exponetial terms
//		switch (TYPE) {
//			case SHORT_RANGE:
//				res = sa*exp(sb*adc1) + sc*exp(sd*adc1);
//				break;
//			case LONG_RANGE:
//				res = la*exp(lb*adc1) + lc*exp(ld*adc1);
//				break;
//		}

		//Evaluate polynomial
		double xpow = 1.0;
		double res = 0.0;
		switch (TYPE) {
			case SHORT_RANGE:
				for (int i = 0;i<8;++i) {
					res += xpow*srcoeffs[i];
					xpow *= adc1;
				}
				break;
			case LONG_RANGE:
				for (int i = 0;i<8;++i) {
					res += xpow*lrcoeffs[i];
					xpow *= adc1;
				}
				break;
		}

		//double sensor_value = distance_sensor_->sample(distance);
		std_msgs::Float64 cdmsg;
		cdmsg.data = res;
		cdistance_publisher_.publish(cdmsg);

	}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ir_verifier_node");

	ir_verifier_node irvn;

	irvn.init();

	ros::Rate loop_rate(10.0);

	while(irvn.n_.ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
