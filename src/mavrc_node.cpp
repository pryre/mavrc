#include <ros/ros.h>

#include <tf/tf.h>
//#include <tf/Quaternion.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/OverrideRCIn.h>

#include <string.h>
#include <math.h>

#define PWM_MIN 1000
#define PWM_MAX 2000

//================================//
// Callback Functions             //
//================================//

mavros_msgs::AttitudeTarget attIn;

void cmd_att_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg) {
	attIn = *msg;
}

uint16_t scaleInput(const double input, const double val_min, const double val_max) {
	double val = 0.0;
	uint16_t out = 0.0;

	if( input < val_min )
		val = val_min;
	if( input > val_max )
		val = val_max;

	const uint16_t pwm_w = PWM_MAX - PWM_MIN;
	const double val_w = val_max - val_min;

	return ( uint16_t( ( val / val_w ) * pwm_w ) + PWM_MIN );
}

//================================//
// Main Function                  //
//================================//
int main(int argc, char **argv) {
	//================================//
	// Initialize node                //
	//================================//
	ros::init(argc, argv, "mavel" );
	ros::NodeHandle nh( ros::this_node::getName() );

	double commandTimeout = 0.1; //TODO: Params
	bool freshCommands = false;		//The thread will be locked to only sending safety values

	double param_loop_rate = 20.0;
	double param_trottle_threshold = 0.1;
	bool thr_threshold_passed = false;

	double param_rate_yaw_max = 1.0;
	double param_tilt_max = 0.39;	//22.5 Deg

	std::string param_cmd_att = "input";
	std::string param_cmd_rc = "/mavros/rc/override";

	//Subscribers
	ros::Subscriber cmd_att_sub = nh.subscribe<mavros_msgs::AttitudeTarget>
		(param_cmd_att, 10, cmd_att_cb);

	ros::Publisher rc_pub = nh.advertise<mavros_msgs::OverrideRCIn>
		(param_cmd_rc, 10);

	ros::Rate rate(param_loop_rate);
	ros::spinOnce();

	//Sit and wait until the inputs are ready
	while( ros::ok() && !freshCommands ) {
		freshCommands = commandTimeout > ( ros::Time::now() - attIn.header.stamp ).toSec();

		ROS_INFO_THROTTLE( 2.0, "Waiting for inputs to become avaliable: [cmd: %d]", freshCommands );

		ros::spinOnce();
		rate.sleep();
	}

	ROS_INFO( "Inputs avaliable: [cmd: %d]", freshCommands );
	ROS_INFO( "Ready to begin broadcasting!" );

	//================================//
	// Main Loop                      //
	//================================//
	while( ros::ok() ) {
		mavros_msgs::OverrideRCIn rc_out;

		//Check to make sure there have been new commands
		if( commandTimeout < ( ros::Time::now() - attIn.header.stamp ).toSec() ) {
			freshCommands = false;
			ROS_ERROR_ONCE( "[Timeout] No new velocity commands" );
		}

		if(attIn.type_mask != (attIn.IGNORE_ROLL_RATE + attIn.IGNORE_PITCH_RATE) ) {
			freshCommands = false;
			ROS_ERROR_ONCE( "[Type Error] Attitude commands do not specify expected inputs" );
		}

		//Main control loop
		//If inputs are lost at all, panic
		if( freshCommands && thr_threshold_passed ) {
			ROS_INFO_THROTTLE( 2.0, "Publishing attitude setpoints..." );

			double roll;
			double pitch;
			double yaw;
			tf::Quaternion q;
			tf::quaternionMsgToTF(attIn.orientation, q);
			tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

			rc_out.channels[0] = scaleInput(roll, -param_tilt_max, param_tilt_max);	//Roll Angle
			rc_out.channels[1] = scaleInput(pitch, -param_tilt_max, param_tilt_max);	//Pitch Angle
			rc_out.channels[2] = scaleInput( attIn.thrust, 0.0, 1.0); //Thrust
			rc_out.channels[3] = scaleInput( attIn.body_rate.z, -param_rate_yaw_max, param_rate_yaw_max );	//Yaw Rate

			rc_out.channels[4] = rc_out.CHAN_NOCHANGE;
			rc_out.channels[5] = rc_out.CHAN_NOCHANGE;
			rc_out.channels[6] = rc_out.CHAN_NOCHANGE;
			rc_out.channels[7] = rc_out.CHAN_NOCHANGE;

			ROS_INFO_THROTTLE( 2.0, "Goal Commands: [%0.2f, %0.2f, %0.2f, %0.2f]", roll, pitch, attIn.thrust, attIn.body_rate.z );
		} else {
			ROS_INFO_THROTTLE( 2.0, "Publishing to hold orientation, and low thrust..." );

			if(attIn.thrust > param_trottle_threshold) {
				thr_threshold_passed = true;
				ROS_INFO_ONCE("Velocity commands non-zero, activating flight");
			}
		}

		rc_pub.publish(rc_out);

		//Sleep //================================================================
		rate.sleep();
		ros::spinOnce();
	}

	return 0;
}

