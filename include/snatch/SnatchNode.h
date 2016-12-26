/**
 * \file fcu_io.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef SNATCH_NODE_H
#define SNATCH_NODE_H

#include <map>
#include <string>

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/Range.h>

#include <snatch/Attitude.h>
#include <snatch/Command.h>
#include <snatch/GetValue.h>
#include <snatch/SetValue.h>
#include <snatch/SnatchSerial.h>
#include <snatch/SnatchParser.h>

#include <std_srvs/Trigger.h>



namespace snatch {

class SnatchNode: public SnatchParserListener {
public:
	SnatchNode();
	~SnatchNode();

	virtual void handleParserEvent(const snatch_imu_event_t * const event);
	virtual void handleParserEvent(const snatch_rx_event_t * const event);
	virtual void handleParserEvent(const snatch_status_event_t * const event);

private:
	ros::Time toRosTime(uint32_t fc_time);

	// ROS message callbacks
	void commandCallback(Command::ConstPtr msg);

	bool getValueSrvCallback(GetValue::Request &req, GetValue::Response &res);
	bool setValueSrvCallback(SetValue::Request &req, SetValue::Response &res);

	SnatchSerial *serial_;

	ros::NodeHandle nh_;

	ros::Subscriber command_sub_;

	ros::Publisher attitude_pub_;

	ros::ServiceServer get_value_srv_;

	ros::ServiceServer set_value_srv_;

	std::string frame_id_;
};

} // namespace snatch

#endif // SNATCH_NODE_H
