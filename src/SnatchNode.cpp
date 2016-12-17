#include <ros/ros.h>

#include <snatch/SnatchNode.h>
#include <snatch/SnatchSerial.h>
#include <snatch/SnatchSerialException.h>
#include <string>
#include <stdint.h>

namespace snatch {

SnatchNode::SnatchNode() {
	command_sub_ = nh_.subscribe("command", 1, &SnatchNode::commandCallback, this);

	attitude_pub_ = nh_.advertise<std_msgs::Bool>("attitude", 1, true);

	get_value_srv_ = nh_.advertiseService("getValue", &SnatchNode::getValueSrvCallback, this);
	set_value_srv_ = nh_.advertiseService("setValue", &SnatchNode::setValueSrvCallback, this);

	ros::NodeHandle nh_private("~");

	frame_id_ = nh_private.param<std::string>("frame_id", "FCU");

	std::string port = nh_private.param<std::string>("port", "/dev/ttyUSB0");
	int baud_rate = nh_private.param<int>("baud_rate", 115200);

	try {
		serial_ = new SnatchSerial(port, baud_rate, new SnatchParser(this));
	} catch (snatch::SnatchSerialException e) {
		ROS_FATAL("%s", e.what());
		ros::shutdown();
	}

	std_msgs::Bool unsaved_msg;
	unsaved_msg.data = false;
}

SnatchNode::~SnatchNode() {
	delete serial_;
}

void SnatchNode::handleParserEvent(const snatch_imu_event_t * const event) {

}

void SnatchNode::handleParserEvent(const snatch_rx_event_t * const event) {

}

void SnatchNode::handleParserEvent(const snatch_status_event_t * const event) {

}

void SnatchNode::commandCallback(snatch::Command::ConstPtr msg) {

}

bool SnatchNode::getValueSrvCallback(snatch::GetValue::Request &req, snatch::GetValue::Response &res) {
	return false;
}

bool SnatchNode::setValueSrvCallback(snatch::SetValue::Request &req, snatch::SetValue::Response &res) {
	return false;
}

} // namespace snatch

int main(int argc, char **argv) {
	ros::init(argc, argv, "SnatchNode");
	snatch::SnatchNode node;
	ros::spin();
}

