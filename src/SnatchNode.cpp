#include <ros/ros.h>

#include <snatch/SnatchNode.h>
#include <snatch/SnatchSerial.h>
#include <snatch/SnatchSerialException.h>
#include <string>
#include <stdint.h>

namespace snatch {

SnatchNode::SnatchNode() {
	command_sub_ = nh_.subscribe("command", 1, &SnatchNode::commandCallback, this);

	get_state_srv_ = nh_.advertiseService("getState", &SnatchNode::getStateSrvCallback, this);
	get_value_srv_ = nh_.advertiseService("getValue", &SnatchNode::getValueSrvCallback, this);
	set_value_srv_ = nh_.advertiseService("setValue", &SnatchNode::setValueSrvCallback, this);

	ros::NodeHandle nh_private("~");

	frame_id_ = nh_private.param<std::string>("frame_id", "Sparky");

	std::string port = nh_private.param<std::string>("port", "/dev/ttyUSB0");
	int baud_rate = nh_private.param<int>("baud_rate", 921600); //115200);

	try {
		serial_ = new SnatchSerial(port, baud_rate, new SnatchParser(this));
	} catch (snatch::SnatchSerialException &e) {
		ROS_FATAL("%s", e.what());
		ros::shutdown();
	}

	// Stop streaming.
	serial_->sendCommandChar('+');
	state_.streamming = true;

	// Fallback to RX.
	serial_->sendCommandChar('<');
	state_.fallback = true;

	// Get the current status.
	serial_->sendCommandChar('?');
}

SnatchNode::~SnatchNode() {
	serial_->sendCommandChar('-');
	state_.streamming = false;
	delete serial_;
}

ros::Time SnatchNode::toRosTime(uint32_t fc_time) {
	// TODO Time sync with FC and calculation
	return ros::Time::now();
}

void SnatchNode::handleParserEvent(const snatch_imu_event_t * const event) {
//	std::cerr << "IMU Time:" << event->time;
//	std::cerr << " Roll:" << event->roll << " Pitch:" << event->pitch << " Yaw:" << event->yaw << "\t";
//	std::cerr << "A(" << event->accX << "\t" << event->accY << "\t" << event->accZ << ")\t";
//	std::cerr << " G(" << event->gyroX << "\t" << event->gyroY << "\t" << event->gyroZ << ")\t";
//	std::cerr << "M(" << event->magX << "\t" << event->magY << "\t" << event->magZ << ")\n";

	Attitude attitude_msg;
	attitude_msg.header.frame_id = frame_id_;
	attitude_msg.header.stamp = toRosTime(event->time);
	attitude_msg.roll = event->roll;
	attitude_msg.pitch = event->pitch;
	attitude_msg.yaw = event->yaw;

	if (attitude_pub_.getTopic().empty()) {
		attitude_pub_ = nh_.advertise<Attitude>("attitude", false);
	}
	attitude_pub_.publish(attitude_msg);
}

#define channelTo0to1(v)	(((float) (v)) / 1000)
#define channelTo1to1(v)	((((float)(v)) - 500) / 500)

void SnatchNode::handleParserEvent(const snatch_rx_event_t * const event) {
//	std::cerr << "RX  Time:" << event->time;
//	std::cerr << " Channel values:";
//	std::cerr << event->channels[0] << ":";
//	std::cerr << event->channels[1] << ":";
//	std::cerr << event->channels[2] << ":";
//	std::cerr << event->channels[3] << ":";
//	std::cerr << event->channels[4] << ":";
//	std::cerr << event->channels[5] << ":";
//	std::cerr << event->channels[6] << ":";
//	std::cerr << event->channels[7] << ":";
//	std::cerr << event->channels[8] << ":";
//	std::cerr << event->channels[9] << ":";
//	std::cerr << event->channels[10] << ":";
//	std::cerr << event->channels[11] << ":";
//	std::cerr << event->channels[12] << ":";
//	std::cerr << event->channels[13] << ":";
//	std::cerr << event->channels[14] << ":";
//	std::cerr << event->channels[15] << "\n";

	Command command;
	command.header.stamp = toRosTime(event->time);
	updateChannels(event->channels, command.channels);

	if (rx_command_pub_.getTopic().empty()) {
		rx_command_pub_ = nh_.advertise<Command>("rx_command", 1);
	}
	rx_command_pub_.publish(command);
}

void SnatchNode::handleParserEvent(const snatch_status_event_t * const event) {
//	std::cerr << "ST  Time:" << event->time;
//	std::cerr << " Channel values:";
//	std::cerr << event->channels[0] << ":";
//	std::cerr << event->channels[1] << ":";
//	std::cerr << event->channels[2] << ":";
//	std::cerr << event->channels[3] << ":";
//	std::cerr << event->channels[4] << ":";
//	std::cerr << event->channels[5] << ":";
//	std::cerr << event->channels[6] << ":";
//	std::cerr << event->channels[7] << ":";
//	std::cerr << event->channels[8] << ":";
//	std::cerr << event->channels[9] << ":";
//	std::cerr << event->channels[10] << ":";
//	std::cerr << event->channels[11] << ":";
//	std::cerr << event->channels[12] << ":";
//	std::cerr << event->channels[13] << ":";
//	std::cerr << event->channels[14] << ":";
//	std::cerr << event->channels[15] << "\n";
	updateChannels(event->channels, state_.channels);
}

#define channelFrom0to1(v)	((uint16_t)(v * 1000))
#define channelFrom1to1(v)	((uint16_t)(v * 500) + 500)

void SnatchNode::commandCallback(Command::ConstPtr command) {
	serial_->setChannelValue(0, channelFrom1to1(command->channels.roll));
	serial_->setChannelValue(1, channelFrom1to1(command->channels.pitch));
	serial_->setChannelValue(2, channelFrom1to1(command->channels.yaw));
	serial_->setChannelValue(3, channelFrom0to1(command->channels.throttle));

	for (int i = 0; i < 8; i++) {
		serial_->setChannelValue(i + 4, channelFrom0to1(command->channels.aux[i]));
	}

	serial_->sendChannelValues();
}

bool SnatchNode::getStateSrvCallback(GetState::Request &req, GetState::Response &res) {
	// Copy state_ to response.
	res = state_;
	return true;
}

bool SnatchNode::getValueSrvCallback(GetValue::Request &req, GetValue::Response &res) {
	if (req.key == "streaming") {
		res.value = state_.streamming ? 1.0 : 0.0;
		return true;
	} else if (req.key == "fallback") {
		res.value = state_.fallback ? 1.0 : 0.0;
		return true;
	}
	return false;
}

bool SnatchNode::setValueSrvCallback(SetValue::Request &req, SetValue::Response &res) {
	if (req.key == "streaming") {
		res.oldValue = state_.streamming ? 1.0 : 0.0;
		state_.streamming = req.value == 1.0;
		serial_->sendCommandChar(state_.streamming ? '+' : '-');
		return true;
	} else if (req.key == "fallback") {
		res.oldValue = state_.fallback ? 1.0 : 0.0;
		state_.fallback = req.value == 1.0;
		serial_->sendCommandChar(state_.fallback ? '<' : '>');
		return true;
	}
	return false;
}

void updateChannels(const uint16_t* const from, Channels &to) {
	to.roll = channelTo1to1(from[0]);
	to.pitch = channelTo1to1(from[1]);
	to.yaw = channelTo1to1(from[2]);
	to.throttle = channelTo0to1(from[3]);

	for (int i = 0; i < 8; i++) {
		to.aux[i] = channelTo0to1(from[i + 4]);
	}
}

} // namespace snatch

int main(int argc, char **argv) {
	ros::init(argc, argv, "SnatchNode");
	snatch::SnatchNode node;
	ros::spin();
}

