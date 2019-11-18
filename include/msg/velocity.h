#ifndef NOVATEL_MSGS_VELOCITY_H
#define NOVATEL_MSGS_VELOCITY_H
#include <stdint.h>
#include <string>

#include "base.h"
#include <boost/shared_ptr.hpp>
#include "messageheader.h"


using namespace base;

namespace novatel_gps_msgs
{
	typedef struct Velocity
	{

		MessageHeader novatel_msg_header;

		std::string solution_status;

		std::string velocity_type;
		
		//A measure of the latency in the velocity time tag in seconds.
		float32 latency;

		// Differential age in seconds
		float32 age;

		//Horizontal speed over ground, meters / second
		float64 horizontal_speed;

		//Direction of motion over ground with respect to true North, degrees
		float64 track_ground;

		//Vertical speed, where positive values correspond to increasing altitude, meters / second
		float64 vertical_speed;

	}VELOCITY;

	typedef boost::shared_ptr< novatel_gps_msgs::Velocity > VelocityPtr;
	typedef boost::shared_ptr< novatel_gps_msgs::Velocity const> VelocityConstPtr;
}// end of namespace
#endif