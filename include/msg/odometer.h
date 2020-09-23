#ifndef NOVATEL_MSGS_ODOMETER_H
#define NOVATEL_MSGS_ODOMETER_H
#include <stdint.h>
#include <string>
#include <boost/shared_ptr.hpp>
#include "messageheader.h"
#include "base.h"

using namespace base;

namespace novatel_gps_msgs
{
	typedef struct Odometer
	{
		MessageHeader novatel_msg_header;
		uint32  gps_week_num;
		float64 gps_seconds;
		uint8   mode;  // 2 increaded wheel_tick
		float64 speed;
		uint8   fwd;
		uint64  wheel_tick;
	}Odometer;

	typedef boost::shared_ptr< novatel_gps_msgs::Odometer > OdometerPtr;
	typedef boost::shared_ptr< novatel_gps_msgs::Odometer const> OdometerConstPtr;
} // end of namespace
#endif