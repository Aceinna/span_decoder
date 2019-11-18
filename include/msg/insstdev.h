#ifndef NOVATEL_MSGS_INSSTDEV_H
#define NOVATEL_MSGS_INSSTDEV_H
#include <stdint.h>
#include <string>
#include <boost/shared_ptr.hpp>
#include "base.h"
#include "messageheader.h"
#include "extendedsolutionstatus.h"

using namespace base;

namespace novatel_gps_msgs
{
	typedef struct Insstdev
	{

		MessageHeader novatel_msg_header;

		float32 latitude_dev;
		float32 longitude_dev;
		float32 height_dev;
		float32 north_velocity_dev;
		float32 east_velocity_dev;
		float32 up_velocity_dev;
		float32 roll_dev;
		float32 pitch_dev;
		float32 azimuth_dev;
		ExtendedSolutionStatus extended_solution_status;
		uint16 time_since_update;
	}INSSTDEV;
	typedef boost::shared_ptr< novatel_gps_msgs::Insstdev > InsstdevPtr;
	typedef boost::shared_ptr< novatel_gps_msgs::Insstdev const> InsstdevConstPtr;
}// end of namespace
#endif