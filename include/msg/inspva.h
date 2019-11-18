#ifndef NOVATEL_MSGS_INSPVA_H
#define NOVATEL_MSGS_INSPVA_H
#include <stdint.h>
#include <string>
#include <boost/shared_ptr.hpp>
#include "base.h"
#include "messageheader.h"
#include "extendedsolutionstatus.h"

using namespace base;

namespace novatel_gps_msgs
{
	typedef struct Inspva
	{

		MessageHeader novatel_msg_header;

		uint32 week;
		float64 seconds;

		float64 latitude;
		float64 longitude;
		float64 height;


		float64 north_velocity;
		float64 east_velocity;
		float64 up_velocity;

		float64 roll;
		float64 pitch;
		float64 azimuth;

		std::string  status;
	}INSPVA;
	typedef boost::shared_ptr< novatel_gps_msgs::Inspva > InspvaPtr;
	typedef boost::shared_ptr< novatel_gps_msgs::Inspva const> InspvaConstPtr;
}// end of namespace
#endif