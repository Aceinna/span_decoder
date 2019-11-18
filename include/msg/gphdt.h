#ifndef NOVATEL_MSGS_GPHDT_H
#define NOVATEL_MSGS_GPHDT_H
#include <stdint.h>
#include <string>
#include <boost/shared_ptr.hpp>
#include <vector>
#include "base.h"

using namespace base;

namespace novatel_gps_msgs
{
	typedef struct Gphdt
	{
		std::string message_id;

		//Heading in degrees(clockwise)
		float64 heading;

		//T: Indicates heading relative to True North
		std::string t;

	}GPHDT;

	typedef boost::shared_ptr< novatel_gps_msgs::Gphdt > GphdtPtr;
	typedef boost::shared_ptr< novatel_gps_msgs::Gphdt const> GphdtConstPtr;
} // end of namespace
#endif