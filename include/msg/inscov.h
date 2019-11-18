#ifndef NOVATEL_MSGS_INSCOV_H
#define NOVATEL_MSGS_INSCOV_H
#include <stdint.h>
#include <string>
#include <boost/shared_ptr.hpp>
#include "base.h"
#include "messageheader.h"

using namespace base;

namespace novatel_gps_msgs
{
	typedef struct Inscov
	{

		MessageHeader novatel_msg_header;
		uint32 week;
		float64 seconds;
		float64 position_covariance[9];
		float64 attitude_covariance[9];
		float64 velocity_covariance[9];
	}INSCOV;
	typedef boost::shared_ptr< novatel_gps_msgs::Inscov > InscovPtr;
	typedef boost::shared_ptr< novatel_gps_msgs::Inscov const> InscovConstPtr;
}// end of namespace
#endif