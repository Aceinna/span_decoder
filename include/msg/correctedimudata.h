#ifndef NOVATEL_MSGS_CORRECTEDIMUDATA_H
#define NOVATEL_MSGS_CORRECTEDIMUDATA_H
#include <stdint.h>
#include <string>
#include <boost/shared_ptr.hpp>
#include "base.h"
#include "messageheader.h"

using namespace base;

namespace novatel_gps_msgs
{
	typedef struct CorrectedImuData
	{

		MessageHeader novatel_msg_header;

		uint32 gps_week_num;
		float64 gps_seconds;

		//All measurements in this message are instantaneous values;
		//attitude rate is in radians
		float64 pitch_rate;
		float64 roll_rate;
		float64 yaw_rate;

		//accelerations are in m / s
		float64 lateral_acceleration;
		float64 longitudinal_acceleration;
		float64 vertical_acceleration;

	}CORRECTEDIMUDATA;
	typedef boost::shared_ptr< novatel_gps_msgs::CorrectedImuData > CorrectedImuDataPtr;
	typedef boost::shared_ptr< novatel_gps_msgs::CorrectedImuData const> CorrectedImuDataConstPtr;
}// end of namespace
#endif