#ifndef NOVATEL_MSGS_RAWIMU_H
#define NOVATEL_MSGS_RAWIMU_H
#include <stdint.h>
#include <string>
#include <boost/shared_ptr.hpp>

#include "base.h"
#include "messageheader.h"

using namespace base;

namespace novatel_gps_msgs
{
	typedef struct Rawimu
	{
		MessageHeader novatel_msg_header;
		uint32 gps_week_num;
		float64 gps_seconds;
		uint32 IMUStatus;


		int32 x_accel;
		int32 y_accel;
		int32 z_accel;

		int32 x_gyro;
		int32 y_gyro;
		int32 z_gyro;
	}RAWIMU;
	typedef boost::shared_ptr< novatel_gps_msgs::Rawimu > RawimuPtr;
	typedef boost::shared_ptr< novatel_gps_msgs::Rawimu const> RawimuConstPtr;
}// end of namespace
#endif
