#ifndef NOVATEL_MSGS_RAWIMUSX_H
#define NOVATEL_MSGS_RAWIMUSX_H
#include <stdint.h>
#include <string>

#include "base.h"
#include "messageheader.h"
#include <boost/shared_ptr.hpp>

using namespace base;

namespace novatel_gps_msgs
{
	typedef struct Rawimusx
	{
		ShortMessageHeader novatel_msg_header;
        /*IMU Info Bits
          Bit 0: If set, an IMU error was detected. Check the IMU Status field for details.
          Bit 1: If set, the IMU data is encrypted and should not be used.
          Bits 2 to 7: Reserved*/
		uint8 imuinfo;

		/*IMU Type identifier.*/
		uint8  imutype;

		uint32 gps_week_num;
		float64 gps_seconds;

		uint32 IMUStatus;

		int32 x_accel;
		int32 y_accel;
		int32 z_accel;

		int32 x_gyro;
		int32 y_gyro;
		int32 z_gyro;
	}RAWIMUSX;
	typedef boost::shared_ptr< novatel_gps_msgs::Rawimusx > RawimusxPtr;
	typedef boost::shared_ptr< novatel_gps_msgs::Rawimusx const> RawimusxConstPtr;
}// end of namespace
#endif
