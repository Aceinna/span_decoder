#ifndef NOVATEL_MSGS_INSPVAX_H
#define NOVATEL_MSGS_INSPVAX_H
#include <stdint.h>
#include <string>
#include <boost/shared_ptr.hpp>
#include "base.h"
#include "messageheader.h"
#include "extendedsolutionstatus.h"

using namespace base;

namespace novatel_gps_msgs
{
	typedef struct Inspvax
	{

		MessageHeader novatel_msg_header;

		uint16 ins_status_int;
		std::string ins_status;

		uint16 position_type_int;
		std::string position_type;

		float64 latitude;
		float64 longitude;
		float64 altitude;

		float32 undulation;

		float64 north_velocity;
		float64 east_velocity;
		float64 up_velocity;

		float64 roll;
		float64 pitch;
		float64 azimuth;

		float32 latitude_std;
		float32 longitude_std;
		float32 altitude_std;

		float32 north_velocity_std;
		float32 east_velocity_std;
		float32 up_velocity_std;

		float32 roll_std;
		float32 pitch_std;
		float32 azimuth_std;



		ExtendedSolutionStatus extended_status;

		uint16 seconds_since_update;
	}INSPVAX;
	typedef boost::shared_ptr< novatel_gps_msgs::Inspvax > InspvaxPtr;
	typedef boost::shared_ptr< novatel_gps_msgs::Inspvax const> InspvaxConstPtr;
}// end of namespace
#endif