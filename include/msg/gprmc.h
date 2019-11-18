#ifndef NOVATEL_MSGS_GPRMC_H
#define NOVATEL_MSGS_GPRMC_H
#include <stdint.h>
#include <string>
#include <boost/shared_ptr.hpp>
#include "base.h"

using namespace base;

namespace novatel_gps_msgs
{
	typedef struct Gprmc
	{
		std::string message_id;

		float64 utc_seconds;  //UTC seconds from midnight

		float64 lat;
		float64 lon;

		std::string position_status;

		std::string lat_dir;
		std::string lon_dir;

		float32 speed;
		float32 track;
		std::string date;
		float32 mag_var;
		std::string mag_var_direction;
		std::string mode_indicator;
	}GPRMC;

	typedef boost::shared_ptr< novatel_gps_msgs::Gprmc > GprmcPtr;
	typedef boost::shared_ptr< novatel_gps_msgs::Gprmc const> GprmcConstPtr;
}// end of namespace
#endif