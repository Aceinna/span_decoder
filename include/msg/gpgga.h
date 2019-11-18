#ifndef NOVATEL_MSGS_GPGGA_H
#define NOVATEL_MSGS_GPGGA_H
#include <stdint.h>
#include <string>
#include <boost/shared_ptr.hpp>
#include "base.h"

using namespace base;

namespace novatel_gps_msgs
{
	typedef struct Gpgga
	{
		std::string message_id;

		float64 utc_seconds;  //UTC seconds from midnight
		
		float64 lat;
		float64 lon;

		std::string lat_dir;
		std::string lon_dir;

		enum GPS_QUAL
		{
			 GPS_QUAL_INVALID = 0,
			 GPS_QUAL_SINGLE_POINT = 1,
			 GPS_QUAL_PSEUDORANGE_DIFFERENTIAL = 2,
			 GPS_QUAL_RTK_FIXED_AMBIGUITY_SOLUTION = 4,
			 GPS_QUAL_RTK_FLOATING_AMBIGUITY_SOLUTION = 5,
			 GPS_QUAL_DEAD_RECKONING_MODE = 6,
			 GPS_QUAL_MANUAL_INPUT_MODE = 7,
			 GPS_QUAL_SIMULATION_MODE = 8,
			 GPS_QUAL_WASS = 9
		};

		uint32 gps_qual;

		uint32 num_sats;
		float32 hdop;
		float32 alt;
		std::string altitude_units;
		
		float32 undulation;
		std::string undulation_units;
		uint32 diff_age;
		std::string station_id;
	}GPGGA;

	typedef boost::shared_ptr< novatel_gps_msgs::Gpgga > GpggaPtr;
	typedef boost::shared_ptr< novatel_gps_msgs::Gpgga const> GpggaConstPtr;
} // end of namespace
#endif