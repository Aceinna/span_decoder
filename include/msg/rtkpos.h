#ifndef NOVATEL_MSGS_RTKPOS_H
#define NOVATEL_MSGS_RTKPOS_H
#include <stdint.h>
#include <string>

#include "base.h"
#include "messageheader.h"

using namespace base;

namespace novatel_gps_msgs
{
	typedef struct Rtkvel
	{
		MessageHeader novatel_msg_header;

		std::string ins_status;

		std::string position_type;

		float64 latitude;           //Latitude (degrees)
		float64 longitude;          //Longitude (degrees)
		float64 altitude;           //Height above mean sea level (metres)

		float32 undulation;         //Undulation - the relationship between the geoid and the ellipsoid(m) of the chosen datum

		float32 latitude_std;       //Latitude standard deviation (metres)
		float32 longitude_std;      //Longitude standard deviation (metres)
		float32 altitude_std;       //Height standard deviation (metres)

		char Stn_ID[4];             //Base station ID

		float32 Diff_age;           //Differential age in seconds
		float32 Sol_age;            //Solution age in seconds


	}RTKVEL;
}// end of namespace
#endif