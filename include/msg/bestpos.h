#ifndef NOVATEL_MSGS_BESTPOS_H
#define NOVATEL_MSGS_BESTPOS_H
#include <stdint.h>
#include <string>

#include "base.h"
#include <boost/shared_ptr.hpp>
#include "messageheader.h"
#include "extendedsolutionstatus.h"
#include "signalmark.h"


using namespace base;

namespace novatel_gps_msgs
{
	typedef struct BestPos
	{

		MessageHeader novatel_msg_header;

		std::string solution_status;

		std::string position_type;

		float64 lat;           //Latitude (degrees)
		float64 lon;          //Longitude (degrees)
		float64 height;           //Height above mean sea level (metres)

		float32 undulation;         //Undulation - the relationship between the geoid and the ellipsoid(m) of the chosen datum
		std::string datum_id;

		float32 lat_sigma;       //Latitude standard deviation (metres)
		float32 lon_sigma;      //Longitude standard deviation (metres)
		float32 height_sigma;       //Height standard deviation (metres)

		std::string base_station_id;             //Base station ID

		float32 diff_age;           //Differential age in seconds
		float32 solution_age;            //Solution age in seconds

		uint8 num_satellites_tracked;
		uint8 num_satellites_used_in_solution;
		uint8 num_gps_and_glonass_l1_used_in_solution;
		uint8 num_gps_and_glonass_l1_and_l2_used_in_solution;

		ExtendedSolutionStatus extended_solution_status;

		SignalMask signal_mask;

	}BESTPOS;

	typedef boost::shared_ptr< novatel_gps_msgs::BestPos > BestPosPtr;
	typedef boost::shared_ptr< novatel_gps_msgs::BestPos const> BestPosConstPtr;
}// end of namespace
#endif