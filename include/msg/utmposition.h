#ifndef NOVATEL_MSGS_UTMPOSITION_H
#define NOVATEL_MSGS_UTMPOSITION_H
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
	typedef struct UtmPosition
	{

		MessageHeader novatel_msg_header;

		std::string solution_status;

		std::string position_type;
		
		//Postion Data
		uint32 lon_zone_number;
		std::string lat_zone_letter;
		float64 northing;
		float64 easting;
		float64 height;

		float undulation;         //Undulation - the relationship between the geoid and the ellipsoid(m) of the chosen datum
		std::string datum_id;
		//Accuracy Statistics(units ? )
		float32 northing_sigma;
		float32 easting_sigma;
		float32 height_sigma;
		std::string base_station_id;
		float32 diff_age;
		float32 solution_age;

		uint8 num_satellites_tracked;
		uint8 num_satellites_used_in_solution;
		uint8 num_gps_and_glonass_l1_used_in_solution;
		uint8 num_gps_and_glonass_l1_and_l2_used_in_solution;

		ExtendedSolutionStatus extended_solution_status;

		SignalMask signal_mask;

	}UTMPOSITION;

	typedef boost::shared_ptr< novatel_gps_msgs::UtmPosition > UtmPositionPtr;
	typedef boost::shared_ptr< novatel_gps_msgs::UtmPosition const> UtmPositionConstPtr;
}// end of namespace
#endif