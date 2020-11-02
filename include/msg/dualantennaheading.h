#ifndef NOVATEL_MSGS_DUALANTENNAHEADING_H
#define NOVATEL_MSGS_DUALANTENNAHEADING_H
#include <stdint.h>
#include <string>
#include <boost/shared_ptr.hpp>
#include "base.h"
#include "messageheader.h"
#include "extendedsolutionstatus.h"
#include "signalmark.h"
using namespace base;

namespace novatel_gps_msgs
{
	typedef struct DualAntennaHeading
	{

		MessageHeader novatel_msg_header;


		std::string solution_status;

		std::string position_type;

		int32 solution_status_int;
		int32 position_type_int;

		float32 baseline_length;         //Baseline length (m)
		float32 heading;                 //Heading in degrees [0,360)
		float32 pitch;                   //Pitch in degrees +- 90

		/*Orientation Standard Deviations (deg)*/
		float32 heading_sigma;
		float32 pitch_sigma;

		/*Station ids*/
		std::string station_id;

		/*Satellite Usage*/
		uint8 num_satellites_tracked;
		uint8 num_satellites_used_in_solution;
		uint8 num_satellites_above_elevation_mask_angle;
		uint8 num_satellites_above_elevation_mask_angle_l2;

		/*Enum for solution source*/
		uint8 solution_source;

		static uint8 SOURCE_PRIMARY_ANTENNA ;
		static uint8 SOURCE_SECONDARY_ANTENNA ;

		/*Extended Solution Status*/
		ExtendedSolutionStatus extended_solution_status;

		/*Signal Masks*/
		SignalMask signal_mask;
	}DUALANTENNAHEADING;
	typedef boost::shared_ptr< novatel_gps_msgs::DualAntennaHeading > DualAntennaHeadingPtr;
	typedef boost::shared_ptr< novatel_gps_msgs::DualAntennaHeading const> DualAntennaHeadingConstPtr;
}// end of namespace
#endif