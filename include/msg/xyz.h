#ifndef NOVATEL_MSGS_XYZ_H
#define NOVATEL_MSGS_XYZ_H
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
	typedef struct XYZ
	{

		MessageHeader novatel_msg_header;

		std::string solution_status;

		std::string position_type;
		
		float64 x;
		float64 y;
		float64 z;

		//Position Standard Deviation(m)
		float32 x_sigma;
		float32 y_sigma;
		float32 z_sigma;

		//Velocity Data
		std::string velocity_solution_status;
		std::string velocity_type;			
		
		float64 x_vel;
		float64 y_vel;
		float64 z_vel;

		//Velocity Standard Deviation(m / s)
		float32 x_vel_sigma;
		float32 y_vel_sigma;
		float32 z_vel_sigma;

		std::string base_station_id;
		float32 velocity_latency;

		float32 diff_age;
		float32 solution_age;

		uint8 num_satellites_tracked;
		uint8 num_satellites_used_in_solution;
		uint8 num_gps_and_glonass_l1_used_in_solution;
		uint8 num_gps_and_glonass_l1_and_l2_used_in_solution;

		ExtendedSolutionStatus extended_solution_status;

		SignalMask signal_mask;

	}XYZ;

	typedef boost::shared_ptr< novatel_gps_msgs::XYZ > XYZPtr;
	typedef boost::shared_ptr< novatel_gps_msgs::XYZ const> XYZConstPtr;
}// end of namespace
#endif