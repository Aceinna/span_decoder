#ifndef NOVATEL_MSGS_BESTVEL_H
#define NOVATEL_MSGS_BESTVEL_H
#include <stdint.h>
#include <string>

#include "base.h"
#include "messageheader.h"

using namespace base;

namespace novatel_gps_msgs
{
	typedef struct Bestvel
	{
		MessageHeader novatel_msg_header;

		std::string ins_status;

		std::string position_type;

		/*A measure of the latency in the velocity time tag inseconds.
		It should be subtracted from the time to give improved results*/
		float32 latency;

		float32 age;               //Differential age in seconds
		float64 hor_spd;           //Horizontal speed over ground, in metres per second
		float64 trk_gnd;           //Actual direction of motion over ground (track over ground) with respect to True North, in degrees
	   /*Vertical speed, in metres per second, where positive values indicate increasing altitude(up) and
		negative values indicate decreasing altitude(down)*/
		float64 vect_spd;
	}BESTVEL;
}// end of namespace
#endif
