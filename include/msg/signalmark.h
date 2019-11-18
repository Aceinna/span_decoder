#ifndef NOVATEL_MSGS_SIGNALMARK_H
#define NOVATEL_MSGS_SIGNALMARK_H
#include <stdint.h>
#include <string>

#include "base.h"
#include "messageheader.h"

using namespace base;

namespace novatel_gps_msgs
{
	typedef struct SignalMask
	{
	/*Bit    Mask      Description
			#  0     0x01      GPS L1 used in Solution
			#  1     0x02      GPS L2 used in Solution
			#  2     0x04      GPS L5 used in Solution
			#  3     0x08 < Reserved >
			#  4     0x10      GLONASS L1 used in Solution
			#  5     0x20      GLONASS L2 used in Solution
			# 6 - 7  0x40 - 0x80 < Reserved >*/
		uint32 original_mask;
		bool gps_L1_used_in_solution;
		bool gps_L2_used_in_solution;
		bool gps_L3_used_in_solution;
		bool glonass_L1_used_in_solution;
		bool glonass_L2_used_in_solution;
	}SIGNALMARK;
}// end of namespace
#endif

