#ifndef NOVATEL_MSGS_EXTENDEDSOLUTIONSTATUS_H
#define NOVATEL_MSGS_EXTENDEDSOLUTIONSTATUS_H
#include <stdint.h>
#include <string>

#include "base.h"
#include "messageheader.h"

using namespace base;

namespace novatel_gps_msgs
{
	typedef struct ExtendedSolutionStatus
	{
		uint32 original_mask;
		bool advance_rtk_verified;
		std::string psuedorange_iono_correction;
	}RTEXTENDEDSOLUTIONSTATUSPOS;
}// end of namespace
#endif
