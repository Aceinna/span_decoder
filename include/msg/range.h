#ifndef NOVATEL_MSGS_RANGE_H
#define NOVATEL_MSGS_RANGE_H
#include <stdint.h>
#include <string>
#include <boost/shared_ptr.hpp>
#include <vector>


#include "base.h"
#include "messageheader.h"
#include "rangeinformation.h"

using namespace base;

namespace novatel_gps_msgs
{
	typedef struct Range
	{
		MessageHeader novatel_msg_header;
		/*number of observations*/
		int32 numb_of_observ;

		/*range information for the observed*/
		std::vector<RangeInformation> info;
	}RANGE;
	typedef boost::shared_ptr< novatel_gps_msgs::Range > RangePtr;
	typedef boost::shared_ptr< novatel_gps_msgs::Range const> RangeConstPtr;
}// end of namespace
#endif

