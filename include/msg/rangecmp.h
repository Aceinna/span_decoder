#ifndef NOVATEL_MSGS_RANGECMP_H
#define NOVATEL_MSGS_RANGECMP_H
#include <stdint.h>
#include <string>
#include <boost/shared_ptr.hpp>

#include "base.h"
#include "messageheader.h"
#include "rangeinformation.h"
#include <vector>

using namespace base;

namespace novatel_gps_msgs
{
	typedef struct Rangecmp
	{
		MessageHeader novatel_msg_header;
		/*number of observations*/
		int32 numb_of_observ;

		/*range information for the observed*/
		std::vector<RangeInformation> info;
	}RANGECMP;
	typedef boost::shared_ptr< novatel_gps_msgs::Rangecmp > RangecmpPtr;
	typedef boost::shared_ptr< novatel_gps_msgs::Rangecmp const> RangecmpConstPtr;
}// end of namespace
#endif

