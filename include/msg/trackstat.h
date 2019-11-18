#ifndef NOVATEL_MSGS_TRACKSTAT_H
#define NOVATEL_MSGS_TRACKSTAT_H
#include <stdint.h>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

#include "trackstatchannel.h"

#include "base.h"

using namespace base;

namespace novatel_gps_msgs
{
	typedef struct Trackstat
	{
		std::string solution_status;
		std::string position_type;

		//Tracking elevation cutff - off angle
		float32 cutoff;

		std::vector<TrackstatChannel> channels;
	}TRACKSTAT;
	typedef boost::shared_ptr< novatel_gps_msgs::Trackstat > TrackstatPtr;
	typedef boost::shared_ptr< novatel_gps_msgs::Trackstat const> TrackstatConstPtr;
}// end of namespace
#endif

